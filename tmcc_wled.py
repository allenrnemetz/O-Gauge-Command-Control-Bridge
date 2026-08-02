"""TMCC accessory -> WLED bridge (ESP32 at 192.168.0.10 by default).

Listens for TMCC Accessory/Group packets (cmd_type 0b11) and maps address +
data fields to WLED actions via the WLED JSON HTTP API.

Quick use:

    from tmcc_wled import WLEDController

    controller = WLEDController(
        host="192.168.0.10",
        mapping={
            (50, 0x01): "on",        # ACC 50 keypad 1 -> lights on
            (50, 0x02): "off",       # ACC 50 keypad 2 -> lights off
            (50, 0x03): "cycle",     # ACC 50 keypad 3 -> next preset
        },
        pattern_presets=[1, 2, 3],     # optional list of WLED preset IDs to cycle
    )

    # In your TMCC packet loop when cmd_type == 0b11:
    controller.handle_packet(packet_bytes)

Supported actions
- "on" / "off"
- "color:#RRGGBB"
- "brightness:<0-255>"
- "preset:<id>" (loads a WLED preset ID)
- "cycle" (cycles through pattern_presets list if provided)

Notes
- Uses only the Python stdlib (http.client, json); no extra deps.
- Safe to run on non-Pi systems since it is network-only.
- WLED API docs: https://kno.wled.ge/interfaces/json/
"""

from __future__ import annotations

import http.client
import json
import logging
import random
import threading
import time
from typing import Dict, Iterable, List, Optional, Tuple

logger = logging.getLogger(__name__)


# ---------- TMCC Accessory parsing ----------

def parse_tmcc_switch_or_accessory(packet: Iterable[int], cmd_type: Optional[int] = None) -> Optional[Dict[str, int]]:
    """Parse Switch (0x02) or Accessory (0x03) TMCC packets."""
    packet = list(packet)
    if len(packet) != 3 or packet[0] != 0xFE:
        return None

    if cmd_type is None:
        cmd_type = (packet[1] >> 6) & 0x03
    
    # Accept both Switch (0x02) and Accessory (0x03)
    if cmd_type not in (0x02, 0x03):
        return None

    address = ((packet[1] & 0x3F) << 1) | ((packet[2] & 0x80) >> 7)
    cmd_field = (packet[2] & 0x60) >> 5
    data_field = packet[2] & 0x1F

    return {
        "address": address,
        "cmd_field": cmd_field,
        "data_field": data_field,
        "cmd_type": cmd_type,
    }


# ---------- WLED client ----------

class WLEDClient:
    def __init__(self, host: str = "192.168.0.10", port: int = 80, timeout: float = 5.0,
                 max_retries: int = 3, retry_delay: float = 1.0,
                 on_reconnect: Optional[callable] = None) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self._consecutive_failures = 0
        self._last_success = time.time()
        self._backoff_until = 0.0  # timestamp when backoff expires
        self._max_backoff = 60.0   # max 60 seconds between retry batches
        self._on_reconnect = on_reconnect  # callback when connection restored
        self._was_disconnected = False  # track if we were in a failed state

    def post_state(self, payload: dict) -> bool:
        # Check if we're in backoff period (ESP32 was unreachable)
        now = time.time()
        if now < self._backoff_until:
            # Still in backoff - mark as disconnected and skip
            self._was_disconnected = True
            return False
        
        body = json.dumps(payload)
        last_error = None
        
        for attempt in range(self.max_retries):
            conn = None
            try:
                conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
                conn.request(
                    "POST",
                    "/json/state",
                    body=body,
                    headers={"Content-Type": "application/json", "Content-Length": str(len(body))},
                )
                resp = conn.getresponse()
                resp.read()  # drain
                if resp.status < 200 or resp.status >= 300:
                    logger.warning("WLED HTTP %s %s: %s", resp.status, resp.reason, body)
                    return False
                # Success - reset failure counter and backoff
                if self._consecutive_failures > 0 or self._was_disconnected:
                    logger.info("WLED connection restored after %d failures", self._consecutive_failures)
                    self._was_disconnected = False
                    # Trigger reconnect callback to restore full state
                    if self._on_reconnect:
                        threading.Thread(target=self._on_reconnect, daemon=True).start()
                self._consecutive_failures = 0
                self._backoff_until = 0.0
                self._last_success = time.time()
                return True
            except Exception as e:  # pragma: no cover - network
                last_error = e
                if attempt < self.max_retries - 1:
                    logger.warning("WLED request failed (attempt %d/%d): %s, retrying in %.1fs",
                                   attempt + 1, self.max_retries, e, self.retry_delay)
                    time.sleep(self.retry_delay)
            finally:
                if conn:
                    try:
                        conn.close()
                    except Exception:
                        pass
        
        # All retries exhausted - apply exponential backoff
        self._consecutive_failures += 1
        # Backoff: 5s, 10s, 20s, 40s, 60s (capped)
        backoff_time = min(5.0 * (2 ** (self._consecutive_failures - 1)), self._max_backoff)
        self._backoff_until = time.time() + backoff_time
        logger.error("WLED unreachable after %d attempts: %s (failure #%d, backing off %.0fs)",
                     self.max_retries, last_error, self._consecutive_failures, backoff_time)
        return False

    def is_connected(self) -> bool:
        """Check if WLED is reachable."""
        try:
            conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
            conn.request("GET", "/json/state")
            resp = conn.getresponse()
            resp.read()
            conn.close()
            return resp.status == 200
        except Exception:
            return False


# ---------- Controller ----------

class DaylightCycle:
    """Smooth 24-hour daylight simulation compressed to a configurable duration.
    
    Phases blend continuously; moon segment lights during night; lightning
    storm sequence during evening-to-night transition.
    """

    # Default color keyframes (RGB) at virtual hours
    # 0=midnight, 6=sunrise, 12=noon, 18=sunset, 24=midnight
    DEFAULT_KEYFRAMES = {
        0:  (40, 40, 100),     # midnight - visible blue glow
        5:  (50, 50, 120),     # pre-dawn - brighter blue
        6:  (255, 150, 80),    # sunrise orange
        8:  (255, 220, 180),   # morning warm
        12: (255, 255, 240),   # noon bright white
        16: (255, 240, 220),   # afternoon
        18: (255, 140, 60),    # sunset orange
        20: (80, 60, 120),     # dusk purple
        21: (60, 60, 130),     # evening - visible blue
        24: (40, 40, 100),     # midnight - visible blue glow
    }

    # Lightning storm phases (virtual hours)
    # Storm starts at hour 20.5 (late evening), peaks around 22, ends by 2 (mid-night)
    STORM_START = 20.5      # random flickers begin
    STORM_PEAK_START = 21.5 # full flash zone starts
    STORM_PEAK_END = 22.5   # full flash zone ends
    STORM_TAPER_END = 24.0  # random flickers taper off (wraps to 0)
    STORM_STOP = 2.0        # storm completely stops by mid-night

    def __init__(
        self,
        client: WLEDClient,
        cycle_duration_sec: float = 1800,  # 30 min default
        led_count: int = 100,
        moon_start: int = 0,
        moon_length: int = 5,
        lightning_every_n_cycles: int = 3,
        keyframes: Optional[Dict[int, Tuple[int, int, int]]] = None,
        update_interval: float = 1.0,
        thunder_callback: Optional[callable] = None,  # Called after lightning with delay_ms
    ) -> None:
        self.client = client
        self.cycle_duration = cycle_duration_sec
        self.led_count = led_count
        self.moon_start = moon_start
        self.moon_length = moon_length
        self.lightning_every_n = lightning_every_n_cycles
        self.keyframes = keyframes or self.DEFAULT_KEYFRAMES
        self.update_interval = update_interval
        self.thunder_callback = thunder_callback

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cycle_count = 0
        self._start_time = 0.0
        self._last_lightning_time = 0.0
        self._storm_active = False
        self._last_state: Optional[dict] = None  # cache last sent state for reconnect
        
        # Register reconnect callback with client
        self.client._on_reconnect = self._on_reconnect
    
    def _on_reconnect(self) -> None:
        """Called when WLED connection is restored after dropout."""
        if not self._running:
            return
        logger.info("Daylight cycle: restoring state after reconnect")
        # Re-send the last known state to restore LEDs
        if self._last_state:
            # Small delay to let WLED fully initialize
            time.sleep(0.5)
            self.client.post_state(self._last_state)

    # ---- Color interpolation ----
    def _lerp(self, a: int, b: int, t: float) -> int:
        return int(a + (b - a) * t)

    def _lerp_color(
        self, c1: Tuple[int, int, int], c2: Tuple[int, int, int], t: float
    ) -> Tuple[int, int, int]:
        return (
            self._lerp(c1[0], c2[0], t),
            self._lerp(c1[1], c2[1], t),
            self._lerp(c1[2], c2[2], t),
        )

    def _get_sky_color(self, virtual_hour: float) -> Tuple[int, int, int]:
        """Interpolate sky color for a fractional virtual hour (0-24)."""
        hours = sorted(self.keyframes.keys())
        # Find surrounding keyframes
        lower_h = hours[0]
        upper_h = hours[-1]
        for i, h in enumerate(hours):
            if h <= virtual_hour:
                lower_h = h
            if h >= virtual_hour:
                upper_h = h
                break
        if lower_h == upper_h:
            return self.keyframes[lower_h]
        t = (virtual_hour - lower_h) / (upper_h - lower_h)
        return self._lerp_color(self.keyframes[lower_h], self.keyframes[upper_h], t)

    def _is_night(self, virtual_hour: float) -> bool:
        return virtual_hour < 5 or virtual_hour >= 21

    def _is_lightning_window(self, virtual_hour: float) -> bool:
        # Late evening (20-24) or early night (0-4)
        return virtual_hour >= 20 or virtual_hour < 4

    def _get_storm_phase(self, virtual_hour: float) -> str:
        """Determine current storm phase based on virtual hour.
        
        Returns: 'none', 'building', 'peak', 'tapering'
        """
        # Handle wrap-around at midnight
        if virtual_hour >= self.STORM_START:
            if virtual_hour < self.STORM_PEAK_START:
                return 'building'
            elif virtual_hour < self.STORM_PEAK_END:
                return 'peak'
            elif virtual_hour < self.STORM_TAPER_END:
                return 'tapering'
            else:
                return 'none'
        elif virtual_hour < self.STORM_STOP:
            # After midnight, continue tapering until STORM_STOP
            return 'tapering'
        else:
            return 'none'

    def _get_storm_intensity(self, virtual_hour: float) -> float:
        """Get storm intensity 0.0-1.0 based on phase.
        
        Building: 0.0 -> 0.5
        Peak: 1.0
        Tapering: 0.5 -> 0.0
        """
        phase = self._get_storm_phase(virtual_hour)
        if phase == 'none':
            return 0.0
        elif phase == 'building':
            # Linear ramp from STORM_START to STORM_PEAK_START
            progress = (virtual_hour - self.STORM_START) / (self.STORM_PEAK_START - self.STORM_START)
            return progress * 0.5
        elif phase == 'peak':
            return 1.0
        elif phase == 'tapering':
            if virtual_hour >= self.STORM_PEAK_END:
                # From STORM_PEAK_END to midnight
                progress = (virtual_hour - self.STORM_PEAK_END) / (self.STORM_TAPER_END - self.STORM_PEAK_END)
                return 0.5 * (1.0 - progress)
            else:
                # After midnight (0 to STORM_STOP)
                progress = virtual_hour / self.STORM_STOP
                return 0.3 * (1.0 - progress)
        return 0.0

    def _do_random_flicker(self, sky_color: Tuple[int, int, int]) -> None:
        """Flash random subset of LEDs briefly (distant lightning)."""
        # Pick random start and length for the flash (10-25% of strip)
        flash_length = random.randint(self.led_count // 10, self.led_count // 4)
        flash_start = random.randint(0, max(0, self.led_count - flash_length))
        
        # Build individual LED colors: sky color everywhere, white in flash zone
        # Use effect 0 (solid) with individual pixel control via "i" array
        # Simpler approach: just do a quick full flash but dimmer (like distant lightning)
        dim_white = [180, 180, 200]  # Dimmer white for distant effect
        
        # Quick dim flash (1-2 pulses, shorter duration)
        for _ in range(random.randint(1, 2)):
            self.client.post_state({"seg": [{"id": 0, "col": [dim_white]}]})
            time.sleep(0.05 + random.random() * 0.05)  # 50-100ms flash
            self.client.post_state({"seg": [{"id": 0, "col": [list(sky_color)]}]})
            time.sleep(0.03 + random.random() * 0.03)
        
        # Trigger thunder with longer delay (distant)
        logger.info(f"⚡ Random flicker triggered, thunder_callback={self.thunder_callback is not None}")
        if self.thunder_callback:
            delay_ms = random.randint(800, 2500)
            logger.info(f"🌩️ Triggering distant thunder with {delay_ms}ms delay")
            threading.Thread(target=self.thunder_callback, args=(delay_ms,), daemon=True).start()

    def _do_full_flash(self, sky_color: Tuple[int, int, int]) -> None:
        """Full bright flash on all LEDs (close lightning strike)."""
        # Multiple rapid flashes for dramatic effect (2-4 pulses)
        num_pulses = random.randint(2, 4)
        for i in range(num_pulses):
            self.client.post_state({"seg": [{"id": 0, "col": [[255, 255, 255]]}]})
            # First pulse slightly shorter to avoid "stuck on" appearance
            if i == 0:
                time.sleep(0.06 + random.random() * 0.06)  # 60-120ms first flash
            else:
                time.sleep(0.08 + random.random() * 0.10)  # 80-180ms subsequent
            self.client.post_state({"seg": [{"id": 0, "col": [list(sky_color)]}]})
            time.sleep(0.04 + random.random() * 0.06)  # 40-100ms dark
        
        # Trigger thunder with short delay (close)
        logger.info(f"⚡ Full flash triggered, thunder_callback={self.thunder_callback is not None}")
        if self.thunder_callback:
            delay_ms = random.randint(100, 500)
            logger.info(f"🌩️ Triggering close thunder with {delay_ms}ms delay")
            threading.Thread(target=self.thunder_callback, args=(delay_ms,), daemon=True).start()

    def _is_storm_night(self, virtual_hour: float) -> bool:
        """Only a subset of nights get a storm, per lightning_every_n.

        The storm window spans midnight (e.g. 20.5 -> next day's 2.0), so the
        post-midnight tail (virtual_hour < STORM_STOP) still belongs to the
        night that started the previous cycle, not the newly-started cycle.
        """
        night_index = self._cycle_count - 1 if virtual_hour < self.STORM_STOP else self._cycle_count
        if self.lightning_every_n <= 1:
            return True
        return night_index % self.lightning_every_n == 0

    def _maybe_do_lightning(self, virtual_hour: float, sky_color: Tuple[int, int, int]) -> None:
        """Check if we should do lightning based on storm phase and randomness."""
        if not self._is_storm_night(virtual_hour):
            return

        intensity = self._get_storm_intensity(virtual_hour)
        if intensity <= 0:
            return
        
        current_time = time.time()
        phase = self._get_storm_phase(virtual_hour)
        
        # Minimum time between lightning events based on phase
        # Building: 25-35s (slower buildup)
        # Peak: 20-25s (frequent)
        # Tapering: 30-40s (dying down)
        if phase == 'building':
            min_interval = 35.0 - (intensity * 20.0)  # 25-35s
        elif phase == 'peak':
            min_interval = 25.0 - (intensity * 5.0)   # 20-25s
        else:  # tapering
            min_interval = 40.0 - (intensity * 10.0)  # 30-40s
        
        if current_time - self._last_lightning_time < min_interval:
            return
        
        # Higher base chance during building phase so flickers actually happen
        if phase == 'building':
            # Always do a flicker during building if interval passed
            chance = 0.8
        elif phase == 'peak':
            chance = 0.9
        else:
            chance = 0.5 + (intensity * 0.3)
        
        if random.random() > chance:
            return
        
        self._last_lightning_time = current_time
        
        if phase == 'peak':
            # During peak: mostly full flashes, occasional random
            if random.random() < 0.7:
                self._do_full_flash(sky_color)
            else:
                self._do_random_flicker(sky_color)
        elif phase == 'building':
            # Building: ONLY random flickers, no full flashes yet
            self._do_random_flicker(sky_color)
        else:
            # Tapering: mostly random flickers, occasional full
            if random.random() < 0.2:
                self._do_full_flash(sky_color)
            else:
                self._do_random_flicker(sky_color)

    # ---- WLED segment helpers ----
    def _build_segments(self, sky_color: Tuple[int, int, int], virtual_hour: float) -> list:
        """Build WLED segment array: main sky + optional moon."""
        segments = []
        # Main sky segment (all LEDs) - use "mainseg" to ensure it's the primary
        segments.append({
            "id": 0,
            "start": 0,
            "stop": self.led_count,
            "col": [list(sky_color)],
            "fx": 0,  # solid
            "on": True,
            "sel": True,  # select this segment
        })
        # Moon segment - always on to avoid dark spot during day
        # During night: pale bluish white, during day: match sky color
        if self._is_night(virtual_hour) and self.moon_length > 0:
            moon_color = [180, 180, 200]  # pale bluish white at night
        else:
            moon_color = list(sky_color)  # match sky during day
        if self.moon_length > 0:
            segments.append({
                "id": 1,
                "start": self.moon_start,
                "stop": self.moon_start + self.moon_length,
                "col": [moon_color],
                "fx": 0,
                "on": True,
            })
        return segments

    def _flash_lightning(self) -> None:
        """Quick white flash on all LEDs (brief brightness spike)."""
        # Flash by temporarily setting segment 0 to white
        self.client.post_state({"seg": [{"id": 0, "col": [[255, 255, 255]]}]})
        time.sleep(0.05 + random.random() * 0.1)
        # The next loop iteration will restore the correct sky color

    # ---- Main loop ----
    def _loop(self) -> None:
        self._start_time = time.time()
        # Start at afternoon (14:00) instead of midnight
        start_offset = (14.0 / 24.0) * self.cycle_duration
        while self._running:
            elapsed = time.time() - self._start_time + start_offset
            cycle_progress = (elapsed % self.cycle_duration) / self.cycle_duration
            virtual_hour = cycle_progress * 24.0

            # Detect new cycle
            new_cycle_count = int(elapsed // self.cycle_duration)
            if new_cycle_count > self._cycle_count:
                self._cycle_count = new_cycle_count
                logger.info("Daylight cycle #%d started", self._cycle_count)

            sky_color = self._get_sky_color(virtual_hour)
            segments = self._build_segments(sky_color, virtual_hour)
            state = {"on": True, "seg": segments}
            self._last_state = state  # cache for reconnect recovery
            self.client.post_state(state)

            # Check for lightning during storm window
            self._maybe_do_lightning(virtual_hour, sky_color)

            time.sleep(self.update_interval)

    def start(self) -> None:
        if self._running:
            return
        # Initialize segment 0 to cover all LEDs on startup
        self.client.post_state({
            "on": True,
            "mainseg": 0,
            "seg": [{"id": 0, "start": 0, "stop": self.led_count, "on": True}]
        })
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("Daylight cycle started (%.0f sec per 24h)", self.cycle_duration)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        logger.info("Daylight cycle stopped")


class WLEDController:
    """TMCC accessory -> WLED bridge with optional daylight cycle."""

    def __init__(
        self,
        mapping: Dict[Tuple[int, int], str],
        host: str = "192.168.0.10",
        port: int = 80,
        pattern_presets: Optional[List[int]] = None,
        timeout: float = 5.0,
        daylight_cycle: bool = False,
        cycle_duration_sec: float = 1800,
        led_count: int = 100,
        moon_start: int = 0,
        moon_length: int = 5,
        lightning_every_n_cycles: int = 3,
        thunder_callback: Optional[callable] = None,
        auto_start_daylight: bool = False,
    ) -> None:
        self.mapping = mapping
        self.client = WLEDClient(host=host, port=port, timeout=timeout)
        self.pattern_presets = pattern_presets or []
        self.pattern_index = 0

        # Daylight cycle - create but don't auto-start unless explicitly requested
        self.daylight: Optional[DaylightCycle] = None
        if daylight_cycle:
            self.daylight = DaylightCycle(
                client=self.client,
                cycle_duration_sec=cycle_duration_sec,
                led_count=led_count,
                moon_start=moon_start,
                moon_length=moon_length,
                lightning_every_n_cycles=lightning_every_n_cycles,
                thunder_callback=thunder_callback,
            )
            # Only auto-start if explicitly requested (default: False)
            # This allows the bridge to turn LEDs off first on startup
            if auto_start_daylight:
                self.daylight.start()

    def handle_packet(self, packet: Iterable[int], cmd_type: Optional[int] = None) -> bool:
        parsed = parse_tmcc_switch_or_accessory(packet, cmd_type)
        if not parsed:
            return False

        key = (parsed["address"], parsed["data_field"])
        action = self.mapping.get(key)
        if not action:
            logger.debug("WLED ignored addr=%s data=%s cmd_type=%s", parsed["address"], parsed["data_field"], parsed["cmd_type"])
            return False

        logger.info("WLED action '%s' for addr %s data %s", action, parsed["address"], parsed["data_field"])
        self.apply_action(action)
        return True

    def apply_action(self, action: str) -> None:
        if action == "on":
            self.client.post_state({"on": True})
        elif action == "off":
            # Stop daylight cycle if running, then turn off
            if self.daylight:
                self.daylight.stop()
            self.client.post_state({"on": False})
        elif action == "full_white":
            # Stop daylight cycle if running, then set full bright white
            if self.daylight:
                self.daylight.stop()
            self.client.post_state({"on": True, "bri": 255, "seg": [{"id": 0, "col": [[255, 255, 255]]}]})
        elif action == "daylight_start":
            if self.daylight:
                self.daylight.start()
            else:
                logger.warning("Daylight cycle not configured")
        elif action == "daylight_stop":
            if self.daylight:
                self.daylight.stop()
            else:
                logger.warning("Daylight cycle not configured")
        elif action.startswith("color:"):
            hex_color = action.split(":", 1)[1].lstrip("#")
            if len(hex_color) == 6:
                r = int(hex_color[0:2], 16)
                g = int(hex_color[2:4], 16)
                b = int(hex_color[4:6], 16)
                payload = {"seg": [{"id": 0, "col": [[r, g, b]]}]}
                self.client.post_state(payload)
            else:
                logger.warning("Invalid color format: %s", hex_color)
        elif action.startswith("brightness:"):
            try:
                value = int(action.split(":", 1)[1])
                value = max(0, min(255, value))
                self.client.post_state({"bri": value})
            except ValueError:
                logger.warning("Invalid brightness action: %s", action)
        elif action.startswith("preset:"):
            try:
                preset_id = int(action.split(":", 1)[1])
                self.client.post_state({"ps": preset_id})
            except ValueError:
                logger.warning("Invalid preset action: %s", action)
        elif action == "cycle":
            if not self.pattern_presets:
                logger.warning("No pattern_presets configured for cycle action")
                return
            self.pattern_index = (self.pattern_index + 1) % len(self.pattern_presets)
            preset_id = self.pattern_presets[self.pattern_index]
            self.client.post_state({"ps": preset_id})
            logger.info("WLED cycled to preset %s", preset_id)
        else:
            logger.warning("Unhandled WLED action: %s", action)


if __name__ == "__main__":  # basic manual test harness
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    controller = WLEDController(
        host="192.168.0.10",
        mapping={(50, 0x01): "on", (50, 0x02): "off", (50, 0x03): "cycle"},
        pattern_presets=[1, 2, 3],
    )

    sample_packets = [
        [0xFE, 0b11110010, 0b00000001],  # addr ~50, data 0x01 → on
        [0xFE, 0b11110010, 0b00000010],  # addr ~50, data 0x02 → off
        [0xFE, 0b11110010, 0b00000011],  # addr ~50, data 0x03 → cycle presets
    ]

    for pkt in sample_packets:
        controller.handle_packet(pkt)
