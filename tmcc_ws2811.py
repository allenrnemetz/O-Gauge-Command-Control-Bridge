# pylint: disable=C,R,import-error
"""TMCC accessory → WS2811 controller.

This module is standalone so it can be imported without modifying the main
bridge loop yet. It decodes TMCC Accessory/Group packets (command type 0b11)
from Base 3/SER2 and maps address+data combinations to WS2811 actions.

Usage (example):

    from tmcc_ws2811 import WS2811Controller

    controller = WS2811Controller(
        led_count=50,
        mapping={
            # (accessory_address, data_field): action
            (50, 0x11): "on",        # ACC 50 keypad 1
            (50, 0x12): "off",       # ACC 50 keypad 2
            (50, 0x13): "cycle"      # ACC 50 keypad 3
        }
    )

    # Inside your TMCC packet loop, when cmd_type == 0b11:
    handled = controller.handle_packet(packet)

Actions supported: "on", "off", "cycle" (advance pattern), "pattern:<name>",
"color:#RRGGBB", "brightness:<0-255>". Patterns can be customized via the
`patterns` argument.

If `rpi_ws281x` is unavailable, the module gracefully degrades to logging-only
(no hardware writes), so it can still be run on non-Pi systems for testing.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Iterable, Optional, Tuple

logger = logging.getLogger(__name__)

try:  # Optional hardware dependency
    from rpi_ws281x import Color, PixelStrip
except ImportError:  # pragma: no cover - hardware optional
    PixelStrip = None
    Color = None


# ---------- TMCC Accessory parsing ----------

def parse_tmcc_accessory(packet: Iterable[int]) -> Optional[Dict[str, int]]:
    """Parse a raw TMCC accessory/group packet (0xFE prefix).

    Packet layout (per TMCC spec used by engines/trains/accessories):
    - packet[0] = 0xFE (start byte)
    - packet[1] bits 7-6 = command type (0b11 for accessory/group)
    - packet[1] bits 5-0 plus packet[2] bit 7 = 7-bit address (1-127)
    - packet[2] bits 6-5 = command field (ignored here; often 0b00)
    - packet[2] bits 4-0 = data field (keypad/function)

    Returns a dict with address, cmd_field, data_field, or None if not an
    accessory packet.
    """

    packet = list(packet)
    if len(packet) != 3 or packet[0] != 0xFE:
        return None

    cmd_type = (packet[1] >> 6) & 0x03
    if cmd_type != 0x03:  # not accessory/group
        return None

    address = ((packet[1] & 0x3F) << 1) | ((packet[2] & 0x80) >> 7)
    cmd_field = (packet[2] & 0x60) >> 5
    data_field = packet[2] & 0x1F

    return {
        "address": address,
        "cmd_field": cmd_field,
        "data_field": data_field,
    }


# ---------- Hardware layer ----------

class WS2811Hardware:
    """Light wrapper over rpi_ws281x with a safe no-op fallback."""

    def __init__(
        self,
        led_count: int,
        gpio_pin: int = 18,
        freq_hz: int = 800_000,
        dma: int = 10,
        invert: bool = False,
        brightness: int = 128,
        channel: int = 0,
    ) -> None:
        self.led_count = led_count
        self.strip: Any = None

        if PixelStrip is None:
            logger.warning("rpi_ws281x not installed; WS2811 output disabled (log-only mode)")
            return

        self.strip = PixelStrip(
            led_count,
            gpio_pin,
            freq_hz,
            dma,
            invert,
            brightness,
            channel,
        )
        self.strip.begin()
        logger.info("WS2811 hardware initialized: %s pixels on GPIO %s", led_count, gpio_pin)

    @staticmethod
    def _color_tuple_to_rpi(color: Tuple[int, int, int]) -> int:
        if Color is None:
            return 0
        r, g, b = color
        return Color(r, g, b)

    def set_color(self, color: Tuple[int, int, int]) -> None:
        if not self.strip:
            logger.info("WS2811 (noop) set_color %s", color)
            return
        packed = self._color_tuple_to_rpi(color)
        for i in range(self.led_count):
            self.strip.setPixelColor(i, packed)
        self.strip.show()

    def blackout(self) -> None:
        self.set_color((0, 0, 0))

    def set_brightness(self, brightness: int) -> None:
        brightness = max(0, min(255, brightness))
        if not self.strip:
            logger.info("WS2811 (noop) brightness %s", brightness)
            return
        self.strip.setBrightness(brightness)
        self.strip.show()


# ---------- Controller ----------

class WS2811Controller:
    """Maps TMCC accessory commands to WS2811 actions."""

    def __init__(
        self,
        led_count: int,
        mapping: Dict[Tuple[int, int], str],
        patterns: Optional[Dict[str, Tuple[int, int, int]]] = None,
        default_pattern: str = "white",
        **hw_kwargs,
    ) -> None:
        self.mapping = mapping
        self.patterns = patterns or {
            "white": (255, 255, 255),
            "warm": (255, 180, 120),
            "red": (255, 0, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255),
        }
        self.pattern_names = list(self.patterns.keys())
        self.pattern_index = self.pattern_names.index(default_pattern) if default_pattern in self.patterns else 0
        self.hw = WS2811Hardware(led_count=led_count, **hw_kwargs)

    # Public API
    def handle_packet(self, packet: Iterable[int]) -> bool:
        """Handle a TMCC accessory packet. Returns True if consumed."""
        parsed = parse_tmcc_accessory(packet)
        if not parsed:
            return False

        key = (parsed["address"], parsed["data_field"])
        action = self.mapping.get(key)
        if not action:
            logger.debug("WS2811 accessory ignored addr=%s data=%s", parsed["address"], parsed["data_field"])
            return False

        logger.info("WS2811 action '%s' for ACC %s data %s", action, parsed["address"], parsed["data_field"])
        self.apply_action(action)
        return True

    def apply_action(self, action: str) -> None:
        if action == "on":
            self.hw.set_color(self.patterns[self.pattern_names[self.pattern_index]])
        elif action == "off":
            self.hw.blackout()
        elif action == "cycle":
            self.pattern_index = (self.pattern_index + 1) % len(self.pattern_names)
            name = self.pattern_names[self.pattern_index]
            self.hw.set_color(self.patterns[name])
            logger.info("WS2811 pattern cycled to %s", name)
        elif action.startswith("pattern:"):
            name = action.split(":", 1)[1]
            if name in self.patterns:
                self.pattern_index = self.pattern_names.index(name)
                self.hw.set_color(self.patterns[name])
            else:
                logger.warning("Unknown pattern '%s'", name)
        elif action.startswith("color:"):
            hex_color = action.split(":", 1)[1].lstrip("#")
            if len(hex_color) == 6:
                r = int(hex_color[0:2], 16)
                g = int(hex_color[2:4], 16)
                b = int(hex_color[4:6], 16)
                self.hw.set_color((r, g, b))
            else:
                logger.warning("Invalid color format: %s", hex_color)
        elif action.startswith("brightness:"):
            try:
                value = int(action.split(":", 1)[1])
                self.hw.set_brightness(value)
            except ValueError:
                logger.warning("Invalid brightness action: %s", action)
        else:
            logger.warning("Unhandled WS2811 action: %s", action)


if __name__ == "__main__":  # basic manual test harness
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    controller = WS2811Controller(
        led_count=10,
        mapping={(50, 0x01): "on", (50, 0x02): "off", (50, 0x03): "cycle"},
    )

    # Simulated TMCC accessory packets (0xFE, address bits in packet[1]/[2], data in packet[2])
    sample_packets = [
        [0xFE, 0b11110010, 0b00000001],  # addr ~50, data 0x01 → on
        [0xFE, 0b11110010, 0b00000010],  # addr ~50, data 0x02 → off
        [0xFE, 0b11110010, 0b00000011],  # addr ~50, data 0x03 → cycle
    ]

    for pkt in sample_packets:
        controller.handle_packet(pkt)
