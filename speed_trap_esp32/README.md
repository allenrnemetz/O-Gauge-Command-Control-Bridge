# ESP32 Speed Trap (ToF Calibration Rig)

Dual VL53L0X time-of-flight sensors on an ESP32, reporting train crossing
times over WiFi UDP. Used to calibrate the Lionel speed-step → MTH sMPH
conversion curve in the bridge.

## Hardware

- ELEGOO ESP32 dev board (or any standard ESP32)
- 2x VL53L0X breakout boards (AITRIP VL53L0X with cover, Pololu #2490, or similar)

## Wiring

Both sensors share one I2C bus. Sensor B's address is changed at startup
via its XSHUT pin so both can coexist.

```
ESP32 GPIO21 (SDA)  -> SDA on both sensors
ESP32 GPIO22 (SCL)  -> SCL on both sensors
ESP32 GPIO18        -> XSHUT on sensor A (left)
ESP32 GPIO19        -> XSHUT on sensor B (right)
3V3                 -> VDD on both sensors
GND                 -> GND on both sensors
```

## Mounting

- Bridge over **straight, level track**, sensors facing down.
- Both sensors at the **same height and angle**.
- Spacing is **fixed** — do not change between calibration runs. The
  actual distance is derived at runtime from an MTH engine at a known
  sMPH, so you don't need to measure it physically.
- Height: ~200–300mm above railhead works well. The firmware auto-
  calibrates the open-air threshold on startup.

## Arduino IDE Setup

1. Install the **VL53L0X** library by Pololu:
   `Sketch → Include Library → Manage Libraries → search "VL53L0X"`
2. Install **ArduinoJson**:
   `Sketch → Include Library → Manage Libraries → search "ArduinoJson"`
3. Open `speed_trap_esp32.ino`
4. Edit WiFi credentials and bridge PC IP at the top of the file
5. Select your ESP32 board and upload

## UDP Output

All packets are JSON, sent to `BRIDGE_IP:UDP_PORT` (default broadcast
`255.255.255.255:7777`).

| Event       | Fields                                              | When                |
|-------------|-----------------------------------------------------|---------------------|
| `ready`     | `ip`, `a_base_mm`, `b_base_mm`, `thresh_mm`         | Once at startup     |
| `pass`      | `dir`, `t1_ms`, `t2_ms`, `dt_ms`, `a_mm`, `b_mm`   | Each train pass     |
| `heartbeat` | `uptime_s`, `a_mm`, `b_mm`                          | Every 2 seconds     |
| `error`     | `msg`                                               | On fatal errors     |

### Pass packet

```json
{"event":"pass","dir":"fwd","t1_ms":1234567,"t2_ms":1234580,"dt_ms":13.0,"a_mm":82,"b_mm":85}
```

- `dir`: `"fwd"` = sensor A triggered first, `"rev"` = sensor B first
- `dt_ms`: crossing time in milliseconds — this is what the bridge uses
  to compute speed (`speed = D / dt`)
- `a_mm` / `b_mm`: distance readings at each sensor's trigger moment

## Calibration Flow

1. **MTH engine sweep**: run at known sMPH values (1, 20, 40, 60, 80,
   100, 120). The bridge collects `dt` for each, back-calculates sensor
   spacing `D = sMPH × 0.367 × dt` (O scale 1:48), and confirms MTH
   linearity.

2. **Lionel engine sweep**: run at Legacy steps (1, 34, 67, 100, 133,
   166, 199) or TMCC1 steps (1, 6, 11, 16, 21, 26, 31). The bridge
   uses the derived `D` to compute actual sMPH at each step.

3. **Build conversion curve**: for each Lionel step, the DCS sMPH to
   command is the one that produces the same actual speed the Lionel
   engine measured at that step. Stored as a lookup table keyed by
   engine ID.

## Configuration (top of .ino file)

| Setting              | Default          | Notes                              |
|----------------------|------------------|------------------------------------|
| `WIFI_SSID`          | `Nemetz`         | Your WiFi                          |
| `WIFI_PASSWORD`      | `crawdad2`       |                                    |
| `BRIDGE_IP`          | `255.255.255.255`| Bridge PC IP or broadcast          |
| `UDP_PORT`           | `7777`           |                                    |
| `TIMING_BUDGET_US`   | `20000`          | 20ms = fastest practical for L0X   |
| `THRESHOLD_MARGIN_MM`| `60`             | Below open-air baseline = train    |
| `PASS_RESET_MS`      | `500`            | Delay before re-arming after pass  |
| `PASS_TIMEOUT_MS`    | `5000`           | Discard if only one sensor triggers|
