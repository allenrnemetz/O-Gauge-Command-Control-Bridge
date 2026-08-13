# Speed Trap Wiring Guide

## Parts

| Part | Qty | Notes |
|------|-----|-------|
| ELEGOO ESP32 dev board | 1 | Any standard ESP32 works |
| VL53L1X ToF sensor module (with cover) | 2 | 3V-5V, onboard regulator, up to 400cm |
| Hookup wire | — | ~6-8 connections |
| Mounting bracket / bridge | 1 | Over straight level track, sensors facing down |

## VL53L1X Module Pinout

These modules break out 6 pins (pin labels may vary slightly):

```
VCC   — Power (3V or 5V, onboard regulator handles either)
GND   — Ground
SDA   — I2C data
SCL   — I2C clock
GPIO1 — Interrupt output (not used here)
XSHUT — Shutdown / address control (labeled XSHUT or SHUT)
```

The metal cover snaps on — leave it on, it improves ranging accuracy
and narrows the field of view.

## Wiring

Both sensors share the ESP32's I2C bus. Sensor B's address is changed
to `0x2B` at startup using its XSHUT pin, so both can coexist on one
bus without a multiplexer.

### Sensor A (left / "first" in forward direction)

```
VL53L0X A      ESP32
---------      -----
VCC      ->    3V3
GND      ->    GND
SDA      ->    GPIO21
SCL      ->    GPIO22
XSHUT    ->    GPIO18
GPIO1    ->    (leave unconnected)
```

### Sensor B (right / "second" in forward direction)

```
VL53L1X B      ESP32
---------      -----
VCC      ->    3V3
GND      ->    GND
SDA      ->    GPIO21   (same bus)
SCL      ->    GPIO22   (same bus)
XSHUT    ->    GPIO19
GPIO1    ->    (leave unconnected)
```

### Power note

The AITRIP modules accept 3V-5V on VCC thanks to the onboard regulator.
Wire both to the ESP32's **3V3** pin — do not use 5V/VIN. The ESP32's
I2C pins are 3.3V logic and the VL53L1X is a 3.3V part natively; using
3V3 on VCC keeps everything at the same logic level and avoids any
level-shifting. Two sensors + the ESP32's 3V3 regulator can handle this
easily (each VL53L1X draws ~10-20mA in ranging mode).

### Full wiring diagram (text)

```
                    3V3  o----+----------+
                              |          |
                            [VCC A]    [VCC B]

                    GND  o----+----------+
                              |          |
                            [GND A]    [GND B]

   GPIO21 (SDA)  o----+----------+
                       |          |
                     [SDA A]    [SDA B]

   GPIO22 (SCL)  o----+----------+
                       |          |
                     [SCL A]    [SCL B]

   GPIO18        o----[XSHUT A]

   GPIO19        o----[XSHUT B]

   GPIO1 (both)  ->  not connected
```

## Mounting

- **Bridge over straight, level track**, sensors facing down at the
  railhead.
- **Both sensors at the same height and angle.** If one is higher or
  tilted differently, its trigger fires at a different point on the
  train's profile, introducing a fixed timing offset that does NOT
  cancel out between calibration runs.
- **Spacing is fixed** — whatever you build, lock it down. Do not
  remount or adjust between calibration runs. The actual distance is
  derived at runtime from an MTH engine at a known sMPH, so you don't
  need to measure it physically.
- **Recommended height**: 200-300mm (8-12") above the railhead. The
  firmware auto-calibrates the open-air threshold on startup by taking
  baseline readings, so exact height doesn't matter as long as both
  sensors match and the beam hits the train body consistently.
- **Avoid mounting near curves or grade changes.** On a curve the train
  leans (ToF reading shifts) and the path between sensors isn't a
  straight line anymore.

## Sensor A vs B orientation

The firmware reports direction based on which sensor triggers first:
- `dir: "fwd"` = sensor A triggered first
- `dir: "rev"` = sensor B triggered first

Label your sensors when you mount them so you know which is A and
which is B. It doesn't matter which physical direction is "forward"
for your track — the bridge calibration script handles both directions
and the crossing time `dt` is the same either way.

## Power-up sequence

1. Wire everything per the diagram above.
2. Power the ESP32 via USB.
3. Open Serial Monitor at **115200 baud**.
4. You should see:
   ```
   === O-Gauge Speed Trap (VL53L1X) ===
   Sensor A online (addr 0x29)
   Sensor B online (addr 0x2B)
   Calibrating open-air baseline...
   Baseline A=262mm B=259mm -> thresholds A=202mm B=199mm
   Connecting to WiFi 'Nemetz'...
   WiFi connected! IP: 192.168.1.123
   Speed trap running. Waiting for trains...
   ```
5. If you see `sensor A init failed` or `sensor B init failed`:
   - Check wiring (especially XSHUT pins and SDA/SCL)
   - Make sure both sensors have power (VCC + GND)
   - Try swapping the two sensor modules to isolate a bad board

## Troubleshooting

| Symptom | Check |
|---------|-------|
| `sensor A/B init failed` | XSHUT wiring, I2C wiring, power |
| Heartbeat shows `a_mm` or `b_mm` = 0 or >4000 | Sensor obstruction, bad module, sensor too close/far |
| No `pass` events when train goes through | Threshold too low (train doesn't trigger), sensor not aimed at train body |
| Pass events but `dt_ms` looks wrong | One sensor triggering on a tall detail (stack, cab) instead of body — raise threshold margin or remount higher |
| WiFi won't connect | Check SSID/password in `.ino`, check signal at trackside |
| Double-triggers from multi-car trains | Increase `PASS_RESET_MS` in the `.ino` |
