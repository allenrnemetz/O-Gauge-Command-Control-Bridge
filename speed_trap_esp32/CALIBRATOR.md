# Speed Calibrator Tool

Standalone Python tool that listens to the ESP32 speed trap over UDP and
runs an interactive manual calibration sweep to build a Lionel speed-step
→ MTH sMPH conversion curve.

You drive the trains manually with your Cab controller. The tool just
measures what the speed trap sees.

## Requirements

- Python 3.8+ (standard library only — no pip installs)
- ESP32 speed trap running and on the same WiFi network
- Both the Lionel engine and MTH engine on the track

## Quick Start

```bash
cd speed_trap_esp32
python speed_calibrator.py
```

## Workflow

### Step 1: MTH sMPH Sweep (do this first)

1. Place your **MTH engine** on the track.
2. Run the tool, select menu option **1 (MTH sMPH sweep)**.
3. The tool waits for the ESP32 to report "ready."
4. For each speed point (s1, s20, s40, s60, s80, s100, s120):
   - Set the DCS speed on your Cab to the displayed value
   - Let the train run a few laps to settle momentum
   - Press Enter
   - The tool collects 5 passes through the speed trap
   - It records the median crossing time and derives the sensor spacing
5. At the end, it reports the consensus sensor spacing and confirms
   MTH linearity (spread should be < 10% of spacing).

This does three things:
- Derives the physical sensor spacing (you never have to measure it)
- Confirms MTH sMPH is linear with actual speed
- Gives you actual scale MPH numbers for the Lionel sweep

### Step 2: Lionel Sweep (Legacy or TMCC1)

1. Place your **Lionel engine** on the track.
2. Select menu option **2 (Legacy sweep)** or **3 (TMCC1 sweep)**.
3. The tool auto-loads the sensor spacing from Step 1.
4. For each speed point (Legacy: 1, 34, 67, 100, 133, 166, 199):
   - Set the speed step on your Cab
   - Let the train run a few laps to settle
   - Press Enter
   - The tool collects 5 passes and computes actual sMPH
5. Results are saved automatically.

### Step 3: Build the Conversion Curve

1. Select menu option **4 (Build conversion curve)**.
2. The tool lists your Lionel sweep files — pick one.
3. It combines the measured actual sMPH at each step with the confirmed
   MTH linearity to produce the curve: `Legacy step → DCS sMPH`.
4. The curve is saved as `curve_<protocol>_<engine>.json`.

### Step 4: Load into the Bridge (future)

The curve JSON file is what the bridge will load to replace its current
linear formula (`int(legacy_speed * 120 / 199)`) with a per-engine
lookup table. That integration is a separate step — for now, just
produce the curve file.

## Output Files

All files saved to `~/.lionel-mth-bridge/calibration/`:

| File | Contents |
|------|----------|
| `mth_<engine>.json` | MTH sweep results + derived sensor spacing |
| `legacy_<engine>.json` | Legacy 200-step sweep results (actual sMPH per step) |
| `tmcc1_<engine>.json` | TMCC1 32-step sweep results |
| `curve_legacy_<engine>.json` | Final conversion curve (Legacy step → DCS sMPH) |
| `curve_tmcc1_<engine>.json` | Final conversion curve (TMCC1 step → DCS sMPH) |

## Curve File Format

```json
{
  "engine_name": "gp7",
  "protocol": "legacy",
  "type": "conversion_curve",
  "curve": [
    {"step": 1,   "actual_smpH": 1.2,  "dcs_smpH": 1},
    {"step": 34,  "actual_smpH": 18.7, "dcs_smpH": 19},
    {"step": 67,  "actual_smpH": 37.4, "dcs_smpH": 37},
    {"step": 100, "actual_smpH": 56.1, "dcs_smpH": 56},
    {"step": 133, "actual_smpH": 74.8, "dcs_smpH": 75},
    {"step": 166, "actual_smpH": 93.5, "dcs_smpH": 94},
    {"step": 199, "actual_smpH": 112.2,"dcs_smpH": 112}
  ],
  "spacing_mm": 304.8,
  "source_sweep": "legacy_gp7.json",
  "timestamp": "2026-08-12 14:30:00"
}
```

The bridge will interpolate between these points at runtime.

## Tips for Good Measurements

- **Let momentum settle** — after setting a speed, wait a few laps
  before pressing Enter. MTH engines with high momentum settings take
  longer to reach steady state.
- **Run in the same direction** for all points in a sweep. The tool
  records direction, but consistency reduces variance.
- **Straight, level track only** through the speed trap.
- **5 passes per point** is the default — the median filters out
  outliers from sensor jitter or brief speed variations.
- **If a point looks wrong** (way off the trend), just re-run that
  sweep. You can also edit the JSON file manually to remove bad points.
- **Re-run the MTH sweep** if you remount the sensors or change the
  spacing. The Lionel sweep depends on the spacing being correct.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| "No packet received" | Check ESP32 is powered, on same WiFi, Serial Monitor shows "running" |
| Passes not detected | Train too short/slow? Check threshold in ESP32 Serial output |
| Derived spacing varies wildly | Sensors loose, track not level, train not at steady speed |
| MTH linearity warning (>10% spread) | Remount sensors more rigidly, retry, or check for draft/vibration |
| Tool can't find spacing for Lionel sweep | Run MTH sweep first, or enter spacing manually |
