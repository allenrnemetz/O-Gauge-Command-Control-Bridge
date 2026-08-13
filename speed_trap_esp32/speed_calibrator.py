#!/usr/bin/env python3
"""
Speed Calibration Tool for O-Gauge Lionel -> MTH Conversion
============================================================

Standalone tool that listens to the ESP32 speed trap over UDP, runs an
interactive manual calibration sweep, and produces a JSON conversion
curve that the Lionel-MTH bridge can load.

You drive the trains manually with your Cab controller. The tool just
measures what the speed trap sees and prompts you through the sweep.

WORKFLOW
--------
1. Start the ESP32 speed trap (it broadcasts UDP packets on port 7777).
2. Run this script.
3. Choose a calibration mode:
   - "mth"    : Sweep DCS sMPH on an MTH engine. Derives sensor spacing
                and confirms MTH linearity.
   - "legacy" : Sweep Legacy 200-step on a Lionel engine. Measures
                actual sMPH at each step.
   - "tmcc1"  : Sweep TMCC1 32-step on a Lionel engine. Same, 32-step.
   - "build"  : Combine a completed MTH sweep + Lionel sweep into a
                conversion curve JSON file.
4. For mth/legacy/tmcc1: the tool prompts you for each speed point.
   Set the speed on your Cab, let the train run a few laps to settle,
   press Enter. The tool collects N passes through the speed trap,
   takes the median, and records the result.
5. After the Lionel sweep, run "build" to produce the curve file.

OUTPUT FILES
------------
Saved to ~/.lionel-mth-bridge/calibration/:
  - mth_<engine_name>.json     : MTH sweep results + derived spacing
  - legacy_<engine_name>.json  : Legacy sweep results
  - tmcc1_<engine_name>.json   : TMCC1 sweep results
  - curve_<engine_name>.json   : Final conversion curve (Legacy step -> DCS sMPH)

The curve file is what the bridge loads to replace the linear formula.

USAGE
-----
  python speed_calibrator.py

  (interactive prompts guide the rest)

REQUIREMENTS
------------
  Python 3.8+ (standard library only, no pip installs needed)
"""

import socket
import json
import os
import statistics
import sys
import time
from pathlib import Path

# ==================== CONFIG ====================

UDP_PORT = 7777
UDP_BUFFER_SIZE = 1024
RECEIVE_TIMEOUT = 6.0  # seconds — if no packet in this long, trap may be offline

# O scale conversion: 1 scale MPH = 0.367 inches/sec (1:48)
# Speed (in/sec) = sMPH * 0.367
# Speed (mm/sec) = sMPH * 9.317
# Distance (mm) = speed (mm/sec) * time (sec) = sMPH * 9.317 * dt_sec
SMPH_TO_MM_PER_SEC = 9.317

# Number of passes to collect per speed point (median is used)
SAMPLES_PER_POINT = 5

# Default sweep points (min, max, and 5 evenly scattered between)
MTH_SWEEP_POINTS = [1, 20, 40, 60, 80, 100, 120]
LEGACY_SWEEP_POINTS = [1, 34, 67, 100, 133, 166, 199]
TMCC1_SWEEP_POINTS = [1, 6, 11, 16, 21, 26, 31]

# Config directory (matches the bridge's convention)
CONFIG_DIR = Path.home() / ".lionel-mth-bridge" / "calibration"
CONFIG_DIR.mkdir(parents=True, exist_ok=True)


# ==================== UDP LISTENER ====================

class SpeedTrapListener:
    """Listens for UDP packets from the ESP32 speed trap."""

    def __init__(self, port=UDP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(RECEIVE_TIMEOUT)
        self.last_heartbeat = None
        self.trap_ip = None
        self.trap_ready = False
        self.baseline_a = None
        self.baseline_b = None
        self.threshold = None

    def _parse(self, data):
        try:
            return json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None

    def wait_for_ready(self):
        """Block until we receive a 'ready' packet or heartbeat from the trap."""
        print(f"Listening for speed trap on UDP port {self.sock.getsockname()[1]}...")
        print("Waiting for 'ready' packet from ESP32...")
        start = time.time()
        while time.time() - start < 30:
            try:
                data, addr = self.sock.recvfrom(UDP_BUFFER_SIZE)
                msg = self._parse(data)
                if not msg:
                    continue
                self.trap_ip = addr[0]
                event = msg.get("event")
                if event == "ready":
                    self.trap_ready = True
                    self.baseline_a = msg.get("a_base_mm")
                    self.baseline_b = msg.get("b_base_mm")
                    self.threshold = msg.get("thresh_mm")
                    print(f"  Speed trap ready at {self.trap_ip}")
                    print(f"  Baselines: A={self.baseline_a}mm  B={self.baseline_b}mm  "
                          f"Threshold={self.threshold}mm")
                    return True
                elif event == "heartbeat":
                    self.trap_ready = True
                    self.last_heartbeat = time.time()
                    print(f"  Speed trap heartbeat from {self.trap_ip} "
                          f"(A={msg.get('a_mm')}mm B={msg.get('b_mm')}mm)")
                    return True
            except socket.timeout:
                print("  No packet received. Is the ESP32 powered on and on the same WiFi?")
                return False
        return False

    def collect_passes(self, count, timeout_per_pass=30.0):
        """Collect `count` pass events. Returns list of (dt_ms, dir, a_mm, b_mm)."""
        results = []
        deadline = time.time() + timeout_per_pass * count + 30
        print(f"  Collecting {count} pass{'es' if count != 1 else ''}...")

        while len(results) < count:
            remaining = deadline - time.time()
            if remaining <= 0:
                print(f"  Timeout: only got {len(results)}/{count} passes.")
                break
            self.sock.settimeout(min(remaining, 30.0))
            try:
                data, addr = self.sock.recvfrom(UDP_BUFFER_SIZE)
                msg = self._parse(data)
                if not msg:
                    continue
                event = msg.get("event")
                if event == "pass":
                    dt = msg.get("dt_ms")
                    direction = msg.get("dir", "?")
                    a_mm = msg.get("a_mm", 0)
                    b_mm = msg.get("b_mm", 0)
                    results.append((dt, direction, a_mm, b_mm))
                    n = len(results)
                    print(f"    Pass {n}/{count}: dt={dt:.1f}ms  dir={direction}  "
                          f"A={a_mm}mm B={b_mm}mm")
                elif event == "heartbeat":
                    self.last_heartbeat = time.time()
                elif event == "error":
                    print(f"  [ESP32 ERROR] {msg.get('msg')}")
            except socket.timeout:
                print(f"  Waiting for pass {len(results)+1}/{count}... "
                      f"(drive the train through the trap)")
        return results


# ==================== CALIBRATION LOGIC ====================

def median_dt(passes):
    """Extract median dt_ms from a list of pass tuples."""
    dts = [p[0] for p in passes if p[0] and p[0] > 0]
    if not dts:
        return None
    return statistics.median(dts)


def run_mth_sweep(listener):
    """Sweep DCS sMPH on an MTH engine. Derives sensor spacing."""
    engine_name = input("Enter a name for this MTH engine (e.g. 'bigboy'): ").strip()
    if not engine_name:
        engine_name = "mth_engine"

    print("\n=== MTH sMPH Sweep ===")
    print(f"Speed points: {MTH_SWEEP_POINTS}")
    print("For each point: set the DCS sMPH on your Cab, let the train")
    print("run a few laps to settle momentum, then press Enter.")
    print("The tool will collect {} passes per point.\n".format(SAMPLES_PER_POINT))

    results = []  # list of {smpH, dt_ms_median, samples}

    for smpH in MTH_SWEEP_POINTS:
        print(f"\n--- s{smpH} ---")
        input(f"Set DCS speed to s{smpH}, let it settle, then press Enter... ")

        passes = listener.collect_passes(SAMPLES_PER_POINT)
        if len(passes) < 2:
            print(f"  Not enough passes for s{smpH} (got {len(passes)}). Skipping.")
            results.append({"smpH": smpH, "dt_ms": None, "samples": len(passes)})
            continue

        med_dt = median_dt(passes)
        # Derive spacing: D_mm = sMPH * SMPH_TO_MM_PER_SEC * (dt_ms / 1000)
        derived_d = smpH * SMPH_TO_MM_PER_SEC * (med_dt / 1000.0)
        print(f"  Median dt = {med_dt:.1f}ms")
        print(f"  Derived sensor spacing = {derived_d:.1f}mm ({derived_d/25.4:.1f}in)")

        results.append({
            "smpH": smpH,
            "dt_ms": round(med_dt, 2),
            "derived_spacing_mm": round(derived_d, 1),
            "samples": len(passes),
            "raw_dts": [round(p[0], 1) for p in passes],
        })

    # Compute consensus spacing from all valid points
    spacings = [r["derived_spacing_mm"] for r in results
                if r.get("derived_spacing_mm") and r["smpH"] > 5]
    if spacings:
        consensus_d = statistics.median(spacings)
        spread = max(spacings) - min(spacings)
        print(f"\n=== Spacing Consensus ===")
        print(f"  Median spacing: {consensus_d:.1f}mm ({consensus_d/25.4:.1f}in)")
        print(f"  Spread: {spread:.1f}mm")
        if spread > consensus_d * 0.10:
            print(f"  WARNING: spread > 10% of spacing. MTH may not be perfectly")
            print(f"  linear, or measurements are noisy. Check mounting and retry.")
        else:
            print(f"  MTH linearity confirmed (spread < 10%).")
    else:
        consensus_d = None
        print("\nWARNING: Could not derive spacing (not enough valid points).")

    # Save
    out_file = CONFIG_DIR / f"mth_{engine_name}.json"
    data = {
        "engine_name": engine_name,
        "type": "mth_sweep",
        "sweep_points": MTH_SWEEP_POINTS,
        "samples_per_point": SAMPLES_PER_POINT,
        "results": results,
        "consensus_spacing_mm": consensus_d,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    out_file.write_text(json.dumps(data, indent=2))
    print(f"\nSaved to {out_file}")
    return data


def run_lionel_sweep(listener, protocol="legacy"):
    """Sweep Legacy or TMCC1 steps on a Lionel engine. Measures actual sMPH."""
    if protocol == "legacy":
        sweep_points = LEGACY_SWEEP_POINTS
        step_label = "Legacy step"
        step_word = "step"
    else:
        sweep_points = TMCC1_SWEEP_POINTS
        step_label = "TMCC1 step"
        step_word = "step"

    engine_name = input(f"Enter a name for this Lionel engine (e.g. 'gp7'): ").strip()
    if not engine_name:
        engine_name = "lionel_engine"

    # Load spacing — try to find an MTH sweep file
    spacing_mm = load_spacing_from_mth_sweep()
    if spacing_mm is None:
        print("\nNo MTH sweep found. You need to run the MTH sweep first to")
        print("derive the sensor spacing. Alternatively, enter it manually.")
        manual = input("Enter sensor spacing in mm (or press Enter to abort): ").strip()
        if not manual:
            print("Aborting.")
            return None
        try:
            spacing_mm = float(manual)
        except ValueError:
            print("Invalid number. Aborting.")
            return None
    print(f"Using sensor spacing: {spacing_mm:.1f}mm ({spacing_mm/25.4:.1f}in)")

    print(f"\n=== {protocol.upper()} {step_label} Sweep ===")
    print(f"Speed points: {sweep_points}")
    print(f"For each point: set the {step_label} on your Cab, let the train")
    print("run a few laps to settle momentum, then press Enter.")
    print("The tool will collect {} passes per point.\n".format(SAMPLES_PER_POINT))

    results = []

    for step in sweep_points:
        print(f"\n--- {step_label} {step} ---")
        input(f"Set {step_label} to {step}, let it settle, then press Enter... ")

        passes = listener.collect_passes(SAMPLES_PER_POINT)
        if len(passes) < 2:
            print(f"  Not enough passes for {step} (got {len(passes)}). Skipping.")
            results.append({"step": step, "dt_ms": None, "actual_smpH": None,
                            "samples": len(passes)})
            continue

        med_dt = median_dt(passes)
        # actual sMPH = D_mm / (SMPH_TO_MM_PER_SEC * dt_sec)
        actual_smpH = spacing_mm / (SMPH_TO_MM_PER_SEC * (med_dt / 1000.0))
        print(f"  Median dt = {med_dt:.1f}ms")
        print(f"  Actual speed = {actual_smpH:.1f} sMPH")

        results.append({
            "step": step,
            "dt_ms": round(med_dt, 2),
            "actual_smpH": round(actual_smpH, 1),
            "samples": len(passes),
            "raw_dts": [round(p[0], 1) for p in passes],
        })

    # Save
    out_file = CONFIG_DIR / f"{protocol}_{engine_name}.json"
    data = {
        "engine_name": engine_name,
        "type": f"{protocol}_sweep",
        "protocol": protocol,
        "sweep_points": sweep_points,
        "samples_per_point": SAMPLES_PER_POINT,
        "spacing_mm": spacing_mm,
        "results": results,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    out_file.write_text(json.dumps(data, indent=2))
    print(f"\nSaved to {out_file}")
    return data


def load_spacing_from_mth_sweep():
    """Find the most recent MTH sweep file and return its consensus spacing."""
    mth_files = sorted(CONFIG_DIR.glob("mth_*.json"), key=lambda f: f.stat().st_mtime,
                       reverse=True)
    if not mth_files:
        return None
    try:
        data = json.loads(mth_files[0].read_text())
        spacing = data.get("consensus_spacing_mm")
        if spacing and spacing > 0:
            print(f"\nLoaded spacing {spacing:.1f}mm from {mth_files[0].name}")
            return spacing
    except (json.JSONDecodeError, KeyError):
        pass
    return None


def build_curve():
    """Combine an MTH sweep + Lionel sweep into a conversion curve."""
    print("\n=== Build Conversion Curve ===")

    # Find Lionel sweep files
    legacy_files = sorted(CONFIG_DIR.glob("legacy_*.json"),
                          key=lambda f: f.stat().st_mtime, reverse=True)
    tmcc1_files = sorted(CONFIG_DIR.glob("tmcc1_*.json"),
                         key=lambda f: f.stat().st_mtime, reverse=True)

    all_files = [(f, "legacy") for f in legacy_files] + \
                [(f, "tmcc1") for f in tmcc1_files]

    if not all_files:
        print("No Lionel sweep files found. Run a Legacy or TMCC1 sweep first.")
        return

    print("Available Lionel sweep files:")
    for i, (f, proto) in enumerate(all_files):
        try:
            data = json.loads(f.read_text())
            name = data.get("engine_name", "?")
            ts = data.get("timestamp", "?")
            print(f"  [{i+1}] {proto.upper()} '{name}' ({ts})  ->  {f.name}")
        except json.JSONDecodeError:
            print(f"  [{i+1}] {f.name} (parse error)")

    choice = input(f"\nSelect file [1-{len(all_files)}] (or Enter for most recent): ").strip()
    if not choice:
        chosen = all_files[0]
    else:
        try:
            idx = int(choice) - 1
            chosen = all_files[idx]
        except (ValueError, IndexError):
            print("Invalid selection.")
            return

    sweep_file, protocol = chosen
    sweep_data = json.loads(sweep_file.read_text())
    engine_name = sweep_data.get("engine_name", "engine")
    results = sweep_data.get("results", [])

    # Build the curve: step -> round(actual_smpH)
    # If MTH is linear (confirmed by sweep), the DCS sMPH to command
    # is simply the rounded actual sMPH measured at that step.
    curve_points = []
    for r in results:
        step = r.get("step")
        actual = r.get("actual_smpH")
        if step is not None and actual is not None:
            dcs_smpH = round(actual)
            dcs_smpH = max(0, min(120, dcs_smpH))  # clamp to DCS range
            curve_points.append({
                "step": step,
                "actual_smpH": actual,
                "dcs_smpH": dcs_smpH,
            })

    if not curve_points:
        print("No valid data points to build a curve.")
        return

    # Sort by step
    curve_points.sort(key=lambda p: p["step"])

    print(f"\nConversion curve for '{engine_name}' ({protocol.upper()}):")
    print(f"  {'Step':>6}  {'Actual sMPH':>12}  {'DCS sMPH':>10}")
    print(f"  {'----':>6}  {'-----------':>12}  {'--------':>10}")
    for p in curve_points:
        print(f"  {p['step']:>6}  {p['actual_smpH']:>12.1f}  {p['dcs_smpH']:>10}")

    # Save curve
    curve_data = {
        "engine_name": engine_name,
        "protocol": protocol,
        "type": "conversion_curve",
        "curve": curve_points,
        "spacing_mm": sweep_data.get("spacing_mm"),
        "source_sweep": sweep_file.name,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    curve_file = CONFIG_DIR / f"curve_{protocol}_{engine_name}.json"
    curve_file.write_text(json.dumps(curve_data, indent=2))
    print(f"\nCurve saved to {curve_file}")
    print(f"The bridge can load this to replace the linear formula for this engine.")


def view_files():
    """List all calibration files."""
    print("\n=== Calibration Files ===")
    files = sorted(CONFIG_DIR.glob("*.json"), key=lambda f: f.stat().st_mtime,
                   reverse=True)
    if not files:
        print(f"  (none in {CONFIG_DIR})")
        return
    for f in files:
        try:
            data = json.loads(f.read_text())
            name = data.get("engine_name", "?")
            ftype = data.get("type", "?")
            ts = data.get("timestamp", "?")
            print(f"  {f.name:40s}  {ftype:20s}  '{name}'  ({ts})")
        except json.JSONDecodeError:
            print(f"  {f.name:40s}  (parse error)")


# ==================== MAIN ====================

def main():
    print("=" * 60)
    print("  O-Gauge Speed Calibration Tool")
    print("  Lionel speed-step -> MTH sMPH conversion curve builder")
    print("=" * 60)
    print(f"  Calibration files: {CONFIG_DIR}")
    print()

    while True:
        print("\nMain menu:")
        print("  1. MTH sMPH sweep     (derive sensor spacing + confirm linearity)")
        print("  2. Legacy 200-step sweep  (measure Lionel engine speed per step)")
        print("  3. TMCC1 32-step sweep    (measure Lionel engine speed per step)")
        print("  4. Build conversion curve (combine sweeps -> curve JSON)")
        print("  5. View calibration files")
        print("  6. Quit")

        choice = input("\nSelect [1-6]: ").strip()

        if choice == "1":
            listener = SpeedTrapListener()
            if listener.wait_for_ready():
                run_mth_sweep(listener)
            listener.sock.close()
        elif choice == "2":
            listener = SpeedTrapListener()
            if listener.wait_for_ready():
                run_lionel_sweep(listener, protocol="legacy")
            listener.sock.close()
        elif choice == "3":
            listener = SpeedTrapListener()
            if listener.wait_for_ready():
                run_lionel_sweep(listener, protocol="tmcc1")
            listener.sock.close()
        elif choice == "4":
            build_curve()
        elif choice == "5":
            view_files()
        elif choice == "6":
            print("Goodbye.")
            break
        else:
            print("Invalid choice.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(0)
