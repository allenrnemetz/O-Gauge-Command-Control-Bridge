# <img src="docs/readme-icon.png" width="96" style="vertical-align: middle;"> O Gauge Command Control Bridge

**Control MTH DCS trains using your Lionel Cab-1L, Cab-2, or Cab-3 remote**

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)

---

## Acknowledgments

This project would not be possible without the foundational work of:

- **[Mark DiVecchio](http://www.silogic.com/trains/RTC_Running.html)** - Reverse-engineered the MTH DCS protocol and created RTC (Remote Train Control). His documentation of DCS commands, lashup creation, and WTIU communication is the foundation this bridge is built on.

- **[Dave Swindell](https://github.com/cdswindell/PyLegacy)** - Created PyLegacy/PyTrain, the definitive Python library for Lionel Legacy/TMCC control. His protocol analysis and source code were invaluable references for this project.

---

## What Is This?

This bridge translates Lionel TMCC and Legacy commands to MTH DCS commands, letting you control MTH Proto-Sound 2 and Proto-Sound 3 locomotives with your Lionel remote. Full support for both TMCC (32-step speed) and Legacy (200-step speed) protocols.

### Key Features

- **Dual Protocol Support** - TMCC and Legacy protocols fully supported
- **Mixed Consist Support** - Run Lionel and MTH engines together in the same lashup
- **200-Step Speed Control** - Legacy's fine-grained speed control mapped to DCS sMPH
- **ProtoWhistle/Quilling Horn** - Legacy whistle slider controls MTH whistle pitch
- **Extended Startup/Shutdown** - Hold power button for full startup/shutdown sequences
- **PFA Announcements** - Passenger/Freight announcements via CAB3
- **Consist-Aware Headlights** - Automatically sets MTH engine headlights based on consist position and direction (lead engine on, others off)
- **Lighting Translation** - Legacy marker lights and strobe/beacon commands translated to MTH DCS equivalents
- **Smoke Control** - Legacy cycles through smoke levels (off/low/med/high), TMCC1 simple on/off
- **Volume Control** - Proper 0-100 scaling for all volume paths (buttons, speed dial)
- **Momentum Control** - Legacy 0-7 momentum levels and TMCC1 low/med/high mapped to MTH acceleration/deceleration rates

---

## Hardware Required

| Component | Purpose |
|-----------|---------|
| **Lionel Base 3** (6-82972) | Receives commands from remote |
| **Lionel Remote** (Cab-1L, Cab-2, or Cab-3) | Your controller |
| **Lionel LCS SER2** (6-81326) | Serial output from Base 3 |
| **FTDI USB-Serial Adapter** | Connects SER2 to Raspberry Pi |
| **MTH WTIU** (50-1039) | WiFi interface to DCS track |
| **Raspberry Pi** (3B+, 4, or 5) | Runs the bridge software |

**Connection:**
```
Remote → Base 3 → SER2 → FTDI → Raspberry Pi → WiFi → WTIU → Track
```

---

## Installation

There are two ways to install:

- **Easy Desktop Install** (recommended for non-savvy users) — download a ZIP, double-click an icon, follow the GUI wizard. No terminal or SSH required.
- **Manual Install** (for advanced users) — SSH in, clone the repo, run `install.sh` from the terminal.

---

### Easy Desktop Install (GUI, no terminal required)

This method works on any Linux desktop with zenity and systemd. Supported platforms:

| Platform | Desktop | Notes |
|----------|---------|-------|
| Raspberry Pi 3B+ / 4 / 5 | Raspberry Pi OS Desktop | Primary target |
| Linux Mint | Cinnamon, MATE, XFCE | zenity preinstalled |
| Ubuntu | GNOME, KDE, XFCE | zenity preinstalled |
| Debian (with desktop) | GNOME, KDE, XFCE, MATE | |
| Pop!_OS | GNOME (COSMIC) | zenity preinstalled |
| Fedora Workstation + spins | GNOME, KDE, XFCE, etc. | zenity preinstalled |
| Manjaro / Arch (with desktop) | GNOME, KDE, XFCE, etc. | |
| openSUSE (with desktop) | GNOME, KDE, XFCE | |

**Steps:**

1. On the machine that will run the bridge (with its desktop screen visible), open a web browser and go to:
   ```
   https://github.com/allenrnemetz/O-Gauge-Command-Control-Bridge/releases/latest
   ```
2. Download **Source code (zip)** from the Assets section
3. Extract the ZIP (right-click → Extract Here)
4. Open the extracted folder and double-click **Install Bridge**
5. Follow the on-screen wizard — it will ask for your username, WTIU address (defaults to auto-discovery), and whether you have WLED
6. Enter your password when prompted (needed to install the background service)
7. Done — the bridge is running and will start automatically on boot

> **If double-click doesn't work:** Right-click "Install Bridge" → "Allow launching" (or Properties → Permissions → allow executing). Alternatively, open a terminal in the folder and run `bash gui_install.sh`.

> **Hardware requirement:** The FTDI USB-serial adapter must be plugged into the machine running the bridge, and that machine must be on the same network subnet as the MTH WTIU.

See **START HERE.txt** in the download for plain-English instructions.

---

### Manual Install (terminal / SSH)

### Step 1: Raspberry Pi Setup

1. Install **Raspberry Pi OS** (Lite or Desktop) on your Pi
2. Connect to your WiFi network
3. Enable SSH if not already enabled:
   ```bash
   sudo raspi-config
   # Navigate to Interface Options → SSH → Enable
   ```
4. Note your Pi's IP address:
   ```bash
   hostname -I
   ```

> **Important:** Raspberry Pi and WTIU must be on the same network subnet

### Step 2: Download the Bridge Software

SSH into your Raspberry Pi and clone the repository:

```bash
cd ~
git clone https://github.com/allenrnemetz/O-Gauge-Command-Control-Bridge.git lionel-mth-bridge
cd lionel-mth-bridge
```

Or copy the files manually via SCP:
```bash
# From your computer:
scp lionel_mth_bridge.py bridge_config.json install.sh lionel-mth-bridge.service main.py <username>@<pi-ip>:~/lionel-mth-bridge/
```

### Step 3: Run the Installer

SSH into your Raspberry Pi and run:

```bash
cd ~/lionel-mth-bridge
chmod +x install.sh
./install.sh
```

The installer will:
- Create a Python virtual environment
- Install Python dependencies (pyserial, zeroconf)
- Create the configuration file
- Set up and enable the systemd service
- Start the bridge automatically

### Step 4: Connect Hardware

1. Connect the FTDI USB-Serial adapter to the Raspberry Pi's USB port
2. Connect the FTDI cable's DB9 end to the LCS SER2

### Step 5: Add Engines to MTH WTIU

**Before using the bridge**, you must add your MTH engines to the WTIU database using the MTH app:

1. Open the **MTH DCS app** on your phone/tablet
2. Connect to your WTIU
3. Go to **Add Engine** and follow the prompts to add each locomotive
4. Note the engine number shown in the app (e.g., "Engine 48")

> **Important:** The bridge can only control engines that are already in the WTIU database

### Step 6: Configuration (Optional)

The bridge auto-discovers MTH engines and maps them automatically. Most users won't need to change anything.

The configuration file is located at `~/.lionel-mth-bridge/bridge_config.json`.

**Default config:**
```json
{
  "lionel_port": "/dev/ttyUSB0",
  "legacy_enabled": true,
  "mth_host": "auto",
  "mth_port": "auto"
}
```

**How engine mapping works:**
- Set your Lionel remote to the same engine number as shown in the MTH app
- Example: MTH app shows "Engine 48" → Use Lionel engine address 48
- The bridge handles the internal DCS addressing automatically

**Optional manual mapping** (only if needed):
```json
{
  "engine_mappings": {
    "10": 49
  }
}
```
This would map Lionel #10 to MTH engine 48 (use the MTH app number + 1 for the DCS value).

### Step 7: Verify It's Working

Check the service status:
```bash
sudo systemctl status lionel-mth-bridge
```

View live logs:
```bash
sudo journalctl -u lionel-mth-bridge -f
```

You should see:
```
✅ Connected to Lionel Base 3 on /dev/ttyUSB0
✅ Connected to MTH WTIU at 192.168.x.x
🎯 Monitoring Lionel Base 3 for TMCC packets...
```

---

## WLED LED Strip Control (Optional)

The bridge can control WLED-based LED strips via your Lionel remote, including a smooth day/night lighting cycle.

**Requirements:** ESP32 with WLED firmware on your network.

**Setup:** During installation, answer "y" when prompted for WLED configuration and enter your controller's IP address and preferred ACC/Switch ID.

**Default Controls (ACC/Switch 50):**
| Keypad | Action |
|--------|--------|
| 1 | Full white (max brightness) |
| 2 | Turn off all LEDs |
| 3 | Start 15-min day/night cycle |
| 4 | Stop day/night cycle |

The daylight cycle smoothly transitions through morning → afternoon → sunset → night → sunrise, with a moon segment visible during night hours.

---

## Verified Commands

### TMCC Mode

| Button/Control | MTH Action | Status |
|----------------|------------|--------|
| **Speed Knob** | Speed control (32-step → 0-120 sMPH) | ✅ Verified |
| **Direction** | Toggle forward/reverse | ✅ Verified |
| **Whistle** (hold) | Whistle on while held | ✅ Verified |
| **Bell** (press) | Toggle bell on/off | ✅ Verified |
| **AUX1** | Quick engine startup | ✅ Verified |
| **Keypad 2** | PFA announcements (start/advance) | ✅ Verified |
| **Keypad 5** | Quick engine shutdown | ✅ Verified |
| **Keypad 8** | Smoke off | ✅ Verified |
| **Keypad 9** | Smoke on | ✅ Verified |
| **Keypad 1** | Volume up | ✅ Verified |
| **Keypad 4** | Volume down | ✅ Verified |
| **Front Coupler** | Fire front coupler | ✅ Verified |
| **Rear Coupler** | Fire rear coupler | ✅ Verified |

### Legacy Mode

| Button/Control | MTH Action | Status |
|----------------|------------|--------|
| **Speed Knob** | 200-step speed → 0-120 sMPH (fine control) | ✅ Verified |
| **Direction** | Direct forward/reverse control | ✅ Verified |
| **Whistle Slider** | ProtoWhistle with 4-level pitch control | ✅ Verified |
| **Bell** (hold >0.5s) | Toggle bell on/off | ✅ Verified |
| **Bell** (quick press) | Single bell ring | ✅ Verified |
| **Power Button** | Extended startup sequence | ✅ Verified |
| **Shutdown Button** | Extended shutdown sequence | ✅ Verified |
| **AUX1** | Quick startup | ✅ Verified |
| **Keypad 1** | Volume up | ✅ Verified |
| **Keypad 4** | Volume down | ✅ Verified |
| **Keypad 5** | Quick shutdown | ✅ Verified |
| **Keypad 2** | PFA announcements (start/advance) | ✅ Verified |
| **Smoke Up** | Cycle smoke: off → low → med → high | ✅ Verified |
| **Smoke Down** | Cycle smoke: high → med → low → off | ✅ Verified |
| **Front Coupler** | Fire front coupler | ✅ Verified |
| **Rear Coupler** | Fire rear coupler | ✅ Verified |
| **Boost** | Increase speed | ✅ Verified |
| **Brake** | Decrease speed | ✅ Verified |
| **Momentum** | 0-7 levels mapped to MTH acceleration/deceleration | ✅ Verified |
| **Marker Lights** | On/off translated to MTH markers | ✅ Verified |
| **Strobe/Beacon** | On/off (single/double flash) translated to MTH beacon | ✅ Verified |
| **Mars Light** | On/off translated to MTH Mars light | ✅ Verified |
| **Cab/Interior Light** | On/off translated to MTH interior light | ✅ Verified |
| **Ditch Lights** | Off/auto/on/flashing translated to MTH ditch lights | ✅ Verified |

### Engines Tested

| Engine | Type | Status |
|--------|------|--------|
| **Chesapeake and Ohio Allegheny** | Steam (PS1->PS3 Upgrade) | ✅ Verified |
| **Marburger Dairy SW1200** | Diesel (PS3) | ✅ Verified |

---

## Consist/Lashup Support

Build a lashup on your Lionel remote as usual. The bridge automatically detects when engines are added to a consist and creates a matching MTH lashup on the WTIU.

### How It Works

1. **Build your lashup** on the Lionel remote (Cab-2/Cab-3)
2. **Bridge detects** the TRAIN_ADDRESS commands from Base 3
3. **MTH lashup created** automatically on the WTIU
4. **Commands sync** - speed and direction go to all engines, horn/bell to lead only

### Mixed Lionel/MTH Consists

You can run Lionel and MTH engines together in the same lashup:
- Lionel engines respond via Base 3 as normal
- MTH engines receive translated commands via the bridge
- Speed commands are synchronized with 100ms debounce for smooth operation
- Horn/bell commands go to the lead engine only (per prototype)

### Lashup Commands

| Control | Action |
|---------|--------|
| **Speed** | All engines in lashup |
| **Direction** | All engines in lashup |
| **Horn/Whistle** | Lead engine only |
| **Bell** | Lead engine only |
| **Startup/Shutdown** | All MTH engines in lashup |
| **Volume** | All MTH engines in lashup |
| **Couplers** | Correct end engine only (orientation-aware) |
| **Smoke** | Cycle through levels (off/low/med/high) |
| **Momentum** | Low/med/high mapped to MTH acceleration/deceleration |
| **Headlights** | Automatic — lead engine on, others off (updated on direction change) |

### Breaking Up a Lashup

When you clear a lashup on the Lionel side, the bridge automatically breaks up the MTH lashup and resets each engine to standalone mode.

---

## TCP Serial Proxy (PyTrain Integration)

The bridge includes a TCP serial proxy that allows other applications like **PyTrain** to share access to the SER2 serial port.

### How It Works

- The bridge owns the SER2 serial port
- A TCP server runs on port **5111** (configurable)
- Connected clients receive all SER2 data in real-time
- Clients can send commands which are forwarded to the SER2

### Connecting PyTrain

The install script displays the PyTrain connection command with your Pi's IP address at the end of installation.

### Configuration

The TCP proxy is enabled by default. To configure it, edit `~/.lionel-mth-bridge/bridge_config.json`:

```json
{
  "tcp_proxy": {
    "enabled": true,
    "port": 5111
  }
}
```

Set `enabled` to `false` to disable the proxy if not needed.

### Multiple Clients

- Multiple clients can connect simultaneously
- All clients receive SER2 data in real-time
- All clients can send commands to the SER2 (serialized through the bridge)
- Your CAB-1L/2/3 remotes continue to work normally via Base 3

---

## Home Assistant Integration

Monitor your bridge from Home Assistant — see connection status and engine libraries from both the Lionel Base 3 and MTH WTIU, all in your HA dashboard.

[![Open your Home Assistant instance and open a repository inside the Home Assistant Community Store.](https://my.home-assistant.io/badges/hacs_repository.svg)](https://my.home-assistant.io/redirect/hacs_repository/?category=integration&owner=allenrnemetz&repository=O-Gauge-Command-Control-Bridge)

[![Add integration to Home Assistant](https://my.home-assistant.io/badges/config_flow_start.svg)](https://my.home-assistant.io/redirect/config_flow_start?domain=lionel_mth_bridge)

### What You Get

| Entity | Type | Shows |
|--------|------|-------|
| Bridge Online | Binary sensor | Bridge process running |
| Base 3 Connected | Binary sensor | SER2 / Base 3 connection state |
| WTIU Connected | Binary sensor | MTH WTIU connection state |
| WTIU Host | Sensor | WTIU IP address |
| Lionel Engine Count | Sensor | Number of engines in Base 3 library |
| MTH Engine Count | Sensor | Number of MTH engines on track |
| Engine sensors | Sensors | Per-engine name, road number, and type |
| Refresh Engines | Button | Triggers a rescan of both WTIU and Base 3 libraries |

### Prerequisites

- The bridge must be running version **1.4.0** or later (includes the HTTP status endpoint)
- [HACS](https://hacs.xyz) must be installed in your Home Assistant instance
- Home Assistant and the bridge must be on the same network

### Installation

1. Click the blue **HACS** button above — it opens HACS in your HA instance with this repo pre-filled
2. Click **Download** to install the integration
3. **Restart Home Assistant** (Settings → System → Power → Restart) — required for HA to load the new integration
4. Click the blue **Add Integration** button above (or go to Settings → Devices & Services → Add Integration → search "Lionel MTH Bridge")
5. Enter your bridge's IP address and port (default: `8580`)
6. Done — entities appear automatically and update every 30 seconds

### How It Works

The bridge runs a lightweight HTTP endpoint (port 8580) that serves JSON status data. The HA integration polls it every 30 seconds — no MQTT broker, no extra software, no YAML configuration.

The endpoint is read-only. Home Assistant cannot send commands to your trains. The only action HA can take is triggering an engine library rescan via the Refresh button.

---

## Service Commands

| Command | Description |
|---------|-------------|
| `sudo systemctl restart lionel-mth-bridge` | Restart after config changes |
| `sudo journalctl -u lionel-mth-bridge -f` | View live logs |
| `sudo systemctl power cycle the bridge` | Power Cycle the bridge |

---

## Updating the Bridge

There are two ways to update, depending on how you installed:

### Easy Update (GUI, for desktop installer users)

If you installed via the Easy Desktop Install method, just double-click **Update Bridge** in the same folder where you originally installed. The updater will:

- Download the latest release from GitHub automatically (no manual download needed)
- Replace the bridge files (your configuration is preserved)
- Refresh Python dependencies
- Restart the bridge service

You may be asked for your password — this is needed to restart the service.

> **If double-click doesn't work:** Right-click "Update Bridge" → "Allow launching" (or Properties → Permissions → allow executing). Alternatively, open a terminal in the folder and run `bash gui_update.sh`.

### Manual Update (terminal, for SSH/git users)

If you installed via the manual method (git clone), run the updater script to pull the latest code, refresh dependencies, and restart the service:

```bash
cd ~/lionel-mth-bridge
./update.sh
```

If you haven't made it executable yet:

```bash
chmod +x ~/lionel-mth-bridge/update.sh
./update.sh
```

The script will:
- `git pull --rebase`
- `pip install --upgrade -r requirements.txt` (inside the venv)
- `sudo systemctl restart lionel-mth-bridge.service`

---

## Troubleshooting

**WTIU not connecting:**
- Verify WTIU is powered and on WiFi
- Check Raspberry Pi is on the same network subnet

**No response from train:**
- Verify engine is added to WTIU (use MTH app first)
- Check `engine_mappings` in config file
- View logs for error messages

**Commands not recognized:**
- Check log output for raw TMCC packets
- Verify remote is paired with Base 3

---

## License

GNU Affero General Public License v3.0 - Copyright (c) 2026 Allen Nemetz
