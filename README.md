# Lionel-MTH Bridge

**Control MTH DCS trains using your Lionel Cab-1L, Cab-2, or Cab-3 remote**

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Status: Beta](https://img.shields.io/badge/Status-Beta-yellow.svg)]()

---
## Credits

- **Mark DiVecchio** - MTH WTIU protocol research ([silogic.com](http://www.silogic.com/trains/RTC_Running.html))
- **Lionel LLC** - TMCC protocol documentation
- **O Gauge Railroading Forum** - Community support

---

## What Is This?

This project bridges Lionel's TMCC/Legacy command system to MTH's DCS system, allowing you to control MTH DCS locomotives using your existing Lionel remote control.

**Use your Lionel remote → Control MTH trains**

---

## Features

- **Whistle** - Hold button to blow, release to stop
- **Bell** - Toggle on/off with each press  
- **Speed Control** - Smooth relative speed changes
- **Direction** - Toggle forward/reverse
- **Startup/Shutdown** - Full engine sequences
- **Smoke On/Off** - Control smoke unit
- **Volume Up/Down** - Adjust master volume

---

## Hardware Requirements

| Component | Model | Purpose |
|-----------|-------|---------|
| Lionel Base 3 | 6-82972 | TMCC command receiver |
| Lionel Remote | Cab-1L, Cab-2, or Cab-3 | Your controller |
| Lionel LCS SER2 | 6-81326 | Serial output from Base 3 |
| FTDI USB-Serial | Any 9600 baud | Connect SER2 to computer |
| MTH WTIU | 50-1039 | WiFi DCS interface |
| Arduino UNO Q | ABX00162 | Bridge processor |

### Connection Diagram

```
Lionel Remote → Base 3 → SER2 → FTDI → Arduino UNO Q → WiFi → MTH WTIU → Track
```

---

## Quick Start

### 1. Hardware Setup

1. Connect SER2 to Lionel Base 3 LCS port
2. Connect FTDI cable to SER2 DB9 port
3. Connect FTDI USB to Arduino UNO Q
4. Power on MTH WTIU and connect to your WiFi network

### 2. Arduino UNO Q First-Time Setup

If this is your first time using the Arduino UNO Q:

1. **Download Arduino App Lab** from the Arduino website
2. **Connect the board** via USB-C to your computer
3. **Open Arduino App Lab** and follow the setup wizard
4. **Connect to WiFi**:
   - In App Lab, go to **Settings → Network**
   - Select your WiFi network (must be the **same network** as your MTH WTIU)
   - Enter your WiFi password
   - Note the board's IP address once connected
5. **Verify network connectivity**:
   - The WTIU and Arduino must be on the same subnet (e.g., both on `192.168.0.x`)

### 3. Software Installation

SSH into your Arduino UNO Q:

```bash
ssh arduino@<your-board-ip>

# Install dependencies
sudo apt update
sudo apt install -y python3-serial python3-zeroconf python3-pycryptodome

# Copy the bridge script
cd /home/arduino/ArduinoApps
mkdir -p lcs-to-mth-bridge/python
cd lcs-to-mth-bridge/python
```

Copy `lionel_mth_bridge.py` and `lionel-mth-bridge.service` to this directory.

### 4. Configure Auto-Start Service

To have the bridge start automatically when the Arduino boots:

```bash
# Copy the service file to systemd
sudo cp lionel-mth-bridge.service /etc/systemd/system/

# Reload systemd to recognize the new service
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable lionel-mth-bridge.service

# Start the service now
sudo systemctl start lionel-mth-bridge.service

# Check status
sudo systemctl status lionel-mth-bridge.service
```

**Service Commands:**

| Command | Description |
|---------|-------------|
| `sudo systemctl start lionel-mth-bridge` | Start the bridge |
| `sudo systemctl stop lionel-mth-bridge` | Stop the bridge |
| `sudo systemctl restart lionel-mth-bridge` | Restart the bridge |
| `sudo systemctl status lionel-mth-bridge` | Check status |
| `sudo journalctl -u lionel-mth-bridge -f` | View live logs |

### 5. Manual Run (Optional)

If you prefer to run manually instead of using the service:

```bash
python3 lionel_mth_bridge.py
```

You should see:
```
✅ Connected to Lionel Base 3 on /dev/ttyUSB0
✅ Connected to MTH WTIU at 192.168.x.x
🎯 Monitoring Lionel Base 3 for TMCC packets...
```

---

## Remote Control Mapping

### Lionel Cab-1L / Cab-2 / Cab-3

| Button | Function | MTH Action |
|--------|----------|------------|
| **Whistle** | Hold to blow | Whistle on while held |
| **Bell** | Press to toggle | Bell on/off |
| **Speed Knob** | Turn | Relative speed change |
| **Direction** | Press | Toggle forward/reverse |
| **AUX1** | Startup | Engine startup sequence |
| **Keypad 5** | Shutdown | Engine shutdown sequence |
| **Keypad 8** | Smoke Off | Turn smoke unit off |
| **Keypad 9** | Smoke On | Turn smoke unit on |
| **Keypad 1** | Volume Up | Increase master volume |
| **Keypad 4** | Volume Down | Decrease master volume |

---

## Configuration

Edit `lionel_mth_bridge.py` to customize:

```python
# MTH WTIU connection
self.mth_host = '192.168.x.xxxx'  # Your WTIU IP address
self.mth_port = xxxxx           # WTIU port

# Lionel Base 3 serial port
self.lionel_port = '/dev/ttyUSB0'  # FTDI adapter port
```

### Engine Mapping

The bridge maps Lionel engine numbers to MTH DCS engine numbers:

| Lionel Engine # | MTH WTIU Engine # |
|-----------------|-------------------|
| 10 | 11 |
| 11 | 12 |
| 5 | 6 |
| Others | +1 offset |

Adjust the mapping in `send_to_mth()` if needed.

---

## Troubleshooting

### WTIU Connection Issues

1. Verify WTIU is powered on and connected to WiFi
2. Check that Arduino UNO Q is on the same network
3. Try setting the WTIU IP manually in the script

### No Response from Train

1. Verify engine is added to WTIU (use MTH app first)
2. Check engine number mapping in the script
3. Look at log output for error messages

### Commands Not Recognized

Check the log output - it shows the raw TMCC packets received. If you see "Failed to parse packet", the data_field value may need to be added to the mapping.

---

## Technical Details

### TMCC Packet Format

```
Byte 0: 0xFE (sync byte)
Byte 1: Address bits 15-8
Byte 2: Command and data bits 7-0
```

### MTH DCS Commands

| Command | Description |
|---------|-------------|
| `d0` | Direction forward |
| `d1` | Direction reverse |
| `s{0-120}` | Speed (0-120 scale) |
| `w2` | Whistle on |
| `w4` | Bell on |
| `bFFFD` | Whistle off |
| `bFFFB` | Bell off |
| `u4` | Engine startup |
| `u5` | Engine shutdown |
| `abF` | Smoke on |
| `abE` | Smoke off |

---


## License

GNU General Public License v3.0

Copyright (c) 2026 Allen Nemetz

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

---

## Contributing

This is a beta release for user testing. Please report issues on GitHub with:

1. Log output showing the problem
2. Your hardware configuration
3. Steps to reproduce

Pull requests welcome!
