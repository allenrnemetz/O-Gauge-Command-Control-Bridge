# O Gauge Command Control Bridge — Home Assistant Integration

Monitor your O Gauge Command Control Bridge from Home Assistant.
See connection status and engine libraries from both the Lionel Base 3 and MTH WTIU, all in your HA dashboard.

[![Open your Home Assistant instance and open a repository inside the Home Assistant Community Store.](https://my.home-assistant.io/badges/hacs_repository.svg)](https://my.home-assistant.io/redirect/hacs_repository/?category=integration&owner=allenrnemetz&repository=O-Gauge-Command-Control-Bridge)

[![Add integration to Home Assistant](https://my.home-assistant.io/badges/config_flow_start.svg)](https://my.home-assistant.io/redirect/config_flow_start?domain=lionel_mth_bridge)

## What You Get

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

## Prerequisites

- The bridge must be running version **1.4.0** or later (includes the HTTP status endpoint on port 8580)
- [HACS](https://hacs.xyz) must be installed in your Home Assistant instance
- Home Assistant and the bridge (Raspberry Pi or Linux PC) must be on the same network

## Installation

### Step 1: Install via HACS

1. Click the blue **HACS** button at the top of this page — it opens HACS in your HA instance with this repository pre-filled
2. Click **Download** to install the integration
3. **Restart Home Assistant** (Settings → System → Power → Restart) — this is required for HA to load the new integration

Alternatively, in HA:
1. Go to **HACS** → **Integrations**
2. Click the three dots (top right) → **Custom repositories**
3. Add `https://github.com/allenrnemetz/O-Gauge-Command-Control-Bridge` as type **Integration**
4. Find "O Gauge Command Control Bridge" and click **Download**
5. **Restart Home Assistant** (Settings → System → Power → Restart)

### Step 2: Configure

1. Click the blue **Add Integration** button at the top of this page
2. Enter your bridge's IP address (e.g. `192.168.1.50`)
3. Enter the port (default: `8580`)
4. Click **Submit**

Alternatively, in HA:
1. Go to **Settings** → **Devices & Services** → **Add Integration**
2. Search for "O Gauge Command Control Bridge"
3. Enter the bridge IP and port

> **Same machine as the bridge?** If Home Assistant is running on the same machine as the bridge (Raspberry Pi or Linux PC):
> - **HA Supervised (bare metal)**: use `localhost` or `127.0.0.1` as the bridge IP
> - **HA Container (Docker)**: use the machine's LAN IP (e.g. `192.168.1.50`), not `localhost` — inside a Docker container, `localhost` refers to the container itself, not the host
> - **HAOS (Home Assistant OS)**: you cannot run the bridge on HAOS — it doesn't support systemd services or direct USB serial access. Run the bridge on a separate Raspberry Pi or Linux PC.

### Step 3: Use

Entities appear automatically and update every 30 seconds. Add them to your dashboard as you see fit.

If you add or remove engines from your WTIU or Base 3, click the **Refresh Engines** button in HA to trigger an immediate rescan. Engine sensors update within 30 seconds.

## How It Works

The bridge runs a lightweight HTTP endpoint (port 8580) that serves JSON status data. The HA integration polls it every 30 seconds — no MQTT broker, no extra software, no YAML configuration.

The endpoint is **read-only**. Home Assistant cannot send commands to your trains. The only action HA can take is triggering an engine library rescan via the Refresh button.

## Configuration

The HTTP endpoint is configured in the bridge's `bridge_config.json`:

```json
{
    "ha_status": {
        "enabled": true,
        "port": 8580
    }
}
```

Set `enabled` to `false` to disable the endpoint if you don't use Home Assistant.

## Fixing Wrong Engine Names

MTH engine names come from the PS2/PS3 controller board inside each engine. If an engine was upgraded (e.g. PS1 to PS2, or PS2 to PS3), the name stored in the controller may be wrong or outdated. You can fix this in the MTH handheld or app, but those corrections are stored in the handheld — not on the WTIU — so the bridge will keep showing the old name from the controller board.

### Rename from Home Assistant (recommended)

Each engine has an editable text entity (e.g. `text.mth_engine_1_name`). To rename an engine:

1. Find the engine's text entity in Home Assistant (it has a pencil icon)
2. Click it and type the correct name
3. Press Enter — the name is saved to the bridge instantly
4. The engine sensor updates within 30 seconds with the new name

To revert to the original polled name from the controller board, clear the text field and press Enter.

The original polled name is preserved as a `polled_name` attribute on the engine sensor, so you can always see what the controller board reported.

### Rename via config file (alternative)

You can also edit `~/.lionel-mth-bridge/bridge_config.json` directly:

```json
{
    "engine_name_overrides": {
        "mth": {
            "1": "Pennsylvania GG1",
            "3": "Union Pacific Big Boy"
        },
        "lionel": {
            "5": "Chessie Steam Special",
            "12": "New Haven EP5"
        }
    }
}
```

Then restart the bridge: `sudo systemctl restart lionel-mth-bridge`

- The keys are engine IDs as strings (DCS ID for MTH, TMCC ID for Lionel)
- The values are the display names you want to see in Home Assistant
- Remove an entry to revert to the polled name

## Troubleshooting

**Cannot connect during setup:**
- Make sure the bridge is running (`sudo systemctl status lionel-mth-bridge`)
- Verify the IP address and port are correct
- Check that port 8580 is not blocked by a firewall
- Try `curl http://<pi-ip>:8580/status` from another machine on the network

**Entities show as unavailable:**
- The bridge may be offline or restarting
- Check the bridge logs: `sudo journalctl -u lionel-mth-bridge -f`
- The HTTP endpoint starts automatically with the bridge

**Engine sensors not updating after adding engines:**
- Click the **Refresh Engines** button in HA
- Wait up to 30 seconds for the next poll cycle
- For WTIU engines: the bridge also auto-discovers every 60 seconds
- For Base 3 engines: use the Refresh button (no periodic scan by design)
