#!/bin/bash
# gui_update.sh - GUI updater for the O Gauge Command Control Bridge
# Downloads the latest release from GitHub, copies new files over the
# existing installation, refreshes dependencies, and restarts the service.
#
# The user just double-clicks "Update Bridge" — no manual download needed.
#
# This script is meant to be launched by double-clicking "Update Bridge.desktop"
# or by running: bash gui_update.sh

set -e

# Make sure we're in the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# --- Constants ---
REPO="allenrnemetz/O-Gauge-Command-Control-Bridge"
GITHUB_API="https://api.github.com/repos/$REPO/releases/latest"
SERVICE_NAME="lionel-mth-bridge"

# --- Check for zenity; try to install it if missing ---
if ! command -v zenity &> /dev/null; then
    if command -v apt-get &> /dev/null; then
        sudo apt-get install -y zenity || true
    elif command -v dnf &> /dev/null; then
        sudo dnf install -y zenity || true
    elif command -v yum &> /dev/null; then
        sudo yum install -y zenity || true
    elif command -v zypper &> /dev/null; then
        sudo zypper install -y zenity || true
    elif command -v pacman &> /dev/null; then
        sudo pacman -S --noconfirm zenity || true
    fi

    if ! command -v zenity &> /dev/null; then
        echo "Could not install zenity. Please run update.sh in a terminal instead."
        exit 1
    fi
fi

# --- Set up GUI sudo askpass ---
ASKPASS_SCRIPT="$(mktemp)"
cat > "$ASKPASS_SCRIPT" << 'ASKPASS'
#!/bin/bash
zenity --password --title="Administrator Password Required" 2>/dev/null
ASKPASS
chmod +x "$ASKPASS_SCRIPT"
export SUDO_ASKPASS="$ASKPASS_SCRIPT"

cleanup() {
    rm -f "$ASKPASS_SCRIPT"
    rm -rf "$TMP_DIR" 2>/dev/null
}
trap cleanup EXIT

# --- Helper: show error and exit ---
die() {
    zenity --error --title="Update Error" --text="$1" 2>/dev/null
    exit 1
}

# --- Check that the bridge is actually installed ---
if ! systemctl list-unit-files 2>/dev/null | grep -q "$SERVICE_NAME"; then
    zenity --info \
        --title="Bridge Not Installed" \
        --text="The O Gauge Command Control Bridge doesn't appear to be installed on this system.\n\n\
If you haven't installed it yet, use 'Install Bridge' instead.\n\n\
If you believe this is an error, check with:\n  sudo systemctl status lionel-mth-bridge" \
        --width=450 2>/dev/null
    exit 1
fi

# --- Find the existing install directory from the systemd service ---
INSTALL_DIR=$(systemctl show "$SERVICE_NAME" -p WorkingDirectory --value 2>/dev/null)

if [ -z "$INSTALL_DIR" ] || [ ! -d "$INSTALL_DIR" ]; then
    # Fallback: try reading the service file directly
    SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
    if [ -f "$SERVICE_FILE" ]; then
        INSTALL_DIR=$(grep '^WorkingDirectory=' "$SERVICE_FILE" | cut -d= -f2)
    fi
fi

if [ -z "$INSTALL_DIR" ] || [ ! -d "$INSTALL_DIR" ]; then
    die "Could not find the bridge installation directory.\n\n\
The systemd service file doesn't point to a valid directory.\n\
You may need to reinstall using 'Install Bridge'."
fi

# --- Get current version (from the source file — fast) ---
CURRENT_VERSION="unknown"
if [ -n "$INSTALL_DIR" ] && [ -f "$INSTALL_DIR/lionel_mth_bridge.py" ]; then
    CURRENT_VERSION=$(grep -o 'BRIDGE_VERSION = "v[0-9]\+\.[0-9]\+\.[0-9]\+[a-z]*"' "$INSTALL_DIR/lionel_mth_bridge.py" | grep -o 'v[0-9]\+\.[0-9]\+\.[0-9]\+[a-z]*' | tail -1 || true)
fi
# Fallback: check journalctl (last 100 lines only) if file grep failed
if [ -z "$CURRENT_VERSION" ] || [ "$CURRENT_VERSION" = "unknown" ]; then
    if command -v journalctl &> /dev/null; then
        CURRENT_VERSION=$(journalctl -u "$SERVICE_NAME" --no-pager -n 100 2>/dev/null | grep 'Bridge started' | grep -o 'v[0-9]\+\.[0-9]\+\.[0-9]\+[a-z]*' | tail -1 || true)
    fi
fi

# --- Confirm with the user ---
zenity --question \
    --title="Update O Gauge Command Control Bridge" \
    --text="This will update the bridge to the latest version from GitHub.\n\n\
Install directory: $INSTALL_DIR\n\
Current version:   ${CURRENT_VERSION:-unknown}\n\n\
The update will:\n\
  • Download the latest release from GitHub\n\
  • Replace the bridge files (your config is preserved)\n\
  • Refresh Python dependencies\n\
  • Restart the bridge service\n\n\
You may be asked for your administrator password.\n\n\
Click Yes to continue." \
    --width=500 2>/dev/null || exit 0

# --- Check for download tools ---
if command -v curl &> /dev/null; then
    DL_CMD="curl -sL"
elif command -v wget &> /dev/null; then
    DL_CMD="wget -qO-"
else
    # Try to install curl
    if command -v apt-get &> /dev/null; then
        sudo -A apt-get install -y curl 2>/dev/null || true
    elif command -v dnf &> /dev/null; then
        sudo -A dnf install -y curl 2>/dev/null || true
    fi
    if command -v curl &> /dev/null; then
        DL_CMD="curl -sL"
    elif command -v wget &> /dev/null; then
        DL_CMD="wget -qO-"
    else
        die "Neither curl nor wget is installed.\n\
Please install one with: sudo apt install curl"
    fi
fi

# --- Download and update in the background, showing a progress dialog ---
TMP_DIR=$(mktemp -d)
LOG_FILE="/tmp/lionel_mth_bridge_update_$$.log"

(
    echo "# Checking for latest release..." >&2
    # Get the latest release tag from GitHub API
    if command -v curl &> /dev/null; then
        LATEST_TAG=$(curl -sL "$GITHUB_API" 2>/dev/null | grep -o '"tag_name": *"[^"]*"' | head -1 | sed 's/.*"tag_name": *"//;s/"//')
    else
        LATEST_TAG=$(wget -qO- "$GITHUB_API" 2>/dev/null | grep -o '"tag_name": *"[^"]*"' | head -1 | sed 's/.*"tag_name": *"//;s/"//')
    fi

    if [ -z "$LATEST_TAG" ]; then
        echo "FAILED: Could not determine the latest release version from GitHub." >&2
        echo "Check your internet connection and try again." >&2
        exit 1
    fi

    echo "# Latest version: $LATEST_TAG — downloading..." >&2
    ZIP_URL="https://github.com/$REPO/archive/refs/tags/$LATEST_TAG.zip"

    # Download the ZIP
    ZIP_FILE="$TMP_DIR/bridge-update.zip"
    if command -v curl &> /dev/null; then
        curl -sL -o "$ZIP_FILE" "$ZIP_URL" 2>&1 || true
    else
        wget -q -O "$ZIP_FILE" "$ZIP_URL" 2>&1 || true
    fi

    if [ ! -f "$ZIP_FILE" ] || [ ! -s "$ZIP_FILE" ]; then
        echo "FAILED: Could not download the update from GitHub." >&2
        echo "URL: $ZIP_URL" >&2
        exit 1
    fi

    echo "# Extracting update..." >&2
    # Extract
    unzip -q "$ZIP_FILE" -d "$TMP_DIR" 2>&1 || {
        echo "FAILED: Could not extract the downloaded ZIP." >&2
        exit 1
    }

    # Find the extracted folder (named like O-Gauge-Command-Control-Bridge-1.3.0)
    EXTRACTED_DIR=$(find "$TMP_DIR" -maxdepth 1 -type d -name "O-Gauge-*" | head -1)
    if [ -z "$EXTRACTED_DIR" ] || [ ! -d "$EXTRACTED_DIR" ]; then
        echo "FAILED: Could not find extracted files." >&2
        exit 1
    fi

    echo "# Installing update files..." >&2
    # Copy the bridge files over the existing installation
    # Preserve: venv/, logs/, start_bridge.sh (generated by installer), requirements.txt (generated)
    # Replace: lionel_mth_bridge.py, tmcc_wled.py, install.sh, gui_install.sh, gui_update.sh,
    #          bridge_config.json (only if user's config is elsewhere), README.md, etc.
    FILES_TO_COPY="lionel_mth_bridge.py tmcc_wled.py install.sh gui_install.sh gui_update.sh
                   'START HERE.txt'
                   'Install (Easy Method).sh' 'Update (Easy Method).sh' setup.sh
                   README.md LICENSE lionel-mth-bridge.service .gitignore .gitattributes
                   hacs.json requirements.txt"

    for f in $FILES_TO_COPY; do
        if [ -f "$EXTRACTED_DIR/$f" ]; then
            cp "$EXTRACTED_DIR/$f" "$INSTALL_DIR/" 2>/dev/null || true
        fi
    done

    # Fix permissions on .sh files so double-click works
    chmod +x "$INSTALL_DIR"/*.sh 2>/dev/null || true

    # Refresh .desktop launcher icons on the user's Desktop
    cat > "$HOME/Desktop/Update Bridge.desktop" << 'DESKTOPEOF'
[Desktop Entry]
Type=Application
Name=Update Bridge
Comment=Update the Lionel-MTH Command Control Bridge to the latest version
Exec=bash -c 'cd "$HOME/lionel-mth-bridge" && bash gui_update.sh'
Icon=system-software-update
Terminal=false
Categories=Utility;System;
StartupNotify=true
DESKTOPEOF

    cat > "$HOME/Desktop/Install Bridge.desktop" << 'DESKTOPEOF'
[Desktop Entry]
Type=Application
Name=Install Bridge
Comment=Install the Lionel-MTH Command Control Bridge
Exec=bash -c 'cd "$HOME/lionel-mth-bridge" && bash gui_install.sh'
Icon=system-software-install
Terminal=false
Categories=Utility;System;
StartupNotify=true
DESKTOPEOF

    for df in "Install Bridge.desktop" "Update Bridge.desktop"; do
        chmod +x "$HOME/Desktop/$df" 2>/dev/null || true
        gio set "$HOME/Desktop/$df" metadata::trusted true 2>/dev/null || true
    done

    # Copy the home_assistant directory (for HA integration files)
    if [ -d "$EXTRACTED_DIR/home_assistant" ]; then
        rm -rf "$INSTALL_DIR/home_assistant" 2>/dev/null || true
        cp -r "$EXTRACTED_DIR/home_assistant" "$INSTALL_DIR/" 2>/dev/null || true
    fi

    # Copy custom_components directory for HACS if it exists
    if [ -d "$EXTRACTED_DIR/custom_components" ]; then
        rm -rf "$INSTALL_DIR/custom_components" 2>/dev/null || true
        cp -r "$EXTRACTED_DIR/custom_components" "$INSTALL_DIR/" 2>/dev/null || true
    fi

    # Copy calibration curves directory if it exists
    if [ -d "$EXTRACTED_DIR/calibration" ]; then
        rm -rf "$INSTALL_DIR/calibration" 2>/dev/null || true
        cp -r "$EXTRACTED_DIR/calibration" "$INSTALL_DIR/" 2>/dev/null || true
    fi

    # Don't overwrite the user's bridge_config.json in the install dir —
    # the real config lives in ~/.lionel-mth-bridge/bridge_config.json
    # which is never touched by the update.

    echo "# Refreshing Python dependencies..." >&2
    # Refresh dependencies in the existing venv
    if [ -d "$INSTALL_DIR/venv" ]; then
        source "$INSTALL_DIR/venv/bin/activate"
        pip install --upgrade pyserial zeroconf 2>&1 || true
    else
        echo "WARNING: venv not found at $INSTALL_DIR/venv — skipping dependency refresh" >&2
    fi

    echo "# Migrating WTIU config to mDNS auto-discovery..." >&2
    # Migrate old hardcoded WTIU host/port to auto-discovery (v1.5.5+)
    # The WTIU picks a random port on each boot, so hardcoding always breaks
    CONFIG_FILE="$HOME/.lionel-mth-bridge/bridge_config.json"
    if [ -f "$CONFIG_FILE" ]; then
        python3 -c "
import json
config_path = '$CONFIG_FILE'
with open(config_path, 'r') as f:
    config = json.load(f)
migrated = False
if config.get('mth_host', 'auto') != 'auto':
    config['mth_host'] = 'auto'
    migrated = True
if config.get('mth_port', 'auto') != 'auto':
    config['mth_port'] = 'auto'
    migrated = True
if 'connection_settings' not in config:
    config['connection_settings'] = {}
if config['connection_settings'].get('mdns_discovery') is not True:
    config['connection_settings']['mdns_discovery'] = True
    migrated = True
if config['connection_settings'].get('default_port') != 'auto':
    config['connection_settings']['default_port'] = 'auto'
    migrated = True
if migrated:
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=4)
    print('MIGRATED: WTIU config updated to mDNS auto-discovery')
else:
    print('OK: WTIU already using auto-discovery')
" 2>&1 || true
    fi

    echo "# Setting up log cleanup..." >&2
    # Install log cleanup cron job if not already present (added in v1.4)
    CRON_SCRIPT="/etc/cron.daily/lionel-mth-bridge-log-cleanup"
    if [ ! -f "$CRON_SCRIPT" ]; then
        cat << 'CRONEOF' | sudo -A tee "$CRON_SCRIPT" > /dev/null 2>&1
#!/bin/bash
# Clean up old bridge logs — keep last 30 days of journald entries
journalctl --vacuum-time=30d --quiet 2>/dev/null || true

# Also clean up any log files in the install directory (older than 30 days)
find "$HOME/lionel-mth-bridge/logs" -name "*.log" -mtime +30 -delete 2>/dev/null || true
CRONEOF
        sudo -A chmod +x "$CRON_SCRIPT" 2>/dev/null || true
    fi

    echo "# Deploying calibration curves..." >&2
    if [ -d "$INSTALL_DIR/calibration" ]; then
        mkdir -p "$HOME/.lionel-mth-bridge/calibration"
        cp "$INSTALL_DIR/calibration"/curve_*.json "$HOME/.lionel-mth-bridge/calibration/" 2>/dev/null || true
    fi

    echo "# Restarting bridge service..." >&2
    # Restart the service
    sudo -A systemctl restart "$SERVICE_NAME" 2>&1 || {
        echo "WARNING: Could not restart the service automatically." >&2
        echo "Try: sudo systemctl restart lionel-mth-bridge" >&2
    }

    echo "# Update complete!" >&2
    echo "SUCCESS:$LATEST_TAG" >&2

) 2> "$LOG_FILE" &

UPDATE_PID=$!

# Show a pulsing progress dialog while the update runs
while kill -0 $UPDATE_PID 2>/dev/null; do
    echo "100"
    sleep 0.5
done | zenity --progress \
    --title="Updating..." \
    --text="Updating the O Gauge Command Control Bridge.\n\n\
Downloading and installing the latest version...\n\
Please wait, this may take a minute." \
    --pulsate \
    --auto-close \
    --width=450 2>/dev/null

# Wait for the background process to finish
wait $UPDATE_PID
UPDATE_EXIT=$?

# Check the log for success or failure
if grep -q "^SUCCESS:" "$LOG_FILE" 2>/dev/null; then
    NEW_VERSION=$(grep "^SUCCESS:" "$LOG_FILE" | sed 's/SUCCESS://')

    # Get the host IP for display
    HOST_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    if [ -z "$HOST_IP" ]; then
        HOST_IP="(your machine's IP)"
    fi

    # Check service status
    SERVICE_STATUS=$(systemctl is-active "$SERVICE_NAME" 2>/dev/null || echo "unknown")

    if [ "$SERVICE_STATUS" = "active" ]; then
        STATUS_TEXT="The bridge service is running."
    else
        STATUS_TEXT="The bridge service status: $SERVICE_STATUS\n\
Check with: sudo systemctl status lionel-mth-bridge"
    fi

    # Check if HA status endpoint is configured
    HA_INFO=""
    CONFIG_FILE="$HOME/.lionel-mth-bridge/bridge_config.json"

    if [ -f "$CONFIG_FILE" ]; then
        # Check if ha_status is already in the config file
        HA_EXISTS=$(python3 -c "import json; c=json.load(open('$CONFIG_FILE')); print('yes' if 'ha_status' in c else 'no')" 2>/dev/null || echo "no")

        if [ "$HA_EXISTS" = "no" ]; then
            # v1.3 user upgrading — ha_status not in their config yet
            # Ask if they want to enable the HA endpoint
            zenity --question \
                --title="Home Assistant" \
                --text="This version adds Home Assistant integration.\n\n\
The bridge can expose a status endpoint that lets Home\n\
Assistant monitor bridge connections and engine libraries.\n\n\
Do you use Home Assistant?" \
                --width=450 2>/dev/null

            if [ $? -eq 0 ]; then
                # User wants HA — ask for port
                HA_PORT=$(zenity --entry \
                    --title="Home Assistant: Status Port" \
                    --text="Enter the port for the HA status endpoint.\n\n\
Home Assistant will connect to this port to read bridge status.\n\
Default is 8580." \
                    --entry-text="8580" \
                    --width=400 2>/dev/null) || HA_PORT="8580"

                if [ -z "$HA_PORT" ]; then
                    HA_PORT="8580"
                fi

                # Write ha_status to config
                python3 -c "
import json
config_path = '$CONFIG_FILE'
with open(config_path, 'r') as f:
    config = json.load(f)
config['ha_status'] = {'enabled': True, 'port': $HA_PORT}
with open(config_path, 'w') as f:
    json.dump(config, f, indent=4)
" 2>/dev/null

                HA_INFO="\n\nHome Assistant endpoint: http://$HOST_IP:$HA_PORT/status\n\
  To use with HA, install the 'O Gauge Command Control Bridge' integration via HACS.\n\
  See the Home Assistant section in README.md for setup instructions."

                # Restart the service so the endpoint starts
                sudo -A systemctl restart "$SERVICE_NAME" 2>/dev/null || true
            else
                # User doesn't want HA — explicitly disable it
                python3 -c "
import json
config_path = '$CONFIG_FILE'
with open(config_path, 'r') as f:
    config = json.load(f)
config['ha_status'] = {'enabled': False, 'port': 8580}
with open(config_path, 'w') as f:
    json.dump(config, f, indent=4)
" 2>/dev/null
            fi
        else
            # ha_status already configured — just read it
            HA_PORT=$(python3 -c "import json; c=json.load(open('$CONFIG_FILE')); print(c.get('ha_status',{}).get('port',8580))" 2>/dev/null || echo "8580")
            HA_ENABLED=$(python3 -c "import json; c=json.load(open('$CONFIG_FILE')); print(c.get('ha_status',{}).get('enabled',True))" 2>/dev/null || echo "True")
            if [ "$HA_ENABLED" = "True" ]; then
                HA_INFO="\n\nHome Assistant endpoint: http://$HOST_IP:$HA_PORT/status\n\
  To use with HA, install the 'O Gauge Command Control Bridge' integration via HACS.\n\
  See the Home Assistant section in README.md for setup instructions."
            fi
        fi
    fi

    zenity --info \
        --title="Update Complete" \
        --text="The bridge has been updated to version $NEW_VERSION.\n\n\
$STATUS_TEXT\n\n\
TCP Serial Proxy: $HOST_IP:5111$HA_INFO" \
        --width=500 2>/dev/null
else
    # Show error with log tail
    ERROR_TAIL=$(tail -15 "$LOG_FILE" 2>/dev/null | grep -v '^#' || echo "See log for details")
    zenity --error \
        --title="Update Failed" \
        --text="The update did not complete successfully.\n\n\
Output:\n$ERROR_TAIL\n\n\
Full log: $LOG_FILE" \
        --width=500 2>/dev/null
    exit 1
fi

# Clean up
rm -f "$LOG_FILE"

exit 0
