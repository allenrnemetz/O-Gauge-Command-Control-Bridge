#!/bin/bash
# gui_update.sh - GUI updater for the Lionel MTH Bridge
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
REPO="allenrnemetz/O-Gauge-Command-Control-Bridge-on-Pi"
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
        --text="The Lionel MTH Bridge doesn't appear to be installed on this system.\n\n\
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

# --- Get current version (from the running service or file) ---
CURRENT_VERSION="unknown"
if command -v journalctl &> /dev/null; then
    CURRENT_VERSION=$(journalctl -u "$SERVICE_NAME" --no-pager -n 50 2>/dev/null | grep -o 'v[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1 || true)
fi

# --- Confirm with the user ---
zenity --question \
    --title="Update Lionel MTH Bridge" \
    --text="This will update the bridge to the latest version from GitHub.\n\n\
Install directory: $INSTALL_DIR\n\
Current version:   ${CURRENT_VERSION:-unknown}\n\n\
The update will:\n\
  • Download the latest release from GitHub\n\
  • Replace the bridge files (your config is preserved)\n\
  • Refresh Python dependencies\n\
  • Restart the bridge service\n\n\
You may be asked for your administrator password.\n\n\
Click OK to continue." \
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

    # Find the extracted folder (named like O-Gauge-Command-Control-Bridge-on-Pi-1.3.0)
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
                   'Install Bridge.desktop' 'Update Bridge.desktop' 'START HERE.txt'
                   README.md LICENSE lionel-mth-bridge.service .gitignore .gitattributes"

    for f in $FILES_TO_COPY; do
        if [ -f "$EXTRACTED_DIR/$f" ]; then
            cp "$EXTRACTED_DIR/$f" "$INSTALL_DIR/" 2>/dev/null || true
        fi
    done

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
    --text="Updating the Lionel MTH Bridge.\n\n\
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

    zenity --info \
        --title="Update Complete" \
        --text="The bridge has been updated to version $NEW_VERSION.\n\n\
$STATUS_TEXT\n\n\
TCP Serial Proxy: $HOST_IP:5111" \
        --width=450 2>/dev/null
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
