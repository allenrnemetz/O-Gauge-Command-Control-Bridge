#!/bin/bash
# gui_install.sh - GUI installer for the Lionel MTH Bridge
# Uses zenity dialogs so non-savvy users can install without a terminal.
# Falls back to the text-based install.sh if zenity is unavailable.
#
# This script is meant to be launched by double-clicking "Install Bridge.desktop"
# or by running: bash gui_install.sh

set -e

# Make sure we're in the directory containing this script
# (the .desktop launcher may not set CWD correctly)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# --- Colors (for any terminal fallback output) ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# --- Check for zenity; try to install it if missing ---
if ! command -v zenity &> /dev/null; then
    # Try to install zenity via the available package manager
    if command -v apt-get &> /dev/null; then
        PKG_CMD="sudo apt-get install -y zenity"
    elif command -v dnf &> /dev/null; then
        PKG_CMD="sudo dnf install -y zenity"
    elif command -v yum &> /dev/null; then
        PKG_CMD="sudo yum install -y zenity"
    elif command -v zypper &> /dev/null; then
        PKG_CMD="sudo zypper install -y zenity"
    elif command -v pacman &> /dev/null; then
        PKG_CMD="sudo pacman -S --noconfirm zenity"
    else
        PKG_CMD=""
    fi

    if [ -n "$PKG_CMD" ]; then
        echo -e "${YELLOW}zenity is not installed. Attempting to install it...${NC}"
        $PKG_CMD || true
    fi

    # Re-check
    if ! command -v zenity &> /dev/null; then
        echo -e "${RED}Could not install zenity. Falling back to text-based installer.${NC}"
        echo "Please open a terminal in this folder and run:  ./install.sh"
        # Try to launch a terminal with install.sh as a last resort
        if command -v xterm &> /dev/null; then
            xterm -e "bash install.sh; read -p 'Press Enter to close...'"
        elif command -v lxterminal &> /dev/null; then
            lxterminal -e "bash install.sh; read -p 'Press Enter to close...'"
        elif command -v gnome-terminal &> /dev/null; then
            gnome-terminal -- bash -c "bash install.sh; read -p 'Press Enter to close...'"
        fi
        exit 1
    fi
fi

# --- Set up a GUI sudo askpass helper so sudo prompts appear as GUI dialogs ---
ASKPASS_SCRIPT="$(mktemp)"
cat > "$ASKPASS_SCRIPT" << 'ASKPASS'
#!/bin/bash
zenity --password --title="Administrator Password Required" 2>/dev/null
ASKPASS
chmod +x "$ASKPASS_SCRIPT"
export SUDO_ASKPASS="$ASKPASS_SCRIPT"

# Clean up the temp askpass script on exit
cleanup() {
    rm -f "$ASKPASS_SCRIPT"
}
trap cleanup EXIT

# --- Helper: show an error dialog and exit ---
die() {
    zenity --error --title="Installation Error" --text="$1" 2>/dev/null
    exit 1
}

# --- Welcome dialog ---
zenity --info \
    --title="Lionel MTH Bridge Installer" \
    --text="Welcome to the Lionel MTH Bridge Installer!\n\n\
This wizard will install the bridge software that lets you control\n\
MTH DCS trains with your Lionel Cab-1L, Cab-2, or Cab-3 remote.\n\n\
You will need:\n\
  • Your FTDI USB-serial adapter plugged in\n\
  • Your MTH WTIU powered on and on the same network\n\n\
Click OK to continue." \
    --width=450 2>/dev/null || exit 0

# --- Step 1: Username ---
PI_USER=$(zenity --entry \
    --title="Step 1 of 4: Username" \
    --text="Enter the username that the bridge service should run as.\n\n\
This is almost always your current login name.\n\
Just click OK unless you know you need a different user." \
    --entry-text="$USER" \
    --width=400 2>/dev/null) || die "Installation cancelled."

if [ -z "$PI_USER" ]; then
    PI_USER="$USER"
fi

# --- Step 2: MTH WTIU IP ---
MTH_IP=$(zenity --entry \
    --title="Step 2 of 4: MTH WTIU Address" \
    --text="Enter the IP address of your MTH WTIU.\n\n\
If you're not sure, just leave it as 'auto' — the bridge will\n\
find your WTIU automatically on the network (mDNS discovery).\n\n\
Only enter a specific IP if auto-discovery doesn't work for you." \
    --entry-text="auto" \
    --width=450 2>/dev/null) || die "Installation cancelled."

if [ -z "$MTH_IP" ]; then
    MTH_IP="auto"
fi

MTH_PORT="auto"
if [ "$MTH_IP" != "auto" ]; then
    MTH_PORT=$(zenity --entry \
        --title="Step 2b: MTH WTIU Port" \
        --text="Enter the port number for your MTH WTIU.\n\n\
Leave it as 'auto' to discover the port automatically via mDNS.\n\n\
Only enter a specific port number if auto-discovery doesn't work\n\
and you know the exact port your WTIU is using." \
        --entry-text="auto" \
        --width=400 2>/dev/null) || die "Installation cancelled."

    if [ -z "$MTH_PORT" ]; then
        MTH_PORT="auto"
    fi
fi

# --- Step 3: WLED ---
zenity --question \
    --title="Step 3 of 4: WLED LED Controller" \
    --text="Do you have a WLED LED strip controller to configure?\n\n\
WLED is optional — it lets you control LED lighting from your\n\
Lionel remote. If you don't have WLED, click No.\n\n\
Click Yes only if you have a WLED controller on your network." \
    --width=450 2>/dev/null

if [ $? -eq 0 ]; then
    CONFIGURE_WLED="y"

    WLED_IP=$(zenity --entry \
        --title="WLED: IP Address" \
        --text="Enter the IP address of your WLED controller.\n\n\
You can find this in your router's device list or in the\n\
WLED web interface." \
        --entry-text="192.168.0.10" \
        --width=400 2>/dev/null) || die "Installation cancelled."

    if [ -z "$WLED_IP" ]; then
        WLED_IP="192.168.0.10"
    fi

    WLED_ACC_ID=$(zenity --entry \
        --title="WLED: Accessory ID" \
        --text="Enter the ACC/Switch ID (1-99) to use for WLED control.\n\n\
This is the Lionel accessory address you'll use to trigger\n\
lighting effects from your remote. Default is 50." \
        --entry-text="50" \
        --width=400 2>/dev/null) || die "Installation cancelled."

    if [ -z "$WLED_ACC_ID" ]; then
        WLED_ACC_ID="50"
    fi

    WLED_LED_COUNT=$(zenity --entry \
        --title="WLED: LED Count" \
        --text="Enter the total number of LEDs on your strip(s)." \
        --entry-text="100" \
        --width=400 2>/dev/null) || die "Installation cancelled."

    if [ -z "$WLED_LED_COUNT" ]; then
        WLED_LED_COUNT="100"
    fi
else
    CONFIGURE_WLED="n"
fi

# --- Step 4: Home Assistant ---
zenity --question \
    --title="Step 4 of 5: Home Assistant" \
    --text="Do you use Home Assistant?\n\n\
The bridge can expose a status endpoint that lets Home\n\
Assistant monitor bridge connections and engine libraries.\n\n\
If you don't use Home Assistant, click No." \
    --width=450 2>/dev/null

if [ $? -eq 0 ]; then
    CONFIGURE_HA="y"

    HA_PORT=$(zenity --entry \
        --title="Home Assistant: Status Port" \
        --text="Enter the port for the HA status endpoint.\n\n\
Home Assistant will connect to this port to read bridge status.\n\
Default is 8580." \
        --entry-text="8580" \
        --width=400 2>/dev/null) || die "Installation cancelled."

    if [ -z "$HA_PORT" ]; then
        HA_PORT="8580"
    fi
else
    CONFIGURE_HA="n"
fi

# --- Step 5: Confirm ---
CONFIRM_TEXT="Ready to install with these settings:\n\n\
  Username:       $PI_USER\n\
  MTH WTIU:       $MTH_IP"

if [ "$MTH_IP" != "auto" ]; then
    CONFIRM_TEXT="$CONFIRM_TEXT:$MTH_PORT"
fi
CONFIRM_TEXT="$CONFIRM_TEXT\n\
  WLED:           "

if [ "$CONFIGURE_WLED" = "y" ]; then
    CONFIRM_TEXT="$CONFIRM_TEXT Yes (IP: $WLED_IP, ACC: $WLED_ACC_ID, LEDs: $WLED_LED_COUNT)"
else
    CONFIRM_TEXT="$CONFIRM_TEXT No"
fi

CONFIRM_TEXT="$CONFIRM_TEXT\n\
  Home Assistant: "

if [ "$CONFIGURE_HA" = "y" ]; then
    CONFIRM_TEXT="$CONFIRM_TEXT Yes (port: $HA_PORT)"
else
    CONFIRM_TEXT="$CONFIRM_TEXT No"
fi

CONFIRM_TEXT="$CONFIRM_TEXT\n\n\
You may be asked for your administrator password next.\n\
This is needed to install the system service.\n\n\
Click Yes to begin installation."

zenity --question \
    --title="Step 5 of 5: Confirm" \
    --text="$CONFIRM_TEXT" \
    --ok-label="Install" \
    --cancel-label="Cancel" \
    --width=450 2>/dev/null || die "Installation cancelled."

# --- Run the installer non-interactively ---
# Export all collected values as env vars so install.sh skips its prompts
export PI_USER
export MTH_IP
export MTH_PORT
export CONFIGURE_WLED
export WLED_IP
export WLED_ACC_ID
export WLED_LED_COUNT
export CONFIGURE_HA
export HA_PORT
export NONINTERACTIVE=1

# Run install.sh and capture output, showing a progress dialog
LOG_FILE="/tmp/lionel_mth_bridge_install_$$.log"
rm -f "$LOG_FILE"

# Run the installer in the background, teeing output to the log file
( bash install.sh > "$LOG_FILE" 2>&1 ) &
INSTALL_PID=$!

# Show a pulsing progress dialog while the installer runs
while kill -0 $INSTALL_PID 2>/dev/null; do
    echo "100"  # pulsing progress bar stays full
    sleep 0.5
done | zenity --progress \
    --title="Installing..." \
    --text="Installing the Lionel MTH Bridge.\n\n\
This may take a minute or two. Please wait...\n\n\
  • Creating Python environment\n\
  • Installing dependencies\n\
  • Writing configuration\n\
  • Setting up system service" \
    --pulsate \
    --auto-close \
    --width=450 2>/dev/null

# Check if the installer succeeded
wait $INSTALL_PID
INSTALL_EXIT=$?

if [ $INSTALL_EXIT -ne 0 ]; then
    # Show the last 20 lines of the log in an error dialog
    ERROR_TAIL=$(tail -20 "$LOG_FILE" 2>/dev/null || echo "See log for details")
    zenity --error \
        --title="Installation Failed" \
        --text="Installation did not complete successfully.\n\n\
Error output:\n$ERROR_TAIL\n\n\
The full log is at: $LOG_FILE" \
        --width=500 2>/dev/null
    exit 1
fi

# --- Get the host IP for the success message ---
HOST_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
if [ -z "$HOST_IP" ]; then
    HOST_IP="(your machine's IP address)"
fi

# --- Success dialog ---
# Find the install directory (install.sh relocates to ~/lionel-mth-bridge)
INSTALL_DIR=$(systemctl show lionel-mth-bridge -p WorkingDirectory --value 2>/dev/null || echo "$HOME/lionel-mth-bridge")

SUCCESS_TEXT="Installation complete!\n\n\
The bridge is now running and will start automatically on boot.\n\n\
Install location: $INSTALL_DIR\n\n\
TCP Serial Proxy (for PyTrain):\n  $HOST_IP:5111\n\
  Connect with:  pytrain -ser2 $HOST_IP:5111\n"

if [ "$CONFIGURE_HA" = "y" ]; then
    SUCCESS_TEXT="$SUCCESS_TEXT\n\
Home Assistant status endpoint:\n  http://$HOST_IP:$HA_PORT/status\n\
  Enter this IP and port when adding the integration in HA.\n"
fi

SUCCESS_TEXT="$SUCCESS_TEXT\n\
Useful commands (in a terminal):\n\
  sudo systemctl status lionel-mth-bridge\n\
  sudo journalctl -u lionel-mth-bridge -f\n\
  sudo systemctl restart lionel-mth-bridge"

zenity --info \
    --title="Installation Complete" \
    --text="$SUCCESS_TEXT" \
    --width=500 2>/dev/null

# Clean up the log file
rm -f "$LOG_FILE"

exit 0
