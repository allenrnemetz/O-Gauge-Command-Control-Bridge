#!/bin/bash
# Lionel MTH Bridge Installation Script
# Automatically installs dependencies and sets up the bridge
#
# Can be run interactively (default) or non-interactively by setting
# NONINTERACTIVE=1 and providing config via environment variables:
#   PI_USER, MTH_IP, MTH_PORT, CONFIGURE_WLED, WLED_IP, WLED_ACC_ID, WLED_LED_COUNT
# When NONINTERACTIVE=1, any unset variable falls back to its default.

set -e  # Exit on any error

echo "🚂 Lionel MTH Bridge Installation Script"
echo "========================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Helper: prompt for a value if not already set via env var.
# Usage: prompt_with_default VAR_NAME "prompt text" "default value"
# If the env var is already set, uses it without prompting.
# Otherwise prompts interactively (unless NONINTERACTIVE=1, then uses default).
prompt_with_default() {
    local var_name="$1"
    local prompt_text="$2"
    local default_val="$3"
    local current_val="${!var_name}"

    if [[ -n "$current_val" ]]; then
        print_status "$prompt_text: $current_val (from env)"
    elif [[ "$NONINTERACTIVE" == "1" ]]; then
        eval "$var_name=\"$default_val\""
        print_status "$prompt_text: $default_val (default, non-interactive)"
    else
        read -p "$prompt_text [$default_val]: " input
        eval "$var_name=\"${input:-$default_val}\""
    fi
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_warning "Running as root. This may not be necessary for all operations."
fi

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
    if command -v apt-get &> /dev/null; then
        DISTRO="debian"
    elif command -v dnf &> /dev/null; then
        DISTRO="fedora"
    elif command -v yum &> /dev/null; then
        DISTRO="redhat"
    elif command -v zypper &> /dev/null; then
        DISTRO="opensuse"
    elif command -v pacman &> /dev/null; then
        DISTRO="arch"
    else
        print_error "Unsupported Linux distribution"
        exit 1
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
else
    print_error "Unsupported operating system: $OSTYPE"
    exit 1
fi

print_status "Detected OS: $OS"
if [ "$OS" = "linux" ]; then
    print_status "Detected distribution: $DISTRO"
fi

# Prompt for username
echo ""
prompt_with_default PI_USER "Enter the username to run the service as" "$USER"
print_status "Service will run as user: $PI_USER"

# Get the home directory for the specified user
PI_HOME=$(eval echo ~$PI_USER)
print_status "Home directory: $PI_HOME"

# Create virtual environment first (required for PEP 668 compliant systems)
print_status "Creating Python virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install required Python packages in venv
print_status "Installing required Python packages..."
pip install pyserial zeroconf

# Create configuration directory
print_status "Creating configuration directory..."
CONFIG_DIR="$HOME/.lionel-mth-bridge"
mkdir -p "$CONFIG_DIR"

# Copy default configuration if it doesn't exist
if [ ! -f "$CONFIG_DIR/bridge_config.json" ]; then
    print_status "Creating default configuration..."
    cp bridge_config.json "$CONFIG_DIR/bridge_config.json"
else
    print_warning "Configuration already exists at $CONFIG_DIR/bridge_config.json"
fi

# MTH WTIU Configuration
echo ""
echo "🚂 MTH WTIU Configuration"
echo "========================"
prompt_with_default MTH_IP "Enter the IP address of your MTH WTIU (or 'auto' for mDNS discovery)" "auto"

if [ "$MTH_IP" != "auto" ]; then
    prompt_with_default MTH_PORT "Enter the port of your MTH WTIU" "33069"
    MTH_HOST="$MTH_IP:$MTH_PORT"
    print_status "Configuring MTH WTIU at $MTH_HOST"

    # Update the config file with MTH WTIU settings using Python for JSON manipulation
    python3 << EOF
import json

config_path = "$CONFIG_DIR/bridge_config.json"
with open(config_path, 'r') as f:
    config = json.load(f)

config['mth_host'] = "$MTH_HOST"
config['mth_port'] = $MTH_PORT

with open(config_path, 'w') as f:
    json.dump(config, f, indent=4)

print("✅ MTH WTIU configuration saved")
EOF
else
    print_status "Using mDNS auto-discovery for MTH WTIU"
fi

# WLED Configuration
echo ""
echo "🎨 WLED LED Strip Controller Configuration"
echo "==========================================="
prompt_with_default CONFIGURE_WLED "Do you have a WLED controller to configure? (y/n)" "n"

if [[ "$CONFIGURE_WLED" =~ ^[Yy]$ ]]; then
    prompt_with_default WLED_IP "Enter the IP address of your WLED controller" "192.168.0.10"
    prompt_with_default WLED_ACC_ID "Enter the ACC/Switch ID to control WLED (1-99)" "50"
    prompt_with_default WLED_LED_COUNT "Enter the total number of LEDs on your strip(s)" "100"

    print_status "Configuring WLED: IP=$WLED_IP, ACC ID=$WLED_ACC_ID, LED Count=$WLED_LED_COUNT"
    
    # Update the config file with WLED settings using Python for JSON manipulation
    python3 << EOF
import json

config_path = "$CONFIG_DIR/bridge_config.json"
with open(config_path, 'r') as f:
    config = json.load(f)

config['wled'] = {
    "enabled": True,
    "host": "$WLED_IP",
    "port": 80,
    "daylight_cycle": True,
    "cycle_duration_sec": 900,
    "led_count": $WLED_LED_COUNT,
    "moon_start": 0,
    "moon_length": 5,
    "mapping": {
        "$WLED_ACC_ID": {
            "17": "full_white",
            "18": "off",
            "19": "daylight_start",
            "20": "daylight_stop"
        }
    }
}

with open(config_path, 'w') as f:
    json.dump(config, f, indent=4)

print("✅ WLED configuration saved")
EOF

    echo ""
    echo "WLED Control Mapping (ACC/Switch $WLED_ACC_ID):"
    echo "  Keypad 1 = Full white (all LEDs on, max brightness)"
    echo "  Keypad 2 = Turn off all LEDs"
    echo "  Keypad 3 = Start daylight cycle (15-min day/night)"
    echo "  Keypad 4 = Stop daylight cycle"
else
    print_status "Skipping WLED configuration"
fi

# Create systemd service file
print_status "Creating systemd service..."
SERVICE_FILE="/etc/systemd/system/lionel-mth-bridge.service"

# Create service file content using the specified username
INSTALL_DIR=$(pwd)
SERVICE_CONTENT="[Unit]
Description=Lionel MTH Bridge Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$PI_USER
WorkingDirectory=$INSTALL_DIR
ExecStartPre=/bin/sleep 10
ExecStart=$INSTALL_DIR/venv/bin/python $INSTALL_DIR/lionel_mth_bridge.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target"

# Use sudo to write the service file
echo "$SERVICE_CONTENT" | sudo tee "$SERVICE_FILE" > /dev/null

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable lionel-mth-bridge.service
print_status "Systemd service created and enabled"

# Create startup scripts
print_status "Creating startup scripts..."

# Main startup script
cat > start_bridge.sh << 'EOF'
#!/bin/bash
# Start Lionel MTH Bridge

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Change to script directory
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies if needed
pip install -r requirements.txt 2>/dev/null || true

# Start the bridge
echo "🚂 Starting Lionel MTH Bridge..."
python3 lionel_mth_bridge.py
EOF

# Make scripts executable
chmod +x start_bridge.sh

# Create requirements.txt
print_status "Creating requirements.txt..."
cat > requirements.txt << 'EOF'
pyserial>=3.5
zeroconf>=0.39.0
EOF

# Create log directory
print_status "Creating log directory..."
mkdir -p logs

# Test installation
print_status "Testing installation..."
source venv/bin/activate

# Test Python imports
python3 -c "
import serial
import socket
import json
import threading
import time
print('✅ All required modules imported successfully')
"

if [ $? -eq 0 ]; then
    print_status "✅ Installation completed successfully!"
else
    print_error "❌ Installation test failed"
    exit 1
fi

# Start the service
print_status "Starting the bridge service..."
sudo systemctl start lionel-mth-bridge.service

# Get the Pi's IP address for display
PI_IP=$(hostname -I | awk '{print $1}')

# Update README.md with actual Pi IP address
if [ -f "README.md" ]; then
    sed -i "s/<pi-ip>/$PI_IP/g" README.md
    print_status "Updated README.md with Pi IP address: $PI_IP"
fi

# Print next steps
echo ""
echo "🎉 Installation Complete!"
echo "========================"
echo ""
echo "The bridge service is now running and will start automatically on boot."
echo ""
echo "TCP Serial Proxy is available at: $PI_IP:5111"
echo "  - PyTrain can connect using: pytrain -ser2 $PI_IP:5111"
echo ""
echo "Useful commands:"
echo "  sudo systemctl status lionel-mth-bridge   # Check status"
echo "  sudo journalctl -u lionel-mth-bridge -f   # View logs"
echo "  sudo systemctl restart lionel-mth-bridge  # Restart service"
