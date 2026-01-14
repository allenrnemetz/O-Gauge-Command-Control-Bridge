#!/bin/bash
# Lionel MTH Bridge Installation Script
# Automatically installs dependencies and sets up the bridge

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

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_warning "Running as root. This may not be necessary for all operations."
fi

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
    if command -v apt-get &> /dev/null; then
        DISTRO="debian"
    elif command -v yum &> /dev/null; then
        DISTRO="redhat"
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

# Install Python dependencies
print_status "Installing Python dependencies..."

# Check if pip is available
if ! command -v pip3 &> /dev/null; then
    print_status "Installing pip3..."
    if [ "$DISTRO" = "debian" ]; then
        sudo apt-get update
        sudo apt-get install -y python3-pip
    elif [ "$DISTRO" = "redhat" ]; then
        sudo yum install -y python3-pip
    elif [ "$DISTRO" = "arch" ]; then
        sudo pacman -S --noconfirm python-pip
    elif [ "$OS" = "macos" ]; then
        # On macOS, pip should come with python3
        if ! command -v python3 &> /dev/null; then
            print_status "Installing python3..."
            brew install python3
        fi
    fi
fi

# Install required Python packages
print_status "Installing required Python packages..."
pip3 install --user pyserial zeroconf

# Install system dependencies
print_status "Installing system dependencies..."

if [ "$DISTRO" = "debian" ]; then
    sudo apt-get update
    sudo apt-get install -y python3-dev build-essential
elif [ "$DISTRO" = "redhat" ]; then
    sudo yum groupinstall -y "Development Tools"
    sudo yum install -y python3-devel
elif [ "$DISTRO" = "arch" ]; then
    sudo pacman -S --noconfirm base-devel python
elif [ "$OS" = "macos" ]; then
    # Install Xcode command line tools if not present
    if ! xcode-select -p &> /dev/null; then
        print_status "Installing Xcode command line tools..."
        xcode-select --install
    fi
fi

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

# Create systemd service file
print_status "Creating systemd service..."
SERVICE_FILE="/etc/systemd/system/lionel-mth-bridge.service"

if [ -w "/etc/systemd/system" ]; then
    # We can write to systemd directory
    cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Lionel MTH Bridge Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$(pwd)
ExecStart=/usr/bin/python3 $(pwd)/lionel_mth_bridge.py
Restart=always
RestartSec=10
Environment=PYTHONPATH=$(pwd)

[Install]
WantedBy=multi-user.target
EOF

    # Reload systemd and enable service
    sudo systemctl daemon-reload
    sudo systemctl enable lionel-mth-bridge.service
    print_status "Systemd service created and enabled"
else
    print_warning "Cannot create systemd service (no write permissions)"
    print_status "You can manually create the service file at: $SERVICE_FILE"
fi

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

# Create virtual environment
print_status "Creating Python virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

# Install requirements in virtual environment
print_status "Installing requirements in virtual environment..."
source venv/bin/activate
pip install -r requirements.txt

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

# Print next steps
echo ""
echo "🎉 Installation Complete!"
echo "========================"
echo ""
echo "Next steps:"
echo "1. Edit configuration: $CONFIG_DIR/bridge_config.json"
echo "2. Connect your hardware (Lionel Base 3, Arduino, MTH WTIU)"
echo "3. Start the bridge:"
echo "   ./start_bridge.sh"
echo ""
echo "Or start with systemd:"
echo "   sudo systemctl start lionel-mth-bridge"
echo ""
echo "Check logs with:"
echo "   tail -f logs/bridge.log"
echo ""
echo "For help and troubleshooting, see README.md"
