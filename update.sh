#!/bin/bash
# Lionel MTH Bridge Updater
# Pull latest code, update dependencies, and restart service

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ ! -d "venv" ]; then
  echo "Virtual environment not found. Please run install.sh first."
  exit 1
fi

source venv/bin/activate

echo "🚂 Updating repository..."
git pull --rebase

echo "📦 Updating Python dependencies..."
pip install --upgrade -r requirements.txt 2>/dev/null || true
# Ensure zeroconf is always installed (needed for WTIU mDNS auto-discovery)
pip install zeroconf 2>/dev/null || true

# Check if HA status endpoint is configured (added in v1.4)
CONFIG_FILE="$HOME/.lionel-mth-bridge/bridge_config.json"
if [ -f "$CONFIG_FILE" ]; then
    # Migrate old hardcoded WTIU host/port to auto-discovery (v1.5.5+)
    # The WTIU picks a random port on each boot, so hardcoding always breaks
    MTH_HOST=$(python3 -c "import json; c=json.load(open('$CONFIG_FILE')); print(c.get('mth_host','auto'))" 2>/dev/null || echo "auto")
    if [ "$MTH_HOST" != "auto" ]; then
        echo ""
        echo "🔍 Migrating WTIU connection to mDNS auto-discovery..."
        echo "   Your WTIU was configured with a specific IP:port, but the WTIU"
        echo "   changes its port on every reboot. Auto-discovery finds it automatically."
        python3 -c "
import json
config_path = '$CONFIG_FILE'
with open(config_path, 'r') as f:
    config = json.load(f)
config['mth_host'] = 'auto'
config['mth_port'] = 'auto'
if 'connection_settings' not in config:
    config['connection_settings'] = {}
config['connection_settings']['mdns_discovery'] = True
config['connection_settings']['default_port'] = 'auto'
with open(config_path, 'w') as f:
    json.dump(config, f, indent=4)
print('   ✅ WTIU now uses mDNS auto-discovery')
" 2>/dev/null || echo "   ⚠️ Could not migrate config — you may need to set mth_host='auto' manually"
        echo ""
    fi

    HA_EXISTS=$(python3 -c "import json; c=json.load(open('$CONFIG_FILE')); print('yes' if 'ha_status' in c else 'no')" 2>/dev/null || echo "no")
    if [ "$HA_EXISTS" = "no" ]; then
        echo ""
        echo "📊 Home Assistant Integration"
        echo "   This version adds Home Assistant support."
        echo "   The bridge can expose a status endpoint for HA monitoring."
        echo ""
        read -p "   Do you use Home Assistant? (y/n) " -r
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            read -p "   Enter port (default 8580): " HA_PORT
            HA_PORT=${HA_PORT:-8580}
            python3 -c "
import json
config_path = '$CONFIG_FILE'
with open(config_path, 'r') as f:
    config = json.load(f)
config['ha_status'] = {'enabled': True, 'port': $HA_PORT}
with open(config_path, 'w') as f:
    json.dump(config, f, indent=4)
"
            echo "   ✅ HA status endpoint enabled on port $HA_PORT"
        else
            python3 -c "
import json
config_path = '$CONFIG_FILE'
with open(config_path, 'r') as f:
    config = json.load(f)
config['ha_status'] = {'enabled': False, 'port': 8580}
with open(config_path, 'w') as f:
    json.dump(config, f, indent=4)
"
            echo "   HA status endpoint disabled"
        fi
        echo ""
    fi
fi

# Deploy calibration curves (speed-trap conversion curves for speed matching)
CONFIG_DIR="$HOME/.lionel-mth-bridge"
if [ -d "calibration" ]; then
    echo "📐 Deploying speed calibration curves..."
    mkdir -p "$CONFIG_DIR/calibration"
    cp calibration/curve_*.json "$CONFIG_DIR/calibration/" 2>/dev/null || true
    CURVE_COUNT=$(ls -1 "$CONFIG_DIR/calibration"/curve_*.json 2>/dev/null | wc -l)
    if [ "$CURVE_COUNT" -gt 0 ]; then
        echo "   ✅ Deployed $CURVE_COUNT calibration curve(s)"
    else
        echo "   ⚠️ No calibration curve files found in repository"
    fi
fi

# Install log cleanup cron job if not already present (added in v1.4)
CRON_SCRIPT="/etc/cron.daily/lionel-mth-bridge-log-cleanup"
if [ ! -f "$CRON_SCRIPT" ]; then
    echo "🧹 Setting up automatic log cleanup..."
    cat << 'CRONEOF' | sudo tee "$CRON_SCRIPT" > /dev/null
#!/bin/bash
# Clean up old bridge logs — keep last 30 days of journald entries
journalctl --vacuum-time=30d --quiet 2>/dev/null || true

# Also clean up any log files in the install directory (older than 30 days)
find "$HOME/lionel-mth-bridge/logs" -name "*.log" -mtime +30 -delete 2>/dev/null || true
CRONEOF
    sudo chmod +x "$CRON_SCRIPT"
fi

echo "🔄 Restarting service..."
sudo systemctl restart lionel-mth-bridge.service

echo "✅ Update complete."
