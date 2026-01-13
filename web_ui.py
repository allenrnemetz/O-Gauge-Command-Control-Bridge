#!/usr/bin/env python3
"""
Web UI for Lionel MTH Bridge - Remote monitoring and control
Provides Flask-based web interface for status monitoring and manual control
"""

import json
import time
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import threading
from lionel_mth_bridge import LionelMTHBridge

app = Flask(__name__)
app.config['SECRET_KEY'] = 'lionel-mth-bridge-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global bridge instance
bridge = None
bridge_thread = None

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('index.html')

@app.route('/api/status')
def get_status():
    """Get current bridge status"""
    if not bridge:
        return jsonify({'error': 'Bridge not running'})
    
    status = {
        'lionel_connected': bridge.lionel_serial and bridge.lionel_serial.is_open,
        'mth_connected': bridge.mth_connected,
        'mcu_connected': bridge.mcu_connected,
        'current_engine': bridge.current_lionel_engine,
        'engine_speeds': bridge.engine_speeds,
        'engine_directions': bridge.engine_directions,
        'queue_size': bridge.command_queue.get_queue_size() if bridge.command_queue else 0,
        'uptime': time.time() - getattr(bridge, 'start_time', time.time()),
        'last_command': getattr(bridge, 'last_command_time', {}),
        'volume': bridge.master_volume,
        'whistle_state': bridge.quillable_whistle_on,
        'available_mth_engines': bridge.available_mth_engines,
        'discovered_mappings': bridge.discovered_mth_engines,
        'manual_mappings': bridge.engine_mappings
    }
    return jsonify(status)

@app.route('/api/config')
def get_config():
    """Get current configuration"""
    if not bridge:
        return jsonify({'error': 'Bridge not running'})
    
    return jsonify(bridge.settings)

@app.route('/api/config', methods=['POST'])
def update_config():
    """Update configuration"""
    if not bridge:
        return jsonify({'error': 'Bridge not running'})
    
    try:
        new_config = request.json
        # Merge with existing config
        bridge.settings.update(new_config)
        bridge.config.save(bridge.settings)
        return jsonify({'success': True})
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/api/command', methods=['POST'])
def send_command():
    """Send manual command"""
    if not bridge:
        return jsonify({'error': 'Bridge not running'})
    
    try:
        command = request.json
        # Add command to queue
        success = bridge.command_queue.add_command(command)
        return jsonify({'success': success, 'queue_size': bridge.command_queue.get_queue_size()})
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/api/reconnect')
def reconnect():
    """Force reconnection"""
    if not bridge:
        return jsonify({'error': 'Bridge not running'})
    
    try:
        # Reconnect all services
        lionel_ok = bridge.connect_lionel()
        mth_ok = bridge.connect_mth()
        mcu_ok = bridge.connect_mcu()
        
        return jsonify({
            'lionel': lionel_ok,
            'mth': mth_ok, 
            'mcu': mcu_ok
        })
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/api/refresh_engines')
def refresh_engines():
    """Re-discover MTH engines from WTIU"""
    if not bridge:
        return jsonify({'error': 'Bridge not running'})
    
    try:
        bridge.discover_mth_engines()
        return jsonify({
            'success': True,
            'available_engines': bridge.available_mth_engines,
            'discovered_mappings': bridge.discovered_mth_engines
        })
    except Exception as e:
        return jsonify({'error': str(e)})

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    emit('status', get_status().json)

@socketio.on('subscribe')
def handle_subscribe():
    """Subscribe to status updates"""
    # Start status update thread
    def status_updater():
        while True:
            if bridge:
                socketio.emit('status', get_status().json)
            time.sleep(1)
    
    thread = threading.Thread(target=status_updater, daemon=True)
    thread.start()

def create_templates():
    """Create HTML templates"""
    import os
    
    templates_dir = 'templates'
    if not os.path.exists(templates_dir):
        os.makedirs(templates_dir)
    
    # Create index.html
    index_html = '''<!DOCTYPE html>
<html>
<head>
    <title>Lionel MTH Bridge Monitor</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        .status-indicator { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }
        .status-connected { background-color: #10b981; }
        .status-disconnected { background-color: #ef4444; }
        .status-warning { background-color: #f59e0b; }
    </style>
</head>
<body class="bg-gray-900 text-white">
    <div class="container mx-auto p-4">
        <h1 class="text-3xl font-bold mb-6">🚂 Lionel MTH Bridge Monitor</h1>
        
        <!-- Connection Status -->
        <div class="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
            <div class="bg-gray-800 p-4 rounded">
                <h3 class="font-semibold mb-2">Lionel Base 3</h3>
                <span id="lionel-status" class="status-indicator status-disconnected"></span>
                <span id="lionel-text" class="ml-2">Disconnected</span>
            </div>
            <div class="bg-gray-800 p-4 rounded">
                <h3 class="font-semibold mb-2">MTH WTIU</h3>
                <span id="mth-status" class="status-indicator status-disconnected"></span>
                <span id="mth-text" class="ml-2">Disconnected</span>
            </div>
            <div class="bg-gray-800 p-4 rounded">
                <h3 class="font-semibold mb-2">Arduino MCU</h3>
                <span id="mcu-status" class="status-indicator status-disconnected"></span>
                <span id="mcu-text" class="ml-2">Disconnected</span>
            </div>
        </div>
        
        <!-- Engine Status -->
        <div class="bg-gray-800 p-4 rounded mb-6">
            <h2 class="text-xl font-semibold mb-4">Engine Status</h2>
            <div id="engine-info" class="grid grid-cols-1 md:grid-cols-2 gap-4">
                <!-- Engine info will be populated here -->
            </div>
            
            <!-- Engine Mapping -->
            <div class="mt-4">
                <h3 class="text-lg font-semibold mb-2">Engine Mapping</h3>
                <div id="engine-mapping" class="grid grid-cols-1 md:grid-cols-3 gap-4">
                    <!-- Mapping info will be populated here -->
                </div>
            </div>
        </div>
        
        <!-- Queue Status -->
        <div class="bg-gray-800 p-4 rounded mb-6">
            <h2 class="text-xl font-semibold mb-4">Command Queue</h2>
            <div class="flex items-center space-x-4">
                <span>Queue Size:</span>
                <span id="queue-size" class="font-mono bg-gray-700 px-2 py-1 rounded">0</span>
                <button onclick="clearQueue()" class="bg-red-600 hover:bg-red-700 px-3 py-1 rounded">Clear Queue</button>
            </div>
        </div>
        
        <!-- Manual Controls -->
        <div class="bg-gray-800 p-4 rounded mb-6">
            <h2 class="text-xl font-semibold mb-4">Manual Controls</h2>
            <div class="grid grid-cols-2 md:grid-cols-4 gap-4">
                <button onclick="sendCommand('horn')" class="bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded">Horn</button>
                <button onclick="sendCommand('bell')" class="bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded">Bell</button>
                <button onclick="sendCommand('smoke_on')" class="bg-green-600 hover:bg-green-700 px-4 py-2 rounded">Smoke On</button>
                <button onclick="sendCommand('smoke_off')" class="bg-red-600 hover:bg-red-700 px-4 py-2 rounded">Smoke Off</button>
            </div>
        </div>
        
        <!-- Reconnect Controls -->
        <div class="bg-gray-800 p-4 rounded">
            <h2 class="text-xl font-semibold mb-4">Connection Control</h2>
            <button onclick="reconnect()" class="bg-orange-600 hover:bg-orange-700 px-4 py-2 rounded">Force Reconnect</button>
        </div>
    </div>
    
    <script>
        const socket = io();
        
        socket.on('status', function(data) {
            updateStatus(data);
        });
        
        function updateStatus(data) {
            // Update connection indicators
            updateConnectionStatus('lionel', data.lionel_connected);
            updateConnectionStatus('mth', data.mth_connected);
            updateConnectionStatus('mcu', data.mcu_connected);
            
            // Update engine info
            updateEngineInfo(data);
            
            // Update engine mapping
            updateEngineMapping(data);
            
            // Update queue size
            document.getElementById('queue-size').textContent = data.queue_size;
        }
        
        function updateConnectionStatus(prefix, connected) {
            const indicator = document.getElementById(prefix + '-status');
            const text = document.getElementById(prefix + '-text');
            
            if (connected) {
                indicator.className = 'status-indicator status-connected';
                text.textContent = 'Connected';
            } else {
                indicator.className = 'status-indicator status-disconnected';
                text.textContent = 'Disconnected';
            }
        }
        
        function updateEngineInfo(data) {
            const engineInfo = document.getElementById('engine-info');
            engineInfo.innerHTML = '';
            
            // Current engine
            if (data.current_engine > 0) {
                const div = document.createElement('div');
                div.className = 'bg-gray-700 p-3 rounded';
                div.innerHTML = `
                    <h4 class="font-semibold">Engine ${data.current_engine}</h4>
                    <p>Speed: ${data.engine_speeds[data.current_engine] || 0}</p>
                    <p>Direction: ${data.engine_directions[data.current_engine] || 'unknown'}</p>
                `;
                engineInfo.appendChild(div);
            }
            
            // Volume and whistle status
            const statusDiv = document.createElement('div');
            statusDiv.className = 'bg-gray-700 p-3 rounded';
            statusDiv.innerHTML = `
                <h4 class="font-semibold">System Status</h4>
                <p>Volume: ${data.volume}%</p>
                <p>Whistle: ${data.whistle_state ? 'On' : 'Off'}</p>
                <p>Uptime: ${Math.floor(data.uptime / 60)}m</p>
            `;
            engineInfo.appendChild(statusDiv);
        }
        
        function updateEngineMapping(data) {
            const mappingDiv = document.getElementById('engine-mapping');
            mappingDiv.innerHTML = '';
            
            // Available MTH engines
            if (data.available_mth_engines && data.available_mth_engines.length > 0) {
                const mthDiv = document.createElement('div');
                mthDiv.className = 'bg-gray-700 p-3 rounded';
                mthDiv.innerHTML = `
                    <h4 class="font-semibold">Available MTH Engines</h4>
                    <p class="text-green-400">${data.available_mth_engines.join(', ')}</p>
                `;
                mappingDiv.appendChild(mthDiv);
            }
            
            // Discovered mappings
            if (data.discovered_mappings && Object.keys(data.discovered_mappings).length > 0) {
                const autoDiv = document.createElement('div');
                autoDiv.className = 'bg-gray-700 p-3 rounded';
                let mappings = [];
                for (const [lionel, mth] of Object.entries(data.discovered_mappings)) {
                    mappings.push(`Lionel #${lionel} → MTH #${mth}`);
                }
                autoDiv.innerHTML = `
                    <h4 class="font-semibold">Auto-Mapped</h4>
                    <p class="text-blue-400">${mappings.join('<br>')}</p>
                `;
                mappingDiv.appendChild(autoDiv);
            }
            
            // Manual mappings
            if (data.manual_mappings && Object.keys(data.manual_mappings).length > 0) {
                const manualDiv = document.createElement('div');
                manualDiv.className = 'bg-gray-700 p-3 rounded';
                let mappings = [];
                for (const [lionel, mth] of Object.entries(data.manual_mappings)) {
                    mappings.push(`Lionel #${lionel} → MTH #${mth}`);
                }
                manualDiv.innerHTML = `
                    <h4 class="font-semibold">Manual Mappings</h4>
                    <p class="text-yellow-400">${mappings.join('<br>')}</p>
                `;
                mappingDiv.appendChild(manualDiv);
            }
        }
        
        function sendCommand(command) {
            fetch('/api/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({type: 'function', value: command})
            })
            .then(response => response.json())
            .then(data => {
                if (data.error) {
                    alert('Error: ' + data.error);
                }
            });
        }
        
        function reconnect() {
            fetch('/api/reconnect')
            .then(response => response.json())
            .then(data => {
                if (data.error) {
                    alert('Error: ' + data.error);
                } else {
                    alert('Reconnection initiated');
                }
            });
        }
        
        function clearQueue() {
            // This would need to be implemented in the bridge
            alert('Queue clearing not yet implemented');
        }
        
        // Subscribe to updates
        socket.emit('subscribe');
    </script>
</body>
</html>'''
    
    with open(os.path.join(templates_dir, 'index.html'), 'w') as f:
        f.write(index_html)

def start_bridge():
    """Start the bridge in background"""
    global bridge
    bridge = LionelMTHBridge()
    bridge.start_time = time.time()
    bridge.running = True
    
    # Start connections
    bridge.connect_lionel()
    bridge.connect_mth()
    bridge.connect_mcu()
    
    # Start command queue
    bridge.command_queue.start(bridge)
    
    # Start connection monitor
    bridge.start_connection_monitor()

if __name__ == '__main__':
    # Create templates
    create_templates()
    
    # Start bridge
    start_bridge()
    
    # Start web server
    print("🌐 Starting Web UI on http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
