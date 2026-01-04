/*
 * mcu_mth_handler.ino
 * 
 * MTH WTIU Handler for Arduino UNO Q MCU (Sub-processor)
 * Communicates with MPU via serial port and controls MTH trains via WiFi
 * 
 * Author: Allen Nemetz
 * Credits:
 * - Mark DiVecchio for his immense work translating MTH commands to and from the MTH WTIU
 *   http://www.silogic.com/trains/RTC_Running.html
 * - Lionel LLC for publishing TMCC and Legacy protocol specifications
 * - O Gauge Railroading Forum (https://www.ogrforum.com/) for the model railroad community
 * 
 * Disclaimer: This software is provided "as-is" without warranty. The author assumes no liability 
 * for any damages resulting from the use or misuse of this software. Users are responsible for 
 * ensuring safe operation of their model railroad equipment.
 * 
 * Copyright (c) 2026 Allen Nemetz. All rights reserved.
 * 
 * License: GNU General Public License v3.0
 * 
 * This sketch runs on the Arduino UNO Q MCU (sub-processor) and handles:
 * - WiFi communication with MTH WTIU devices
 * - mDNS discovery for automatic WTIU detection
 * - Speck encryption for secure MTH protocol communication
 * - Command reception from MPU via serial port
 * - Real-time train control and status monitoring
 */

// Arduino App Lab compatibility
#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <ArduinoMDNS.h>
#include <Monitor.h>  // Required for App Lab Serial Monitor
#include "speck_functions.h"

// Command constants (must match MPU)
#define CMD_SPEED           1
#define CMD_DIRECTION       2
#define CMD_BELL            3
#define CMD_WHISTLE         4
#define CMD_STARTUP         5
#define CMD_SHUTDOWN        6
#define CMD_ENGINE_SELECT   7
#define CMD_PROTOWHISTLE    8
#define CMD_WLED            9

// Command packet structure (must match MPU)
struct CommandPacket {
  uint8_t command_type;
  uint8_t engine_number;
  uint16_t value;
  bool bool_value;
};

// WiFi configuration - UPDATE THESE VALUES
const char* ssid = "YOUR_WIFI_SSID";        // <-- Your WiFi network name
const char* password = "YOUR_WIFI_PASSWORD";  // <-- Your WiFi password

// MTH WTIU configuration - UPDATE THESE VALUES
const char* wtiu_ip = "192.168.0.100";      // <-- Your MTH WTIU IP address
const int wtiu_port = 8882;                   // MTH WTIU default port

// mDNS service configuration for MTH WTIU
const char* wtiu_service = "wtiu";
const char* wtiu_protocol = "tcp";

// Alternative: Use WiFiManager for configuration (uncomment to enable)
// #include <WiFiManager.h>
// WiFiManager wifiManager;

// Global variables
WiFiClient wtiu_client;
char wtiu_host[16] = "0.0.0.0";       // Will be populated by mDNS
int wtiu_port_dynamic = 8882;           // Will be updated from mDNS
bool wtiu_connected = false;
unsigned long last_connection_attempt = 0;
const unsigned long CONNECTION_RETRY_INTERVAL = 5000; // 5 seconds

// mDNS instance
MDNS mdns;

// MPU-MCU communication via internal serial (Serial1 on Arduino UNO Q)
// Note: Serial is for USB/Monitor, Serial1 is for MPU communication

// Status LED
#define STATUS_LED LED_BUILTIN

// ProtoWhistle state
bool protowhistle_enabled = false;
int protowhistle_pitch = 0; // 0-3 pitch levels

// Speck encryption state
SPECK_TYPE key[4] = {0x0100, 0x0302, 0x0504, 0x0706}; // Default key from RTCRemote
SPECK_TYPE round_keys[SPECK_ROUNDS];
bool encryption_enabled = true;

void setup() {
  // Initialize communication with MPU and App Lab Monitor
  Serial.begin(115200);
  Monitor.begin();  // Initialize App Lab Monitor
  Monitor.println("=== MTH WTIU Handler Starting ===");
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  // Test LED blink to show MCU is running
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);
    delay(200);
  }
  
  Monitor.println("MCU initialized - LED test complete");
  
  // Initialize WiFi
  initializeWiFi();
  
  // Initialize Speck encryption
  speck_expand(key, round_keys);
  Monitor.println("Speck encryption initialized");
  
  // Initialize Serial1 for MPU-MCU communication (internal UART)
  Serial1.begin(115200);
  Monitor.println("Serial1 initialized for MPU communication");
  
  // Discover and connect to MTH WTIU using mDNS
  if (!discoverWTIU()) {
    Monitor.println("Failed to find WTIU on startup, will retry in background");
  }
  
  Monitor.println("=== MTH WTIU Handler Ready ===");
  Monitor.println("ProtoWhistle support enabled");
}

void loop() {
  // Check for commands from MPU via Serial1 (internal UART)
  checkSerialCommands();
  
  // Check for commands from USB Serial (fallback/testing)
  if (Serial.available()) {
    receiveCommandFromMPU();
  }
  
  // Maintain WTIU connection
  maintainWTIUConnection();
  
  delay(10);
}

void initializeWiFi() {
  Monitor.println("=== Starting WiFi Connection ===");
  Monitor.print("Connecting to network: ");
  Monitor.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Monitor.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Monitor.println("\n✅ WiFi connected successfully!");
    Monitor.print("IP address: ");
    Monitor.println(WiFi.localIP());
    Monitor.print("Signal strength (RSSI): ");
    Monitor.print(WiFi.RSSI());
    Monitor.println(" dBm");
    
    // Initialize mDNS for WTIU discovery
    if (mdns.begin("lionel-mth-bridge")) {
      Monitor.println("✅ mDNS responder started successfully");
    } else {
      Monitor.println("⚠️ Error setting up mDNS responder");
    }
  } else {
    Monitor.println("\n❌ WiFi connection failed!");
    Monitor.print("Final status: ");
    Monitor.println(WiFi.status());
  }
}

bool discoverWTIU() {
  Monitor.println("=== Starting WTIU Discovery ===");
  Monitor.println("Searching for MTH WTIU devices via mDNS...");
  
  // Use ArduinoMDNS library for service discovery
  int n = mdns.queryService("wtiu", "tcp");
  Monitor.print("mDNS query completed. Found ");
  Monitor.print(n);
  Monitor.println(" WTIU service(s)");
  
  if (n == 0) {
    Monitor.println("❌ No WTIU services found");
    Monitor.println("💡 Troubleshooting tips:");
    Monitor.println("   - Ensure MTH WTIU is powered on");
    Monitor.println("   - Check WTIU is on same 2.4GHz WiFi network");
    Monitor.println("   - Verify WTIU is advertising mDNS services");
    return false;
  }
  
  Monitor.println("🎯 WTIU services discovered:");
  for (int i = 0; i < n; i++) {
    Monitor.print("  Service ");
    Monitor.print(i + 1);
    Monitor.print(": ");
    Monitor.print(mdns.hostname(i));
    Monitor.print(" (");
    Monitor.print(mdns.IP(i));
    Monitor.print(":");
    Monitor.print(mdns.port(i));
    Monitor.println(")");
    
    // Try to connect to this WTIU
    strcpy(wtiu_host, mdns.IP(i).toString().c_str());
    wtiu_port_dynamic = mdns.port(i);
    
    Monitor.print("🔗 Attempting connection to ");
    Monitor.print(wtiu_host);
    Monitor.print(":");
    Monitor.println(wtiu_port_dynamic);
    
    if (connectToWTIU()) {
      Monitor.print("✅ Successfully connected to WTIU: ");
      Monitor.print(wtiu_host);
      Monitor.print(":");
      Monitor.println(wtiu_port_dynamic);
      return true;
    } else {
      Monitor.println("❌ Connection failed, trying next WTIU...");
    }
  }
  
  Monitor.println("❌ Could not connect to any WTIU devices");
  return false;
}

bool connectToWTIU() {
  if (WiFi.status() != WL_CONNECTED) {
    Monitor.println("❌ WiFi not connected - cannot connect to WTIU");
    return false;
  }
  
  Monitor.print("🔗 Connecting to MTH WTIU at ");
  Monitor.print(wtiu_host);
  Monitor.print(":");
  Monitor.println(wtiu_port_dynamic);
  
  if (wtiu_client.connect(wtiu_host, wtiu_port_dynamic)) {
    wtiu_connected = true;
    Monitor.println("✅ Successfully connected to MTH WTIU!");
    Monitor.print("📡 WTIU connection details: ");
    Monitor.print(wtiu_host);
    Monitor.print(":");
    Monitor.println(wtiu_port_dynamic);
    digitalWrite(STATUS_LED, HIGH); // Turn on LED when connected
    return true;
  } else {
    wtiu_connected = false;
    Monitor.println("❌ Failed to connect to MTH WTIU");
    Monitor.print("⚠️ Connection error details: ");
    Monitor.print(wtiu_host);
    Monitor.print(":");
    Monitor.println(wtiu_port_dynamic);
    digitalWrite(STATUS_LED, LOW);
    return false;
  }
}

void maintainWTIUConnection() {
  // Update mDNS regularly
  mdns.update();
  
  // Try to reconnect if connection is lost
  if (!wtiu_connected || !wtiu_client.connected()) {
    wtiu_connected = false;
    digitalWrite(STATUS_LED, LOW);
    
    unsigned long current_time = millis();
    if (current_time - last_connection_attempt >= CONNECTION_RETRY_INTERVAL) {
      Monitor.println("Attempting to rediscover and reconnect to MTH WTIU...");
      last_connection_attempt = current_time;
      
      // Rediscover WTIU
      if (!discoverWTIU()) {
        Monitor.println("Failed to rediscover WTIU, will retry later");
      }
    }
  }
}

void checkSerialCommands() {
  // Check for commands from MPU via Serial1 (internal UART)
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("CMD:")) {
      // Parse command format: "CMD:type:value"
      int firstColon = command.indexOf(':');
      int secondColon = command.indexOf(':', firstColon + 1);
      
      if (firstColon > 0 && secondColon > firstColon) {
        int cmd_type = command.substring(firstColon + 1, secondColon).toInt();
        int cmd_value = command.substring(secondColon + 1).toInt();
        
        // Create command packet
        CommandPacket cmd;
        cmd.command_type = cmd_type;
        cmd.engine_number = 1; // Default engine number
        cmd.value = cmd_value;
        cmd.bool_value = (cmd_value > 0);
        
        Monitor.print("Received Serial1 command: type=");
        Monitor.print(cmd.command_type);
        Monitor.print(", value=");
        Monitor.println(cmd.value);
        
        executeMTHCommand(&cmd);
      }
    }
  }
}

void receiveCommandFromMPU() {
  static uint8_t buffer[sizeof(CommandPacket)];
  static size_t bytes_received = 0;
  
  while (Serial.available() && bytes_received < sizeof(CommandPacket)) {
    buffer[bytes_received++] = Serial.read();
  }
  
  if (bytes_received == sizeof(CommandPacket)) {
    CommandPacket* cmd = (CommandPacket*)buffer;
    executeMTHCommand(cmd);
    bytes_received = 0;
  }
}

void executeMTHCommand(CommandPacket* cmd) {
  char mth_cmd[32];
  bool command_sent = false;
  
  // Send to Monitor for debugging
  Monitor.print("Executing command: type=");
  Monitor.print(cmd->command_type);
  Monitor.print(", engine=");
  Monitor.print(cmd->engine_number);
  Monitor.print(", value=");
  Monitor.print(cmd->value);
  Monitor.print(", bool=");
  Monitor.println(cmd->bool_value);
  
  switch (cmd->command_type) {
    case CMD_ENGINE_SELECT:
      // MTH uses engine numbers 1-99 (direct mapping, no offset)
      // Map Lionel engine 1-99 to MTH engine 1-99
      snprintf(mth_cmd, sizeof(mth_cmd), "y%d", cmd->engine_number);
      sendMTHCommand(mth_cmd);
      command_sent = true;
      break;
      
    case CMD_SPEED:
      snprintf(mth_cmd, sizeof(mth_cmd), "s%d", cmd->value);
      sendMTHCommand(mth_cmd);
      command_sent = true;
      break;
      
    case CMD_DIRECTION:
      if (cmd->bool_value) {
        sendMTHCommand("d1"); // Reverse
      } else {
        sendMTHCommand("d0"); // Forward
      }
      command_sent = true;
      break;
      
    case CMD_BELL:
      if (cmd->bool_value) {
        sendMTHCommand("w4"); // Bell on
      } else {
        sendMTHCommand("bFFFB"); // Bell off
      }
      command_sent = true;
      break;
      
    case CMD_WHISTLE:
      // Regular whistle - only works if protowhistle is disabled
      if (!protowhistle_enabled) {
        if (cmd->bool_value) {
          sendMTHCommand("w2"); // Whistle on
        } else {
          sendMTHCommand("bFFFD"); // Whistle off
        }
        command_sent = true;
      } else {
        Monitor.println("Regular whistle ignored - ProtoWhistle is enabled");
      }
      break;
      
    case CMD_PROTOWHISTLE:
      // ProtoWhistle control
      if (cmd->value == 0) {
        // ProtoWhistle on/off
        if (cmd->bool_value) {
          sendMTHCommand("ab20"); // Enable protowhistle
          protowhistle_enabled = true;
          Monitor.println("ProtoWhistle ENABLED");
        } else {
          sendMTHCommand("ab21"); // Disable protowhistle
          protowhistle_enabled = false;
          Monitor.println("ProtoWhistle DISABLED");
        }
        command_sent = true;
      } else if (cmd->value == 1) {
        // ProtoWhistle quill (actual whistle sound)
        if (protowhistle_enabled) {
          if (cmd->bool_value) {
            sendMTHCommand("w2"); // Quill the whistle
            Monitor.println("ProtoWhistle QUILL ON");
          } else {
            sendMTHCommand("bFFFD"); // Stop quilling
            Monitor.println("ProtoWhistle QUILL OFF");
          }
          command_sent = true;
        }
      } else if (cmd->value == 2) {
        // Toggle protowhistle state
        protowhistle_enabled = cmd->bool_value;
        if (protowhistle_enabled) {
          sendMTHCommand("ab20"); // Enable protowhistle
          Monitor.println("ProtoWhistle TOGGLED ON");
        } else {
          sendMTHCommand("ab21"); // Disable protowhistle
          Monitor.println("ProtoWhistle TOGGLED OFF");
        }
        command_sent = true;
      } else if (cmd->value >= 10 && cmd->value <= 13) {
        // ProtoWhistle pitch control (0-3 mapped to 10-13)
        protowhistle_pitch = cmd->value - 10;
        char pitch_cmd[10];
        snprintf(pitch_cmd, sizeof(pitch_cmd), "ab%d", protowhistle_pitch + 26);
        sendMTHCommand(pitch_cmd);
        Monitor.print("ProtoWhistle pitch set to ");
        Monitor.println(protowhistle_pitch);
        command_sent = true;
      }
      break;
      
    case CMD_WLED:
      // WLED control commands
      if (cmd->bool_value) {
        snprintf(mth_cmd, sizeof(mth_cmd), "w%d", cmd->engine_number);
        sendMTHCommand(mth_cmd);
        command_sent = true;
      } else {
        // WLED off - no MTH command needed, just log
        Monitor.print("WLED Engine ");
        Monitor.print(cmd->engine_number);
        Monitor.println(" OFF");
        command_sent = true;
      }
      break;
      
    case CMD_STARTUP:
      sendMTHCommand("u4"); // Startup
      command_sent = true;
      break;
      
    case CMD_SHUTDOWN:
      sendMTHCommand("u5"); // Shutdown
      command_sent = true;
      break;
      
    default:
      Monitor.print("Unknown command type: ");
      Monitor.println(cmd->command_type);
      break;
  }
  
  // Send response back to MPU
  CommandPacket response;
  response.command_type = cmd->command_type;
  response.engine_number = cmd->engine_number;
  response.value = cmd->value;
  response.bool_value = cmd->bool_value;
  
  // Send response via Serial1 to MPU
  Serial1.println("ACK");
  
  if (command_sent) {
    Monitor.print("✅ Command executed: ");
    Monitor.println(mth_cmd);
  } else {
    Monitor.println("❌ Command not executed");
  }
}

void sendMTHCommand(const char* cmd) {
  if (!wtiu_connected || !wtiu_client.connected()) {
    Monitor.print("Cannot send command - WTIU not connected: ");
    Monitor.println(cmd);
    Serial.println(cmd);
    return;
  }
  
  Serial.print("Sending to WTIU: ");
  Serial.print(cmd);
  
  if (encryption_enabled) {
    // Encrypt the command using Speck
    size_t cmd_len = strlen(cmd);
    
    // Pad to multiple of 4 bytes (2 words)
    size_t padded_len = ((cmd_len + 3) / 4) * 4;
    uint8_t* padded_cmd = (uint8_t*)malloc(padded_len);
    
    // Check for malloc failure
    if (!padded_cmd) {
      Serial.println("ERROR: Memory allocation failed for encryption");
      return;
    }
    
    memset(padded_cmd, 0, padded_len);
    memcpy(padded_cmd, cmd, cmd_len);
    
    // Encrypt in 4-byte chunks
    for (size_t i = 0; i < padded_len; i += 4) {
      SPECK_TYPE pt[2], ct[2];
      pt[0] = (padded_cmd[i] << 8) | padded_cmd[i+1];
      pt[1] = (padded_cmd[i+2] << 8) | padded_cmd[i+3];
      
      speck_encrypt(pt, ct, round_keys);
      
      // Send encrypted bytes
      wtiu_client.write((uint8_t)(ct[0] >> 8));
      wtiu_client.write((uint8_t)(ct[0] & 0xFF));
      wtiu_client.write((uint8_t)(ct[1] >> 8));
      wtiu_client.write((uint8_t)(ct[1] & 0xFF));
    }
    
    free(padded_cmd);
    Serial.println(" (encrypted)");
  } else {
    // Send plain text
    wtiu_client.printf("%s\r\n", cmd);
    Serial.println(" (plain text)");
  }
  
  wtiu_client.flush();
  
  // Brief delay to allow command processing
  delay(50);
}

void printStatus() {
  Serial.println("=== MTH WTIU Handler Status ===");
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  
  Serial.print("WTIU Connection: ");
  Serial.println(wtiu_connected ? "Connected" : "Disconnected");
  
  if (wtiu_connected) {
    Serial.print("WTIU Address: ");
    Serial.print(wtiu_host);
    Serial.print(":");
    Serial.println(wtiu_port);
  }
  
  Serial.print("ProtoWhistle: ");
  Serial.println(protowhistle_enabled ? "Enabled" : "Disabled");
  
  Serial.println("==============================");
}
