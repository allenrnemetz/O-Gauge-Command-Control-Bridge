/*
  ESP32 Dual ToF Speed Trap for O-Gauge Speed Calibration (VL53L1X)
  ==================================================================

  Two VL53L1X time-of-flight sensors, spaced a fixed distance apart along
  the track. When a train passes through, the firmware records the time
  between each sensor triggering and sends it over WiFi UDP to the bridge
  PC. The bridge computes speed = D / dt and builds the calibration curve.

  HARDWARE
  --------
  - ELEGOO ESP32 dev board (or any standard ESP32)
  - 2x VL53L1X breakout boards

  WIRING
  ------
  Both sensors share the ESP32 I2C bus. Sensor B's I2C address is changed
  at startup via its XSHUT pin, so both can coexist on one bus.

    ESP32 GPIO21 (SDA)  -> SDA on both sensors
    ESP32 GPIO22 (SCL)  -> SCL on both sensors
    ESP32 GPIO18        -> XSHUT on sensor A (left)
    ESP32 GPIO19        -> XSHUT on sensor B (right)
    3V3                 -> VCC on both sensors
    GND                 -> GND on both sensors

  Mount both sensors on a bridge over straight, level track, facing down.
  Same height, same angle. Spacing is fixed and must not change between
  calibration runs. The actual distance value is derived at runtime from
  an MTH engine at a known sMPH (see calibration docs).

  UDP OUTPUT (JSON, one packet per train pass)
  --------------------------------------------
  {"event":"pass","dir":"fwd","t1_ms":1234567,"t2_ms":1234580,"dt_ms":13.0,"a_mm":82,"b_mm":85}
    dir:   "fwd" = sensor A triggered first, "rev" = sensor B first
    t1_ms: millis() when first sensor triggered
    t2_ms: millis() when second sensor triggered
    dt_ms: t2 - t1 (the crossing time the bridge uses for speed)
    a_mm:  distance reading on sensor A at its trigger moment
    b_mm:  distance reading on sensor B at its trigger moment

  {"event":"ready","ip":"192.168.1.123","a_base_mm":285,"b_base_mm":283,"thresh_mm":225}
    Sent once at startup after WiFi connects and thresholds auto-calibrate.

  {"event":"heartbeat","uptime_s":60,"a_mm":285,"b_mm":283}
    Sent every HEARTBEAT_INTERVAL_MS so the bridge knows the rig is alive.

  {"event":"error","msg":"sensor B init failed"}
    Sent on fatal errors.

  LIBRARY REQUIRED
  ----------------
  Install "VL53L1X" by Pololu via Arduino Library Manager.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <VL53L1X.h>

// ==================== CONFIGURATION ====================

// WiFi — same network as the bridge PC.
const char* WIFI_SSIDS[] = {"Nemetz-trainroom", "Nemetz"};
const char* WIFI_PASSWORDS[] = {"crawdad2", "crawdad2"};
const int WIFI_NUM_NETWORKS = 2;

// UDP destination — broadcast to all on the network
const char* BRIDGE_IP = "255.255.255.255";
const int   UDP_PORT  = 7777;

// I2C pins (ESP32 default)
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// XSHUT pins — used to re-address sensor B at startup
const int XSHUT_A = 18;
const int XSHUT_B = 19;

// Sensor B's new I2C address (sensor A stays at default 0x29)
const uint8_t ADDR_B = 0x2B;

// VL53L1X timing. Short distance mode for close-range train detection.
// 20ms timing budget is the fastest practical for VL53L1X.
const uint32_t TIMING_BUDGET_US = 20000;
const uint32_t INTER_MEASUREMENT_MS = 30;  // slightly longer than timing budget

// Trigger threshold: auto-calibrated as (open_air_baseline - THRESHOLD_MARGIN_MM).
// Train is "present" when reading drops below this.
const int THRESHOLD_MARGIN_MM = 60;

// After a pass completes, wait this long for both sensors to clear
// before arming for the next pass.
const unsigned long PASS_RESET_MS = 500;

// If only one sensor triggers and the other doesn't follow within
// this window, discard the event.
const unsigned long PASS_TIMEOUT_MS = 5000;

// Heartbeat interval
const unsigned long HEARTBEAT_INTERVAL_MS = 2000;

// WiFi connect timeout
const unsigned long WIFI_TIMEOUT_MS = 30000;

// Max valid reading (mm). VL53L1X can read up to ~4000mm; filter
// anything above this as invalid.
const int MAX_VALID_MM = 2000;

// ==================== GLOBALS ====================

VL53L1X sensorA;
VL53L1X sensorB;

WiFiUDP udp;
IPAddress bridgeAddr;

unsigned long heartbeatTimer = 0;

// Per-sensor state
struct SensorState {
  bool present;          // train currently in beam
  bool triggered;        // trigger recorded for current pass
  unsigned long trigMs;  // millis() at trigger
  int trigMm;            // distance at trigger
  int lastMm;            // most recent reading
};

SensorState sA = { false, false, 0, 0, 0 };
SensorState sB = { false, false, 0, 0, 0 };

// Pass state
enum PassState { ARMED, FIRST_TRIGGERED, COMPLETE, RESETTING };
PassState passState = ARMED;
unsigned long passTimer = 0;

// Auto-calibrated thresholds
int threshA = 0;
int threshB = 0;

// ==================== HELPERS ====================

void sendJson(JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(bridgeAddr, UDP_PORT);
    udp.print(out);
    udp.endPacket();
  }
  Serial.println(out);
}

void sendError(const char* msg) {
  StaticJsonDocument<256> doc;
  doc["event"] = "error";
  doc["msg"] = msg;
  sendJson(doc);
}

void sendHeartbeat() {
  StaticJsonDocument<256> doc;
  doc["event"] = "heartbeat";
  doc["uptime_s"] = (long)(millis() / 1000);
  doc["a_mm"] = sA.lastMm;
  doc["b_mm"] = sB.lastMm;
  sendJson(doc);
}

void sendPass(char dir, unsigned long t1, unsigned long t2,
              int aMm, int bMm) {
  StaticJsonDocument<256> doc;
  doc["event"] = "pass";
  doc["dir"] = (dir == 'f') ? "fwd" : "rev";
  doc["t1_ms"] = (long)t1;
  doc["t2_ms"] = (long)t2;
  doc["dt_ms"] = (double)(t2 - t1);
  doc["a_mm"] = aMm;
  doc["b_mm"] = bMm;
  sendJson(doc);
}

// Poll one sensor. VL53L1X read() blocks until new data is available.
// Returns true if a new trigger edge occurred (train entered beam).
bool pollSensor(VL53L1X& sensor, SensorState& ss, int threshold) {
  sensor.read();
  int mm = sensor.ranging_data.range_mm;

  ss.lastMm = mm;

  // Ignore invalid readings (0 or out-of-range)
  if (mm == 0 || mm > MAX_VALID_MM) return false;

  bool wasPresent = ss.present;
  ss.present = (mm < threshold);

  // Trigger on rising edge of "present" (open air -> train)
  if (ss.present && !wasPresent && !ss.triggered) {
    ss.triggered = true;
    ss.trigMs = millis();
    ss.trigMm = mm;
    return true;
  }

  return false;
}

// Read a single sample from a sensor for calibration (no state update).
// Returns the distance in mm, or -1 if invalid.
int readSample(VL53L1X& sensor) {
  sensor.read();
  int mm = sensor.ranging_data.range_mm;
  if (mm == 0 || mm > MAX_VALID_MM) return -1;
  return mm;
}

void resetPass() {
  sA.triggered = false;
  sB.triggered = false;
  passState = ARMED;
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== O-Gauge Speed Trap (VL53L1X) ===");

  // XSHUT pins — start both LOW (sensors held in shutdown)
  pinMode(XSHUT_A, OUTPUT);
  pinMode(XSHUT_B, OUTPUT);
  digitalWrite(XSHUT_A, LOW);
  digitalWrite(XSHUT_B, LOW);
  delay(10);

  // Init I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // --- Bring up sensor A at default address 0x29 ---
  digitalWrite(XSHUT_A, HIGH);
  delay(10);

  sensorA.setTimeout(500);
  if (!sensorA.init()) {
    sendError("sensor A init failed");
    Serial.println("ERROR: sensor A init failed");
    while (1) delay(1000);
  }
  sensorA.setAddress(0x29);  // explicit (already default)
  sensorA.setDistanceMode(VL53L1X::Short);
  sensorA.setMeasurementTimingBudget(TIMING_BUDGET_US);
  sensorA.startContinuous(INTER_MEASUREMENT_MS);
  Serial.println("Sensor A online (addr 0x29)");

  // --- Bring up sensor B, change address to 0x2B ---
  digitalWrite(XSHUT_B, HIGH);
  delay(10);

  sensorB.setTimeout(500);
  if (!sensorB.init()) {
    sendError("sensor B init failed");
    Serial.println("ERROR: sensor B init failed");
    while (1) delay(1000);
  }
  sensorB.setAddress(ADDR_B);
  sensorB.setDistanceMode(VL53L1X::Short);
  sensorB.setMeasurementTimingBudget(TIMING_BUDGET_US);
  sensorB.startContinuous(INTER_MEASUREMENT_MS);
  Serial.print("Sensor B online (addr 0x");
  Serial.print(ADDR_B, HEX);
  Serial.println(")");

  // --- Auto-calibrate open-air thresholds ---
  Serial.println("Calibrating open-air baseline (keep track clear)...");
  long baseA = 0, baseB = 0;
  const int CAL_SAMPLES = 20;
  int validA = 0, validB = 0;

  // Discard first few readings after startContinuous (warm-up)
  for (int i = 0; i < 5; i++) {
    readSample(sensorA);
    readSample(sensorB);
    delay(30);
  }

  for (int i = 0; i < CAL_SAMPLES; i++) {
    int mmA = readSample(sensorA);
    if (mmA > 0) { baseA += mmA; validA++; }

    int mmB = readSample(sensorB);
    if (mmB > 0) { baseB += mmB; validB++; }

    delay(30);
  }

  if (validA > 0 && validB > 0) {
    baseA /= validA;
    baseB /= validB;
    threshA = baseA - THRESHOLD_MARGIN_MM;
    threshB = baseB - THRESHOLD_MARGIN_MM;
    Serial.printf("Baseline A=%dmm B=%dmm -> thresholds A=%dmm B=%dmm\n",
                  (int)baseA, (int)baseB, threshA, threshB);
  } else {
    threshA = 200;
    threshB = 200;
    baseA = 260;
    baseB = 260;
    Serial.println("WARNING: calibration failed, using default threshold 200mm");
  }

  // --- Connect WiFi (try each known SSID in order) ---
  WiFi.mode(WIFI_STA);
  bool wifiOk = false;

  for (int i = 0; i < WIFI_NUM_NETWORKS && !wifiOk; i++) {
    Serial.printf("Connecting to WiFi '%s'...\n", WIFI_SSIDS[i]);
    WiFi.begin(WIFI_SSIDS[i], WIFI_PASSWORDS[i]);

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_TIMEOUT_MS) {
      delay(200);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiOk = true;
      Serial.printf("\nWiFi connected via '%s'! IP: %s\n",
                    WIFI_SSIDS[i], WiFi.localIP().toString().c_str());
    } else {
      Serial.printf("\n'%s' failed, trying next...\n", WIFI_SSIDS[i]);
      WiFi.disconnect();
      delay(500);
    }
  }

  if (!wifiOk) {
    Serial.println("\nWiFi connect FAILED on all networks — continuing without UDP (serial only)");
    bridgeAddr = IPAddress(0, 0, 0, 0);
  } else {
    if (!bridgeAddr.fromString(BRIDGE_IP)) {
      bridgeAddr = IPAddress(255, 255, 255, 255);
    }
    udp.begin(UDP_PORT);

    // Send ready packet
    StaticJsonDocument<256> doc;
    doc["event"] = "ready";
    doc["ip"] = WiFi.localIP().toString();
    doc["a_base_mm"] = (int)baseA;
    doc["b_base_mm"] = (int)baseB;
    doc["thresh_mm"] = threshA;
    sendJson(doc);
  }

  Serial.println("Speed trap running. Waiting for trains...");
}

// ==================== MAIN LOOP ====================

void loop() {
  // Poll both sensors (sequential — each blocks until new reading)
  bool trigA = pollSensor(sensorA, sA, threshA);
  bool trigB = pollSensor(sensorB, sB, threshB);

  // Pass state machine
  switch (passState) {

    case ARMED:
      if (trigA) {
        passState = FIRST_TRIGGERED;
        passTimer = millis();
        Serial.println("Trigger: A first");
      } else if (trigB) {
        passState = FIRST_TRIGGERED;
        passTimer = millis();
        Serial.println("Trigger: B first");
      }
      break;

    case FIRST_TRIGGERED: {
      bool aFirst = sA.triggered && !sB.triggered;
      bool bFirst = sB.triggered && !sA.triggered;

      if (aFirst && trigB) {
        sendPass('f', sA.trigMs, sB.trigMs, sA.trigMm, sB.trigMm);
        passState = RESETTING;
        passTimer = millis();
      } else if (bFirst && trigA) {
        sendPass('r', sB.trigMs, sA.trigMs, sA.trigMm, sB.trigMm);
        passState = RESETTING;
        passTimer = millis();
      } else if (millis() - passTimer > PASS_TIMEOUT_MS) {
        Serial.println("Pass timeout (single sensor), discarding");
        resetPass();
      }
      break;
    }

    case COMPLETE:
      break;

    case RESETTING:
      if (!sA.present && !sB.present && millis() - passTimer > PASS_RESET_MS) {
        resetPass();
        Serial.println("Re-armed");
      }
      if (millis() - passTimer > PASS_TIMEOUT_MS * 2) {
        resetPass();
        Serial.println("Reset timeout, re-arming");
      }
      break;
  }

  // Heartbeat
  if (millis() - heartbeatTimer > HEARTBEAT_INTERVAL_MS) {
    heartbeatTimer = millis();
    if (WiFi.status() == WL_CONNECTED) {
      sendHeartbeat();
    }
  }

  delay(1);
}
