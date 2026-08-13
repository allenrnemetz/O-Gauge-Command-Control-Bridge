/*
  VL53L1X Single Sensor Test (Pololu library)
  ===========================================
*/

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== VL53L1X Test ===");

  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  delay(50);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  sensor.setTimeout(500);
  Serial.println("Calling sensor.init()...");
  if (!sensor.init()) {
    Serial.println("init() FAILED");
    while (1) delay(2000);
  }

  Serial.println("init() OK!");

  // Short distance mode is better for close-range train detection
  sensor.setDistanceMode(VL53L1X::Short);
  // 20ms timing budget, 50ms between measurements
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(50);

  Serial.println("Reading distance... (mm)");
}

void loop() {
  sensor.read();  // blocks until new data available
  Serial.print(sensor.ranging_data.range_mm);
  Serial.println(" mm");
  delay(50);
}
