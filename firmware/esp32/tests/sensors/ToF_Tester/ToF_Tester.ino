#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // Wait for Serial Monitor
  while (!Serial) {
    delay(10);
  }

  Serial.println("VL53L0X Test");

  // Initialize I2C
  Wire.begin(33, 16);   // SDA, SCL

  // Initialize sensor
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1) {
      delay(10);
    }
  }

  Serial.println("Sensor initialized!");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {   // Valid measurement
    Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }

  delay(100);
}