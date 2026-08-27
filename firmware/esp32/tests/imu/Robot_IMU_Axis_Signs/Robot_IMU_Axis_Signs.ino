#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 33
#define SCL_PIN 16

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!lsm.begin()) {
    Serial.println("Could not find LSM9DS1.");
    while (1) {
      delay(100);
    }
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  Serial.println("Move the robot slowly into different orientations.");
  Serial.println("Whichever axis is close to +9.8 is pointing UP.");
  Serial.println("Whichever axis is close to -9.8 has its NEGATIVE direction pointing UP.");
}

void loop() {
  sensors_event_t accel;
  sensors_event_t mag;
  sensors_event_t gyro;
  sensors_event_t temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);

  Serial.print("X: ");
  Serial.print(accel.acceleration.x, 3);

  Serial.print("   Y: ");
  Serial.print(accel.acceleration.y, 3);

  Serial.print("   Z: ");
  Serial.println(accel.acceleration.z, 3);

  delay(250);
}