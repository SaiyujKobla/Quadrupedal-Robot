#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 33
#define SCL_PIN 16

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

//==================================================
// Calibration Constants
//==================================================
// OLD VALUES
// const float ACCEL_X_OFFSET = -0.10205126;
// const float ACCEL_Y_OFFSET =  0.05219603;
// const float ACCEL_Z_OFFSET =  3.28486419;

// const float ACCEL_X_SCALE  = 0.99835628;
// const float ACCEL_Y_SCALE  = 0.99660498;
// const float ACCEL_Z_SCALE  = 0.99073857;

// const float GYRO_X_OFFSET = 0.02374484;
// const float GYRO_Y_OFFSET = 0.02552957;
// const float GYRO_Z_OFFSET = 0.01015970;

//==================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("LSM9DS1 Calibration Test");
  Serial.println("======================================");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lsm.begin()) {
    Serial.println("ERROR: Could not find LSM9DS1!");
    while (1) {
      delay(100);
    }
  }

  Serial.println("LSM9DS1 Found!");

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
}

void loop() {

  sensors_event_t accel;
  sensors_event_t mag;
  sensors_event_t gyro;
  sensors_event_t temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);

  //------------------------------------------------
  // Raw values
  //------------------------------------------------

  float rawAx = accel.acceleration.x;
  float rawAy = accel.acceleration.y;
  float rawAz = accel.acceleration.z;

  float rawGx = gyro.gyro.x;
  float rawGy = gyro.gyro.y;
  float rawGz = gyro.gyro.z;

  //------------------------------------------------
  // Corrected values
  //------------------------------------------------

  float ax = (rawAx - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  float ay = (rawAy - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  float az = (rawAz - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

  float gx = rawGx - GYRO_X_OFFSET;
  float gy = rawGy - GYRO_Y_OFFSET;
  float gz = rawGz - GYRO_Z_OFFSET;

  //------------------------------------------------
  // Acceleration magnitude
  //------------------------------------------------

  float accelMagnitude =
      sqrt(ax * ax +
           ay * ay +
           az * az);

  //------------------------------------------------
  // Roll / Pitch from accelerometer
  //------------------------------------------------

  float roll =
      atan2(ay, az) * 180.0 / PI;

  float pitch =
      atan2(-ax,
            sqrt(ay * ay + az * az))
      * 180.0 / PI;

  //------------------------------------------------
  // Print everything
  //------------------------------------------------

  Serial.println();
  Serial.println("======================================");

  Serial.println("RAW ACCEL (m/s^2)");
  Serial.print("X: ");
  Serial.print(rawAx, 3);
  Serial.print("   Y: ");
  Serial.print(rawAy, 3);
  Serial.print("   Z: ");
  Serial.println(rawAz, 3);

  Serial.println();

  Serial.println("CALIBRATED ACCEL (m/s^2)");
  Serial.print("X: ");
  Serial.print(ax, 3);
  Serial.print("   Y: ");
  Serial.print(ay, 3);
  Serial.print("   Z: ");
  Serial.println(az, 3);

  Serial.print("Magnitude: ");
  Serial.print(accelMagnitude, 3);
  Serial.println(" m/s^2");

  Serial.println();

  Serial.println("RAW GYRO (rad/s)");
  Serial.print("X: ");
  Serial.print(rawGx, 4);
  Serial.print("   Y: ");
  Serial.print(rawGy, 4);
  Serial.print("   Z: ");
  Serial.println(rawGz, 4);

  Serial.println();

  Serial.println("CALIBRATED GYRO (rad/s)");
  Serial.print("X: ");
  Serial.print(gx, 4);
  Serial.print("   Y: ");
  Serial.print(gy, 4);
  Serial.print("   Z: ");
  Serial.println(gz, 4);

  Serial.println();

  Serial.println("MAGNETOMETER (uT)");
  Serial.print("X: ");
  Serial.print(mag.magnetic.x, 2);
  Serial.print("   Y: ");
  Serial.print(mag.magnetic.y, 2);
  Serial.print("   Z: ");
  Serial.println(mag.magnetic.z, 2);

  Serial.println();

  Serial.println("ORIENTATION");

  Serial.print("Roll : ");
  Serial.print(roll, 2);
  Serial.println(" deg");

  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.println(" deg");

  Serial.println("======================================");

  delay(250);
}