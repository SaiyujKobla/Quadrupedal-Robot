#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define SDA_PIN 33
#define SCL_PIN 16

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// OLD VALUES
// const float ACCEL_X_OFFSET = -0.10205126f;
// const float ACCEL_Y_OFFSET = 0.05219603f;
// const float ACCEL_Z_OFFSET = 3.28486419f;

// const float ACCEL_X_SCALE = 0.99835628f;
// const float ACCEL_Y_SCALE = 0.99660498f;
// const float ACCEL_Z_SCALE = 0.99073857f;

// const float GYRO_X_OFFSET = 0.02374484f;
// const float GYRO_Y_OFFSET = 0.02552957f;
// const float GYRO_Z_OFFSET = 0.01015970f;

float yaw = 0.0f;

unsigned long previousTimeUs = 0;
unsigned long previousPrintTimeMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!lsm.begin()) {
    Serial.println("ERROR");

    while (true) {
      delay(100);
    }
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  delay(500);

  previousTimeUs = micros();

  Serial.println(
    "roll,pitch,yaw,ax,ay,az,roll_rate,pitch_rate,yaw_rate"
  );
}

void loop() {
  sensors_event_t accel;
  sensors_event_t mag;
  sensors_event_t gyro;
  sensors_event_t temp;

  lsm.getEvent(
    &accel,
    &mag,
    &gyro,
    &temp
  );

  unsigned long currentTimeUs = micros();

  float dt =
    (currentTimeUs - previousTimeUs) /
    1000000.0f;

  previousTimeUs = currentTimeUs;

  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  float ax =
    (accel.acceleration.x - ACCEL_X_OFFSET) *
    ACCEL_X_SCALE;

  float ay =
    (accel.acceleration.y - ACCEL_Y_OFFSET) *
    ACCEL_Y_SCALE;

  float az =
    (accel.acceleration.z - ACCEL_Z_OFFSET) *
    ACCEL_Z_SCALE;

  float gyroX =
    gyro.gyro.x - GYRO_X_OFFSET;

  float gyroY =
    gyro.gyro.y - GYRO_Y_OFFSET;

  float gyroZ =
    gyro.gyro.z - GYRO_Z_OFFSET;

  float rollRate =
    gyroX * RAD_TO_DEG;

  float pitchRate =
    gyroY * RAD_TO_DEG;

  float yawRate =
    gyroZ * RAD_TO_DEG;

  float roll =
    atan2(
      ay,
      az
    ) * RAD_TO_DEG;

  float pitch =
    atan2(
      -ax,
      sqrt(
        ay * ay +
        az * az
      )
    ) * RAD_TO_DEG;

  yaw += yawRate * dt;

  if (
    millis() - previousPrintTimeMs >= 20
  ) {
    previousPrintTimeMs = millis();

    Serial.print(roll, 4);
    Serial.print(",");

    Serial.print(pitch, 4);
    Serial.print(",");

    Serial.print(yaw, 4);
    Serial.print(",");

    Serial.print(ax, 4);
    Serial.print(",");

    Serial.print(ay, 4);
    Serial.print(",");

    Serial.print(az, 4);
    Serial.print(",");

    Serial.print(rollRate, 4);
    Serial.print(",");

    Serial.print(pitchRate, 4);
    Serial.print(",");

    Serial.println(yawRate, 4);
  }
}