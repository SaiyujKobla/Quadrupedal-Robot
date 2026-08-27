#include "IMUOrientation.h"


IMUOrientation::IMUOrientation(
  int sda,
  int scl
)
  : lsm() {

  sdaPin = sda;
  sclPin = scl;

  accelXOffset = -0.11622900f;
  accelYOffset = 0.08589700f;
  accelZOffset = 1.30524301f;

  accelXScale = 0.99895497f;
  accelYScale = 0.99418946f;
  accelZScale = 0.99225240f;

  gyroXOffset = 0.00766f;
  gyroYOffset = 0.01859f;
  gyroZOffset = 0.00223f;

  rawAccelX = 0.0f;
  rawAccelY = 0.0f;
  rawAccelZ = 0.0f;

  calibratedAccelX = 0.0f;
  calibratedAccelY = 0.0f;
  calibratedAccelZ = 0.0f;

  rawGyroX = 0.0f;
  rawGyroY = 0.0f;
  rawGyroZ = 0.0f;

  calibratedGyroX = 0.0f;
  calibratedGyroY = 0.0f;
  calibratedGyroZ = 0.0f;

  accelRoll = 0.0f;
  accelPitch = 0.0f;

  roll = 0.0f;
  pitch = 0.0f;
  yaw = 0.0f;

  rollRate = 0.0f;
  pitchRate = 0.0f;
  yawRate = 0.0f;

  accelerationMagnitude = 0.0f;

  complementaryAlpha = 0.98f;

  previousTimeMicros = 0;

  firstUpdate = true;
}


bool IMUOrientation::begin() {

  Wire.begin(
    sdaPin,
    sclPin
  );

  Wire.setClock(
    400000
  );

  if (!lsm.begin()) {
    return false;
  }

  lsm.setupAccel(
    lsm.LSM9DS1_ACCELRANGE_2G
  );

  lsm.setupGyro(
    lsm.LSM9DS1_GYROSCALE_245DPS
  );

  lsm.setupMag(
    lsm.LSM9DS1_MAGGAIN_4GAUSS
  );

  delay(100);

  firstUpdate = true;

  previousTimeMicros =
    micros();

  return true;
}


float IMUOrientation::normalizeAngle(
  float angle
) const {

  while (angle > 180.0f) {
    angle -= 360.0f;
  }

  while (angle < -180.0f) {
    angle += 360.0f;
  }

  return angle;
}


float IMUOrientation::shortestAngleDifference(
  float target,
  float current
) const {

  return normalizeAngle(
    target - current
  );
}


bool IMUOrientation::update() {

  sensors_event_t accelEvent;
  sensors_event_t magEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  lsm.getEvent(
    &accelEvent,
    &magEvent,
    &gyroEvent,
    &tempEvent
  );


  unsigned long currentTimeMicros =
    micros();

  float dt =
    (
      currentTimeMicros -
      previousTimeMicros
    ) /
    1000000.0f;

  previousTimeMicros =
    currentTimeMicros;


  rawAccelX =
    accelEvent.acceleration.x;

  rawAccelY =
    accelEvent.acceleration.y;

  rawAccelZ =
    accelEvent.acceleration.z;


  calibratedAccelX =
    (
      rawAccelX -
      accelXOffset
    ) *
    accelXScale;

  calibratedAccelY =
    (
      rawAccelY -
      accelYOffset
    ) *
    accelYScale;

  calibratedAccelZ =
    (
      rawAccelZ -
      accelZOffset
    ) *
    accelZScale;


  rawGyroX =
    gyroEvent.gyro.x;

  rawGyroY =
    gyroEvent.gyro.y;

  rawGyroZ =
    gyroEvent.gyro.z;


  calibratedGyroX =
    rawGyroX -
    gyroXOffset;

  calibratedGyroY =
    rawGyroY -
    gyroYOffset;

  calibratedGyroZ =
    rawGyroZ -
    gyroZOffset;


  rollRate =
    -calibratedGyroX *
    RAD_TO_DEG;

  pitchRate =
    -calibratedGyroY *
    RAD_TO_DEG;

  yawRate =
    calibratedGyroZ *
    RAD_TO_DEG;


  accelRoll =
    atan2(
      calibratedAccelY,
      calibratedAccelZ
    ) *
    RAD_TO_DEG;


  accelPitch =
    atan2(
      -calibratedAccelX,
      sqrt(
        calibratedAccelY *
        calibratedAccelY +

        calibratedAccelZ *
        calibratedAccelZ
      )
    ) *
    RAD_TO_DEG;


  accelerationMagnitude =
    sqrt(
      calibratedAccelX *
      calibratedAccelX +

      calibratedAccelY *
      calibratedAccelY +

      calibratedAccelZ *
      calibratedAccelZ
    );


  if (
    !isfinite(accelRoll) ||
    !isfinite(accelPitch) ||
    !isfinite(rollRate) ||
    !isfinite(pitchRate) ||
    !isfinite(yawRate) ||
    !isfinite(accelerationMagnitude)
  ) {
    return false;
  }


  if (firstUpdate) {

    roll =
      accelRoll;

    pitch =
      accelPitch;

    yaw =
      0.0f;

    firstUpdate =
      false;

    return true;
  }


  if (
    dt <= 0.0f ||
    dt > 0.1f
  ) {

    roll =
      accelRoll;

    pitch =
      accelPitch;

    return false;
  }


  float gyroRoll =
    normalizeAngle(
      roll +
      rollRate *
      dt
    );


  float gyroPitch =
    normalizeAngle(
      pitch +
      pitchRate *
      dt
    );


  float rollError =
    shortestAngleDifference(
      accelRoll,
      gyroRoll
    );


  float pitchError =
    shortestAngleDifference(
      accelPitch,
      gyroPitch
    );


  roll =
    normalizeAngle(
      gyroRoll +
      (
        1.0f -
        complementaryAlpha
      ) *
      rollError
    );


  pitch =
    normalizeAngle(
      gyroPitch +
      (
        1.0f -
        complementaryAlpha
      ) *
      pitchError
    );


  yaw +=
    yawRate *
    dt;

  yaw =
    normalizeAngle(
      yaw
    );


  if (
    !isfinite(roll) ||
    !isfinite(pitch) ||
    !isfinite(yaw)
  ) {

    roll =
      accelRoll;

    pitch =
      accelPitch;

    yaw =
      0.0f;

    return false;
  }


  return true;
}


void IMUOrientation::setAccelerometerCalibration(
  float xOffset,
  float yOffset,
  float zOffset,
  float xScale,
  float yScale,
  float zScale
) {

  accelXOffset =
    xOffset;

  accelYOffset =
    yOffset;

  accelZOffset =
    zOffset;

  accelXScale =
    xScale;

  accelYScale =
    yScale;

  accelZScale =
    zScale;
}


void IMUOrientation::setGyroscopeCalibration(
  float xOffset,
  float yOffset,
  float zOffset
) {

  gyroXOffset =
    xOffset;

  gyroYOffset =
    yOffset;

  gyroZOffset =
    zOffset;
}


void IMUOrientation::setComplementaryAlpha(
  float alpha
) {

  complementaryAlpha =
    constrain(
      alpha,
      0.0f,
      1.0f
    );
}


void IMUOrientation::resetOrientation() {

  roll =
    0.0f;

  pitch =
    0.0f;

  yaw =
    0.0f;

  firstUpdate =
    true;

  previousTimeMicros =
    micros();
}


void IMUOrientation::resetYaw() {

  yaw =
    0.0f;
}


float IMUOrientation::getRoll() const {
  return roll;
}


float IMUOrientation::getPitch() const {
  return pitch;
}


float IMUOrientation::getYaw() const {
  return yaw;
}


float IMUOrientation::getAccelRoll() const {
  return accelRoll;
}


float IMUOrientation::getAccelPitch() const {
  return accelPitch;
}


float IMUOrientation::getRollRate() const {
  return rollRate;
}


float IMUOrientation::getPitchRate() const {
  return pitchRate;
}


float IMUOrientation::getYawRate() const {
  return yawRate;
}


float IMUOrientation::getRawAccelX() const {
  return rawAccelX;
}


float IMUOrientation::getRawAccelY() const {
  return rawAccelY;
}


float IMUOrientation::getRawAccelZ() const {
  return rawAccelZ;
}


float IMUOrientation::getAccelX() const {
  return calibratedAccelX;
}


float IMUOrientation::getAccelY() const {
  return calibratedAccelY;
}


float IMUOrientation::getAccelZ() const {
  return calibratedAccelZ;
}


float IMUOrientation::getRawGyroX() const {
  return rawGyroX;
}


float IMUOrientation::getRawGyroY() const {
  return rawGyroY;
}


float IMUOrientation::getRawGyroZ() const {
  return rawGyroZ;
}


float IMUOrientation::getGyroX() const {
  return calibratedGyroX;
}


float IMUOrientation::getGyroY() const {
  return calibratedGyroY;
}


float IMUOrientation::getGyroZ() const {
  return calibratedGyroZ;
}


float IMUOrientation::getAccelerationMagnitude() const {
  return accelerationMagnitude;
}