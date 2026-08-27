#ifndef IMU_ORIENTATION_H
#define IMU_ORIENTATION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>

class IMUOrientation {
private:
  Adafruit_LSM9DS1 lsm;

  int sdaPin;
  int sclPin;

  float accelXOffset;
  float accelYOffset;
  float accelZOffset;

  float accelXScale;
  float accelYScale;
  float accelZScale;

  float gyroXOffset;
  float gyroYOffset;
  float gyroZOffset;

  float rawAccelX;
  float rawAccelY;
  float rawAccelZ;

  float calibratedAccelX;
  float calibratedAccelY;
  float calibratedAccelZ;

  float rawGyroX;
  float rawGyroY;
  float rawGyroZ;

  float calibratedGyroX;
  float calibratedGyroY;
  float calibratedGyroZ;

  float accelRoll;
  float accelPitch;

  float roll;
  float pitch;
  float yaw;

  float rollRate;
  float pitchRate;
  float yawRate;

  float accelerationMagnitude;

  float complementaryAlpha;

  unsigned long previousTimeMicros;

  bool firstUpdate;

  float normalizeAngle(
    float angle
  ) const;

  float shortestAngleDifference(
    float target,
    float current
  ) const;

public:
  IMUOrientation(
    int sda,
    int scl
  );

  bool begin();

  bool update();

  void setAccelerometerCalibration(
    float xOffset,
    float yOffset,
    float zOffset,
    float xScale,
    float yScale,
    float zScale
  );

  void setGyroscopeCalibration(
    float xOffset,
    float yOffset,
    float zOffset
  );

  void setComplementaryAlpha(
    float alpha
  );

  void resetOrientation();

  void resetYaw();

  float getRoll() const;
  float getPitch() const;
  float getYaw() const;

  float getAccelRoll() const;
  float getAccelPitch() const;

  float getRollRate() const;
  float getPitchRate() const;
  float getYawRate() const;

  float getRawAccelX() const;
  float getRawAccelY() const;
  float getRawAccelZ() const;

  float getAccelX() const;
  float getAccelY() const;
  float getAccelZ() const;

  float getRawGyroX() const;
  float getRawGyroY() const;
  float getRawGyroZ() const;

  float getGyroX() const;
  float getGyroY() const;
  float getGyroZ() const;

  float getAccelerationMagnitude() const;
};

#endif