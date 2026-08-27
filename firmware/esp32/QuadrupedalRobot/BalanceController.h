#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>

struct FootTarget {
  float x;
  float y;
  float z;
};

struct BalanceOutput {
  FootTarget frontLeft;
  FootTarget frontRight;
  FootTarget backLeft;
  FootTarget backRight;

  float rollError;
  float pitchError;

  float rollCorrectionAngle;
  float pitchCorrectionAngle;

  bool safetyStopped;
};

class BalanceController {
private:

  float bodyLength;
  float bodyWidth;

  float rotationCenterOffsetX;
  float rotationCenterOffsetZ;

  float targetRoll;
  float targetPitch;

  float rollKp;
  float rollKd;

  float pitchKp;
  float pitchKd;

  float rollCorrectionSign;
  float pitchCorrectionSign;

  float maximumRollCorrectionAngle;
  float maximumPitchCorrectionAngle;

  float maximumSafeAngle;

  float angleDeadband;
  float outputFilterAlpha;

  float filteredRollCorrectionAngle;
  float filteredPitchCorrectionAngle;

  bool enabled;

  float normalizeAngle(float angle) const;

  float applyDeadband(float value) const;

  float clampValue(float value, float minimum, float maximum) const;

  FootTarget calculateCorrectedFootTarget(
    const FootTarget &nominalTarget,

    float hipX,
    float hipZ,

    bool isLeftLeg,

    float rollAngleDegrees,
    float pitchAngleDegrees
  ) const;

public:

  BalanceController(
    float initialBodyLength,
    float initialBodyWidth,

    float initialRollKp,
    float initialRollKd,

    float initialPitchKp,
    float initialPitchKd
  );

  BalanceOutput update(
    float measuredRoll,
    float measuredPitch,

    float measuredRollRate,
    float measuredPitchRate,

    const FootTarget &nominalFrontLeft,
    const FootTarget &nominalFrontRight,
    const FootTarget &nominalBackLeft,
    const FootTarget &nominalBackRight
  );

  void reset();

  void setEnabled(bool state);

  bool isEnabled() const;

  void setGeometry(float newBodyLength, float newBodyWidth);

  void setRotationCenterOffset(float offsetX, float offsetZ);

  void setTargetAngles(float rollDegrees, float pitchDegrees);

  void setCorrectionSigns(float rollSign, float pitchSign);

  void setRollGains(float kp, float kd);

  void setPitchGains(float kp, float kd);

  void setMaximumCorrectionAngles(float maximumRollDegrees, float maximumPitchDegrees);

  void setMaximumSafeAngle(float angleDegrees);

  void setDeadband(float deadbandDegrees);

  void setOutputFilterAlpha(float alpha);
};

#endif