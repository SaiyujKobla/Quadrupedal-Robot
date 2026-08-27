#include "BalanceController.h"
#include <math.h>

BalanceController::BalanceController(
  float initialBodyLength,
  float initialBodyWidth,

  float initialRollKp,
  float initialRollKd,

  float initialPitchKp,
  float initialPitchKd
) {

  bodyLength = fabs(initialBodyLength);
  bodyWidth = fabs(initialBodyWidth);

  rotationCenterOffsetX = 0.0f;
  rotationCenterOffsetZ = 0.0f;

  targetRoll = 0.0f;
  targetPitch = 0.0f;

  rollKp = initialRollKp;
  rollKd = initialRollKd;
  pitchKp = initialPitchKp;
  pitchKd = initialPitchKd;

  rollCorrectionSign = 1.0f;
  pitchCorrectionSign = 1.0f;

  maximumRollCorrectionAngle = 10.0f;
  maximumPitchCorrectionAngle = 10.0f;
  maximumSafeAngle = 35.0f;
  angleDeadband = 0.2f;
  outputFilterAlpha = 0.75f;

  filteredRollCorrectionAngle = 0.0f;
  filteredPitchCorrectionAngle = 0.0f;

  enabled = false;
}


float BalanceController::normalizeAngle(float angle) const {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }

  while (angle < -180.0f) {
    angle += 360.0f;
  }

  return angle;
}


float BalanceController::applyDeadband(float value) const {
  if (fabs(value) <= angleDeadband) {
    return 0.0f;
  }

  if (value > 0.0f) {
    return value - angleDeadband;
  }

  return value + angleDeadband;
}


float BalanceController::clampValue(float value, float minimum, float maximum) const {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}


FootTarget BalanceController::calculateCorrectedFootTarget(
  const FootTarget &nominalTarget,

  float hipX,
  float hipZ,

  bool isLeftLeg,

  float rollAngleDegrees,
  float pitchAngleDegrees
) const {

  FootTarget output;

  /*
    BODY COORDINATE SYSTEM

      +X = forward
      +Y = downward
      +Z = robot's right


    LEG-LOCAL COORDINATE SYSTEM

      +X = backward
      +Y = downward
      +Z = inward


    Therefore:

      local X -> body X:
          bodyX = -localX

      local Y -> body Y:
          bodyY = localY

      LEFT leg local Z points right:
          bodyZ = +localZ

      RIGHT leg local Z points left:
          bodyZ = -localZ
  */


  float zAxisSign;

  if (isLeftLeg) {
    zAxisSign = 1.0f;
  }
  else {
    zAxisSign = -1.0f;
  }

  /*
    Convert the nominal leg-local foot target
    into a displacement expressed in BODY axes.
  */
  float bodyRelativeFootX = -nominalTarget.x;
  float bodyRelativeFootY = nominalTarget.y;
  float bodyRelativeFootZ = zAxisSign * nominalTarget.z;

  /*
    Now convert the hip-relative foot vector
    into a vector from the BODY ROTATION CENTER
    to the foot.
  */
  float centerToFootX = hipX + bodyRelativeFootX;
  float centerToFootY = bodyRelativeFootY;
  float centerToFootZ = hipZ + bodyRelativeFootZ;

  /*
    Convert correction angles from degrees
    to radians.
  */
  float roll = rollAngleDegrees * PI / 180.0f;
  float pitch = pitchAngleDegrees * PI / 180.0f;

  float cosRoll = cos(roll);
  float sinRoll = sin(roll);

  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  /*
    INVERSE PITCH ROTATION

    The foot is assumed fixed relative
    to the ground.

    If the body rotates, the foot's coordinates
    relative to the body move in the opposite
    rotational direction.

    Pitch acts in the body X-Y plane.
  */
  float pitchCorrectedX = cosPitch * centerToFootX + sinPitch * centerToFootY;
  float pitchCorrectedY = -sinPitch * centerToFootX + cosPitch * centerToFootY;
  float pitchCorrectedZ = centerToFootZ;

  /*
    INVERSE ROLL ROTATION

    Roll acts in the body Y-Z plane.
  */
  float rollCorrectedX = pitchCorrectedX;
  float rollCorrectedY = cosRoll * pitchCorrectedY + sinRoll * pitchCorrectedZ;
  float rollCorrectedZ = -sinRoll * pitchCorrectedY + cosRoll * pitchCorrectedZ;

  /*
    Convert from rotation-center coordinates
    back to a displacement relative to this
    particular hip joint.

    These values are STILL expressed using
    BODY axes.
  */
  float correctedBodyRelativeX = rollCorrectedX - hipX;
  float correctedBodyRelativeY = rollCorrectedY;
  float correctedBodyRelativeZ = rollCorrectedZ - hipZ;

  /*
    Convert BODY axes back into this leg's
    LOCAL IK coordinate system.

    Body +X is forward but local +X is backward,
    so X is negated.

    Y is identical.

    Z depends on the side:
      left  local +Z = body +Z
      right local +Z = body -Z
  */

  output.x = -correctedBodyRelativeX;
  output.y = correctedBodyRelativeY;
  output.z = zAxisSign * correctedBodyRelativeZ;

  return output;
}


BalanceOutput BalanceController::update(
  float measuredRoll,
  float measuredPitch,

  float measuredRollRate,
  float measuredPitchRate,

  const FootTarget &nominalFrontLeft,
  const FootTarget &nominalFrontRight,
  const FootTarget &nominalBackLeft,
  const FootTarget &nominalBackRight
) {

  BalanceOutput output;

  output.frontLeft = nominalFrontLeft;
  output.frontRight = nominalFrontRight;
  output.backLeft = nominalBackLeft;
  output.backRight = nominalBackRight;

  output.rollError = 0.0f;
  output.pitchError = 0.0f;

  output.rollCorrectionAngle = 0.0f;
  output.pitchCorrectionAngle = 0.0f;

  output.safetyStopped = false;

  if (!enabled) {
    return output;
  }

  float rawRollError = normalizeAngle(targetRoll - measuredRoll);
  float rawPitchError = normalizeAngle(targetPitch - measuredPitch);

  if (fabs(rawRollError) > maximumSafeAngle || fabs(rawPitchError) > maximumSafeAngle) {

    enabled = false;
    reset();
    output.safetyStopped = true;
    return output;
  }


  float rollError = applyDeadband(rawRollError);
  float pitchError = applyDeadband(rawPitchError);

  output.rollError = rollError;
  output.pitchError = pitchError;

  /*
    The PD controllers produce desired
    CORRECTIVE BODY ANGLES.

    They no longer directly produce
    centimeters of leg displacement.
  */

  float rawRollCorrectionAngle = rollCorrectionSign * (rollKp * rollError - rollKd * measuredRollRate);
  float rawPitchCorrectionAngle = pitchCorrectionSign * (pitchKp * pitchError - pitchKd * measuredPitchRate);

  rawRollCorrectionAngle = clampValue(rawRollCorrectionAngle, -maximumRollCorrectionAngle, maximumRollCorrectionAngle);
  rawPitchCorrectionAngle = clampValue(rawPitchCorrectionAngle, -maximumPitchCorrectionAngle, maximumPitchCorrectionAngle);

  /*
    Smooth the commanded body correction angle.
  */
  filteredRollCorrectionAngle += outputFilterAlpha * (rawRollCorrectionAngle - filteredRollCorrectionAngle);
  filteredPitchCorrectionAngle += outputFilterAlpha * (rawPitchCorrectionAngle - filteredPitchCorrectionAngle);

  output.rollCorrectionAngle = filteredRollCorrectionAngle;
  output.pitchCorrectionAngle = filteredPitchCorrectionAngle;

  /*
    Hip locations in the BODY frame.

    Body +X = forward
    Body +Z = right
  */
  float halfLength = bodyLength / 2.0f;
  float halfWidth = bodyWidth / 2.0f;

  /*
    rotationCenterOffsetX > 0:
      rotation center is shifted toward front.

    rotationCenterOffsetZ > 0:
      rotation center is shifted toward right.
  */

  float frontHipX = halfLength - rotationCenterOffsetX;
  float backHipX = -halfLength - rotationCenterOffsetX;

  float leftHipZ = -halfWidth - rotationCenterOffsetZ;

  float rightHipZ = halfWidth - rotationCenterOffsetZ;

  /*
    FRONT LEFT

    Left-side leg:
      local +Z = body +Z
  */
  output.frontLeft = calculateCorrectedFootTarget(nominalFrontLeft, frontHipX, leftHipZ, true, filteredRollCorrectionAngle, filteredPitchCorrectionAngle);

  /*
    FRONT RIGHT

    Right-side leg:
      local +Z = body -Z
  */
  output.frontRight = calculateCorrectedFootTarget(nominalFrontRight, frontHipX, rightHipZ, false, filteredRollCorrectionAngle, filteredPitchCorrectionAngle);

  /*
    BACK LEFT
  */
  output.backLeft = calculateCorrectedFootTarget(nominalBackLeft, backHipX, leftHipZ, true, filteredRollCorrectionAngle, filteredPitchCorrectionAngle);

  /*
    BACK RIGHT
  */

  output.backRight = calculateCorrectedFootTarget(nominalBackRight, backHipX, rightHipZ, false, filteredRollCorrectionAngle, filteredPitchCorrectionAngle);

  /*
    Basic numerical safety check.
  */
  if (
    !isfinite(output.frontLeft.x) ||
    !isfinite(output.frontLeft.y) ||
    !isfinite(output.frontLeft.z) ||

    !isfinite(output.frontRight.x) ||
    !isfinite(output.frontRight.y) ||
    !isfinite(output.frontRight.z) ||

    !isfinite(output.backLeft.x) ||
    !isfinite(output.backLeft.y) ||
    !isfinite(output.backLeft.z) ||

    !isfinite(output.backRight.x) ||
    !isfinite(output.backRight.y) ||
    !isfinite(output.backRight.z)
  ) {

    enabled =
      false;

    reset();

    output.frontLeft = nominalFrontLeft;
    output.frontRight = nominalFrontRight;
    output.backLeft = nominalBackLeft;
    output.backRight = nominalBackRight;

    output.safetyStopped =true;
  }

  return output;
}

void BalanceController::reset() {

  filteredRollCorrectionAngle =0.0f;
  filteredPitchCorrectionAngle =  0.0f;
}


void BalanceController::setEnabled(bool state) {

  enabled = state;

  if (!enabled) {
    reset();
  }
}

bool BalanceController::isEnabled() const {

  return enabled;
}

void BalanceController::setGeometry(float newBodyLength, float newBodyWidth) {
  bodyLength = fabs(newBodyLength);

  bodyWidth = fabs(newBodyWidth);
}


void BalanceController::setRotationCenterOffset(float offsetX, float offsetZ) {

  rotationCenterOffsetX = offsetX;
  rotationCenterOffsetZ = offsetZ;
}


void BalanceController::setTargetAngles(float rollDegrees, float pitchDegrees) {
  targetRoll = rollDegrees;
  targetPitch = pitchDegrees;
}


void BalanceController::setCorrectionSigns(float rollSign, float pitchSign) {

  rollCorrectionSign =
    rollSign >= 0.0f
      ? 1.0f
      : -1.0f;


  pitchCorrectionSign =
    pitchSign >= 0.0f
      ? 1.0f
      : -1.0f;
}

void BalanceController::setRollGains(float kp, float kd) {
  rollKp = kp;
  rollKd =kd;
}

void BalanceController::setPitchGains(float kp, float kd) {

  pitchKp = kp;
  pitchKd = kd;
}

void BalanceController::setMaximumCorrectionAngles(float maximumRollDegrees, float maximumPitchDegrees) {

  maximumRollCorrectionAngle = fabs(maximumRollDegrees);
  maximumPitchCorrectionAngle = fabs(maximumPitchDegrees);
}

void BalanceController::setMaximumSafeAngle(float angleDegrees) {
  maximumSafeAngle = fabs(angleDegrees);
}

void BalanceController::setDeadband(float deadbandDegrees) {
  angleDeadband = fabs(deadbandDegrees);
}

void BalanceController::setOutputFilterAlpha(float alpha) {
  outputFilterAlpha = clampValue(alpha, 0.0f, 1.0f);
}