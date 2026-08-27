#include "InverseKinematics.h"


// Restricts a double value so it stays between a specified minimum and maximum.
static double clampDouble(double value, double minimum, double maximum) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}


// Stores the servo pins and leg orientation number for this leg.
InverseKinematics::InverseKinematics(int up, int low, int side, int o) {
  upperPin = up;
  lowerPin = low;
  sidePin = side;

  orientation = o;
}


// Attaches each servo to its corresponding ESP32 pin and sets the pulse-width range.
void InverseKinematics::begin() {
  upperMotor.attach(upperPin, 500, 2500);
  lowerMotor.attach(lowerPin, 500, 2500);
  sideMotor.attach(sidePin, 500, 2500);
}


// Converts a desired Cartesian foot position into the three joint angles required to reach it.
void InverseKinematics::updateTarget(double targetX, double targetY, double targetZ) {

  // Rejects the target if any coordinate is NaN or infinite.
  if (!isfinite(targetX) || !isfinite(targetY) || !isfinite(targetZ)) {
    return;
  }


  // Converts the commanded Z position from the neutral leg plane to the true Z position relative to the side joint's axis of rotation.
  double targetZPhysical = targetZ - sideOffset;


  // Calculates the effective Y distance inside the rotated leg plane.
  double targetYPrimeSquared = targetY * targetY + targetZPhysical * targetZPhysical - sideOffset * sideOffset;

  // Rejects targets that cannot geometrically exist because of the fixed side-joint offset.
  if (targetYPrimeSquared < 0.0) {
    return;
  }

  double targetYPrime = sqrt(targetYPrimeSquared);


  // Calculates and limits the side joint angle required to reach the requested Y-Z position.
  double sideAngle = atan2(targetZPhysical, targetY) + atan2(sideOffset, targetYPrime);

  // Positive rotation moves the foot into the frame and is limited to 18 degrees.
  // Negative rotation moves the foot away from the frame and is limited to 15 degrees.
  sideAngle = clampDouble(sideAngle, radians(-15.0), radians(18.0));


  // Uses the law of cosines to calculate the lower joint angle.
  double lowerArgument = (targetX * targetX + targetYPrime * targetYPrime - upperLeg * upperLeg - lowerLeg * lowerLeg) / (2.0 * upperLeg * lowerLeg);

  // Rejects targets that are physically outside the planar reach of the leg.
  if (lowerArgument < -1.000001 || lowerArgument > 1.000001) {
    return;
  }

  // Protects acos from small floating-point errors near -1 and 1.
  lowerArgument = clampDouble(lowerArgument, -1.0, 1.0);

  double lowerAngle = acos(lowerArgument);


  // Calculates the upper joint angle using the geometry of the two-link leg.
  double upperAngle = atan2(targetYPrime, targetX) - atan2(lowerLeg * sin(lowerAngle), upperLeg + lowerLeg * cos(lowerAngle));


  // Rejects the target if any calculated joint angle is invalid.
  if (!isfinite(sideAngle) || !isfinite(upperAngle) || !isfinite(lowerAngle)) {
    return;
  }


  // Converts the mathematical joint angles into calibrated servo commands.
  double sideCommand = calibrateSideAngle(sideAngle);
  double upperCommand = calibrateUpperAngle(upperAngle);
  double lowerCommand = calibrateLowerAngle(lowerAngle);

  // Rejects the target if any calibrated servo command is invalid.
  if (!isfinite(sideCommand) || !isfinite(upperCommand) || !isfinite(lowerCommand)) {
    return;
  }


  // Sends the calibrated commands to all three servos.
  sideMotor.write(sideCommand);
  upperMotor.write(upperCommand);
  lowerMotor.write(lowerCommand);
}


// Converts the calculated upper joint angle into the calibrated servo command for each leg.
double InverseKinematics::calibrateUpperAngle(double angle) {
  double correctedAngle;


  // Front-left upper servo calibration.
  if (orientation == 0) {
    angle = clampDouble(90.0 - degrees(angle), 0.0, 180.0);

    correctedAngle = 7.0 + (angle / 180.0) * (173.0 - 7.0);
  }

  // Back-left upper servo calibration.
  else if (orientation == 1) {
    angle = clampDouble(90.0 - degrees(angle), 0.0, 180.0);

    correctedAngle = 6.0 + (angle / 180.0) * (165.0 - 6.0);
  }

  // Front-right upper servo calibration.
  // The angle direction is reversed because of the mirrored mounting.
  else if (orientation == 2) {
    angle = clampDouble(90.0 + degrees(angle), 0.0, 180.0);

    correctedAngle = 2.0 + (angle / 180.0) * (165.0 - 2.0);
  }

  // Back-right upper servo calibration.
  // The angle direction is reversed because of the mirrored mounting.
  else if (orientation == 3) {
    angle = clampDouble(90.0 + degrees(angle), 0.0, 180.0);

    correctedAngle = 0.000154321 * angle * angle + 0.930555556 * angle + 13.0;;
  }

  // Rejects an invalid leg orientation.
  else {
    return NAN;
  }


  // Applies the servo scaling factor and keeps the final command within the Servo library's range.
  return clampDouble(correctedAngle * 0.75, 0.0, 180.0);
}


// Converts the calculated side joint angle into the calibrated servo command for each leg.
double InverseKinematics::calibrateSideAngle(double angle) {
  double correctedAngle;


  // Front-left side servo calibration.
  if (orientation == 0) {
    angle = clampDouble(90.0 - degrees(angle), 0.0, 180.0);

    correctedAngle = -0.000246914 * angle * angle + 0.988889 * angle;
  }

  // Back-left side servo calibration.
  else if (orientation == 1) {
    angle = clampDouble(90.0 + degrees(angle), 0.0, 180.0);

    correctedAngle = 0.00030864 * angle * angle + 0.833333 * angle + 7.5;
  }

  // Front-right side servo calibration.
  else if (orientation == 2) {
    angle = clampDouble(90.0 + degrees(angle), 0.0, 180.0);

    correctedAngle = -0.0000617284 * angle * angle + 0.95 * angle + 13.0;
  }

  // Back-right side servo calibration.
  else if (orientation == 3) {
    angle = clampDouble(90.0 - degrees(angle), 0.0, 180.0);

    correctedAngle = -0.000185185 * angle * angle + 0.983333333 * angle + 3.0;
  }

  // Rejects an invalid leg orientation.
  else {
    return NAN;
  }


  // Applies the servo scaling factor and keeps the final command within the Servo library's range.
  return clampDouble(correctedAngle * 0.75, 0.0, 180.0);
}


// Converts the calculated lower joint angle into the calibrated servo command for each leg.
double InverseKinematics::calibrateLowerAngle(double angle) {
  double correctedAngle;


  // Front-left lower servo calibration.
  if (orientation == 0) {
    angle = clampDouble(degrees(angle), 0.0, 180.0);

    correctedAngle = 0.955555556 * angle;
  }

  // Back-left lower servo calibration.
  else if (orientation == 1) {
    angle = clampDouble(degrees(angle), 0.0, 180.0);

    correctedAngle = 0.000246914 * angle * angle + 0.900000000 * angle + 10.0;
  }

  // Front-right lower servo calibration.
  // This motor uses a 90-to-270-degree physical reference range.
  else if (orientation == 2) {
    angle = clampDouble(270.0 - degrees(angle), 90.0, 270.0);

    correctedAngle = 0.944444444 * angle - 27.0;
  }

  // Back-right lower servo calibration.
  // This motor also uses a 90-to-270-degree physical reference range.
  else if (orientation == 3) {
    angle = clampDouble(270.0 - degrees(angle), 90.0, 270.0);

    correctedAngle = 0.001666667 * angle * angle + 0.494444444 * angle + 15.0;
  }

  // Rejects an invalid leg orientation.
  else {
    return NAN;
  }


  // Applies the servo scaling factor and keeps the final command within the Servo library's range.
  return clampDouble(correctedAngle * 0.75, 0.0, 180.0);
}