#include "InverseKinematics.h"
#include "GaitCycle.h"
#include "IMUOrientation.h"
#include "BalanceController.h"
#include "SitToStand.h"
#include <math.h>


// Creates each leg using its upper, lower, and side motor pins and its orientation number.
InverseKinematics frontLeftLeg(17, 19, 18, 0);
InverseKinematics backLeftLeg(23, 21, 22, 1);
InverseKinematics frontRightLeg(13, 27, 14, 2);
InverseKinematics backRightLeg(32, 26, 25, 3);


// Creates the IMU using its SDA and SCL pins.
IMUOrientation orientation(33, 16);


// Accelerometer calibration offsets.
const float ACCEL_X_OFFSET = -0.11622900f;
const float ACCEL_Y_OFFSET = 0.08589700f;
const float ACCEL_Z_OFFSET = 1.30524301f;


// Accelerometer calibration scale factors.
const float ACCEL_X_SCALE = 0.99895497f;
const float ACCEL_Y_SCALE = 0.99418946f;
const float ACCEL_Z_SCALE = 0.99225240f;


// Gyroscope calibration offsets.
const float GYRO_X_OFFSET = 0.00766f;
const float GYRO_Y_OFFSET = 0.01859f;
const float GYRO_Z_OFFSET = 0.00223f;


// Determines how strongly the complementary filter favors gyroscope data.
const float COMPLEMENTARY_ALPHA = 0.98f;


// Cartesian foot position while the robot is sitting.
const float SIT_X = 0.0f;
const float SIT_Y = 8.5f;
const float SIT_Z = 0.0f;


// Cartesian foot position while the robot is standing.
const float STANDING_X = 0.0f;
const float STANDING_Y = 20.0f;
const float STANDING_Z = 0.0f;


// Distance between the front and back hip pivots and between the left and right hip pivots.
const float BODY_LENGTH = 20.0f;
const float BODY_WIDTH = 19.45f;


// Actual center-of-mass offset relative to the geometric center of the robot.
//
// Total robot weight = 6.0 lb.
// Weight measured on the two front legs = 3.6 lb.
//
// Front weight fraction:
// 3.6 / 6.0 = 0.60
//
// Assuming a 20 cm front-to-rear support distance:
// COM position from rear = 20 * 0.60 = 12 cm.
// Geometric center = 20 / 2 = 10 cm.
// COM offset = 12 - 10 = +2.0 cm.
//
// Positive X is toward the front of the robot.
// The left-right weight distribution is assumed centered for now.
const float COM_OFFSET_X = 2.0f;
const float COM_OFFSET_Z = 0.0f;


// Desired body orientation while balancing.
const float TARGET_ROLL = 0.0f;
const float TARGET_PITCH = 0.0f;


// Roll PD controller gains.
// Derivative gain remains zero for this test so the new support-planning system can be evaluated independently.
const float ROLL_KP = 1.0f;
const float ROLL_KD = 0.0f;


// Pitch PD controller gains.
const float PITCH_KP = 1.0f;
const float PITCH_KD = 0.0f;


// Determines the physical direction of roll and pitch corrections.
const float ROLL_CORRECTION_SIGN = 1.0f;
const float PITCH_CORRECTION_SIGN = 1.0f;


// Each step now contains a dedicated four-foot body-shift phase followed by a one-leg swing phase.
const double GAIT_SHIFT_TIME = 350.0;
const double GAIT_SWING_TIME = 500.0;
const double GAIT_STEP_TIME = GAIT_SHIFT_TIME + GAIT_SWING_TIME;
const double GAIT_CYCLE_TIME = 4.0 * GAIT_STEP_TIME;


// The swing fraction is determined by the 500 ms swing inside the full 3400 ms gait cycle.
const double GAIT_SWING_FRACTION = GAIT_SWING_TIME / GAIT_CYCLE_TIME;


// Defines the fore-aft stride and vertical foot clearance.
const double GAIT_STRIDE_LENGTH = 5.0;
const double GAIT_HEIGHT = 3.0;
const double GAIT_COMPRESSION = 0.0;


// Positive direction makes the stance feet move backward relative to the robot, producing forward body movement.
const double GAIT_DIRECTION = 1.0;


// Each swing begins only after a 350 ms all-foot support shift.
//
// Cycle:
// shift -> FL swing -> shift -> BR swing -> shift -> FR swing -> shift -> BL swing.
const double FRONT_LEFT_SWING_START = GAIT_SHIFT_TIME / GAIT_CYCLE_TIME;
const double BACK_RIGHT_SWING_START = (GAIT_STEP_TIME + GAIT_SHIFT_TIME) / GAIT_CYCLE_TIME;
const double FRONT_RIGHT_SWING_START = (2.0 * GAIT_STEP_TIME + GAIT_SHIFT_TIME) / GAIT_CYCLE_TIME;
const double BACK_LEFT_SWING_START = (3.0 * GAIT_STEP_TIME + GAIT_SHIFT_TIME) / GAIT_CYCLE_TIME;


// Planned body shifts used to move the COM farther inside the upcoming three-foot support triangle.
//
// Because the measured COM is already 2 cm toward the front, a rear-leg swing does not need an additional
// forward X shift. A front-leg swing still uses a small 1 cm backward shift.
//
// The robot also shifts 1.5 cm laterally toward the side opposite the leg that is about to swing.
const double FRONT_LEG_BODY_SHIFT_X = -1.0;
const double REAR_LEG_BODY_SHIFT_X = 0.0;
const double BODY_SHIFT_Z = 1.5;


// Creates the roll and pitch balance controller.
BalanceController balance(BODY_LENGTH, BODY_WIDTH, ROLL_KP, ROLL_KD, PITCH_KP, PITCH_KD);


// Creates the gait generator for each leg.
GaitCycle frontLeftGait(STANDING_X, STANDING_Y, GAIT_STRIDE_LENGTH, GAIT_HEIGHT, FRONT_LEFT_SWING_START, GAIT_CYCLE_TIME, GAIT_SWING_FRACTION, GAIT_COMPRESSION, GAIT_DIRECTION);
GaitCycle backRightGait(STANDING_X, STANDING_Y, GAIT_STRIDE_LENGTH, GAIT_HEIGHT, BACK_RIGHT_SWING_START, GAIT_CYCLE_TIME, GAIT_SWING_FRACTION, GAIT_COMPRESSION, GAIT_DIRECTION);
GaitCycle frontRightGait(STANDING_X, STANDING_Y, GAIT_STRIDE_LENGTH, GAIT_HEIGHT, FRONT_RIGHT_SWING_START, GAIT_CYCLE_TIME, GAIT_SWING_FRACTION, GAIT_COMPRESSION, GAIT_DIRECTION);
GaitCycle backLeftGait(STANDING_X, STANDING_Y, GAIT_STRIDE_LENGTH, GAIT_HEIGHT, BACK_LEFT_SWING_START, GAIT_CYCLE_TIME, GAIT_SWING_FRACTION, GAIT_COMPRESSION, GAIT_DIRECTION);


// Creates the sit-to-stand controller for each leg.
SitToStand frontLeftStand(frontLeftLeg, SIT_X, SIT_Y, SIT_Z, STANDING_X, STANDING_Y, STANDING_Z);
SitToStand frontRightStand(frontRightLeg, SIT_X, SIT_Y, SIT_Z, STANDING_X, STANDING_Y, STANDING_Z);
SitToStand backLeftStand(backLeftLeg, SIT_X, SIT_Y, SIT_Z, STANDING_X, STANDING_Y, STANDING_Z);
SitToStand backRightStand(backRightLeg, SIT_X, SIT_Y, SIT_Z, STANDING_X, STANDING_Y, STANDING_Z);


// Stores the default standing position for each leg.
const FootTarget STANDING_FRONT_LEFT = { STANDING_X, STANDING_Y, STANDING_Z };
const FootTarget STANDING_FRONT_RIGHT = { STANDING_X, STANDING_Y, STANDING_Z };
const FootTarget STANDING_BACK_LEFT = { STANDING_X, STANDING_Y, STANDING_Z };
const FootTarget STANDING_BACK_RIGHT = { STANDING_X, STANDING_Y, STANDING_Z };


// Sets the balance and gait control loop to run every 20 ms, or 50 Hz.
const unsigned long CONTROL_PERIOD_US = 20000;


// Gives the IMU time to settle after the robot finishes standing.
const unsigned long BALANCE_DELAY_AFTER_STAND_MS = 3000;


// Lets the balance controller stabilize the robot before walking begins.
const unsigned long BALANCE_HOLD_BEFORE_WALK_MS = 1500;


// Stores timestamps used for sit-to-stand, balance startup, gait startup, and the control loop.
unsigned long standFinishedTime = 0;
unsigned long balanceStartedTime = 0;
unsigned long gaitStartTime = 0;
unsigned long previousControlTime = 0;


// Tracks the current state of the robot.
bool imuWorking = false;
bool standFinished = false;
bool balanceStarted = false;
bool gaitStarted = false;
bool safetyStopped = false;


// Creates a smooth interpolation for the planned body shift.
double smoothBodyShift(double value) {
  return 10.0 * pow(value, 3) - 15.0 * pow(value, 4) + 6.0 * pow(value, 5);
}


// Sends four Cartesian foot targets to their corresponding inverse kinematics objects.
void commandFootTargets(const FootTarget &frontLeft, const FootTarget &frontRight, const FootTarget &backLeft, const FootTarget &backRight) {
  frontLeftLeg.updateTarget(frontLeft.x, frontLeft.y, frontLeft.z);
  frontRightLeg.updateTarget(frontRight.x, frontRight.y, frontRight.z);
  backLeftLeg.updateTarget(backLeft.x, backLeft.y, backLeft.z);
  backRightLeg.updateTarget(backRight.x, backRight.y, backRight.z);
}


// Commands all four legs to the standard standing position.
void commandStandingPosition() {
  commandFootTargets(STANDING_FRONT_LEFT, STANDING_FRONT_RIGHT, STANDING_BACK_LEFT, STANDING_BACK_RIGHT);
}


// Updates all four sit-to-stand transitions and detects when standing is complete.
void updateSitToStand() {
  frontLeftStand.updateTransition();
  frontRightStand.updateTransition();
  backLeftStand.updateTransition();
  backRightStand.updateTransition();

  if (frontLeftStand.isFinished() && frontRightStand.isFinished() && backLeftStand.isFinished() && backRightStand.isFinished()) {
    standFinished = true;
    standFinishedTime = millis();

    commandStandingPosition();
  }
}


// Resets all four gait generators using the same timestamp so their phases stay synchronized.
void resetAllGaits(unsigned long sharedStartTime) {
  frontLeftGait.resetCycle(sharedStartTime);
  backRightGait.resetCycle(sharedStartTime);
  frontRightGait.resetCycle(sharedStartTime);
  backLeftGait.resetCycle(sharedStartTime);

  gaitStartTime = sharedStartTime;
}


// Gets the current Cartesian target from one gait generator and converts it into a FootTarget.
FootTarget getGaitTarget(GaitCycle &gait, unsigned long currentTime) {
  double x;
  double y;
  double z;

  gait.getTarget(currentTime, x, y, z);

  FootTarget target = { (float)x, (float)y, (float)z };

  return target;
}


// Returns the desired body shift for the support triangle used before each leg swings.
void getBodyShiftTarget(int stepIndex, double &targetX, double &targetZ) {

  // Before the front-left leg swings, shift backward and right toward the back-right support corner.
  if (stepIndex == 0) {
    targetX = FRONT_LEG_BODY_SHIFT_X;
    targetZ = BODY_SHIFT_Z;
  }


  // Before the back-right leg swings, shift left.
  // No additional forward X shift is used because the measured COM is already front-heavy.
  else if (stepIndex == 1) {
    targetX = REAR_LEG_BODY_SHIFT_X;
    targetZ = -BODY_SHIFT_Z;
  }


  // Before the front-right leg swings, shift backward and left toward the back-left support corner.
  else if (stepIndex == 2) {
    targetX = FRONT_LEG_BODY_SHIFT_X;
    targetZ = -BODY_SHIFT_Z;
  }


  // Before the back-left leg swings, shift right.
  // No additional forward X shift is used because the measured COM is already front-heavy.
  else {
    targetX = REAR_LEG_BODY_SHIFT_X;
    targetZ = BODY_SHIFT_Z;
  }
}


// Calculates the planned body translation for the current point in the static crawl.
void getPlannedBodyShift(unsigned long currentTime, double &bodyShiftX, double &bodyShiftZ) {
  double elapsed = (double)(currentTime - gaitStartTime);
  double cycleElapsed = fmod(elapsed, GAIT_CYCLE_TIME);

  int stepIndex = (int)(cycleElapsed / GAIT_STEP_TIME);

  if (stepIndex < 0) {
    stepIndex = 0;
  }

  if (stepIndex > 3) {
    stepIndex = 3;
  }


  // Finds how far the robot is through the current shift/swing pair.
  double stepElapsed = cycleElapsed - stepIndex * GAIT_STEP_TIME;


  // Gets the body position that should be held during the upcoming swing.
  double targetX;
  double targetZ;

  getBodyShiftTarget(stepIndex, targetX, targetZ);


  // During the first part of each step, all four feet remain down while the body moves into the support triangle.
  if (stepElapsed < GAIT_SHIFT_TIME) {
    int previousStepIndex = (stepIndex + 3) % 4;

    double startX;
    double startZ;


    // The first body shift begins from the neutral standing position instead of the previous cycle's position.
    if (elapsed < GAIT_SHIFT_TIME && stepIndex == 0) {
      startX = 0.0;
      startZ = 0.0;
    }

    else {
      getBodyShiftTarget(previousStepIndex, startX, startZ);
    }


    double shiftProgress = stepElapsed / GAIT_SHIFT_TIME;
    double r = smoothBodyShift(shiftProgress);

    bodyShiftX = startX + (targetX - startX) * r;
    bodyShiftZ = startZ + (targetZ - startZ) * r;
  }


  // During the swing phase, the body shift is held constant while one foot moves.
  else {
    bodyShiftX = targetX;
    bodyShiftZ = targetZ;
  }
}


// Applies a planned body translation to all four leg-local foot targets.
//
// Body +X is forward.
// Leg-local +X is backward.
// Therefore a forward body shift requires a positive local-X foot displacement.
//
// Body +Z is toward the robot's right.
// Left-leg local +Z follows body +Z, while right-leg local +Z is mirrored.
void applyPlannedBodyShift(
  FootTarget &frontLeft,
  FootTarget &frontRight,
  FootTarget &backLeft,
  FootTarget &backRight,
  double bodyShiftX,
  double bodyShiftZ) {
  frontLeft.x += bodyShiftX;
  frontRight.x += bodyShiftX;
  backLeft.x += bodyShiftX;
  backRight.x += bodyShiftX;

  frontLeft.z -= bodyShiftZ;
  backLeft.z -= bodyShiftZ;

  frontRight.z += bodyShiftZ;
  backRight.z += bodyShiftZ;
}


// Blends between the nominal gait target and the fully balanced target.
FootTarget blendFootTarget(const FootTarget &nominal, const FootTarget &balanced, double balanceWeight) {

  // Keeps the requested balance percentage between 0 and 100 percent.
  if (balanceWeight < 0.0) {
    balanceWeight = 0.0;
  }

  if (balanceWeight > 1.0) {
    balanceWeight = 1.0;
  }


  // Applies only the requested portion of the balance correction.
  FootTarget target = {
    nominal.x + (balanced.x - nominal.x) * (float)balanceWeight,
    nominal.y + (balanced.y - nominal.y) * (float)balanceWeight,
    nominal.z + (balanced.z - nominal.z) * (float)balanceWeight
  };

  return target;
}


// Initializes the servos for all four legs.
void setup() {
  frontLeftLeg.begin();
  backLeftLeg.begin();
  frontRightLeg.begin();
  backRightLeg.begin();
}


// Runs the sit-to-stand, IMU, balance, support-shift, gait, and safety-control sequence.
void loop() {

  // Performs the sit-to-stand transition before enabling the IMU or walking.
  if (!standFinished) {
    updateSitToStand();

    return;
  }


  // Initializes and configures the IMU and balance controller after the robot is standing.
  if (!imuWorking) {
    orientation.setAccelerometerCalibration(ACCEL_X_OFFSET, ACCEL_Y_OFFSET, ACCEL_Z_OFFSET, ACCEL_X_SCALE, ACCEL_Y_SCALE, ACCEL_Z_SCALE);
    orientation.setGyroscopeCalibration(GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET);
    orientation.setComplementaryAlpha(COMPLEMENTARY_ALPHA);

    imuWorking = orientation.begin();


    // Keeps the robot standing if the IMU fails to initialize.
    if (!imuWorking) {
      commandStandingPosition();

      return;
    }


    // Configures the physical geometry and measured center of mass used by the balance controller.
    balance.setGeometry(BODY_LENGTH, BODY_WIDTH);
    balance.setRotationCenterOffset(COM_OFFSET_X, COM_OFFSET_Z);
    balance.setTargetAngles(TARGET_ROLL, TARGET_PITCH);
    balance.setCorrectionSigns(ROLL_CORRECTION_SIGN, PITCH_CORRECTION_SIGN);


    // Limits the balance controller so it corrects attitude without overpowering the gait.
    balance.setMaximumCorrectionAngles(12.0f, 12.0f);
    balance.setMaximumSafeAngle(60.0f);
    balance.setDeadband(0.2f);
    balance.setOutputFilterAlpha(1.0f);
    balance.setEnabled(false);


    // Starts the control-loop timing and IMU settling period.
    previousControlTime = micros();
    standFinishedTime = millis();

    return;
  }


  // Updates the current roll, pitch, and angular velocity measurements.
  bool validImuReading = orientation.update();


  // Holds the standing position while the complementary filter settles.
  if (millis() - standFinishedTime < BALANCE_DELAY_AFTER_STAND_MS) {
    commandStandingPosition();

    return;
  }


  // Limits the main balance and gait controller to 50 Hz.
  unsigned long currentTime = micros();

  if (currentTime - previousControlTime < CONTROL_PERIOD_US) {
    return;
  }

  previousControlTime = currentTime;


  // Keeps the previous servo positions if the current IMU reading is invalid.
  if (!validImuReading) {
    return;
  }


  // Stops sending new motor commands after the balance controller enters its safety state.
  if (safetyStopped) {
    return;
  }


  // Enables the balance controller before allowing the robot to begin walking.
  if (!balanceStarted) {
    balance.reset();
    balance.setEnabled(true);

    balanceStarted = true;
    balanceStartedTime = millis();
  }


  // Uses one shared timestamp for gait generation and body-shift planning.
  unsigned long gaitTime = millis();


  // Gives the balance controller time to stabilize the standing robot before beginning the crawl.
  if (!gaitStarted && gaitTime - balanceStartedTime >= BALANCE_HOLD_BEFORE_WALK_MS) {
    resetAllGaits(gaitTime);

    gaitStarted = true;
  }


  // Uses the normal standing position until the static crawl begins.
  FootTarget nominalFrontLeft = STANDING_FRONT_LEFT;
  FootTarget nominalFrontRight = STANDING_FRONT_RIGHT;
  FootTarget nominalBackLeft = STANDING_BACK_LEFT;
  FootTarget nominalBackRight = STANDING_BACK_RIGHT;


  // Generates the walking trajectory and planned COM-support shift once walking begins.
  if (gaitStarted) {
    nominalFrontLeft = getGaitTarget(frontLeftGait, gaitTime);
    nominalFrontRight = getGaitTarget(frontRightGait, gaitTime);
    nominalBackLeft = getGaitTarget(backLeftGait, gaitTime);
    nominalBackRight = getGaitTarget(backRightGait, gaitTime);


    // Calculates where the body should move before the next leg is allowed to swing.
    double bodyShiftX;
    double bodyShiftZ;

    getPlannedBodyShift(gaitTime, bodyShiftX, bodyShiftZ);


    // Adds the support-polygon body translation to the normal walking targets.
    applyPlannedBodyShift(
      nominalFrontLeft,
      nominalFrontRight,
      nominalBackLeft,
      nominalBackRight,
      bodyShiftX,
      bodyShiftZ);
  }


  // Calculates the complete IMU roll and pitch correction around the measured COM.
  BalanceOutput output = balance.update(
    orientation.getRoll(),
    orientation.getPitch(),
    orientation.getRollRate(),
    orientation.getPitchRate(),
    nominalFrontLeft,
    nominalFrontRight,
    nominalBackLeft,
    nominalBackRight);


  // Freezes the robot if the balance controller detects an unsafe body angle.
  if (output.safetyStopped) {
    balance.setEnabled(false);

    safetyStopped = true;

    return;
  }


  // During walking, full balance authority remains on stance legs while the swing leg follows its gait trajectory.
  if (gaitStarted) {
    double frontLeftBalanceWeight = frontLeftGait.getStanceBlend(gaitTime);
    double frontRightBalanceWeight = frontRightGait.getStanceBlend(gaitTime);
    double backLeftBalanceWeight = backLeftGait.getStanceBlend(gaitTime);
    double backRightBalanceWeight = backRightGait.getStanceBlend(gaitTime);

    output.frontLeft = blendFootTarget(nominalFrontLeft, output.frontLeft, frontLeftBalanceWeight);
    output.frontRight = blendFootTarget(nominalFrontRight, output.frontRight, frontRightBalanceWeight);
    output.backLeft = blendFootTarget(nominalBackLeft, output.backLeft, backLeftBalanceWeight);
    output.backRight = blendFootTarget(nominalBackRight, output.backRight, backRightBalanceWeight);
  }


  // Sends the final combination of gait, planned body shift, and IMU balance correction to inverse kinematics.
  commandFootTargets(output.frontLeft, output.frontRight, output.backLeft, output.backRight);
}