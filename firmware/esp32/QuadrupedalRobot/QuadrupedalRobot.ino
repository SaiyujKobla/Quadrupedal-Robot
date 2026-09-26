#include "InverseKinematics.h"
#include "GaitCycle.h"
#include "IMUOrientation.h"
#include "BalanceController.h"
#include "SitToStand.h"
#include <math.h>


// ------------------------------------------------------------
// LEGS
// ------------------------------------------------------------

InverseKinematics frontLeftLeg(17, 19, 18, 0);
InverseKinematics backLeftLeg(23, 21, 22, 1);
InverseKinematics frontRightLeg(13, 27, 14, 2);
InverseKinematics backRightLeg(32, 26, 25, 3);


// ------------------------------------------------------------
// IMU
// ------------------------------------------------------------

IMUOrientation orientation(33, 16);


const float ACCEL_X_OFFSET = -0.11622900f;
const float ACCEL_Y_OFFSET = 0.08589700f;
const float ACCEL_Z_OFFSET = 1.30524301f;


const float ACCEL_X_SCALE = 0.99895497f;
const float ACCEL_Y_SCALE = 0.99418946f;
const float ACCEL_Z_SCALE = 0.99225240f;


const float GYRO_X_OFFSET = 0.00766f;
const float GYRO_Y_OFFSET = 0.01859f;
const float GYRO_Z_OFFSET = 0.00223f;


const float COMPLEMENTARY_ALPHA = 0.98f;


bool imuReady = false;


// ------------------------------------------------------------
// ROBOT POSITIONS
// ------------------------------------------------------------

const float SIT_X = 0.0f;
const float SIT_Y = 8.5f;
const float SIT_Z = 0.0f;


const float STANDING_X = 0.0f;
const float STANDING_Y = 20.0f;
const float STANDING_Z = 0.0f;


// ------------------------------------------------------------
// BODY GEOMETRY
// ------------------------------------------------------------

const float BODY_LENGTH = 20.0f;
const float BODY_WIDTH = 19.45f;


// Measured center-of-mass offset.
//
// Total robot weight = 6.0 lb.
// Weight measured on front two legs = 3.6 lb.
//
// This gives approximately a +2 cm forward COM offset.
const float COM_OFFSET_X = 2.0f;
const float COM_OFFSET_Z = 0.0f;


// ------------------------------------------------------------
// BALANCE CONTROLLER
// ------------------------------------------------------------

const float TARGET_ROLL = 0.0f;
const float TARGET_PITCH = 0.0f;


const float ROLL_KP = 1.0f;
const float ROLL_KD = 0.0f;


const float PITCH_KP = 1.0f;
const float PITCH_KD = 0.0f;


const float ROLL_CORRECTION_SIGN = 1.0f;
const float PITCH_CORRECTION_SIGN = 1.0f;


// Robot is considered reasonably level inside this range.
const float LEVEL_TOLERANCE = 2.0f;


// Gives the IMU time to settle after standing.
const unsigned long IMU_SETTLE_TIME = 3000;


// Initial balance timing.
const unsigned long LEVEL_HOLD_TIME = 200;
const unsigned long MAX_BALANCE_TIME = 2000;


// Rebalance timing between steps.
const unsigned long REBALANCE_LEVEL_HOLD_TIME = 150;
const unsigned long MAX_REBALANCE_TIME = 1200;
const unsigned long REBALANCE_BLEND_TIME = 400;
const unsigned long REBALANCE_SETTLE_TIME = 300;


// Wait after each rebalance before beginning another shift.
const unsigned long WAIT_AFTER_REBALANCE = 500;


// Wait after the initial balance.
const unsigned long WAIT_AFTER_INITIAL_BALANCE = 2000;


// Main control update period.
const unsigned long CONTROL_PERIOD_MS = 20;


// Balance output filtering.
const float BALANCE_OUTPUT_FILTER_ALPHA = 0.25f;


// Ignore extremely small errors.
const float BALANCE_DEADBAND = 0.5f;


// ------------------------------------------------------------
// SUPPORT-POLYGON SHIFTS
// ------------------------------------------------------------

// FRONT LEFT
//
// Front-left now uses the same backward X shift as front-right.
const double FRONT_LEFT_BODY_SHIFT_X = -2.5;
const double FRONT_LEFT_BODY_SHIFT_Z = 3.0;


// BACK RIGHT
//
// Shift forward and left.
const double BACK_RIGHT_BODY_SHIFT_X = 1.5;
const double BACK_RIGHT_BODY_SHIFT_Z = -3.0;


// FRONT RIGHT
//
// Shift backward and farther left.
const double FRONT_RIGHT_BODY_SHIFT_X = -2.5;
const double FRONT_RIGHT_BODY_SHIFT_Z = -3.5;


// BACK LEFT
//
// Lateral shift only.
const double BACK_LEFT_BODY_SHIFT_X = 0.0;
const double BACK_LEFT_BODY_SHIFT_Z = 3.0;


// Time used to move into and out of the support position.
const unsigned long BODY_SHIFT_TIME = 1200;


// Gives the robot time to finish transferring weight
// before the swing begins.
const unsigned long SHIFT_SETTLE_TIME = 1000;


// Gives the returned foot time to accept weight.
const unsigned long TOUCHDOWN_SETTLE_TIME = 600;


// ------------------------------------------------------------
// GAIT
// ------------------------------------------------------------

// Previous walking code used a 5 cm stride.
//
// Negative local X moves the foot forward relative to the body.
const double GAIT_STRIDE_LENGTH = 5.0;


// Increased during physical testing to verify reliable clearance.
const double GAIT_HEIGHT = 8.0;


// One swing is deliberately slow for the first walking tests.
const unsigned long GAIT_SWING_TIME = 900;


// GaitCycle expects the swing duration to be represented as
// a fraction of a complete cycle.
//
// We use the GaitCycle class only to generate the smooth swing
// trajectory. The three stance legs remain fixed.
const double GAIT_REFERENCE_CYCLE_TIME = 4000.0;

const double GAIT_SWING_FRACTION =
  (double)GAIT_SWING_TIME /
  GAIT_REFERENCE_CYCLE_TIME;


// Positive gait direction produces the forward swing direction
// used in the previous walking controller.
const double GAIT_DIRECTION = 1.0;


// Once all four feet have stepped forward, all four feet remain
// on the ground while the body moves forward.
const unsigned long BODY_ADVANCE_TIME = 1800;


// Number of complete four-leg crawl cycles to execute.
//
// Keep this at 1 for the first physical test.
// Increase it after the complete cycle works reliably.
const int CRAWL_CYCLES_TO_RUN = 1;


// ------------------------------------------------------------
// OBJECTS
// ------------------------------------------------------------

BalanceController balance(
  BODY_LENGTH,
  BODY_WIDTH,
  ROLL_KP,
  ROLL_KD,
  PITCH_KP,
  PITCH_KD
);


// This GaitCycle object is used only as a SWING trajectory
// generator.
//
// initialX = 0 and initialY = 0 because its output is treated
// as a displacement added to the current balanced foot target.
GaitCycle swingGait(
  0.0,
  0.0,
  GAIT_STRIDE_LENGTH,
  GAIT_HEIGHT,
  0.0,
  GAIT_REFERENCE_CYCLE_TIME,
  GAIT_SWING_FRACTION,
  0.0,
  GAIT_DIRECTION
);


// Sit-to-stand controllers.
SitToStand frontLeftStand(
  frontLeftLeg,
  SIT_X, SIT_Y, SIT_Z,
  STANDING_X, STANDING_Y, STANDING_Z
);


SitToStand frontRightStand(
  frontRightLeg,
  SIT_X, SIT_Y, SIT_Z,
  STANDING_X, STANDING_Y, STANDING_Z
);


SitToStand backLeftStand(
  backLeftLeg,
  SIT_X, SIT_Y, SIT_Z,
  STANDING_X, STANDING_Y, STANDING_Z
);


SitToStand backRightStand(
  backRightLeg,
  SIT_X, SIT_Y, SIT_Z,
  STANDING_X, STANDING_Y, STANDING_Z
);


// ------------------------------------------------------------
// ORIGINAL STANDING TARGETS
// ------------------------------------------------------------

const FootTarget STANDING_FRONT_LEFT = {
  STANDING_X,
  STANDING_Y,
  STANDING_Z
};


const FootTarget STANDING_FRONT_RIGHT = {
  STANDING_X,
  STANDING_Y,
  STANDING_Z
};


const FootTarget STANDING_BACK_LEFT = {
  STANDING_X,
  STANDING_Y,
  STANDING_Z
};


const FootTarget STANDING_BACK_RIGHT = {
  STANDING_X,
  STANDING_Y,
  STANDING_Z
};


// ------------------------------------------------------------
// PLANNED WALKING TARGETS
//
// These contain the gait position WITHOUT balance corrections.
//
// This distinction is important.
//
// The gait changes these targets.
// The balance controller corrects around them.
//
// Balance corrections therefore do not accumulate from step
// to step.
// ------------------------------------------------------------

FootTarget plannedFrontLeft = STANDING_FRONT_LEFT;
FootTarget plannedFrontRight = STANDING_FRONT_RIGHT;
FootTarget plannedBackLeft = STANDING_BACK_LEFT;
FootTarget plannedBackRight = STANDING_BACK_RIGHT;


// ------------------------------------------------------------
// BALANCED TARGETS
//
// These contain the most recent physical level stance.
//
// Support shifts begin from these targets.
// ------------------------------------------------------------

FootTarget balancedFrontLeft = STANDING_FRONT_LEFT;
FootTarget balancedFrontRight = STANDING_FRONT_RIGHT;
FootTarget balancedBackLeft = STANDING_BACK_LEFT;
FootTarget balancedBackRight = STANDING_BACK_RIGHT;


// ------------------------------------------------------------
// LEG IDENTIFIERS
// ------------------------------------------------------------

enum LegIndex {
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
};


// ------------------------------------------------------------
// BASIC TARGET FUNCTIONS
// ------------------------------------------------------------

FootTarget blendTarget(
  const FootTarget &startTarget,
  const FootTarget &endTarget,
  float weight
) {

  if (weight < 0.0f) {
    weight = 0.0f;
  }

  if (weight > 1.0f) {
    weight = 1.0f;
  }


  FootTarget target;


  target.x =
    startTarget.x +
    (endTarget.x - startTarget.x) *
    weight;


  target.y =
    startTarget.y +
    (endTarget.y - startTarget.y) *
    weight;


  target.z =
    startTarget.z +
    (endTarget.z - startTarget.z) *
    weight;


  return target;
}


// Sends four Cartesian targets to the IK controllers.
void commandFootTargets(
  const FootTarget &frontLeft,
  const FootTarget &frontRight,
  const FootTarget &backLeft,
  const FootTarget &backRight
) {

  frontLeftLeg.updateTarget(
    frontLeft.x,
    frontLeft.y,
    frontLeft.z
  );


  frontRightLeg.updateTarget(
    frontRight.x,
    frontRight.y,
    frontRight.z
  );


  backLeftLeg.updateTarget(
    backLeft.x,
    backLeft.y,
    backLeft.z
  );


  backRightLeg.updateTarget(
    backRight.x,
    backRight.y,
    backRight.z
  );
}


// Commands the original standing position.
void commandStandingPosition() {

  commandFootTargets(
    STANDING_FRONT_LEFT,
    STANDING_FRONT_RIGHT,
    STANDING_BACK_LEFT,
    STANDING_BACK_RIGHT
  );
}


// Commands the current balanced walking stance.
void commandBalancedPosition() {

  commandFootTargets(
    balancedFrontLeft,
    balancedFrontRight,
    balancedBackLeft,
    balancedBackRight
  );
}


// ------------------------------------------------------------
// IMU UPDATES
// ------------------------------------------------------------

void updateIMU() {

  if (imuReady) {
    orientation.update();
  }
}


// Waits without allowing the IMU filter timing to become stale.
void waitWithIMU(
  unsigned long duration
) {

  unsigned long startTime =
    millis();


  while (
    millis() - startTime <
    duration
  ) {

    updateIMU();


    delay(
      CONTROL_PERIOD_MS
    );
  }
}


// ------------------------------------------------------------
// SMOOTH BODY MOVEMENT
// ------------------------------------------------------------

double smoothBodyShift(
  double value
) {

  return
    10.0 * pow(value, 3) -
    15.0 * pow(value, 4) +
    6.0 * pow(value, 5);
}


// ------------------------------------------------------------
// SUPPORT SHIFT TARGETS
// ------------------------------------------------------------

void getShiftedTargets(
  double bodyShiftX,
  double bodyShiftZ,
  FootTarget &frontLeft,
  FootTarget &frontRight,
  FootTarget &backLeft,
  FootTarget &backRight
) {

  // Start from the CURRENT balanced stance.
  frontLeft =
    balancedFrontLeft;

  frontRight =
    balancedFrontRight;

  backLeft =
    balancedBackLeft;

  backRight =
    balancedBackRight;


  // Front/back body translation.
  frontLeft.x +=
    bodyShiftX;

  frontRight.x +=
    bodyShiftX;

  backLeft.x +=
    bodyShiftX;

  backRight.x +=
    bodyShiftX;


  // Left/right body translation.
  frontLeft.z -=
    bodyShiftZ;

  backLeft.z -=
    bodyShiftZ;


  frontRight.z +=
    bodyShiftZ;

  backRight.z +=
    bodyShiftZ;
}


// Commands one support-shift position.
void commandBodyShift(
  double bodyShiftX,
  double bodyShiftZ
) {

  FootTarget frontLeft;
  FootTarget frontRight;
  FootTarget backLeft;
  FootTarget backRight;


  getShiftedTargets(
    bodyShiftX,
    bodyShiftZ,
    frontLeft,
    frontRight,
    backLeft,
    backRight
  );


  commandFootTargets(
    frontLeft,
    frontRight,
    backLeft,
    backRight
  );
}


// Smoothly moves the body while all four feet remain down.
void moveBodyShift(
  double startX,
  double startZ,
  double endX,
  double endZ
) {

  const int steps =
    100;


  for (
    int i = 0;
    i <= steps;
    i++
  ) {

    double progress =
      (double)i /
      steps;


    double r =
      smoothBodyShift(
        progress
      );


    double bodyShiftX =
      startX +
      (endX - startX) *
      r;


    double bodyShiftZ =
      startZ +
      (endZ - startZ) *
      r;


    commandBodyShift(
      bodyShiftX,
      bodyShiftZ
    );


    updateIMU();


    delay(
      BODY_SHIFT_TIME /
      steps
    );
  }
}


// ------------------------------------------------------------
// LEG-SPECIFIC SUPPORT SHIFTS
// ------------------------------------------------------------

void getBodyShiftForLeg(
  LegIndex leg,
  double &bodyShiftX,
  double &bodyShiftZ
) {

  if (leg == FRONT_LEFT) {

    bodyShiftX =
      FRONT_LEFT_BODY_SHIFT_X;

    bodyShiftZ =
      FRONT_LEFT_BODY_SHIFT_Z;
  }


  else if (leg == BACK_RIGHT) {

    bodyShiftX =
      BACK_RIGHT_BODY_SHIFT_X;

    bodyShiftZ =
      BACK_RIGHT_BODY_SHIFT_Z;
  }


  else if (leg == FRONT_RIGHT) {

    bodyShiftX =
      FRONT_RIGHT_BODY_SHIFT_X;

    bodyShiftZ =
      FRONT_RIGHT_BODY_SHIFT_Z;
  }


  else {

    bodyShiftX =
      BACK_LEFT_BODY_SHIFT_X;

    bodyShiftZ =
      BACK_LEFT_BODY_SHIFT_Z;
  }
}


// ------------------------------------------------------------
// MODIFY ONE LEG TARGET
// ------------------------------------------------------------

void addXToLeg(
  LegIndex leg,
  double amount
) {

  if (leg == FRONT_LEFT) {

    plannedFrontLeft.x +=
      amount;

    balancedFrontLeft.x +=
      amount;
  }


  else if (leg == FRONT_RIGHT) {

    plannedFrontRight.x +=
      amount;

    balancedFrontRight.x +=
      amount;
  }


  else if (leg == BACK_LEFT) {

    plannedBackLeft.x +=
      amount;

    balancedBackLeft.x +=
      amount;
  }


  else {

    plannedBackRight.x +=
      amount;

    balancedBackRight.x +=
      amount;
  }
}


// ------------------------------------------------------------
// COMMAND ONE GAIT-SWING FRAME
// ------------------------------------------------------------

void commandSwingFrame(
  LegIndex leg,
  double bodyShiftX,
  double bodyShiftZ,
  double gaitDeltaX,
  double gaitDeltaY
) {

  FootTarget frontLeft;
  FootTarget frontRight;
  FootTarget backLeft;
  FootTarget backRight;


  // Start with the three-leg support pose.
  getShiftedTargets(
    bodyShiftX,
    bodyShiftZ,
    frontLeft,
    frontRight,
    backLeft,
    backRight
  );


  // Apply the gait displacement ONLY to the swing leg.
  if (leg == FRONT_LEFT) {

    frontLeft.x +=
      gaitDeltaX;

    frontLeft.y +=
      gaitDeltaY;
  }


  else if (leg == FRONT_RIGHT) {

    frontRight.x +=
      gaitDeltaX;

    frontRight.y +=
      gaitDeltaY;
  }


  else if (leg == BACK_LEFT) {

    backLeft.x +=
      gaitDeltaX;

    backLeft.y +=
      gaitDeltaY;
  }


  else {

    backRight.x +=
      gaitDeltaX;

    backRight.y +=
      gaitDeltaY;
  }


  commandFootTargets(
    frontLeft,
    frontRight,
    backLeft,
    backRight
  );
}


// ------------------------------------------------------------
// GAIT SWING
//
// This replaces the old straight-up leg lift.
//
// The three support legs DO NOT move during this phase.
// The balance controller is also disabled.
//
// The selected foot:
//
// 1. starts at its current location
// 2. rises
// 3. travels forward
// 4. lowers onto the ground one stride ahead
// ------------------------------------------------------------

void performSwing(
  LegIndex leg,
  double bodyShiftX,
  double bodyShiftZ
) {

  unsigned long swingStartTime =
    millis();


  // ----------------------------------------------------------
  // IMPORTANT GAITCYCLE DETAIL
  //
  // GaitCycle normally ramps its amplitude during its first
  // full cycle.
  //
  // We deliberately place its virtual start one full cycle in
  // the past so the swing uses the full stride and full 8 cm
  // height immediately.
  // ----------------------------------------------------------

  unsigned long virtualStartTime =
    swingStartTime -
    (unsigned long)GAIT_REFERENCE_CYCLE_TIME;


  swingGait.resetCycle(
    virtualStartTime
  );


  // At phase = 0, the original GaitCycle swing starts at
  // +stride/2.
  //
  // We subtract this value so that THIS swing begins with
  // zero displacement from the current foot position.
  const double gaitStartX =
    GAIT_STRIDE_LENGTH /
    2.0;


  while (true) {

    unsigned long currentTime =
      millis();


    unsigned long elapsed =
      currentTime -
      swingStartTime;


    if (
      elapsed >
      GAIT_SWING_TIME
    ) {

      elapsed =
        GAIT_SWING_TIME;
    }


    unsigned long sampleTime =
      swingStartTime +
      elapsed;


    double gaitX;
    double gaitY;
    double gaitZ;


    swingGait.getTarget(
      sampleTime,
      gaitX,
      gaitY,
      gaitZ
    );


    // Converts the GaitCycle absolute X output into a
    // displacement from this leg's CURRENT position.
    double gaitDeltaX =
      gaitX -
      gaitStartX;


    // initialY was set to zero, so gaitY itself is the
    // vertical displacement.
    //
    // Negative Y raises the foot.
    double gaitDeltaY =
      gaitY;


    commandSwingFrame(
      leg,
      bodyShiftX,
      bodyShiftZ,
      gaitDeltaX,
      gaitDeltaY
    );


    updateIMU();


    if (
      elapsed >=
      GAIT_SWING_TIME
    ) {

      break;
    }


    delay(
      CONTROL_PERIOD_MS
    );
  }


  // ----------------------------------------------------------
  // TOUCHDOWN
  //
  // The swing begins at zero X displacement and ends one
  // complete stride forward.
  //
  // In the robot's local coordinates:
  //
  // negative X = forward
  // ----------------------------------------------------------

  commandSwingFrame(
    leg,
    bodyShiftX,
    bodyShiftZ,
    -GAIT_STRIDE_LENGTH,
    0.0
  );


  // Save the new foot location.
  //
  // This is critical. Without this, returning the body shift
  // to center would drag the foot back to its old position.
  addXToLeg(
    leg,
    -GAIT_STRIDE_LENGTH
  );


  // Re-command the shifted stance using the NEW landed foot
  // location.
  commandBodyShift(
    bodyShiftX,
    bodyShiftZ
  );


  waitWithIMU(
    TOUCHDOWN_SETTLE_TIME
  );
}


// ------------------------------------------------------------
// SIT TO STAND
// ------------------------------------------------------------

void runSitToStand() {

  while (
    !frontLeftStand.isFinished() ||
    !frontRightStand.isFinished() ||
    !backLeftStand.isFinished() ||
    !backRightStand.isFinished()
  ) {

    frontLeftStand.updateTransition();
    frontRightStand.updateTransition();
    backLeftStand.updateTransition();
    backRightStand.updateTransition();
  }


  commandStandingPosition();
}


// ------------------------------------------------------------
// INITIALIZE IMU
// ------------------------------------------------------------

bool initializeIMU() {

  orientation.setAccelerometerCalibration(
    ACCEL_X_OFFSET,
    ACCEL_Y_OFFSET,
    ACCEL_Z_OFFSET,
    ACCEL_X_SCALE,
    ACCEL_Y_SCALE,
    ACCEL_Z_SCALE
  );


  orientation.setGyroscopeCalibration(
    GYRO_X_OFFSET,
    GYRO_Y_OFFSET,
    GYRO_Z_OFFSET
  );


  orientation.setComplementaryAlpha(
    COMPLEMENTARY_ALPHA
  );


  imuReady =
    orientation.begin();


  return imuReady;
}


// ------------------------------------------------------------
// IMU SETTLING
// ------------------------------------------------------------

void settleIMU() {

  unsigned long startTime =
    millis();


  while (
    millis() - startTime <
    IMU_SETTLE_TIME
  ) {

    orientation.update();


    commandStandingPosition();


    delay(
      CONTROL_PERIOD_MS
    );
  }
}


// ------------------------------------------------------------
// BALANCE CONFIGURATION
// ------------------------------------------------------------

void configureBalanceController() {

  balance.setGeometry(
    BODY_LENGTH,
    BODY_WIDTH
  );


  balance.setRotationCenterOffset(
    COM_OFFSET_X,
    COM_OFFSET_Z
  );


  balance.setTargetAngles(
    TARGET_ROLL,
    TARGET_PITCH
  );


  balance.setCorrectionSigns(
    ROLL_CORRECTION_SIGN,
    PITCH_CORRECTION_SIGN
  );


  balance.setMaximumCorrectionAngles(
    12.0f,
    12.0f
  );


  balance.setMaximumSafeAngle(
    60.0f
  );


  balance.setDeadband(
    BALANCE_DEADBAND
  );


  balance.setOutputFilterAlpha(
    BALANCE_OUTPUT_FILTER_ALPHA
  );


  balance.reset();


  balance.setEnabled(
    true
  );
}


// ------------------------------------------------------------
// BALANCE CURRENT FOUR-FOOT STANCE
//
// Balance is ONLY enabled inside this function.
//
// During:
// - support shifting
// - swing
// - touchdown
//
// the balance controller is disabled.
//
// The controller corrects around the PLANNED gait positions,
// not around the previous balance correction.
// ------------------------------------------------------------

bool balanceCurrentPlan(
  unsigned long maximumBalanceTime,
  unsigned long requiredLevelTime,
  unsigned long blendTime
) {

  FootTarget startingFrontLeft =
    balancedFrontLeft;

  FootTarget startingFrontRight =
    balancedFrontRight;

  FootTarget startingBackLeft =
    balancedBackLeft;

  FootTarget startingBackRight =
    balancedBackRight;


  configureBalanceController();


  unsigned long balanceStartTime =
    millis();


  unsigned long levelStartTime =
    0;


  int levelSampleCount =
    0;


  double flX = 0.0;
  double flY = 0.0;
  double flZ = 0.0;


  double frX = 0.0;
  double frY = 0.0;
  double frZ = 0.0;


  double blX = 0.0;
  double blY = 0.0;
  double blZ = 0.0;


  double brX = 0.0;
  double brY = 0.0;
  double brZ = 0.0;


  FootTarget smoothedFrontLeft =
    startingFrontLeft;

  FootTarget smoothedFrontRight =
    startingFrontRight;

  FootTarget smoothedBackLeft =
    startingBackLeft;

  FootTarget smoothedBackRight =
    startingBackRight;


  bool haveSmoothedPose =
    false;


  bool stablePoseFound =
    false;


  while (
    millis() - balanceStartTime <
    maximumBalanceTime
  ) {

    unsigned long loopStartTime =
      millis();


    bool validReading =
      orientation.update();


    if (validReading) {

      BalanceOutput output =
        balance.update(
          orientation.getRoll(),
          orientation.getPitch(),
          orientation.getRollRate(),
          orientation.getPitchRate(),
          plannedFrontLeft,
          plannedFrontRight,
          plannedBackLeft,
          plannedBackRight
        );


      if (
        output.safetyStopped
      ) {

        balance.setEnabled(
          false
        );


        return false;
      }


      float blendWeight =
        1.0f;


      if (
        blendTime >
        0
      ) {

        blendWeight =
          (float)(
            millis() -
            balanceStartTime
          ) /
          (float)blendTime;


        if (
          blendWeight >
          1.0f
        ) {

          blendWeight =
            1.0f;
        }
      }


      FootTarget commandedFrontLeft =
        blendTarget(
          startingFrontLeft,
          output.frontLeft,
          blendWeight
        );


      FootTarget commandedFrontRight =
        blendTarget(
          startingFrontRight,
          output.frontRight,
          blendWeight
        );


      FootTarget commandedBackLeft =
        blendTarget(
          startingBackLeft,
          output.backLeft,
          blendWeight
        );


      FootTarget commandedBackRight =
        blendTarget(
          startingBackRight,
          output.backRight,
          blendWeight
        );


      commandFootTargets(
        commandedFrontLeft,
        commandedFrontRight,
        commandedBackLeft,
        commandedBackRight
      );


      // ------------------------------------------------------
      // SMOOTH FALLBACK POSE
      // ------------------------------------------------------

      if (
        !haveSmoothedPose
      ) {

        smoothedFrontLeft =
          commandedFrontLeft;

        smoothedFrontRight =
          commandedFrontRight;

        smoothedBackLeft =
          commandedBackLeft;

        smoothedBackRight =
          commandedBackRight;


        haveSmoothedPose =
          true;
      }

      else {

        const float poseSmoothing =
          0.15f;


        smoothedFrontLeft =
          blendTarget(
            smoothedFrontLeft,
            commandedFrontLeft,
            poseSmoothing
          );


        smoothedFrontRight =
          blendTarget(
            smoothedFrontRight,
            commandedFrontRight,
            poseSmoothing
          );


        smoothedBackLeft =
          blendTarget(
            smoothedBackLeft,
            commandedBackLeft,
            poseSmoothing
          );


        smoothedBackRight =
          blendTarget(
            smoothedBackRight,
            commandedBackRight,
            poseSmoothing
          );
      }


      float roll =
        orientation.getRoll();


      float pitch =
        orientation.getPitch();


      bool blendFinished =
        blendWeight >=
        0.999f;


      if (
        blendFinished &&
        fabs(roll) <= LEVEL_TOLERANCE &&
        fabs(pitch) <= LEVEL_TOLERANCE
      ) {

        if (
          levelStartTime ==
          0
        ) {

          levelStartTime =
            millis();


          levelSampleCount =
            0;


          flX = 0.0;
          flY = 0.0;
          flZ = 0.0;


          frX = 0.0;
          frY = 0.0;
          frZ = 0.0;


          blX = 0.0;
          blY = 0.0;
          blZ = 0.0;


          brX = 0.0;
          brY = 0.0;
          brZ = 0.0;
        }


        flX += commandedFrontLeft.x;
        flY += commandedFrontLeft.y;
        flZ += commandedFrontLeft.z;


        frX += commandedFrontRight.x;
        frY += commandedFrontRight.y;
        frZ += commandedFrontRight.z;


        blX += commandedBackLeft.x;
        blY += commandedBackLeft.y;
        blZ += commandedBackLeft.z;


        brX += commandedBackRight.x;
        brY += commandedBackRight.y;
        brZ += commandedBackRight.z;


        levelSampleCount++;


        if (
          millis() -
          levelStartTime >=
          requiredLevelTime
        ) {

          if (
            levelSampleCount >
            0
          ) {

            balancedFrontLeft = {
              (float)(flX / levelSampleCount),
              (float)(flY / levelSampleCount),
              (float)(flZ / levelSampleCount)
            };


            balancedFrontRight = {
              (float)(frX / levelSampleCount),
              (float)(frY / levelSampleCount),
              (float)(frZ / levelSampleCount)
            };


            balancedBackLeft = {
              (float)(blX / levelSampleCount),
              (float)(blY / levelSampleCount),
              (float)(blZ / levelSampleCount)
            };


            balancedBackRight = {
              (float)(brX / levelSampleCount),
              (float)(brY / levelSampleCount),
              (float)(brZ / levelSampleCount)
            };


            stablePoseFound =
              true;
          }


          break;
        }
      }

      else {

        levelStartTime =
          0;


        levelSampleCount =
          0;
      }
    }


    unsigned long elapsedTime =
      millis() -
      loopStartTime;


    if (
      elapsedTime <
      CONTROL_PERIOD_MS
    ) {

      delay(
        CONTROL_PERIOD_MS -
        elapsedTime
      );
    }
  }


  // If the exact level condition was not achieved,
  // keep a smooth corrected pose instead.
  if (
    !stablePoseFound &&
    haveSmoothedPose
  ) {

    balancedFrontLeft =
      smoothedFrontLeft;

    balancedFrontRight =
      smoothedFrontRight;

    balancedBackLeft =
      smoothedBackLeft;

    balancedBackRight =
      smoothedBackRight;
  }


  // ----------------------------------------------------------
  // BALANCE IS DISABLED HERE.
  //
  // Nothing in the following swing phase can change the
  // balance controller output.
  // ----------------------------------------------------------

  balance.setEnabled(
    false
  );


  commandBalancedPosition();


  return true;
}


// ------------------------------------------------------------
// INITIAL BALANCE
// ------------------------------------------------------------

bool initialBalance() {

  return balanceCurrentPlan(
    MAX_BALANCE_TIME,
    LEVEL_HOLD_TIME,
    0
  );
}


// ------------------------------------------------------------
// REBALANCE
// ------------------------------------------------------------

bool rebalanceRobot() {

  waitWithIMU(
    REBALANCE_SETTLE_TIME
  );


  return balanceCurrentPlan(
    MAX_REBALANCE_TIME,
    REBALANCE_LEVEL_HOLD_TIME,
    REBALANCE_BLEND_TIME
  );
}


// ------------------------------------------------------------
// COMPLETE SINGLE CRAWL STEP
//
// This is the core walking operation.
//
// 1. Robot begins balanced with four feet down.
// 2. Shift COM into support triangle.
// 3. Stop active balancing.
// 4. Swing one leg forward.
// 5. Foot touches ground.
// 6. Return body shift to center.
// 7. Rebalance with four feet down.
// ------------------------------------------------------------

bool runCrawlStep(
  LegIndex leg
) {

  double bodyShiftX;
  double bodyShiftZ;


  getBodyShiftForLeg(
    leg,
    bodyShiftX,
    bodyShiftZ
  );


  // ----------------------------------------------------------
  // 1. SHIFT COM
  // ----------------------------------------------------------

  moveBodyShift(
    0.0,
    0.0,
    bodyShiftX,
    bodyShiftZ
  );


  commandBodyShift(
    bodyShiftX,
    bodyShiftZ
  );


  waitWithIMU(
    SHIFT_SETTLE_TIME
  );


  // ----------------------------------------------------------
  // 2. SWING LEG
  //
  // NO ACTIVE BALANCE OCCURS HERE.
  // ----------------------------------------------------------

  performSwing(
    leg,
    bodyShiftX,
    bodyShiftZ
  );


  // ----------------------------------------------------------
  // 3. RETURN BODY SHIFT TO CENTER
  //
  // All four feet are now on the ground.
  // ----------------------------------------------------------

  moveBodyShift(
    bodyShiftX,
    bodyShiftZ,
    0.0,
    0.0
  );


  commandBalancedPosition();


  // ----------------------------------------------------------
  // 4. REBALANCE WITH FOUR FEET DOWN
  // ----------------------------------------------------------

  if (
    !rebalanceRobot()
  ) {

    return false;
  }


  waitWithIMU(
    WAIT_AFTER_REBALANCE
  );


  return true;
}


// ------------------------------------------------------------
// BODY ADVANCE
//
// After all four legs have stepped forward, every planned
// foot X target is approximately:
//
//     original X - GAIT_STRIDE_LENGTH
//
// All four feet are now planted.
//
// Increasing all four local X coordinates by one stride moves
// the feet backward relative to the body.
//
// Because the feet are planted, the physical body moves forward.
//
// At the end, the leg coordinates return near their original
// X values and the next crawl cycle can begin.
// ------------------------------------------------------------

void advanceBodyAfterCycle() {

  const int steps =
    120;


  FootTarget startFrontLeft =
    balancedFrontLeft;

  FootTarget startFrontRight =
    balancedFrontRight;

  FootTarget startBackLeft =
    balancedBackLeft;

  FootTarget startBackRight =
    balancedBackRight;


  for (
    int i = 0;
    i <= steps;
    i++
  ) {

    double progress =
      (double)i /
      steps;


    double r =
      smoothBodyShift(
        progress
      );


    double xAdvance =
      GAIT_STRIDE_LENGTH *
      r;


    FootTarget frontLeft =
      startFrontLeft;

    FootTarget frontRight =
      startFrontRight;

    FootTarget backLeft =
      startBackLeft;

    FootTarget backRight =
      startBackRight;


    frontLeft.x +=
      xAdvance;

    frontRight.x +=
      xAdvance;

    backLeft.x +=
      xAdvance;

    backRight.x +=
      xAdvance;


    commandFootTargets(
      frontLeft,
      frontRight,
      backLeft,
      backRight
    );


    updateIMU();


    delay(
      BODY_ADVANCE_TIME /
      steps
    );
  }


  // Save the new planned positions.
  plannedFrontLeft.x +=
    GAIT_STRIDE_LENGTH;

  plannedFrontRight.x +=
    GAIT_STRIDE_LENGTH;

  plannedBackLeft.x +=
    GAIT_STRIDE_LENGTH;

  plannedBackRight.x +=
    GAIT_STRIDE_LENGTH;


  // Save the corresponding balanced positions.
  balancedFrontLeft.x +=
    GAIT_STRIDE_LENGTH;

  balancedFrontRight.x +=
    GAIT_STRIDE_LENGTH;

  balancedBackLeft.x +=
    GAIT_STRIDE_LENGTH;

  balancedBackRight.x +=
    GAIT_STRIDE_LENGTH;


  commandBalancedPosition();
}


// ------------------------------------------------------------
// EMERGENCY HOLD
// ------------------------------------------------------------

void haltRobot() {

  balance.setEnabled(
    false
  );


  commandBalancedPosition();


  while (true) {

    updateIMU();


    delay(
      CONTROL_PERIOD_MS
    );
  }
}


// ------------------------------------------------------------
// ONE COMPLETE CRAWL CYCLE
//
// Order:
//
// FL -> BR -> FR -> BL
// ------------------------------------------------------------

bool runCrawlCycle() {

  if (
    !runCrawlStep(
      FRONT_LEFT
    )
  ) {

    return false;
  }


  if (
    !runCrawlStep(
      BACK_RIGHT
    )
  ) {

    return false;
  }


  if (
    !runCrawlStep(
      FRONT_RIGHT
    )
  ) {

    return false;
  }


  if (
    !runCrawlStep(
      BACK_LEFT
    )
  ) {

    return false;
  }


  // ----------------------------------------------------------
  // ALL FOUR FEET HAVE NOW STEPPED FORWARD.
  //
  // Move the chassis forward while all four feet are planted.
  // ----------------------------------------------------------

  advanceBodyAfterCycle();


  // Re-level the robot after the body translation.
  if (
    !rebalanceRobot()
  ) {

    return false;
  }


  waitWithIMU(
    WAIT_AFTER_REBALANCE
  );


  return true;
}


// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup() {

  // Initialize all twelve servos.
  frontLeftLeg.begin();
  backLeftLeg.begin();
  frontRightLeg.begin();
  backRightLeg.begin();


  // ----------------------------------------------------------
  // 1. SIT TO STAND
  // ----------------------------------------------------------

  runSitToStand();


  // ----------------------------------------------------------
  // 2. INITIALIZE IMU
  // ----------------------------------------------------------

  if (
    !initializeIMU()
  ) {

    commandStandingPosition();


    while (true) {

      delay(
        1000
      );
    }
  }


  // ----------------------------------------------------------
  // 3. IMU SETTLING
  // ----------------------------------------------------------

  settleIMU();


  // ----------------------------------------------------------
  // 4. INITIAL BALANCE
  // ----------------------------------------------------------

  if (
    !initialBalance()
  ) {

    haltRobot();
  }


  waitWithIMU(
    WAIT_AFTER_INITIAL_BALANCE
  );


  // ----------------------------------------------------------
  // 5. WALK
  //
  // For the first test this runs one complete crawl cycle.
  // ----------------------------------------------------------

  for (
    int cycle = 0;
    cycle < CRAWL_CYCLES_TO_RUN;
    cycle++
  ) {

    if (
      !runCrawlCycle()
    ) {

      haltRobot();
    }
  }


  // ----------------------------------------------------------
  // 6. FINAL BALANCED HOLD
  // ----------------------------------------------------------

  commandBalancedPosition();
}


// ------------------------------------------------------------
// LOOP
//
// The first version deliberately runs only the requested number
// of gait cycles and then holds the robot.
//
// Once one full crawl cycle works reliably, this can easily be
// changed to continuous walking.
// ------------------------------------------------------------

void loop() {

  updateIMU();


  delay(
    CONTROL_PERIOD_MS
  );
}