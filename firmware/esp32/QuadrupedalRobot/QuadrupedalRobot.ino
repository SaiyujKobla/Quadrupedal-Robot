#include "InverseKinematics.h"
#include "IMUOrientation.h"
#include "BalanceController.h"
#include "SitToStand.h"
#include "DanceController.h"

#include <math.h>

// ============================================================
// LEGS
// ============================================================

InverseKinematics frontLeftLeg(17, 19, 18, 0);
InverseKinematics backLeftLeg(23, 21, 22, 1);
InverseKinematics frontRightLeg(13, 27, 14, 2);
InverseKinematics backRightLeg(32, 26, 25, 3);

// ============================================================
// IMU
// ============================================================

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

// ============================================================
// ROBOT POSITIONS
// ============================================================

const float SIT_X = 0.0f;
const float SIT_Y = 8.5f;
const float SIT_Z = 0.0f;

const float STANDING_X = 0.0f;
const float STANDING_Y = 20.0f;
const float STANDING_Z = 0.0f;

// ============================================================
// BODY GEOMETRY / COM
// ============================================================

const float BODY_LENGTH = 20.0f;
const float BODY_WIDTH = 19.45f;

// Existing measured COM offset:
// +X = toward front
// +Z = toward robot's right
const float COM_OFFSET_X = 2.0f;
const float COM_OFFSET_Z = 0.0f;

// ============================================================
// BALANCE CONTROLLER SETTINGS
// ============================================================

const float TARGET_ROLL = 0.0f;
const float TARGET_PITCH = 0.0f;

const float ROLL_KP = 1.0f;
const float ROLL_KD = 0.0f;

const float PITCH_KP = 1.0f;
const float PITCH_KD = 0.0f;

const float ROLL_CORRECTION_SIGN = 1.0f;
const float PITCH_CORRECTION_SIGN = 1.0f;

const float LEVEL_TOLERANCE = 2.0f;

const unsigned long IMU_SETTLE_TIME = 3000;

const unsigned long LEVEL_HOLD_TIME = 200;
const unsigned long MAX_BALANCE_TIME = 2000;

const unsigned long REBALANCE_LEVEL_HOLD_TIME = 150;
const unsigned long MAX_REBALANCE_TIME = 1000;
const unsigned long REBALANCE_BLEND_TIME = 250;
const unsigned long REBALANCE_SETTLE_TIME = 100;

const unsigned long CONTROL_PERIOD_MS = 10;

const float BALANCE_OUTPUT_FILTER_ALPHA = 0.25f;
const float BALANCE_DEADBAND = 0.5f;

// ============================================================
// ROUTINE TIMING
// ============================================================

// Requested pause immediately after sit-to-stand.
const unsigned long WAIT_AFTER_STAND_MS = 2000;

// "Pause for a bit" after the initial balance.
const unsigned long WAIT_AFTER_INITIAL_BALANCE_MS = 1000;

// Small pause after each real rebalance before beginning the next
// scripted section.
const unsigned long WAIT_AFTER_SECTION_REBALANCE_MS = 800;

// After a real IMU rebalance, return smoothly to the exact neutral
// leg targets captured after the FIRST successful balance. This
// prevents the dance neutral from drifting across repetitions.
const unsigned long RETURN_TO_FIXED_NEUTRAL_MS = 500;

// Pause after the entire routine before it starts again.
const unsigned long WAIT_BEFORE_FULL_REPEAT_MS = 1000;

// ============================================================
// CONTROLLERS
// ============================================================

BalanceController balance(
  BODY_LENGTH,
  BODY_WIDTH,
  ROLL_KP,
  ROLL_KD,
  PITCH_KP,
  PITCH_KD
);

DanceController dance(
  BODY_LENGTH,
  BODY_WIDTH,
  COM_OFFSET_X,
  COM_OFFSET_Z
);

// ============================================================
// SIT TO STAND
// ============================================================

SitToStand frontLeftStand(
  frontLeftLeg,
  SIT_X,
  SIT_Y,
  SIT_Z,
  STANDING_X,
  STANDING_Y,
  STANDING_Z
);

SitToStand frontRightStand(
  frontRightLeg,
  SIT_X,
  SIT_Y,
  SIT_Z,
  STANDING_X,
  STANDING_Y,
  STANDING_Z
);

SitToStand backLeftStand(
  backLeftLeg,
  SIT_X,
  SIT_Y,
  SIT_Z,
  STANDING_X,
  STANDING_Y,
  STANDING_Z
);

SitToStand backRightStand(
  backRightLeg,
  SIT_X,
  SIT_Y,
  SIT_Z,
  STANDING_X,
  STANDING_Y,
  STANDING_Z
);

// ============================================================
// STANDING / BALANCED TARGETS
// ============================================================

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

// During the FIRST balance these begin at the geometric standing
// pose. After that first balance succeeds, they are locked to the
// exact initial balanced pose and are never allowed to drift.
FootTarget plannedFrontLeft = STANDING_FRONT_LEFT;
FootTarget plannedFrontRight = STANDING_FRONT_RIGHT;
FootTarget plannedBackLeft = STANDING_BACK_LEFT;
FootTarget plannedBackRight = STANDING_BACK_RIGHT;

// Working balance targets. A rebalance may temporarily update these,
// but after the rebalance they are smoothly restored to the fixed
// dance-neutral targets below.
FootTarget balancedFrontLeft = STANDING_FRONT_LEFT;
FootTarget balancedFrontRight = STANDING_FRONT_RIGHT;
FootTarget balancedBackLeft = STANDING_BACK_LEFT;
FootTarget balancedBackRight = STANDING_BACK_RIGHT;

// IMMUTABLE DANCE NEUTRAL
//
// These are captured exactly once, immediately after the initial
// active balance. Every pitch/roll and square sequence is centered on
// these same four targets for the entire power-on session.
FootTarget danceNeutralFrontLeft = STANDING_FRONT_LEFT;
FootTarget danceNeutralFrontRight = STANDING_FRONT_RIGHT;
FootTarget danceNeutralBackLeft = STANDING_BACK_LEFT;
FootTarget danceNeutralBackRight = STANDING_BACK_RIGHT;

bool danceNeutralLocked = false;

// ============================================================
// FIXED DANCE-NEUTRAL HELPERS
// ============================================================

// Capture the exact pose produced by the FIRST successful active
// balance. This becomes the permanent neutral reference for every
// dance iteration until the robot is power-cycled/reset.
void lockDanceNeutralPose() {
  danceNeutralFrontLeft = balancedFrontLeft;
  danceNeutralFrontRight = balancedFrontRight;
  danceNeutralBackLeft = balancedBackLeft;
  danceNeutralBackRight = balancedBackRight;

  // From this point onward, active rebalancing also computes its
  // corrections around this fixed initial balanced stance instead of
  // around the raw geometric X=0, Y=20, Z=0 standing pose.
  plannedFrontLeft = danceNeutralFrontLeft;
  plannedFrontRight = danceNeutralFrontRight;
  plannedBackLeft = danceNeutralBackLeft;
  plannedBackRight = danceNeutralBackRight;

  danceNeutralLocked = true;
}

// Same seventh-order smootherstep already used elsewhere in the
// project. It lets the legs return to the fixed neutral without a snap.
float smoothNeutralReturnProgress(float progress) {
  if (progress < 0.0f) {
    progress = 0.0f;
  }

  if (progress > 1.0f) {
    progress = 1.0f;
  }

  float p2 = progress * progress;
  float p4 = p2 * p2;
  float p5 = p4 * progress;
  float p6 = p5 * progress;
  float p7 = p6 * progress;

  return
    35.0f * p4
    - 84.0f * p5
    + 70.0f * p6
    - 20.0f * p7;
}

// Smoothly move from whatever temporary correction pose the active
// rebalance ended on back to the exact initial balanced stance.
void returnToFixedDanceNeutral() {
  if (!danceNeutralLocked) {
    return;
  }

  FootTarget startFrontLeft = balancedFrontLeft;
  FootTarget startFrontRight = balancedFrontRight;
  FootTarget startBackLeft = balancedBackLeft;
  FootTarget startBackRight = balancedBackRight;

  unsigned long startTime = millis();

  while (true) {
    unsigned long elapsed = millis() - startTime;

    if (elapsed > RETURN_TO_FIXED_NEUTRAL_MS) {
      elapsed = RETURN_TO_FIXED_NEUTRAL_MS;
    }

    float progress = 1.0f;

    if (RETURN_TO_FIXED_NEUTRAL_MS > 0) {
      progress =
        (float)elapsed
        / (float)RETURN_TO_FIXED_NEUTRAL_MS;
    }

    float weight =
      smoothNeutralReturnProgress(progress);

    FootTarget frontLeft =
      blendTarget(
        startFrontLeft,
        danceNeutralFrontLeft,
        weight
      );

    FootTarget frontRight =
      blendTarget(
        startFrontRight,
        danceNeutralFrontRight,
        weight
      );

    FootTarget backLeft =
      blendTarget(
        startBackLeft,
        danceNeutralBackLeft,
        weight
      );

    FootTarget backRight =
      blendTarget(
        startBackRight,
        danceNeutralBackRight,
        weight
      );

    commandFootTargets(
      frontLeft,
      frontRight,
      backLeft,
      backRight
    );

    updateIMU();

    if (elapsed >= RETURN_TO_FIXED_NEUTRAL_MS) {
      break;
    }

    delay(CONTROL_PERIOD_MS);
  }

  // Reset the working state as well, so the next rebalance starts from
  // exactly the same neutral instead of inheriting the previous one's
  // small correction offset.
  balancedFrontLeft = danceNeutralFrontLeft;
  balancedFrontRight = danceNeutralFrontRight;
  balancedBackLeft = danceNeutralBackLeft;
  balancedBackRight = danceNeutralBackRight;

  commandFootTargets(
    danceNeutralFrontLeft,
    danceNeutralFrontRight,
    danceNeutralBackLeft,
    danceNeutralBackRight
  );
}

// ============================================================
// BASIC TARGET HELPERS
// ============================================================

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
    startTarget.x
    + (endTarget.x - startTarget.x) * weight;

  target.y =
    startTarget.y
    + (endTarget.y - startTarget.y) * weight;

  target.z =
    startTarget.z
    + (endTarget.z - startTarget.z) * weight;

  return target;
}

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

void commandStandingPosition() {
  commandFootTargets(
    STANDING_FRONT_LEFT,
    STANDING_FRONT_RIGHT,
    STANDING_BACK_LEFT,
    STANDING_BACK_RIGHT
  );
}

void commandBalancedPosition() {
  commandFootTargets(
    balancedFrontLeft,
    balancedFrontRight,
    balancedBackLeft,
    balancedBackRight
  );
}

void commandDanceOutput(
  const DanceOutput &output
) {
  commandFootTargets(
    output.frontLeft,
    output.frontRight,
    output.backLeft,
    output.backRight
  );
}

// ============================================================
// IMU HELPERS
// ============================================================

void updateIMU() {
  if (imuReady) {
    orientation.update();
  }
}

void waitWithIMU(
  unsigned long duration
) {
  unsigned long startTime = millis();

  while (millis() - startTime < duration) {
    updateIMU();
    delay(CONTROL_PERIOD_MS);
  }
}

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

  imuReady = orientation.begin();

  return imuReady;
}

void settleIMU() {
  unsigned long startTime = millis();

  while (millis() - startTime < IMU_SETTLE_TIME) {
    orientation.update();
    commandStandingPosition();
    delay(CONTROL_PERIOD_MS);
  }
}

// ============================================================
// SIT TO STAND
// ============================================================

void runSitToStand() {
  while (
    !frontLeftStand.isFinished()
    || !frontRightStand.isFinished()
    || !backLeftStand.isFinished()
    || !backRightStand.isFinished()
  ) {
    frontLeftStand.updateTransition();
    frontRightStand.updateTransition();
    backLeftStand.updateTransition();
    backRightStand.updateTransition();
  }

  commandStandingPosition();
}

// ============================================================
// ACTIVE BALANCE
// ============================================================
//
// This is your existing feedback balance algorithm.
//
// IMPORTANT:
// Active balancing is used ONLY:
//   1. after startup, before the scripted routine
//   2. after the pitch/roll section
//   3. after the square section
//
// DanceController itself never calls BalanceController::update().
// ============================================================

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
  balance.setEnabled(true);
}

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
    millis() - balanceStartTime
    < maximumBalanceTime
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

      if (output.safetyStopped) {
        balance.setEnabled(false);
        return false;
      }

      float blendWeight =
        1.0f;

      if (blendTime > 0) {
        blendWeight =
          (float)(millis() - balanceStartTime)
          / (float)blendTime;

        if (blendWeight > 1.0f) {
          blendWeight = 1.0f;
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

      if (!haveSmoothedPose) {
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
        blendWeight >= 0.999f;

      if (
        blendFinished
        && fabs(roll) <= LEVEL_TOLERANCE
        && fabs(pitch) <= LEVEL_TOLERANCE
      ) {
        if (levelStartTime == 0) {
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
          millis() - levelStartTime
          >= requiredLevelTime
        ) {
          if (levelSampleCount > 0) {
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
      millis() - loopStartTime;

    if (elapsedTime < CONTROL_PERIOD_MS) {
      delay(
        CONTROL_PERIOD_MS
        - elapsedTime
      );
    }
  }

  // Preserve the existing fallback behavior if the strict
  // level-hold condition was not reached before timeout.
  if (
    !stablePoseFound
    && haveSmoothedPose
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

  balance.setEnabled(false);

  commandBalancedPosition();

  return true;
}

bool initialBalance() {
  return balanceCurrentPlan(
    MAX_BALANCE_TIME,
    LEVEL_HOLD_TIME,
    0
  );
}

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

// ============================================================
// EMERGENCY HOLD
// ============================================================

void haltRobot() {
  balance.setEnabled(false);
  dance.forceNeutral();

  commandBalancedPosition();

  Serial.println(
    "HALT: balance safety stop or IMU failure."
  );

  while (true) {
    updateIMU();
    delay(CONTROL_PERIOD_MS);
  }
}

// ============================================================
// ROUTINE STATE MACHINE
// ============================================================
//
// Full repeating routine:
//
// sit -> stand
// wait 2 s
// IMU settle
// real balance
// pause
//
// pitch/roll sequence x2
// neutral
// real rebalance
// pause
//
// square x2
// neutral
// real rebalance
// pause
//
// repeat from pitch/roll section
// ============================================================

enum RoutineState {
  START_PITCH_ROLL,
  RUN_PITCH_ROLL,

  REBALANCE_AFTER_PITCH_ROLL,

  START_SQUARE,
  RUN_SQUARE,

  REBALANCE_AFTER_SQUARE
};

RoutineState routineState =
  START_PITCH_ROLL;

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  frontLeftLeg.begin();
  backLeftLeg.begin();
  frontRightLeg.begin();
  backRightLeg.begin();

  // ----------------------------------------------------------
  // 1. SIT -> STAND
  // ----------------------------------------------------------

  Serial.println(
    "1. Sit-to-stand."
  );

  runSitToStand();

  // ----------------------------------------------------------
  // 2. EXACT REQUESTED 2 SECOND PAUSE
  // ----------------------------------------------------------

  Serial.println(
    "2. Standing. Waiting 2 seconds."
  );

  delay(
    WAIT_AFTER_STAND_MS
  );

  // ----------------------------------------------------------
  // 3. INITIALIZE + SETTLE IMU
  // ----------------------------------------------------------

  Serial.println(
    "3. Initializing IMU."
  );

  if (!initializeIMU()) {
    haltRobot();
  }

  Serial.println(
    "IMU settling."
  );

  settleIMU();

  // ----------------------------------------------------------
  // 4. REAL ACTIVE BALANCE
  // ----------------------------------------------------------

  Serial.println(
    "4. Initial active balance."
  );

  if (!initialBalance()) {
    haltRobot();
  }

  // Capture the one true neutral pose. Do this ONCE only.
  lockDanceNeutralPose();

  // ----------------------------------------------------------
  // 5. PAUSE AFTER BALANCE
  // ----------------------------------------------------------

  Serial.println(
    "5. Balanced. Pausing before dance."
  );

  waitWithIMU(
    WAIT_AFTER_INITIAL_BALANCE_MS
  );

  // Dance is centered on the balanced physical stance.
  dance.begin(
    danceNeutralFrontLeft,
    danceNeutralFrontRight,
    danceNeutralBackLeft,
    danceNeutralBackRight
  );

  routineState =
    START_PITCH_ROLL;

  Serial.println(
    "NERC routine ready."
  );
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  unsigned long loopStartTime =
    millis();

  // Keep IMU orientation current during scripted choreography,
  // but DO NOT use it for active feedback during the dance.
  updateIMU();

  switch (routineState) {

    // --------------------------------------------------------
    // START PITCH / ROLL
    // --------------------------------------------------------

    case START_PITCH_ROLL: {
      Serial.println(
        "Pitch/roll section: start."
      );

      dance.startPitchRollSequence();

      routineState =
        RUN_PITCH_ROLL;

      break;
    }

    // --------------------------------------------------------
    // RUN PITCH / ROLL
    // --------------------------------------------------------

    case RUN_PITCH_ROLL: {
      DanceOutput output =
        dance.update();

      commandDanceOutput(
        output
      );

      if (dance.isFinished()) {
        Serial.println(
          "Pitch/roll section complete. Neutral reached."
        );

        routineState =
          REBALANCE_AFTER_PITCH_ROLL;
      }

      break;
    }

    // --------------------------------------------------------
    // REAL REBALANCE AFTER PITCH / ROLL
    // --------------------------------------------------------

    case REBALANCE_AFTER_PITCH_ROLL: {
      Serial.println(
        "Rebalancing after pitch/roll."
      );

      if (!rebalanceRobot()) {
        haltRobot();
      }

      // The rebalance is allowed to make temporary corrections,
      // but it must NOT redefine the dance neutral. Return the legs
      // to the exact initial balanced targets before the square starts.
      returnToFixedDanceNeutral();

      dance.begin(
        danceNeutralFrontLeft,
        danceNeutralFrontRight,
        danceNeutralBackLeft,
        danceNeutralBackRight
      );

      waitWithIMU(
        WAIT_AFTER_SECTION_REBALANCE_MS
      );

      routineState =
        START_SQUARE;

      break;
    }

    // --------------------------------------------------------
    // START SQUARE
    // --------------------------------------------------------

    case START_SQUARE: {
      Serial.println(
        "Square section: start."
      );

      dance.startSquareSequence();

      routineState =
        RUN_SQUARE;

      break;
    }

    // --------------------------------------------------------
    // RUN SQUARE
    // --------------------------------------------------------

    case RUN_SQUARE: {
      DanceOutput output =
        dance.update();

      commandDanceOutput(
        output
      );

      if (dance.isFinished()) {
        Serial.println(
          "Square section complete. Neutral reached."
        );

        routineState =
          REBALANCE_AFTER_SQUARE;
      }

      break;
    }

    // --------------------------------------------------------
    // REAL REBALANCE, THEN REPEAT WHOLE DANCE
    // --------------------------------------------------------

    case REBALANCE_AFTER_SQUARE: {
      Serial.println(
        "Rebalancing after square."
      );

      if (!rebalanceRobot()) {
        haltRobot();
      }

      // Again, discard any small temporary correction offset from
      // this rebalance and restore the same fixed neutral pose.
      returnToFixedDanceNeutral();

      dance.begin(
        danceNeutralFrontLeft,
        danceNeutralFrontRight,
        danceNeutralBackLeft,
        danceNeutralBackRight
      );

      Serial.println(
        "Full routine complete. Fixed neutral restored before repeat."
      );

      waitWithIMU(
        WAIT_BEFORE_FULL_REPEAT_MS
      );

      routineState =
        START_PITCH_ROLL;

      break;
    }
  }

  // Maintain the existing 10 ms control period whenever the
  // current state did not execute a blocking rebalance/wait.
  unsigned long elapsedTime =
    millis() - loopStartTime;

  if (elapsedTime < CONTROL_PERIOD_MS) {
    delay(
      CONTROL_PERIOD_MS
      - elapsedTime
    );
  }
}
