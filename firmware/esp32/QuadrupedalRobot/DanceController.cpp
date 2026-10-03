#include "DanceController.h"
#include <math.h>

// ============================================================
// TUNING VALUES
// ============================================================
//
// The forward pitch and roll use 12 degrees because the current
// robot code already uses 12 degrees as the normal maximum
// BalanceController correction angle. The requested nose-up pose
// is kept at 20 degrees above horizontal.
//
// Change these constants first if you want to tune the routine.
// ============================================================

static const float FORWARD_PITCH_DEG = 12.0f;
static const float LEFT_ROLL_DEG = 12.0f;
static const float RIGHT_ROLL_DEG = -12.0f;
static const float NOSE_UP_PITCH_DEG = -20.0f;

static const unsigned long TILT_MOVE_MS = 900;
static const unsigned long TILT_HOLD_MS = 300;

// The 20-degree nose-up pose is a larger body excursion than the
// ordinary 12-degree pitch/roll moves. Give transitions into that
// pose extra time so the robot does not throw its mass backward.
static const unsigned long NOSE_UP_MOVE_MS = 1500;

// At the start of pass 2 the pitch reverses directly from -20 deg
// nose-up to +12 deg nose-down: a 32-degree change. This is the
// single largest angular transition in the routine, so slow only
// this move substantially.
static const unsigned long LARGE_PITCH_REVERSAL_MS = 1900;

// Keep the larger 3 cm square requested during testing. The square
// already uses the 7th-order smootherstep below; doubling the square
// from 1.5 cm to 3.0 cm without changing 700 ms doubled its speed.
// 1400 ms restores roughly the old translation speed while the
// smootherstep keeps velocity/acceleration gentle at each corner.
static const float SQUARE_SIDE_CM = 3.00f;
static const unsigned long SQUARE_MOVE_MS = 1400;
static const unsigned long SQUARE_HOLD_MS = 220;

// Helper macro only used to make the choreography tables readable.
#define POSE(FWD, RIGHT, HEIGHT, ROLL, PITCH, TIME_MS) \
  { (FWD), (RIGHT), (HEIGHT), (ROLL), (PITCH), (TIME_MS) }

// ============================================================
// PITCH / ROLL SEQUENCE
// ============================================================
//
// One pass:
//   neutral
//   -> pitch forward
//   -> roll left
//   -> roll right
//   -> pitch 20 deg above horizontal
//
// Then the exact pass is repeated once more.
//
// The repeated identical frame after each move is simply a hold.
// No active balance feedback is used during these poses.
// ============================================================

static const DancePose PITCH_ROLL_SEQUENCE[] = {
  POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 300),

  // Pass 1
  POSE(0.0f, 0.0f, 0.0f, 0.0f, FORWARD_PITCH_DEG, TILT_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, 0.0f, FORWARD_PITCH_DEG, TILT_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, LEFT_ROLL_DEG, 0.0f, TILT_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, LEFT_ROLL_DEG, 0.0f, TILT_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, RIGHT_ROLL_DEG, 0.0f, TILT_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, RIGHT_ROLL_DEG, 0.0f, TILT_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, 0.0f, NOSE_UP_PITCH_DEG, NOSE_UP_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, 0.0f, NOSE_UP_PITCH_DEG, TILT_HOLD_MS),

  // Pass 2
  // Largest pitch reversal: -20 deg nose-up -> +12 deg nose-down.
  POSE(0.0f, 0.0f, 0.0f, 0.0f, FORWARD_PITCH_DEG, LARGE_PITCH_REVERSAL_MS),
  POSE(0.0f, 0.0f, 0.0f, 0.0f, FORWARD_PITCH_DEG, TILT_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, LEFT_ROLL_DEG, 0.0f, TILT_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, LEFT_ROLL_DEG, 0.0f, TILT_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, RIGHT_ROLL_DEG, 0.0f, TILT_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, RIGHT_ROLL_DEG, 0.0f, TILT_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, 0.0f, NOSE_UP_PITCH_DEG, NOSE_UP_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, 0.0f, NOSE_UP_PITCH_DEG, TILT_HOLD_MS),

  // Return to the balanced neutral baseline before the real rebalance.
  POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 900)
};

static const size_t PITCH_ROLL_FRAME_COUNT =
  sizeof(PITCH_ROLL_SEQUENCE) / sizeof(PITCH_ROLL_SEQUENCE[0]);

// ============================================================
// SQUARE BODY-TRANSLATION SEQUENCE
// ============================================================
//
// This follows the user's literal movement order:
//
//   forward -> right -> backward -> left -> neutral
//
// Because DancePose translations are absolute relative to neutral,
// the four corners are:
//   (0,0)
//   (forward,0)
//   (forward,right)
//   (0,right)
//   (0,0)
//
// That traces a real little square rather than a diamond.
//
// The same requested direction is performed twice.
//
// Every segment is interpolated by DanceController::update() using
// smoothProgress(), the same 7th-order smootherstep used by the
// robot's support-shift code. SQUARE_MOVE_MS controls how slowly
// the 3 cm translation travels along each edge.
// ============================================================

static const DancePose SQUARE_SEQUENCE[] = {
  POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 300),

  // Square 1
  POSE(SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),
  POSE(SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, 0.0f, SQUARE_HOLD_MS),

  POSE(SQUARE_SIDE_CM, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),
  POSE(SQUARE_SIDE_CM, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_HOLD_MS),

  POSE(0.0f, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),
  POSE(0.0f, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),
  POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, SQUARE_HOLD_MS),

  // Square 2
  POSE(SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),
  POSE(SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, 0.0f, SQUARE_HOLD_MS),

  POSE(SQUARE_SIDE_CM, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),
  POSE(SQUARE_SIDE_CM, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_HOLD_MS),

  POSE(0.0f, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),
  POSE(0.0f, SQUARE_SIDE_CM, 0.0f, 0.0f, 0.0f, SQUARE_HOLD_MS),

  POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, SQUARE_MOVE_MS),

  // End exactly at neutral before the real rebalance.
  POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 500)
};

static const size_t SQUARE_FRAME_COUNT =
  sizeof(SQUARE_SEQUENCE) / sizeof(SQUARE_SEQUENCE[0]);

#undef POSE

// ============================================================
// CONSTRUCTOR
// ============================================================

DanceController::DanceController(
  float bodyLengthCm,
  float bodyWidthCm,
  float rotationCenterOffsetXcm,
  float rotationCenterOffsetZcm
) {
  bodyLength = fabs(bodyLengthCm);
  bodyWidth = fabs(bodyWidthCm);

  rotationCenterOffsetX = rotationCenterOffsetXcm;
  rotationCenterOffsetZ = rotationCenterOffsetZcm;

  baseFrontLeft = {0.0f, 0.0f, 0.0f};
  baseFrontRight = {0.0f, 0.0f, 0.0f};
  baseBackLeft = {0.0f, 0.0f, 0.0f};
  baseBackRight = {0.0f, 0.0f, 0.0f};

  fromPose = neutralPose();
  currentPose = neutralPose();

  activeSequence = nullptr;
  activeFrameCount = 0;
  frameIndex = 0;
  frameStartTime = 0;

  initialized = false;
  running = false;
  finished = false;
}

// ============================================================
// BASIC POSE HELPERS
// ============================================================

DancePose DanceController::neutralPose() const {
  DancePose pose;
  pose.forwardCm = 0.0f;
  pose.rightCm = 0.0f;
  pose.heightCm = 0.0f;
  pose.rollDeg = 0.0f;
  pose.pitchDeg = 0.0f;
  pose.durationMs = 0;
  return pose;
}

float DanceController::smoothProgress(float progress) const {
  if (progress < 0.0f) {
    progress = 0.0f;
  }

  if (progress > 1.0f) {
    progress = 1.0f;
  }

  // Same 7th-order smootherstep already used in the robot's
  // support-shift code.
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

DancePose DanceController::interpolatePose(
  const DancePose &startPose,
  const DancePose &endPose,
  float weight
) const {
  DancePose result;

  result.forwardCm =
    startPose.forwardCm
    + (endPose.forwardCm - startPose.forwardCm) * weight;

  result.rightCm =
    startPose.rightCm
    + (endPose.rightCm - startPose.rightCm) * weight;

  result.heightCm =
    startPose.heightCm
    + (endPose.heightCm - startPose.heightCm) * weight;

  result.rollDeg =
    startPose.rollDeg
    + (endPose.rollDeg - startPose.rollDeg) * weight;

  result.pitchDeg =
    startPose.pitchDeg
    + (endPose.pitchDeg - startPose.pitchDeg) * weight;

  result.durationMs = 0;

  return result;
}

// ============================================================
// EXACT BALANCE-CONTROLLER ROTATION MATH
// ============================================================
//
// This is intentionally the same coordinate conversion and the
// same inverse pitch / inverse roll equations used by the current
// BalanceController::calculateCorrectedFootTarget().
//
// The dance does NOT call BalanceController::update() and does NOT
// use IMU feedback while a scripted motion is running.
// ============================================================

FootTarget DanceController::rotateTargetUsingBalanceMath(
  const FootTarget &baseTarget,
  float hipX,
  float hipZ,
  bool isLeftLeg,
  float rollDeg,
  float pitchDeg
) const {
  float zAxisSign = isLeftLeg ? 1.0f : -1.0f;

  // Leg-local -> body-frame hip-relative vector.
  float bodyRelativeFootX = -baseTarget.x;
  float bodyRelativeFootY = baseTarget.y;
  float bodyRelativeFootZ = zAxisSign * baseTarget.z;

  // Hip-relative -> rotation-center-relative.
  float centerToFootX = hipX + bodyRelativeFootX;
  float centerToFootY = bodyRelativeFootY;
  float centerToFootZ = hipZ + bodyRelativeFootZ;

  float roll = rollDeg * PI / 180.0f;
  float pitch = pitchDeg * PI / 180.0f;

  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Inverse pitch rotation: identical to BalanceController.
  float pitchCorrectedX =
    cosPitch * centerToFootX
    + sinPitch * centerToFootY;

  float pitchCorrectedY =
    -sinPitch * centerToFootX
    + cosPitch * centerToFootY;

  float pitchCorrectedZ = centerToFootZ;

  // Inverse roll rotation: identical to BalanceController.
  float rollCorrectedX = pitchCorrectedX;

  float rollCorrectedY =
    cosRoll * pitchCorrectedY
    + sinRoll * pitchCorrectedZ;

  float rollCorrectedZ =
    -sinRoll * pitchCorrectedY
    + cosRoll * pitchCorrectedZ;

  // Rotation-center-relative -> hip-relative.
  float correctedBodyRelativeX =
    rollCorrectedX - hipX;

  float correctedBodyRelativeY =
    rollCorrectedY;

  float correctedBodyRelativeZ =
    rollCorrectedZ - hipZ;

  // Body axes -> leg-local IK axes.
  FootTarget output;

  output.x = -correctedBodyRelativeX;
  output.y = correctedBodyRelativeY;
  output.z = zAxisSign * correctedBodyRelativeZ;

  return output;
}

// ============================================================
// APPLY SCRIPTED ROTATION + SCRIPTED TRANSLATION
// ============================================================

FootTarget DanceController::transformTarget(
  const FootTarget &baseTarget,
  float hipX,
  float hipZ,
  bool isLeftLeg,
  const DancePose &pose
) const {
  FootTarget output =
    rotateTargetUsingBalanceMath(
      baseTarget,
      hipX,
      hipZ,
      isLeftLeg,
      pose.rollDeg,
      pose.pitchDeg
    );

  // Body forward:
  // leg-local +X is backward, so all feet move +X relative to body.
  output.x += pose.forwardCm;

  // Body right:
  // left-leg local +Z points body-right, so the fixed foot moves
  // negative local Z. Right-leg local +Z points body-left, so the
  // fixed foot moves positive local Z.
  if (isLeftLeg) {
    output.z -= pose.rightCm;
  }
  else {
    output.z += pose.rightCm;
  }

  // Body rises -> feet are farther below the body.
  output.y += pose.heightCm;

  return output;
}

// ============================================================
// BUILD FOUR FOOT TARGETS
// ============================================================

DanceOutput DanceController::buildOutput(
  const DancePose &pose
) const {
  float halfLength = bodyLength / 2.0f;
  float halfWidth = bodyWidth / 2.0f;

  // Same COM-based rotation center convention as BalanceController.
  float frontHipX =
    halfLength - rotationCenterOffsetX;

  float backHipX =
    -halfLength - rotationCenterOffsetX;

  float leftHipZ =
    -halfWidth - rotationCenterOffsetZ;

  float rightHipZ =
    halfWidth - rotationCenterOffsetZ;

  DanceOutput output;

  output.frontLeft =
    transformTarget(
      baseFrontLeft,
      frontHipX,
      leftHipZ,
      true,
      pose
    );

  output.frontRight =
    transformTarget(
      baseFrontRight,
      frontHipX,
      rightHipZ,
      false,
      pose
    );

  output.backLeft =
    transformTarget(
      baseBackLeft,
      backHipX,
      leftHipZ,
      true,
      pose
    );

  output.backRight =
    transformTarget(
      baseBackRight,
      backHipX,
      rightHipZ,
      false,
      pose
    );

  return output;
}

// ============================================================
// INITIALIZE / START
// ============================================================

void DanceController::begin(
  const FootTarget &frontLeft,
  const FootTarget &frontRight,
  const FootTarget &backLeft,
  const FootTarget &backRight
) {
  baseFrontLeft = frontLeft;
  baseFrontRight = frontRight;
  baseBackLeft = backLeft;
  baseBackRight = backRight;

  fromPose = neutralPose();
  currentPose = neutralPose();

  activeSequence = nullptr;
  activeFrameCount = 0;
  frameIndex = 0;
  frameStartTime = millis();

  initialized = true;
  running = false;
  finished = false;
}

void DanceController::startSequence(
  const DancePose *sequence,
  size_t frameCount
) {
  if (!initialized || sequence == nullptr || frameCount == 0) {
    return;
  }

  activeSequence = sequence;
  activeFrameCount = frameCount;

  // Every section is deliberately started from the same fixed neutral
  // captured after the robot's first successful active balance.
  fromPose = neutralPose();
  currentPose = neutralPose();

  frameIndex = 0;
  frameStartTime = millis();

  running = true;
  finished = false;
}

void DanceController::startPitchRollSequence() {
  startSequence(
    PITCH_ROLL_SEQUENCE,
    PITCH_ROLL_FRAME_COUNT
  );
}

void DanceController::startSquareSequence() {
  startSequence(
    SQUARE_SEQUENCE,
    SQUARE_FRAME_COUNT
  );
}

void DanceController::forceNeutral() {
  if (!initialized) {
    return;
  }

  fromPose = neutralPose();
  currentPose = neutralPose();

  activeSequence = nullptr;
  activeFrameCount = 0;
  frameIndex = 0;
  frameStartTime = millis();

  running = false;
  finished = false;
}

bool DanceController::isActive() const {
  return running;
}

bool DanceController::isFinished() const {
  return finished;
}

// ============================================================
// UPDATE
// ============================================================

DanceOutput DanceController::update() {
  if (!initialized) {
    return buildOutput(neutralPose());
  }

  if (!running) {
    return buildOutput(currentPose);
  }

  unsigned long now = millis();

  const DancePose &targetPose =
    activeSequence[frameIndex];

  unsigned long duration =
    targetPose.durationMs;

  unsigned long elapsed =
    now - frameStartTime;

  float progress = 1.0f;

  if (duration > 0) {
    progress =
      (float)elapsed / (float)duration;
  }

  if (progress >= 1.0f) {
    currentPose = targetPose;
    currentPose.durationMs = 0;

    fromPose = currentPose;
    frameIndex++;
    frameStartTime = now;

    if (frameIndex >= activeFrameCount) {
      running = false;
      finished = true;
    }

    return buildOutput(currentPose);
  }

  float weight =
    smoothProgress(progress);

  currentPose =
    interpolatePose(
      fromPose,
      targetPose,
      weight
    );

  return buildOutput(currentPose);
}
