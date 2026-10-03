#include "DanceController.h"
#include <math.h>


// ============================================================
// NERC DANCE CHOREOGRAPHY
// ============================================================
//
// FORMAT:
//
// {
//   forward cm,
//   right cm,
//   height cm,
//   roll degrees,
//   pitch degrees,
//   transition time milliseconds
// }
//
// IMPORTANT:
//
// Start with these conservative values.
//
// Once the complete routine works reliably, the easiest place
// to make the dance more dramatic is THIS array.
//
// Do not immediately jump to 20-30 degree tilts.
// ============================================================

static const DancePose DANCE_SEQUENCE[] = {

  // ----------------------------------------------------------
  // 1. INTRO / NEUTRAL HOLD
  // ----------------------------------------------------------

  {
    0.00f,
    0.00f,
    0.00f,

    0.0f,
    0.0f,

    700
  },


  // ----------------------------------------------------------
  // 2. LEFT SWAY
  // ----------------------------------------------------------

  {
    0.00f,
    -1.20f,
    0.00f,

    4.5f,
    0.0f,

    850
  },


  // ----------------------------------------------------------
  // 3. RIGHT SWAY
  // ----------------------------------------------------------

  {
    0.00f,
    1.20f,
    0.00f,

    -4.5f,
    0.0f,

    950
  },


  // ----------------------------------------------------------
  // 4. CENTER
  // ----------------------------------------------------------

  {
    0.00f,
    0.00f,
    0.00f,

    0.0f,
    0.0f,

    650
  },


  // ----------------------------------------------------------
  // 5. FORWARD BOW
  // ----------------------------------------------------------

  {
    1.00f,
    0.00f,
    -0.55f,

    0.0f,
    4.0f,

    850
  },


  // ----------------------------------------------------------
  // 6. BACKWARD LEAN
  // ----------------------------------------------------------

  {
    -1.00f,
    0.00f,
    0.00f,

    0.0f,
    -4.0f,

    950
  },


  // ----------------------------------------------------------
  // 7. CENTER
  // ----------------------------------------------------------

  {
    0.00f,
    0.00f,
    0.00f,

    0.0f,
    0.0f,

    650
  },


  // ----------------------------------------------------------
  // 8. SIGNATURE BOTTOM-LEFT / BACKWARD DIP
  //
  // This is based on the move you discussed:
  //
  // left
  // backward
  // downward
  // left lean
  // nose slightly upward
  // ----------------------------------------------------------

  {
    -1.00f,
    -1.10f,
    -0.95f,

    4.5f,
    -3.5f,

    950
  },


  // Hold that pose briefly.

  {
    -1.00f,
    -1.10f,
    -0.95f,

    4.5f,
    -3.5f,

    450
  },


  // Return to center.

  {
    0.00f,
    0.00f,
    0.00f,

    0.0f,
    0.0f,

    850
  },


  // ==========================================================
  // 9. FOUR DIAGONAL DIRECTIONS
  // ==========================================================


  // ----------------------------------------------------------
  // FRONT-LEFT
  // ----------------------------------------------------------

  {
    0.90f,
    -0.90f,
    -0.20f,

    2.5f,
    2.5f,

    650
  },


  // ----------------------------------------------------------
  // FRONT-RIGHT
  // ----------------------------------------------------------

  {
    0.90f,
    0.90f,
    -0.20f,

    -2.5f,
    2.5f,

    650
  },


  // ----------------------------------------------------------
  // BACK-RIGHT
  // ----------------------------------------------------------

  {
    -0.90f,
    0.90f,
    -0.20f,

    -2.5f,
    -2.5f,

    650
  },


  // ----------------------------------------------------------
  // BACK-LEFT
  // ----------------------------------------------------------

  {
    -0.90f,
    -0.90f,
    -0.20f,

    2.5f,
    -2.5f,

    650
  },


  // ----------------------------------------------------------
  // CENTER
  // ----------------------------------------------------------

  {
    0.00f,
    0.00f,
    0.00f,

    0.0f,
    0.0f,

    700
  },


  // ==========================================================
  // 10. VERTICAL BOUNCES
  // ==========================================================


  // Down.

  {
    0.00f,
    0.00f,
    -0.85f,

    0.0f,
    0.0f,

    500
  },


  // Up slightly.

  {
    0.00f,
    0.00f,
    0.30f,

    0.0f,
    0.0f,

    500
  },


  // Down again.

  {
    0.00f,
    0.00f,
    -0.85f,

    0.0f,
    0.0f,

    500
  },


  // Center.

  {
    0.00f,
    0.00f,
    0.00f,

    0.0f,
    0.0f,

    600
  },


  // ==========================================================
  // 11. QUICK SIDE ACCENTS
  // ==========================================================


  {
    0.00f,
    -0.85f,
    0.00f,

    3.2f,
    0.0f,

    380
  },


  {
    0.00f,
    0.85f,
    0.00f,

    -3.2f,
    0.0f,

    380
  },


  {
    0.00f,
    -0.85f,
    0.00f,

    3.2f,
    0.0f,

    380
  },


  {
    0.00f,
    0.85f,
    0.00f,

    -3.2f,
    0.0f,

    380
  },


  // ==========================================================
  // 12. FINALE / RETURN TO NEUTRAL
  // ==========================================================

  {
    0.00f,
    0.00f,
    0.00f,

    0.0f,
    0.0f,

    900
  }
};


// Number of poses in the choreography.

static const size_t DANCE_FRAME_COUNT =
  sizeof(DANCE_SEQUENCE) /
  sizeof(DANCE_SEQUENCE[0]);


// ============================================================
// CONSTRUCTOR
// ============================================================

DanceController::DanceController(
  float bodyLengthCm,
  float bodyWidthCm,

  float rotationCenterOffsetXcm,
  float rotationCenterOffsetZcm
) {

  bodyLength =
    fabs(bodyLengthCm);

  bodyWidth =
    fabs(bodyWidthCm);


  rotationCenterOffsetX =
    rotationCenterOffsetXcm;

  rotationCenterOffsetZ =
    rotationCenterOffsetZcm;


  baseFrontLeft = {
    0.0f,
    0.0f,
    0.0f
  };

  baseFrontRight = {
    0.0f,
    0.0f,
    0.0f
  };

  baseBackLeft = {
    0.0f,
    0.0f,
    0.0f
  };

  baseBackRight = {
    0.0f,
    0.0f,
    0.0f
  };


  fromPose =
    neutralPose();

  currentPose =
    neutralPose();

  stopFromPose =
    neutralPose();


  frameIndex =
    0;

  frameStartTime =
    0;

  stopStartTime =
    0;


  initialized =
    false;

  running =
    false;

  stopping =
    false;
}


// ============================================================
// NEUTRAL POSE
// ============================================================

DancePose DanceController::neutralPose() const {

  DancePose pose;

  pose.forwardCm =
    0.0f;

  pose.rightCm =
    0.0f;

  pose.heightCm =
    0.0f;


  pose.rollDeg =
    0.0f;

  pose.pitchDeg =
    0.0f;


  pose.durationMs =
    0;


  return pose;
}


// ============================================================
// BEGIN
// ============================================================
//
// IMPORTANT:
//
// The supplied foot targets should be the robot's BALANCED
// standing targets.
//
// That means the dance is built on top of the robot's real
// balanced position rather than assuming every leg is exactly
// at X=0, Y=20, Z=0.
// ============================================================

void DanceController::begin(
  const FootTarget &frontLeft,
  const FootTarget &frontRight,
  const FootTarget &backLeft,
  const FootTarget &backRight
) {

  baseFrontLeft =
    frontLeft;

  baseFrontRight =
    frontRight;

  baseBackLeft =
    backLeft;

  baseBackRight =
    backRight;


  fromPose =
    neutralPose();

  currentPose =
    neutralPose();

  stopFromPose =
    neutralPose();


  frameIndex =
    0;

  frameStartTime =
    millis();

  stopStartTime =
    0;


  initialized =
    true;

  running =
    false;

  stopping =
    false;
}


// ============================================================
// START
// ============================================================

void DanceController::start() {

  if (!initialized) {
    return;
  }


  fromPose =
    currentPose;


  frameIndex =
    0;

  frameStartTime =
    millis();


  running =
    true;

  stopping =
    false;
}


// ============================================================
// STOP
// ============================================================
//
// This does NOT instantly snap the legs back.
//
// It smoothly returns the body to the balanced neutral pose.
// ============================================================

void DanceController::stop() {

  if (!initialized) {
    return;
  }


  stopFromPose =
    currentPose;

  stopStartTime =
    millis();


  running =
    false;

  stopping =
    true;
}


// ============================================================
// FORCE NEUTRAL
// ============================================================

void DanceController::forceNeutral() {

  if (!initialized) {
    return;
  }


  fromPose =
    neutralPose();

  currentPose =
    neutralPose();

  stopFromPose =
    neutralPose();


  frameIndex =
    0;

  frameStartTime =
    millis();

  stopStartTime =
    0;


  running =
    false;

  stopping =
    false;
}


// ============================================================
// IS ACTIVE
// ============================================================

bool DanceController::isActive() const {

  return running || stopping;
}


// ============================================================
// SMOOTH PROGRESS
// ============================================================
//
// 7th-order smootherstep.
//
// Position, velocity, acceleration, and jerk all transition
// smoothly at the endpoints.
//
// This is similar to the smoothing already used for the body
// support shifts in QuadrupedalRobot.ino.
// ============================================================

float DanceController::smoothProgress(
  float progress
) const {

  if (progress < 0.0f) {
    progress =
      0.0f;
  }


  if (progress > 1.0f) {
    progress =
      1.0f;
  }


  float p2 =
    progress * progress;

  float p4 =
    p2 * p2;

  float p5 =
    p4 * progress;

  float p6 =
    p5 * progress;

  float p7 =
    p6 * progress;


  return
    35.0f * p4
    - 84.0f * p5
    + 70.0f * p6
    - 20.0f * p7;
}


// ============================================================
// INTERPOLATE POSE
// ============================================================

DancePose DanceController::interpolatePose(
  const DancePose &startPose,
  const DancePose &endPose,
  float weight
) const {

  DancePose result;


  result.forwardCm =
    startPose.forwardCm
    + (
        endPose.forwardCm
        - startPose.forwardCm
      )
    * weight;


  result.rightCm =
    startPose.rightCm
    + (
        endPose.rightCm
        - startPose.rightCm
      )
    * weight;


  result.heightCm =
    startPose.heightCm
    + (
        endPose.heightCm
        - startPose.heightCm
      )
    * weight;


  result.rollDeg =
    startPose.rollDeg
    + (
        endPose.rollDeg
        - startPose.rollDeg
      )
    * weight;


  result.pitchDeg =
    startPose.pitchDeg
    + (
        endPose.pitchDeg
        - startPose.pitchDeg
      )
    * weight;


  result.durationMs =
    0;


  return result;
}


// ============================================================
// TRANSFORM ONE FOOT TARGET
// ============================================================

FootTarget DanceController::transformTarget(
  const FootTarget &baseTarget,

  float hipX,
  float hipZ,

  bool isLeftLeg,

  const DancePose &pose
) const {

  /*
    SAME COORDINATE CONVENTION AS THE EXISTING ROBOT CODE

    BODY:

      +X = forward
      +Y = downward
      +Z = robot's right


    LEG LOCAL:

      +X = backward
      +Y = downward
      +Z = inward


    Therefore:

      local X -> body X:

        bodyX = -localX


      LEFT local Z -> body Z:

        bodyZ = +localZ


      RIGHT local Z -> body Z:

        bodyZ = -localZ
  */


  float zAxisSign;

  if (isLeftLeg) {

    zAxisSign =
      1.0f;
  }

  else {

    zAxisSign =
      -1.0f;
  }


  // ----------------------------------------------------------
  // LEG-LOCAL TARGET -> BODY-FRAME VECTOR FROM HIP
  // ----------------------------------------------------------

  float bodyRelativeX =
    -baseTarget.x;

  float bodyRelativeY =
    baseTarget.y;

  float bodyRelativeZ =
    zAxisSign
    * baseTarget.z;


  // ----------------------------------------------------------
  // VECTOR FROM BODY ROTATION CENTER -> FOOT
  // ----------------------------------------------------------

  float centerToFootX =
    hipX
    + bodyRelativeX;

  float centerToFootY =
    bodyRelativeY;

  float centerToFootZ =
    hipZ
    + bodyRelativeZ;


  // ----------------------------------------------------------
  // BODY TRANSLATION
  // ----------------------------------------------------------
  //
  // Feet are treated as fixed on the floor.
  //
  // So if the body moves forward,
  // the feet move backward relative to the body.
  // ----------------------------------------------------------

  centerToFootX -=
    pose.forwardCm;

  centerToFootZ -=
    pose.rightCm;


  // If the body rises, feet become farther below it.

  centerToFootY +=
    pose.heightCm;


  // ----------------------------------------------------------
  // ANGLES
  // ----------------------------------------------------------

  float roll =
    pose.rollDeg
    * PI
    / 180.0f;


  float pitch =
    pose.pitchDeg
    * PI
    / 180.0f;


  float cosRoll =
    cos(roll);

  float sinRoll =
    sin(roll);


  float cosPitch =
    cos(pitch);

  float sinPitch =
    sin(pitch);


  // ----------------------------------------------------------
  // INVERSE PITCH ROTATION
  // ----------------------------------------------------------
  //
  // Same convention already used by BalanceController.
  // ----------------------------------------------------------

  float pitchCorrectedX =
    cosPitch * centerToFootX
    + sinPitch * centerToFootY;


  float pitchCorrectedY =
    -sinPitch * centerToFootX
    + cosPitch * centerToFootY;


  float pitchCorrectedZ =
    centerToFootZ;


  // ----------------------------------------------------------
  // INVERSE ROLL ROTATION
  // ----------------------------------------------------------

  float rollCorrectedX =
    pitchCorrectedX;


  float rollCorrectedY =
    cosRoll * pitchCorrectedY
    + sinRoll * pitchCorrectedZ;


  float rollCorrectedZ =
    -sinRoll * pitchCorrectedY
    + cosRoll * pitchCorrectedZ;


  // ----------------------------------------------------------
  // ROTATION CENTER -> PARTICULAR HIP
  // ----------------------------------------------------------

  float correctedBodyRelativeX =
    rollCorrectedX
    - hipX;


  float correctedBodyRelativeY =
    rollCorrectedY;


  float correctedBodyRelativeZ =
    rollCorrectedZ
    - hipZ;


  // ----------------------------------------------------------
  // BODY AXES -> LEG-LOCAL IK AXES
  // ----------------------------------------------------------

  FootTarget output;


  output.x =
    -correctedBodyRelativeX;


  output.y =
    correctedBodyRelativeY;


  output.z =
    zAxisSign
    * correctedBodyRelativeZ;


  return output;
}


// ============================================================
// BUILD ALL FOUR FOOT TARGETS
// ============================================================

DanceOutput DanceController::buildOutput(
  const DancePose &pose
) const {

  float halfLength =
    bodyLength / 2.0f;


  float halfWidth =
    bodyWidth / 2.0f;


  // Use the same COM-based rotation center as the existing
  // balance controller.

  float frontHipX =
    halfLength
    - rotationCenterOffsetX;


  float backHipX =
    -halfLength
    - rotationCenterOffsetX;


  float leftHipZ =
    -halfWidth
    - rotationCenterOffsetZ;


  float rightHipZ =
    halfWidth
    - rotationCenterOffsetZ;


  DanceOutput output;


  // ----------------------------------------------------------
  // FRONT LEFT
  // ----------------------------------------------------------

  output.frontLeft =
    transformTarget(
      baseFrontLeft,

      frontHipX,
      leftHipZ,

      true,

      pose
    );


  // ----------------------------------------------------------
  // FRONT RIGHT
  // ----------------------------------------------------------

  output.frontRight =
    transformTarget(
      baseFrontRight,

      frontHipX,
      rightHipZ,

      false,

      pose
    );


  // ----------------------------------------------------------
  // BACK LEFT
  // ----------------------------------------------------------

  output.backLeft =
    transformTarget(
      baseBackLeft,

      backHipX,
      leftHipZ,

      true,

      pose
    );


  // ----------------------------------------------------------
  // BACK RIGHT
  // ----------------------------------------------------------

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
// UPDATE
// ============================================================

DanceOutput DanceController::update() {

  // ----------------------------------------------------------
  // NOT INITIALIZED
  // ----------------------------------------------------------

  if (!initialized) {

    return
      buildOutput(
        neutralPose()
      );
  }


  unsigned long now =
    millis();


  // ==========================================================
  // SMOOTH STOP
  // ==========================================================

  if (stopping) {

    unsigned long elapsed =
      now
      - stopStartTime;


    float progress =
      (float)elapsed
      / (float)STOP_RETURN_TIME_MS;


    if (progress >= 1.0f) {

      currentPose =
        neutralPose();


      fromPose =
        currentPose;


      stopping =
        false;


      return
        buildOutput(
          currentPose
        );
    }


    float weight =
      smoothProgress(
        progress
      );


    currentPose =
      interpolatePose(
        stopFromPose,
        neutralPose(),
        weight
      );


    return
      buildOutput(
        currentPose
      );
  }


  // ==========================================================
  // NOT RUNNING
  // ==========================================================

  if (!running) {

    currentPose =
      neutralPose();


    return
      buildOutput(
        currentPose
      );
  }


  // ==========================================================
  // RUN CURRENT CHOREOGRAPHY FRAME
  // ==========================================================

  const DancePose &targetPose =
    DANCE_SEQUENCE[
      frameIndex
    ];


  unsigned long duration =
    targetPose.durationMs;


  unsigned long elapsed =
    now
    - frameStartTime;


  float progress =
    1.0f;


  if (duration > 0) {

    progress =
      (float)elapsed
      / (float)duration;
  }


  // ----------------------------------------------------------
  // CURRENT FRAME FINISHED
  // ----------------------------------------------------------

  if (progress >= 1.0f) {

    currentPose =
      targetPose;


    currentPose.durationMs =
      0;


    fromPose =
      currentPose;


    frameIndex++;


    // Loop back to the beginning forever.

    if (
      frameIndex
      >= DANCE_FRAME_COUNT
    ) {

      frameIndex =
        0;
    }


    frameStartTime =
      now;


    return
      buildOutput(
        currentPose
      );
  }


  // ----------------------------------------------------------
  // CURRENT FRAME STILL MOVING
  // ----------------------------------------------------------

  float weight =
    smoothProgress(
      progress
    );


  currentPose =
    interpolatePose(
      fromPose,
      targetPose,
      weight
    );


  return
    buildOutput(
      currentPose
    );
}