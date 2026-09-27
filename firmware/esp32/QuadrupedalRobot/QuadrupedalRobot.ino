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

const double HALF_BODY_LENGTH =

  BODY_LENGTH / 2.0;

const double HALF_BODY_WIDTH =

  BODY_WIDTH / 2.0;

// ------------------------------------------------------------

// CENTER OF MASS

//

// Robot weight = 6.0 lb.

// Front two legs measured = 3.6 lb.

//

// Calculated COM:

//

// X = +2.0 cm toward front

// Z = 0.0 cm

// ------------------------------------------------------------

const float COM_OFFSET_X = 2.0f;

const float COM_OFFSET_Z = 0.0f;

// ------------------------------------------------------------

// REFERENCE SUPPORT-POLYGON CALIBRATION

//

// These are the experimentally determined stable shifts.

//

// They are converted into barycentric coordinates and then

// transferred onto the CURRENT three-foot support polygon.

//

// There is no averaging of leg positions.

// ------------------------------------------------------------

// FRONT LEFT

const double REFERENCE_FRONT_LEFT_SHIFT_X = -2.5;

const double REFERENCE_FRONT_LEFT_SHIFT_Z = 3.0;

// BACK RIGHT

const double REFERENCE_BACK_RIGHT_SHIFT_X = 1.5;

const double REFERENCE_BACK_RIGHT_SHIFT_Z = -3.0;

// FRONT RIGHT

const double REFERENCE_FRONT_RIGHT_SHIFT_X = -2.5;

const double REFERENCE_FRONT_RIGHT_SHIFT_Z = -3.5;

// BACK LEFT

const double REFERENCE_BACK_LEFT_SHIFT_X = 1.5;

const double REFERENCE_BACK_LEFT_SHIFT_Z = 2.5;

// ------------------------------------------------------------

// SUPPORT-POLYGON SAFETY

// ------------------------------------------------------------

const double MIN_SUPPORT_EDGE_DISTANCE = 1.0;

// The old 4.0 cm X limit was too small.

//

// BL naturally reaches approximately +3.75 cm during the

// gait even before small balance corrections are considered.

//

// Give the support-polygon planner enough room without

// removing the safety bound entirely.

const double MAX_SUPPORT_SHIFT_X = 5.0;

const double MAX_SUPPORT_SHIFT_Z = 4.5;

const double GEOMETRY_EPSILON = 0.000001;

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

const float LEVEL_TOLERANCE = 2.0f;

const unsigned long IMU_SETTLE_TIME = 3000;

// Initial balance remains conservative.

const unsigned long LEVEL_HOLD_TIME = 200;

const unsigned long MAX_BALANCE_TIME = 2000;

// Faster end-of-cycle rebalance.

const unsigned long REBALANCE_LEVEL_HOLD_TIME = 150;

const unsigned long MAX_REBALANCE_TIME = 1000;

const unsigned long REBALANCE_BLEND_TIME = 250;

const unsigned long REBALANCE_SETTLE_TIME = 100;

const unsigned long WAIT_AFTER_REBALANCE = 150;

const unsigned long WAIT_AFTER_INITIAL_BALANCE = 2000;

const unsigned long CONTROL_PERIOD_MS = 10;

const float BALANCE_OUTPUT_FILTER_ALPHA = 0.25f;

const float BALANCE_DEADBAND = 0.5f;

// ------------------------------------------------------------

// WALKING TIMING

// ------------------------------------------------------------

// The support transitions are deliberately slower than the failed

// 60 ms version so that several loaded servos are not commanded

// through a large body shift almost instantaneously.

//

// The next swing still overlaps the END of the COM transition,

// but only after most of the weight transfer has already occurred.

//

// IMPORTANT:

//

// The swing is NOT compressed into this transition time.

// Once it starts, it still progresses according to real elapsed

// time and GAIT_SWING_TIME.

// Neutral -> FL support.

const unsigned long INITIAL_SUPPORT_SHIFT_TIME = 325;

// One support position -> the next support position.

const unsigned long SUPPORT_TRANSITION_TIME = 550;

// Slower transition only when preparing to lift the Back Left leg.
const unsigned long BACK_LEFT_SUPPORT_TRANSITION_TIME = 650;

// The next swing begins during the final fixed 90 ms of the
// support transition. Using a fixed overlap time means changing
// the body-shift duration does not accidentally change how much
// of the swing overlaps the shift.

const unsigned long SWING_OVERLAP_TIME = 145;

// No additional pause after the support transition.

const unsigned long SHIFT_SETTLE_TIME = 0;

// Gives the newly landed foot time to accept load before the next

// full-body support transition begins.

const unsigned long TOUCHDOWN_SETTLE_TIME = 50;

// Update period for the combined COM-shift / early-lift motion.

const unsigned long OVERLAP_UPDATE_PERIOD_MS = 10;

// ------------------------------------------------------------

// GAIT

// GAIT

// ------------------------------------------------------------

const double GAIT_STRIDE_LENGTH = 5.0;

const double GAIT_HEIGHT = 6;

// Use a conservative swing time while validating the new

// COM-shift / swing-overlap logic. The previous code's comment

// said 700 ms, but the actual constant was 350 ms.

const unsigned long GAIT_SWING_TIME = 700;

const double GAIT_REFERENCE_CYCLE_TIME = 4000.0;

const double GAIT_SWING_FRACTION =

  (double)GAIT_SWING_TIME / GAIT_REFERENCE_CYCLE_TIME;

const double GAIT_DIRECTION = 1.0;

// ------------------------------------------------------------

// STANCE

// ------------------------------------------------------------

// 5 cm total stride / 4 steps.

//

// This advancement is now performed DURING the direct

// transition between support positions.

const double STANCE_ADVANCE_PER_STEP =

  GAIT_STRIDE_LENGTH / 4.0;

// ------------------------------------------------------------

// WALKING TEST

// ------------------------------------------------------------

const int CRAWL_CYCLES_TO_RUN = 4;

// ------------------------------------------------------------

// OBJECTSc

// ------------------------------------------------------------

BalanceController balance(

  BODY_LENGTH,

  BODY_WIDTH,

  ROLL_KP,

  ROLL_KD,

  PITCH_KP,

  PITCH_KD);

GaitCycle swingGait(

  0.0,

  0.0,

  GAIT_STRIDE_LENGTH,

  GAIT_HEIGHT,

  0.0,

  GAIT_REFERENCE_CYCLE_TIME,

  GAIT_SWING_FRACTION,

  0.0,

  GAIT_DIRECTION);

// ------------------------------------------------------------

// SIT TO STAND

// ------------------------------------------------------------

SitToStand frontLeftStand(

  frontLeftLeg,

  SIT_X,

  SIT_Y,

  SIT_Z,

  STANDING_X,

  STANDING_Y,

  STANDING_Z);

SitToStand frontRightStand(

  frontRightLeg,

  SIT_X,

  SIT_Y,

  SIT_Z,

  STANDING_X,

  STANDING_Y,

  STANDING_Z);

SitToStand backLeftStand(

  backLeftLeg,

  SIT_X,

  SIT_Y,

  SIT_Z,

  STANDING_X,

  STANDING_Y,

  STANDING_Z);

SitToStand backRightStand(

  backRightLeg,

  SIT_X,

  SIT_Y,

  SIT_Z,

  STANDING_X,

  STANDING_Y,

  STANDING_Z);

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

// PLANNED GAIT TARGETS

// ------------------------------------------------------------

FootTarget plannedFrontLeft =

  STANDING_FRONT_LEFT;

FootTarget plannedFrontRight =

  STANDING_FRONT_RIGHT;

FootTarget plannedBackLeft =

  STANDING_BACK_LEFT;

FootTarget plannedBackRight =

  STANDING_BACK_RIGHT;

// ------------------------------------------------------------

// BALANCED / PHYSICAL BASELINE TARGETS

//

// Temporary support shifts are NOT stored here.

//

// Swing touchdown and stance advancement ARE stored here.

//

// Therefore these values represent the baseline from which

// the next support polygon is calculated.

// ------------------------------------------------------------

FootTarget balancedFrontLeft =

  STANDING_FRONT_LEFT;

FootTarget balancedFrontRight =

  STANDING_FRONT_RIGHT;

FootTarget balancedBackLeft =

  STANDING_BACK_LEFT;

FootTarget balancedBackRight =

  STANDING_BACK_RIGHT;

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

// 2D POINT

// ------------------------------------------------------------

struct Point2D {

  double x;

  double z;
};

// ------------------------------------------------------------

// TARGET BLENDING

// ------------------------------------------------------------

FootTarget blendTarget(

  const FootTarget &startTarget,

  const FootTarget &endTarget,

  float weight) {

  if (weight < 0.0f) {

    weight = 0.0f;
  }

  if (weight > 1.0f) {

    weight = 1.0f;
  }

  FootTarget target;

  target.x =

    startTarget.x + (endTarget.x - startTarget.x) * weight;

  target.y =

    startTarget.y + (endTarget.y - startTarget.y) * weight;

  target.z =

    startTarget.z + (endTarget.z - startTarget.z) * weight;

  return target;
}

// ------------------------------------------------------------

// SEND TARGETS TO IK

// ------------------------------------------------------------

void commandFootTargets(

  const FootTarget &frontLeft,

  const FootTarget &frontRight,

  const FootTarget &backLeft,

  const FootTarget &backRight) {

  frontLeftLeg.updateTarget(

    frontLeft.x,

    frontLeft.y,

    frontLeft.z);

  frontRightLeg.updateTarget(

    frontRight.x,

    frontRight.y,

    frontRight.z);

  backLeftLeg.updateTarget(

    backLeft.x,

    backLeft.y,

    backLeft.z);

  backRightLeg.updateTarget(

    backRight.x,

    backRight.y,

    backRight.z);
}

// ------------------------------------------------------------

// BASIC POSITIONS

// ------------------------------------------------------------

void commandStandingPosition() {

  commandFootTargets(

    STANDING_FRONT_LEFT,

    STANDING_FRONT_RIGHT,

    STANDING_BACK_LEFT,

    STANDING_BACK_RIGHT);
}

void commandBalancedPosition() {

  commandFootTargets(

    balancedFrontLeft,

    balancedFrontRight,

    balancedBackLeft,

    balancedBackRight);
}

// ------------------------------------------------------------

// IMU

// ------------------------------------------------------------

void updateIMU() {

  if (imuReady) {

    orientation.update();
  }
}

void waitWithIMU(

  unsigned long duration) {

  unsigned long startTime =

    millis();

  while (

    millis() - startTime < duration) {

    updateIMU();

    delay(

      CONTROL_PERIOD_MS);
  }
}

// ------------------------------------------------------------

// 7TH-ORDER SMOOTHERSTEP

// ------------------------------------------------------------

double smoothBodyShift(

  double value) {

  if (value < 0.0) {

    value = 0.0;
  }

  if (value > 1.0) {

    value = 1.0;
  }

  // 7th-order smootherstep.
  //
  // Position, velocity, acceleration, and jerk all transition
  // smoothly at the endpoints. This reduces the end-to-end snap
  // of the body support shifts.

  return 35.0 * pow(value, 4) - 84.0 * pow(value, 5) + 70.0 * pow(value, 6) - 20.0 * pow(value, 7);
}

// ------------------------------------------------------------

// SWING PROGRESS FROM REAL OVERLAP TIME

//

// The swing does not begin until the COM transition is nearly

// complete. Once overlap begins, swing progress is based only on

// the real number of elapsed swing milliseconds divided by

// GAIT_SWING_TIME.

//

// This preserves the normal swing speed while still allowing the

// end of the COM shift and the beginning of the swing to overlap.

// ------------------------------------------------------------

double getOverlapSwingProgress(

  unsigned long transitionElapsed,

  unsigned long transitionDuration) {

  if (

    transitionDuration == 0) {

    return 0.0;
  }

  unsigned long overlapStart =

    0;

  if (

    transitionDuration > SWING_OVERLAP_TIME) {

    overlapStart =

      transitionDuration - SWING_OVERLAP_TIME;
  }

  if (

    transitionElapsed <= overlapStart) {

    return 0.0;
  }

  unsigned long overlapElapsed =

    transitionElapsed - overlapStart;

  double swingProgress =

    (double)overlapElapsed / (double)GAIT_SWING_TIME;

  if (

    swingProgress > 1.0) {

    swingProgress =

      1.0;
  }

  return swingProgress;
}

// ------------------------------------------------------------

// GET SWING DELTAS AT A SPECIFIC SWING PROGRESS

//

// 0.0 = beginning of swing

// 0.5 = middle of swing

// 1.0 = touchdown

//

// This samples the existing GaitCycle trajectory, so the

// overlapped lift and the normal swing use the same path.

// ------------------------------------------------------------

void getSwingDeltasAtProgress(

  unsigned long trajectoryBaseTime,

  double swingProgress,

  double &gaitDeltaX,

  double &gaitDeltaY) {

  if (

    swingProgress < 0.0) {

    swingProgress =

      0.0;
  }

  if (

    swingProgress >= 1.0) {

    gaitDeltaX =

      -GAIT_STRIDE_LENGTH;

    gaitDeltaY =

      0.0;

    return;
  }

  unsigned long sampleTime =

    trajectoryBaseTime + (unsigned long)(swingProgress * GAIT_SWING_TIME);

  double gaitX;

  double gaitY;

  double gaitZ;

  swingGait.getTarget(

    sampleTime,

    gaitX,

    gaitY,

    gaitZ);

  const double gaitStartX =

    GAIT_STRIDE_LENGTH / 2.0;

  gaitDeltaX =

    gaitX - gaitStartX;

  gaitDeltaY =

    gaitY;
}

// ------------------------------------------------------------

// APPLY SWING OFFSET TO ONE LEG IN A SET OF TARGETS

// ------------------------------------------------------------

void applySwingOffsetToTargets(

  LegIndex leg,

  double gaitDeltaX,

  double gaitDeltaY,

  FootTarget &frontLeft,

  FootTarget &frontRight,

  FootTarget &backLeft,

  FootTarget &backRight) {

  if (

    leg == FRONT_LEFT) {

    frontLeft.x +=

      gaitDeltaX;

    frontLeft.y +=

      gaitDeltaY;

  }

  else if (

    leg == FRONT_RIGHT) {

    frontRight.x +=

      gaitDeltaX;

    frontRight.y +=

      gaitDeltaY;

  }

  else if (

    leg == BACK_LEFT) {

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
}

// ============================================================

// SUPPORT POLYGON GEOMETRY

// ============================================================

// ------------------------------------------------------------

// CROSS PRODUCT

// ------------------------------------------------------------

double cross2D(

  const Point2D &a,

  const Point2D &b,

  const Point2D &c) {

  return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

// ------------------------------------------------------------

// REFERENCE FOOT POSITION

// ------------------------------------------------------------

Point2D getReferenceFootPosition(

  LegIndex leg) {

  Point2D foot;

  if (leg == FRONT_LEFT) {

    foot.x =

      HALF_BODY_LENGTH;

    foot.z =

      -HALF_BODY_WIDTH;

  }

  else if (leg == FRONT_RIGHT) {

    foot.x =

      HALF_BODY_LENGTH;

    foot.z =

      HALF_BODY_WIDTH;

  }

  else if (leg == BACK_LEFT) {

    foot.x =

      -HALF_BODY_LENGTH;

    foot.z =

      -HALF_BODY_WIDTH;

  }

  else {

    foot.x =

      -HALF_BODY_LENGTH;

    foot.z =

      HALF_BODY_WIDTH;
  }

  return foot;
}

// ------------------------------------------------------------

// CURRENT FOOT POSITION

//

// Converts the baseline IK position into the current

// body-frame foot location.

//

// +X body = forward

// +Z body = right

//

// leg-local +X = backward

// ------------------------------------------------------------

Point2D getCurrentFootPosition(

  LegIndex leg) {

  Point2D foot;

  if (leg == FRONT_LEFT) {

    foot.x =

      HALF_BODY_LENGTH - balancedFrontLeft.x;

    foot.z =

      -HALF_BODY_WIDTH + balancedFrontLeft.z;

  }

  else if (leg == FRONT_RIGHT) {

    foot.x =

      HALF_BODY_LENGTH - balancedFrontRight.x;

    foot.z =

      HALF_BODY_WIDTH - balancedFrontRight.z;

  }

  else if (leg == BACK_LEFT) {

    foot.x =

      -HALF_BODY_LENGTH - balancedBackLeft.x;

    foot.z =

      -HALF_BODY_WIDTH + balancedBackLeft.z;

  }

  else {

    foot.x =

      -HALF_BODY_LENGTH - balancedBackRight.x;

    foot.z =

      HALF_BODY_WIDTH - balancedBackRight.z;
  }

  return foot;
}

// ------------------------------------------------------------

// SELECT THREE SUPPORT FEET

// ------------------------------------------------------------

void selectSupportTriangle(

  LegIndex swingLeg,

  const Point2D &frontLeft,

  const Point2D &frontRight,

  const Point2D &backLeft,

  const Point2D &backRight,

  Point2D &a,

  Point2D &b,

  Point2D &c) {

  if (swingLeg == FRONT_LEFT) {

    a = frontRight;

    b = backRight;

    c = backLeft;

  }

  else if (swingLeg == BACK_RIGHT) {

    a = frontLeft;

    b = backLeft;

    c = frontRight;

  }

  else if (swingLeg == FRONT_RIGHT) {

    a = frontLeft;

    b = backLeft;

    c = backRight;

  }

  else {

    a = frontLeft;

    b = frontRight;

    c = backRight;
  }
}

// ------------------------------------------------------------

// REFERENCE SUPPORT TRIANGLE

// ------------------------------------------------------------

void getReferenceSupportTriangle(

  LegIndex swingLeg,

  Point2D &a,

  Point2D &b,

  Point2D &c) {

  Point2D frontLeft =

    getReferenceFootPosition(

      FRONT_LEFT);

  Point2D frontRight =

    getReferenceFootPosition(

      FRONT_RIGHT);

  Point2D backLeft =

    getReferenceFootPosition(

      BACK_LEFT);

  Point2D backRight =

    getReferenceFootPosition(

      BACK_RIGHT);

  selectSupportTriangle(

    swingLeg,

    frontLeft,

    frontRight,

    backLeft,

    backRight,

    a,

    b,

    c);
}

// ------------------------------------------------------------

// CURRENT SUPPORT TRIANGLE

// ------------------------------------------------------------

void getCurrentSupportTriangle(

  LegIndex swingLeg,

  Point2D &a,

  Point2D &b,

  Point2D &c) {

  Point2D frontLeft =

    getCurrentFootPosition(

      FRONT_LEFT);

  Point2D frontRight =

    getCurrentFootPosition(

      FRONT_RIGHT);

  Point2D backLeft =

    getCurrentFootPosition(

      BACK_LEFT);

  Point2D backRight =

    getCurrentFootPosition(

      BACK_RIGHT);

  selectSupportTriangle(

    swingLeg,

    frontLeft,

    frontRight,

    backLeft,

    backRight,

    a,

    b,

    c);
}

// ------------------------------------------------------------

// REFERENCE SHIFT

// ------------------------------------------------------------

void getReferenceShift(

  LegIndex leg,

  double &shiftX,

  double &shiftZ) {

  if (leg == FRONT_LEFT) {

    shiftX =

      REFERENCE_FRONT_LEFT_SHIFT_X;

    shiftZ =

      REFERENCE_FRONT_LEFT_SHIFT_Z;

  }

  else if (leg == BACK_RIGHT) {

    shiftX =

      REFERENCE_BACK_RIGHT_SHIFT_X;

    shiftZ =

      REFERENCE_BACK_RIGHT_SHIFT_Z;

  }

  else if (leg == FRONT_RIGHT) {

    shiftX =

      REFERENCE_FRONT_RIGHT_SHIFT_X;

    shiftZ =

      REFERENCE_FRONT_RIGHT_SHIFT_Z;

  }

  else {

    shiftX =

      REFERENCE_BACK_LEFT_SHIFT_X;

    shiftZ =

      REFERENCE_BACK_LEFT_SHIFT_Z;
  }
}

// ------------------------------------------------------------

// BARYCENTRIC COORDINATES

// ------------------------------------------------------------

bool calculateBarycentricCoordinates(

  const Point2D &point,

  const Point2D &a,

  const Point2D &b,

  const Point2D &c,

  double &weightA,

  double &weightB,

  double &weightC) {

  double denominator =

    ((b.z - c.z) * (a.x - c.x)) + ((c.x - b.x) * (a.z - c.z));

  if (

    fabs(denominator) < GEOMETRY_EPSILON) {

    return false;
  }

  weightA =

    ((

       (b.z - c.z) * (point.x - c.x))

     + ((c.x - b.x) * (point.z - c.z)))

    / denominator;

  weightB =

    ((

       (c.z - a.z) * (point.x - c.x))

     + ((a.x - c.x) * (point.z - c.z)))

    / denominator;

  weightC =

    1.0 - weightA - weightB;

  return isfinite(weightA) && isfinite(weightB) && isfinite(weightC);
}

// ------------------------------------------------------------

// RECONSTRUCT BARYCENTRIC POINT

// ------------------------------------------------------------

Point2D barycentricPoint(

  const Point2D &a,

  const Point2D &b,

  const Point2D &c,

  double weightA,

  double weightB,

  double weightC) {

  Point2D point;

  point.x =

    weightA * a.x + weightB * b.x + weightC * c.x;

  point.z =

    weightA * a.z + weightB * b.z + weightC * c.z;

  return point;
}

// ------------------------------------------------------------

// DISTANCE TO TRIANGLE EDGE

// ------------------------------------------------------------

double distanceToEdge(

  const Point2D &point,

  const Point2D &a,

  const Point2D &b) {

  double dx =

    b.x - a.x;

  double dz =

    b.z - a.z;

  double edgeLength =

    sqrt(

      dx * dx + dz * dz);

  if (

    edgeLength < GEOMETRY_EPSILON) {

    return 0.0;
  }

  return fabs(

           cross2D(

             a,

             b,

             point))

         / edgeLength;
}

// ------------------------------------------------------------

// MINIMUM TRIANGLE EDGE DISTANCE

// ------------------------------------------------------------

double minimumTriangleEdgeDistance(

  const Point2D &point,

  const Point2D &a,

  const Point2D &b,

  const Point2D &c) {

  double distanceAB =

    distanceToEdge(

      point,

      a,

      b);

  double distanceBC =

    distanceToEdge(

      point,

      b,

      c);

  double distanceCA =

    distanceToEdge(

      point,

      c,

      a);

  double minimumDistance =

    distanceAB;

  if (

    distanceBC < minimumDistance) {

    minimumDistance =

      distanceBC;
  }

  if (

    distanceCA < minimumDistance) {

    minimumDistance =

      distanceCA;
  }

  return minimumDistance;
}

// ------------------------------------------------------------

// CALCULATE CURRENT SUPPORT-POLYGON SHIFT

// ------------------------------------------------------------

bool calculateSupportPolygonShift(

  LegIndex swingLeg,

  double &bodyShiftX,

  double &bodyShiftZ) {

  Point2D referenceA;

  Point2D referenceB;

  Point2D referenceC;

  getReferenceSupportTriangle(

    swingLeg,

    referenceA,

    referenceB,

    referenceC);

  double referenceShiftX;

  double referenceShiftZ;

  getReferenceShift(

    swingLeg,

    referenceShiftX,

    referenceShiftZ);

  Point2D referenceTargetCOM = {

    COM_OFFSET_X + referenceShiftX,

    COM_OFFSET_Z + referenceShiftZ

  };

  double weightA;

  double weightB;

  double weightC;

  if (

    !calculateBarycentricCoordinates(

      referenceTargetCOM,

      referenceA,

      referenceB,

      referenceC,

      weightA,

      weightB,

      weightC)) {

    return false;
  }

  if (

    weightA < -GEOMETRY_EPSILON || weightB < -GEOMETRY_EPSILON || weightC < -GEOMETRY_EPSILON) {

    return false;
  }

  if (

    weightA > 1.0 + GEOMETRY_EPSILON || weightB > 1.0 + GEOMETRY_EPSILON || weightC > 1.0 + GEOMETRY_EPSILON) {

    return false;
  }

  Point2D currentA;

  Point2D currentB;

  Point2D currentC;

  getCurrentSupportTriangle(

    swingLeg,

    currentA,

    currentB,

    currentC);

  if (

    fabs(

      cross2D(

        currentA,

        currentB,

        currentC))

    < GEOMETRY_EPSILON) {

    return false;
  }

  Point2D targetCOM =

    barycentricPoint(

      currentA,

      currentB,

      currentC,

      weightA,

      weightB,

      weightC);

  double edgeDistance =

    minimumTriangleEdgeDistance(

      targetCOM,

      currentA,

      currentB,

      currentC);

  if (

    edgeDistance < MIN_SUPPORT_EDGE_DISTANCE) {

    return false;
  }

  Point2D currentCOM = {

    COM_OFFSET_X,

    COM_OFFSET_Z

  };

  bodyShiftX =

    targetCOM.x - currentCOM.x;

  bodyShiftZ =

    targetCOM.z - currentCOM.z;

  if (

    !isfinite(bodyShiftX) || !isfinite(bodyShiftZ)) {

    return false;
  }

  if (

    fabs(bodyShiftX) > MAX_SUPPORT_SHIFT_X) {

    return false;
  }

  if (

    fabs(bodyShiftZ) > MAX_SUPPORT_SHIFT_Z) {

    return false;
  }

  return true;
}

// ============================================================

// BODY SHIFT HELPERS

// ============================================================

// ------------------------------------------------------------

// APPLY TEMPORARY SUPPORT SHIFT TO A SET OF TARGETS

// ------------------------------------------------------------

void applyBodyShiftToTargets(

  double bodyShiftX,

  double bodyShiftZ,

  FootTarget &frontLeft,

  FootTarget &frontRight,

  FootTarget &backLeft,

  FootTarget &backRight) {

  frontLeft.x +=

    bodyShiftX;

  frontRight.x +=

    bodyShiftX;

  backLeft.x +=

    bodyShiftX;

  backRight.x +=

    bodyShiftX;

  frontLeft.z -=

    bodyShiftZ;

  backLeft.z -=

    bodyShiftZ;

  frontRight.z +=

    bodyShiftZ;

  backRight.z +=

    bodyShiftZ;
}

// ------------------------------------------------------------

// GET CURRENT BASELINE + TEMPORARY BODY SHIFT

// ------------------------------------------------------------

void getShiftedTargets(

  double bodyShiftX,

  double bodyShiftZ,

  FootTarget &frontLeft,

  FootTarget &frontRight,

  FootTarget &backLeft,

  FootTarget &backRight) {

  frontLeft =

    balancedFrontLeft;

  frontRight =

    balancedFrontRight;

  backLeft =

    balancedBackLeft;

  backRight =

    balancedBackRight;

  applyBodyShiftToTargets(

    bodyShiftX,

    bodyShiftZ,

    frontLeft,

    frontRight,

    backLeft,

    backRight);
}

// ------------------------------------------------------------

// COMMAND BODY SHIFT

// ------------------------------------------------------------

void commandBodyShift(

  double bodyShiftX,

  double bodyShiftZ) {

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

    backRight);

  commandFootTargets(

    frontLeft,

    frontRight,

    backLeft,

    backRight);
}

// ------------------------------------------------------------

// MOVE FROM NEUTRAL BASELINE INTO INITIAL SUPPORT POSITION

//

// This is needed only once at the beginning of each

// four-leg crawl cycle.

// ------------------------------------------------------------

void moveIntoInitialSupportShiftAndPreLift(

  LegIndex swingLeg,

  double endX,

  double endZ,

  double &swingStartProgress) {

  swingStartProgress =

    0.0;

  // If there is no transition time, command the support target

  // immediately and do not overlap the swing.

  if (

    INITIAL_SUPPORT_SHIFT_TIME == 0) {

    commandBodyShift(

      endX,

      endZ);

    return;
  }

  unsigned long transitionStartTime =

    millis();

  // This is the time corresponding to swingProgress = 0.

  //

  // The swing does not actually begin until SWING_OVERLAP_DELAY

  // milliseconds into the COM transition.

  unsigned long trajectoryBaseTime =

    transitionStartTime;

  unsigned long virtualStartTime =

    trajectoryBaseTime - (unsigned long)GAIT_REFERENCE_CYCLE_TIME;

  swingGait.resetCycle(

    virtualStartTime);

  while (true) {

    unsigned long elapsed =

      millis() - transitionStartTime;

    if (

      elapsed > INITIAL_SUPPORT_SHIFT_TIME) {

      elapsed =

        INITIAL_SUPPORT_SHIFT_TIME;
    }

    double transitionProgress =

      (double)elapsed / (double)INITIAL_SUPPORT_SHIFT_TIME;

    double bodyProgress =

      smoothBodyShift(

        transitionProgress);

    double bodyShiftX =

      endX * bodyProgress;

    double bodyShiftZ =

      endZ * bodyProgress;

    FootTarget frontLeft =

      balancedFrontLeft;

    FootTarget frontRight =

      balancedFrontRight;

    FootTarget backLeft =

      balancedBackLeft;

    FootTarget backRight =

      balancedBackRight;

    applyBodyShiftToTargets(

      bodyShiftX,

      bodyShiftZ,

      frontLeft,

      frontRight,

      backLeft,

      backRight);

    // Begin the normal-speed swing only during the final fixed

    // SWING_OVERLAP_TIME of the COM transition.

    // At that point the smootherstep body shift has already

    // completed most of the physical weight transfer.

    double swingProgress =

      getOverlapSwingProgress(

        elapsed,

        INITIAL_SUPPORT_SHIFT_TIME);

    if (

      swingProgress > 0.0) {

      double gaitDeltaX;

      double gaitDeltaY;

      getSwingDeltasAtProgress(

        trajectoryBaseTime,

        swingProgress,

        gaitDeltaX,

        gaitDeltaY);

      applySwingOffsetToTargets(

        swingLeg,

        gaitDeltaX,

        gaitDeltaY,

        frontLeft,

        frontRight,

        backLeft,

        backRight);
    }

    commandFootTargets(

      frontLeft,

      frontRight,

      backLeft,

      backRight);

    updateIMU();

    if (

      elapsed >= INITIAL_SUPPORT_SHIFT_TIME) {

      break;
    }

    delay(

      OVERLAP_UPDATE_PERIOD_MS);
  }

  // Store exactly how much of the swing really occurred during

  // the COM transition.

  swingStartProgress =

    getOverlapSwingProgress(

      INITIAL_SUPPORT_SHIFT_TIME,

      INITIAL_SUPPORT_SHIFT_TIME);

  // Send the exact final COM + pre-lift target so the following

  // performSwing() begins from the same physical command.

  FootTarget finalFrontLeft =

    balancedFrontLeft;

  FootTarget finalFrontRight =

    balancedFrontRight;

  FootTarget finalBackLeft =

    balancedBackLeft;

  FootTarget finalBackRight =

    balancedBackRight;

  applyBodyShiftToTargets(

    endX,

    endZ,

    finalFrontLeft,

    finalFrontRight,

    finalBackLeft,

    finalBackRight);

  if (

    swingStartProgress > 0.0) {

    double gaitDeltaX;

    double gaitDeltaY;

    getSwingDeltasAtProgress(

      trajectoryBaseTime,

      swingStartProgress,

      gaitDeltaX,

      gaitDeltaY);

    applySwingOffsetToTargets(

      swingLeg,

      gaitDeltaX,

      gaitDeltaY,

      finalFrontLeft,

      finalFrontRight,

      finalBackLeft,

      finalBackRight);
  }

  commandFootTargets(

    finalFrontLeft,

    finalFrontRight,

    finalBackLeft,

    finalBackRight);
}

// ============================================================

// GAIT

// ============================================================

// GAIT

// ============================================================

// ------------------------------------------------------------

// MODIFY ONE LEG X LOCATION AFTER TOUCHDOWN

// ------------------------------------------------------------

void addXToLeg(

  LegIndex leg,

  double amount) {

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

// COMMAND ONE SWING FRAME

// ------------------------------------------------------------

void commandSwingFrame(

  LegIndex leg,

  double bodyShiftX,

  double bodyShiftZ,

  double gaitDeltaX,

  double gaitDeltaY) {

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

    backRight);

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

    backRight);
}

// ------------------------------------------------------------

// SWING PHASE

// ------------------------------------------------------------

void performSwing(

  LegIndex leg,

  double bodyShiftX,

  double bodyShiftZ,

  double startSwingProgress) {

  if (

    startSwingProgress < 0.0) {

    startSwingProgress =

      0.0;
  }

  if (

    startSwingProgress > 1.0) {

    startSwingProgress =

      1.0;
  }

  unsigned long swingStartTime =

    millis();

  unsigned long trajectoryBaseTime =

    swingStartTime;

  unsigned long virtualStartTime =

    trajectoryBaseTime - (unsigned long)GAIT_REFERENCE_CYCLE_TIME;

  swingGait.resetCycle(

    virtualStartTime);

  unsigned long remainingSwingTime =

    (unsigned long)(GAIT_SWING_TIME * (1.0 - startSwingProgress));

  while (

    remainingSwingTime > 0) {

    unsigned long elapsed =

      millis() - swingStartTime;

    if (

      elapsed > remainingSwingTime) {

      elapsed =

        remainingSwingTime;
    }

    double remainingProgress =

      (double)elapsed / (double)remainingSwingTime;

    double swingProgress =

      startSwingProgress + (1.0 - startSwingProgress) * remainingProgress;

    double gaitDeltaX;

    double gaitDeltaY;

    getSwingDeltasAtProgress(

      trajectoryBaseTime,

      swingProgress,

      gaitDeltaX,

      gaitDeltaY);

    commandSwingFrame(

      leg,

      bodyShiftX,

      bodyShiftZ,

      gaitDeltaX,

      gaitDeltaY);

    updateIMU();

    if (

      elapsed >= remainingSwingTime) {

      break;
    }

    delay(

      CONTROL_PERIOD_MS);
  }

  // ----------------------------------------------------------

  // EXACT TOUCHDOWN

  // ----------------------------------------------------------

  commandSwingFrame(

    leg,

    bodyShiftX,

    bodyShiftZ,

    -GAIT_STRIDE_LENGTH,

    0.0);

  // Save landed foot location.

  addXToLeg(

    leg,

    -GAIT_STRIDE_LENGTH);

  commandBodyShift(

    bodyShiftX,

    bodyShiftZ);

  waitWithIMU(

    TOUCHDOWN_SETTLE_TIME);
}

// ------------------------------------------------------------

// ADD 1.25 CM STANCE ADVANCE

// ------------------------------------------------------------

// ADD 1.25 CM STANCE ADVANCE TO THE LOGICAL BASELINE

// ------------------------------------------------------------

void addStanceAdvanceToState() {

  plannedFrontLeft.x +=

    STANCE_ADVANCE_PER_STEP;

  plannedFrontRight.x +=

    STANCE_ADVANCE_PER_STEP;

  plannedBackLeft.x +=

    STANCE_ADVANCE_PER_STEP;

  plannedBackRight.x +=

    STANCE_ADVANCE_PER_STEP;

  balancedFrontLeft.x +=

    STANCE_ADVANCE_PER_STEP;

  balancedFrontRight.x +=

    STANCE_ADVANCE_PER_STEP;

  balancedBackLeft.x +=

    STANCE_ADVANCE_PER_STEP;

  balancedBackRight.x +=

    STANCE_ADVANCE_PER_STEP;
}

// ------------------------------------------------------------

// DIRECT SUPPORT-TO-SUPPORT TRANSITION

//

// THIS IS THE MAJOR SPEED CHANGE.

//

// Previously:

//

// current support

//     ↓

// neutral

//     ↓

// move forward 1.25 cm

//     ↓

// next support

//

// Now:

//

// current support

//     ↓

// directly to next support

//

// WHILE simultaneously performing the 1.25 cm stance advance.

//

// All four feet are on the ground during this movement.

// ------------------------------------------------------------

bool transitionAfterTouchdown(

  double currentShiftX,

  double currentShiftZ,

  bool hasNextLeg,

  LegIndex nextLeg,

  double &nextShiftX,

  double &nextShiftZ,

  double &nextSwingStartProgress) {

  nextSwingStartProgress =

    0.0;

  // Use a slightly slower support transfer only when the next
  // swing leg is Back Left. The other transitions stay faster.

  unsigned long transitionDuration =

    SUPPORT_TRANSITION_TIME;

  if (

    hasNextLeg &&

    nextLeg == BACK_LEFT) {

    transitionDuration =

      BACK_LEFT_SUPPORT_TRANSITION_TIME;
  }

  // ----------------------------------------------------------

  // SAVE OLD BASELINE

  // ----------------------------------------------------------

  FootTarget oldBalancedFrontLeft =

    balancedFrontLeft;

  FootTarget oldBalancedFrontRight =

    balancedFrontRight;

  FootTarget oldBalancedBackLeft =

    balancedBackLeft;

  FootTarget oldBalancedBackRight =

    balancedBackRight;

  FootTarget oldPlannedFrontLeft =

    plannedFrontLeft;

  FootTarget oldPlannedFrontRight =

    plannedFrontRight;

  FootTarget oldPlannedBackLeft =

    plannedBackLeft;

  FootTarget oldPlannedBackRight =

    plannedBackRight;

  // ----------------------------------------------------------

  // CURRENT PHYSICAL SUPPORT POSITION

  // ----------------------------------------------------------

  FootTarget startFrontLeft =

    oldBalancedFrontLeft;

  FootTarget startFrontRight =

    oldBalancedFrontRight;

  FootTarget startBackLeft =

    oldBalancedBackLeft;

  FootTarget startBackRight =

    oldBalancedBackRight;

  applyBodyShiftToTargets(

    currentShiftX,

    currentShiftZ,

    startFrontLeft,

    startFrontRight,

    startBackLeft,

    startBackRight);

  // ----------------------------------------------------------

  // LOGICALLY ADVANCE THE CHASSIS 1.25 CM

  // ----------------------------------------------------------

  addStanceAdvanceToState();

  // ----------------------------------------------------------

  // CALCULATE NEXT SUPPORT SHIFT USING THE ADVANCED GEOMETRY

  // ----------------------------------------------------------

  if (hasNextLeg) {

    if (

      !calculateSupportPolygonShift(

        nextLeg,

        nextShiftX,

        nextShiftZ)) {

      // Restore logical state if the next support polygon is

      // invalid.

      balancedFrontLeft =

        oldBalancedFrontLeft;

      balancedFrontRight =

        oldBalancedFrontRight;

      balancedBackLeft =

        oldBalancedBackLeft;

      balancedBackRight =

        oldBalancedBackRight;

      plannedFrontLeft =

        oldPlannedFrontLeft;

      plannedFrontRight =

        oldPlannedFrontRight;

      plannedBackLeft =

        oldPlannedBackLeft;

      plannedBackRight =

        oldPlannedBackRight;

      return false;
    }

  }

  else {

    // After BL, finish the stance advance at the neutral

    // four-foot baseline before rebalancing.

    nextShiftX =

      0.0;

    nextShiftZ =

      0.0;
  }

  // ----------------------------------------------------------

  // END POSITION = ADVANCED BASELINE + NEXT SUPPORT SHIFT

  // ----------------------------------------------------------

  FootTarget endFrontLeft =

    balancedFrontLeft;

  FootTarget endFrontRight =

    balancedFrontRight;

  FootTarget endBackLeft =

    balancedBackLeft;

  FootTarget endBackRight =

    balancedBackRight;

  applyBodyShiftToTargets(

    nextShiftX,

    nextShiftZ,

    endFrontLeft,

    endFrontRight,

    endBackLeft,

    endBackRight);

  // ----------------------------------------------------------

  // ZERO-TIME FALLBACK

  // ----------------------------------------------------------

  if (

    transitionDuration == 0) {

    commandFootTargets(

      endFrontLeft,

      endFrontRight,

      endBackLeft,

      endBackRight);

    return true;
  }

  // ----------------------------------------------------------

  // DIRECT SUPPORT TRANSITION + NORMAL-SPEED EARLY LIFT

  // ----------------------------------------------------------

  unsigned long transitionStartTime =

    millis();

  unsigned long trajectoryBaseTime =

    transitionStartTime;

  unsigned long virtualStartTime =

    trajectoryBaseTime - (unsigned long)GAIT_REFERENCE_CYCLE_TIME;

  swingGait.resetCycle(

    virtualStartTime);

  while (true) {

    unsigned long elapsed =

      millis() - transitionStartTime;

    if (

      elapsed > transitionDuration) {

      elapsed =

        transitionDuration;
    }

    double transitionProgress =

      (double)elapsed / (double)transitionDuration;

    double bodyProgress =

      smoothBodyShift(

        transitionProgress);

    FootTarget frontLeft =

      blendTarget(

        startFrontLeft,

        endFrontLeft,

        bodyProgress);

    FootTarget frontRight =

      blendTarget(

        startFrontRight,

        endFrontRight,

        bodyProgress);

    FootTarget backLeft =

      blendTarget(

        startBackLeft,

        endBackLeft,

        bodyProgress);

    FootTarget backRight =

      blendTarget(

        startBackRight,

        endBackRight,

        bodyProgress);

    // Only the upcoming leg receives a swing offset, and that

    // offset advances at the normal GAIT_SWING_TIME rate.

    if (hasNextLeg) {

      double swingProgress =

        getOverlapSwingProgress(

          elapsed,

          transitionDuration);

      if (

        swingProgress > 0.0) {

        double gaitDeltaX;

        double gaitDeltaY;

        getSwingDeltasAtProgress(

          trajectoryBaseTime,

          swingProgress,

          gaitDeltaX,

          gaitDeltaY);

        applySwingOffsetToTargets(

          nextLeg,

          gaitDeltaX,

          gaitDeltaY,

          frontLeft,

          frontRight,

          backLeft,

          backRight);
      }
    }

    commandFootTargets(

      frontLeft,

      frontRight,

      backLeft,

      backRight);

    updateIMU();

    if (

      elapsed >= transitionDuration) {

      break;
    }

    delay(

      OVERLAP_UPDATE_PERIOD_MS);
  }

  // ----------------------------------------------------------

  // EXACT FINAL COMMAND

  // ----------------------------------------------------------

  if (hasNextLeg) {

    nextSwingStartProgress =

      getOverlapSwingProgress(

        transitionDuration,

        transitionDuration);

    if (

      nextSwingStartProgress > 0.0) {

      double gaitDeltaX;

      double gaitDeltaY;

      getSwingDeltasAtProgress(

        trajectoryBaseTime,

        nextSwingStartProgress,

        gaitDeltaX,

        gaitDeltaY);

      applySwingOffsetToTargets(

        nextLeg,

        gaitDeltaX,

        gaitDeltaY,

        endFrontLeft,

        endFrontRight,

        endBackLeft,

        endBackRight);
    }
  }

  commandFootTargets(

    endFrontLeft,

    endFrontRight,

    endBackLeft,

    endBackRight);

  return true;
}

// ============================================================

// SIT TO STAND / IMU

// ============================================================

// SIT TO STAND / IMU

// ============================================================

void runSitToStand() {

  while (

    !frontLeftStand.isFinished() || !frontRightStand.isFinished() || !backLeftStand.isFinished() || !backRightStand.isFinished()) {

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

    ACCEL_Z_SCALE);

  orientation.setGyroscopeCalibration(

    GYRO_X_OFFSET,

    GYRO_Y_OFFSET,

    GYRO_Z_OFFSET);

  orientation.setComplementaryAlpha(

    COMPLEMENTARY_ALPHA);

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

    millis() - startTime < IMU_SETTLE_TIME) {

    orientation.update();

    commandStandingPosition();

    delay(

      CONTROL_PERIOD_MS);
  }
}

// ============================================================

// BALANCE

// ============================================================

void configureBalanceController() {

  balance.setGeometry(

    BODY_LENGTH,

    BODY_WIDTH);

  balance.setRotationCenterOffset(

    COM_OFFSET_X,

    COM_OFFSET_Z);

  balance.setTargetAngles(

    TARGET_ROLL,

    TARGET_PITCH);

  balance.setCorrectionSigns(

    ROLL_CORRECTION_SIGN,

    PITCH_CORRECTION_SIGN);

  balance.setMaximumCorrectionAngles(

    12.0f,

    12.0f);

  balance.setMaximumSafeAngle(

    60.0f);

  balance.setDeadband(

    BALANCE_DEADBAND);

  balance.setOutputFilterAlpha(

    BALANCE_OUTPUT_FILTER_ALPHA);

  balance.reset();

  balance.setEnabled(

    true);
}

// ------------------------------------------------------------

// BALANCE CURRENT PLAN

//

// Active balancing is used:

//

// 1. once before walking

// 2. once after each complete four-leg crawl cycle

// ------------------------------------------------------------

bool balanceCurrentPlan(

  unsigned long maximumBalanceTime,

  unsigned long requiredLevelTime,

  unsigned long blendTime) {

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

    millis() - balanceStartTime < maximumBalanceTime) {

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

          plannedBackRight);

      if (

        output.safetyStopped) {

        balance.setEnabled(

          false);

        return false;
      }

      float blendWeight =

        1.0f;

      if (

        blendTime > 0) {

        blendWeight =

          (float)(millis() - balanceStartTime) / (float)blendTime;

        if (

          blendWeight > 1.0f) {

          blendWeight =

            1.0f;
        }
      }

      FootTarget commandedFrontLeft =

        blendTarget(

          startingFrontLeft,

          output.frontLeft,

          blendWeight);

      FootTarget commandedFrontRight =

        blendTarget(

          startingFrontRight,

          output.frontRight,

          blendWeight);

      FootTarget commandedBackLeft =

        blendTarget(

          startingBackLeft,

          output.backLeft,

          blendWeight);

      FootTarget commandedBackRight =

        blendTarget(

          startingBackRight,

          output.backRight,

          blendWeight);

      commandFootTargets(

        commandedFrontLeft,

        commandedFrontRight,

        commandedBackLeft,

        commandedBackRight);

      if (

        !haveSmoothedPose) {

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

            poseSmoothing);

        smoothedFrontRight =

          blendTarget(

            smoothedFrontRight,

            commandedFrontRight,

            poseSmoothing);

        smoothedBackLeft =

          blendTarget(

            smoothedBackLeft,

            commandedBackLeft,

            poseSmoothing);

        smoothedBackRight =

          blendTarget(

            smoothedBackRight,

            commandedBackRight,

            poseSmoothing);
      }

      float roll =

        orientation.getRoll();

      float pitch =

        orientation.getPitch();

      bool blendFinished =

        blendWeight >= 0.999f;

      if (

        blendFinished && fabs(roll) <= LEVEL_TOLERANCE && fabs(pitch) <= LEVEL_TOLERANCE) {

        if (

          levelStartTime == 0) {

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

        flX +=

          commandedFrontLeft.x;

        flY +=

          commandedFrontLeft.y;

        flZ +=

          commandedFrontLeft.z;

        frX +=

          commandedFrontRight.x;

        frY +=

          commandedFrontRight.y;

        frZ +=

          commandedFrontRight.z;

        blX +=

          commandedBackLeft.x;

        blY +=

          commandedBackLeft.y;

        blZ +=

          commandedBackLeft.z;

        brX +=

          commandedBackRight.x;

        brY +=

          commandedBackRight.y;

        brZ +=

          commandedBackRight.z;

        levelSampleCount++;

        if (

          millis() - levelStartTime >= requiredLevelTime) {

          if (

            levelSampleCount > 0) {

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

    if (

      elapsedTime < CONTROL_PERIOD_MS) {

      delay(

        CONTROL_PERIOD_MS - elapsedTime);
    }
  }

  if (

    !stablePoseFound && haveSmoothedPose) {

    balancedFrontLeft =

      smoothedFrontLeft;

    balancedFrontRight =

      smoothedFrontRight;

    balancedBackLeft =

      smoothedBackLeft;

    balancedBackRight =

      smoothedBackRight;
  }

  balance.setEnabled(

    false);

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

    0);
}

// ------------------------------------------------------------

// END-OF-CYCLE REBALANCE

// ------------------------------------------------------------

bool rebalanceRobot() {

  waitWithIMU(

    REBALANCE_SETTLE_TIME);

  return balanceCurrentPlan(

    MAX_REBALANCE_TIME,

    REBALANCE_LEVEL_HOLD_TIME,

    REBALANCE_BLEND_TIME);
}

// ============================================================

// WALKING

// ============================================================

// ------------------------------------------------------------

// RUN ONE SWING WHEN THE ROBOT IS ALREADY IN THAT LEG'S

// SUPPORT POSITION.

//

// Afterwards it moves DIRECTLY into the next support position

// while simultaneously advancing the stance.

// ------------------------------------------------------------

bool runPreparedCrawlStep(

  LegIndex leg,

  double currentShiftX,

  double currentShiftZ,

  double currentSwingStartProgress,

  bool hasNextLeg,

  LegIndex nextLeg,

  double &nextShiftX,

  double &nextShiftZ,

  double &nextSwingStartProgress) {

  if (

    SHIFT_SETTLE_TIME > 0) {

    waitWithIMU(

      SHIFT_SETTLE_TIME);
  }

  // Continue the swing from the progress already completed

  // during the preceding COM transition.

  performSwing(

    leg,

    currentShiftX,

    currentShiftZ,

    currentSwingStartProgress);

  // After touchdown, advance the chassis and move directly to

  // the next support position. The next leg begins its normal

  // swing near the end of that transition.

  if (

    !transitionAfterTouchdown(

      currentShiftX,

      currentShiftZ,

      hasNextLeg,

      nextLeg,

      nextShiftX,

      nextShiftZ,

      nextSwingStartProgress)) {

    return false;
  }

  return true;
}

// ------------------------------------------------------------

// EMERGENCY HOLD

// ------------------------------------------------------------

// EMERGENCY HOLD

// ------------------------------------------------------------

void haltRobot() {

  balance.setEnabled(

    false);

  commandBalancedPosition();

  while (true) {

    updateIMU();

    delay(

      CONTROL_PERIOD_MS);
  }
}

// ------------------------------------------------------------

// COMPLETE CRAWL CYCLE

//

// NEW FLOW:

//

// neutral

//   ↓

// FL support

//   ↓

// FL swing

//   ↓

// directly to BR support + stance advance

//   ↓

// BR swing

//   ↓

// directly to FR support + stance advance

//   ↓

// FR swing

//   ↓

// directly to BL support + stance advance

//   ↓

// BL swing

//   ↓

// neutral + final stance advance

//   ↓

// one rebalance

//

// The robot no longer visits neutral between every leg.

// ------------------------------------------------------------

bool runCrawlCycle() {

  double currentShiftX;

  double currentShiftZ;

  double nextShiftX;

  double nextShiftZ;

  double currentSwingStartProgress =

    0.0;

  double nextSwingStartProgress =

    0.0;

  // ----------------------------------------------------------

  // CALCULATE FRONT-LEFT SUPPORT POSITION

  // ----------------------------------------------------------

  if (

    !calculateSupportPolygonShift(

      FRONT_LEFT,

      currentShiftX,

      currentShiftZ)) {

    return false;
  }

  // Move toward FL support. FL begins its normal-speed swing only

  // during the final portion of the COM transition.

  moveIntoInitialSupportShiftAndPreLift(

    FRONT_LEFT,

    currentShiftX,

    currentShiftZ,

    currentSwingStartProgress);

  // ----------------------------------------------------------

  // FRONT LEFT

  // ----------------------------------------------------------

  if (

    !runPreparedCrawlStep(

      FRONT_LEFT,

      currentShiftX,

      currentShiftZ,

      currentSwingStartProgress,

      true,

      BACK_RIGHT,

      nextShiftX,

      nextShiftZ,

      nextSwingStartProgress)) {

    return false;
  }

  currentShiftX =

    nextShiftX;

  currentShiftZ =

    nextShiftZ;

  currentSwingStartProgress =

    nextSwingStartProgress;

  // ----------------------------------------------------------

  // BACK RIGHT

  // ----------------------------------------------------------

  if (

    !runPreparedCrawlStep(

      BACK_RIGHT,

      currentShiftX,

      currentShiftZ,

      currentSwingStartProgress,

      true,

      FRONT_RIGHT,

      nextShiftX,

      nextShiftZ,

      nextSwingStartProgress)) {

    return false;
  }

  currentShiftX =

    nextShiftX;

  currentShiftZ =

    nextShiftZ;

  currentSwingStartProgress =

    nextSwingStartProgress;

  // ----------------------------------------------------------

  // FRONT RIGHT

  // ----------------------------------------------------------

  if (

    !runPreparedCrawlStep(

      FRONT_RIGHT,

      currentShiftX,

      currentShiftZ,

      currentSwingStartProgress,

      true,

      BACK_LEFT,

      nextShiftX,

      nextShiftZ,

      nextSwingStartProgress)) {

    return false;
  }

  currentShiftX =

    nextShiftX;

  currentShiftZ =

    nextShiftZ;

  currentSwingStartProgress =

    nextSwingStartProgress;

  // ----------------------------------------------------------

  // BACK LEFT

  // ----------------------------------------------------------

  if (

    !runPreparedCrawlStep(

      BACK_LEFT,

      currentShiftX,

      currentShiftZ,

      currentSwingStartProgress,

      false,

      FRONT_LEFT,

      nextShiftX,

      nextShiftZ,

      nextSwingStartProgress)) {

    return false;
  }

  // ----------------------------------------------------------

  // ONE REBALANCE AFTER ALL FOUR LEGS

  // ----------------------------------------------------------

  if (

    !rebalanceRobot()) {

    return false;
  }

  waitWithIMU(

    WAIT_AFTER_REBALANCE);

  return true;
}

// ============================================================

// SETUP

// ============================================================

// SETUP

// ============================================================

void setup() {

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

    !initializeIMU()) {

    commandStandingPosition();

    while (true) {

      delay(

        1000);
    }
  }

  // ----------------------------------------------------------

  // 3. SETTLE IMU

  // ----------------------------------------------------------

  settleIMU();

  // ----------------------------------------------------------

  // 4. INITIAL BALANCE

  // ----------------------------------------------------------

  if (

    !initialBalance()) {

    haltRobot();
  }

  waitWithIMU(

    WAIT_AFTER_INITIAL_BALANCE);

  // ----------------------------------------------------------

  // 5. WALK

  // ----------------------------------------------------------

  for (

    int cycle = 0;

    cycle < CRAWL_CYCLES_TO_RUN;

    cycle++) {

    if (

      !runCrawlCycle()) {

      haltRobot();
    }
  }

  // ----------------------------------------------------------

  // 6. FINAL HOLD

  // ----------------------------------------------------------

  commandBalancedPosition();
}

// ------------------------------------------------------------

// LOOP

// ------------------------------------------------------------

void loop() {

  updateIMU();

  delay(

    CONTROL_PERIOD_MS);
}