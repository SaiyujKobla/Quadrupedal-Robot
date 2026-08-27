#include "InverseKinematics.h"
#include "IMUOrientation.h"
#include "BalanceController.h"
#include "SitToStand.h"

InverseKinematics frontLeftLeg(17, 19, 18, 0);
InverseKinematics backLeftLeg(23, 21, 22, 1);
InverseKinematics frontRightLeg(13, 27, 14, 2);
InverseKinematics backRightLeg(32, 26, 25, 3);

IMUOrientation orientation(33, 16);

const float ACCEL_X_OFFSET = -0.11622900f;
const float ACCEL_Y_OFFSET =  0.08589700f;
const float ACCEL_Z_OFFSET =  1.30524301f;

const float ACCEL_X_SCALE = 0.99895497f;
const float ACCEL_Y_SCALE = 0.99418946f;
const float ACCEL_Z_SCALE = 0.99225240f;

const float GYRO_X_OFFSET = 0.00766f;
const float GYRO_Y_OFFSET = 0.01859f;
const float GYRO_Z_OFFSET = 0.00223f;

const float COMPLEMENTARY_ALPHA = 0.98f;

const float SIT_X = 0.0f;
const float SIT_Y = 8.5f;
const float SIT_Z = 0.0f;

const float STANDING_X = 0.0f;
const float STANDING_Y = 20.0f;
const float STANDING_Z = 0.0f;

/*
  These need to be HIP-PIVOT spacings,
  not necessarily foot-center spacings.

  Replace them if your measured hip-to-hip
  dimensions differ.
*/

const float BODY_LENGTH = 20.0f;
const float BODY_WIDTH = 19.45f;

/*
  Rotation-center offset.

  +X = toward front
  +Z = toward robot's right

  Keep zero initially.
*/

const float COM_OFFSET_X = 0.0f;
const float COM_OFFSET_Z = 0.0f;


const float TARGET_ROLL = 0.0f;
const float TARGET_PITCH = 0.0f;


/*
  TEST ONE AXIS AT A TIME.

  Pitch is enabled here.
*/

const float ROLL_KP = 1.0f;
const float ROLL_KD = 0.0f;

const float PITCH_KP = 1.0f;
const float PITCH_KD = 0.0f;


/*
  These may need to be flipped after the
  first very small physical test.

  They determine whether the corrective
  geometry opposes or reinforces the tilt.
*/

const float ROLL_CORRECTION_SIGN = 1.0f;
const float PITCH_CORRECTION_SIGN = 1.0f;


BalanceController balance(
  BODY_LENGTH,
  BODY_WIDTH,

  ROLL_KP,
  ROLL_KD,

  PITCH_KP,
  PITCH_KD
);


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


const unsigned long CONTROL_PERIOD_US =
  20000;


const unsigned long BALANCE_DELAY_AFTER_STAND_MS =
  3000;


unsigned long standFinishedTime =
  0;


unsigned long previousControlTime =
  0;


bool imuWorking =
  false;


bool standFinished =
  false;


bool balanceStarted =
  false;


bool safetyStopped =
  false;


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


void updateSitToStand() {

  frontLeftStand.updateTransition();

  frontRightStand.updateTransition();

  backLeftStand.updateTransition();

  backRightStand.updateTransition();


  if (
    frontLeftStand.isFinished() &&
    frontRightStand.isFinished() &&
    backLeftStand.isFinished() &&
    backRightStand.isFinished()
  ) {

    standFinished =
      true;


    standFinishedTime =
      millis();


    commandStandingPosition();
  }
}


void setup() {

  frontLeftLeg.begin();

  backLeftLeg.begin();

  frontRightLeg.begin();

  backRightLeg.begin();
}


void loop() {

  /*
    1. Known-good sit-to-stand.
  */

  if (!standFinished) {

    updateSitToStand();

    return;
  }


  /*
    2. Initialize the IMU only once the
       robot is standing.
  */

  if (!imuWorking) {

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


    imuWorking =
      orientation.begin();


    if (!imuWorking) {

      commandStandingPosition();

      return;
    }


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


    /*
      Keep this conservative for the first
      test of the NEW geometry.

      A 5 degree maximum correction is safer
      than immediately allowing 10 degrees.
    */

    balance.setMaximumCorrectionAngles(
      30.0f,
      30.0f
    );


    balance.setMaximumSafeAngle(
      60.0f
    );


    balance.setDeadband(
      0.2f
    );


    balance.setOutputFilterAlpha(
      1.0f
    );


    balance.setEnabled(
      false
    );


    previousControlTime =
      micros();


    standFinishedTime =
      millis();


    return;
  }


  /*
    3. Continuously update orientation.
  */

  bool validImuReading =
    orientation.update();


  /*
    4. Allow the complementary filter
       to settle before enabling balance.
  */

  if (
    millis() -
    standFinishedTime <
    BALANCE_DELAY_AFTER_STAND_MS
  ) {

    commandStandingPosition();

    return;
  }


  unsigned long currentTime =
    micros();


  if (
    currentTime -
    previousControlTime <
    CONTROL_PERIOD_US
  ) {

    return;
  }


  previousControlTime =
    currentTime;


  /*
    Hold the previous motor command if a
    single IMU sample is invalid.
  */

  if (!validImuReading) {

    return;
  }


  if (safetyStopped) {

    commandStandingPosition();

    return;
  }


  if (!balanceStarted) {

    balance.reset();


    balance.setEnabled(
      true
    );


    balanceStarted =
      true;
  }


  BalanceOutput output =
    balance.update(

      orientation.getRoll(),
      orientation.getPitch(),

      orientation.getRollRate(),
      orientation.getPitchRate(),

      STANDING_FRONT_LEFT,
      STANDING_FRONT_RIGHT,
      STANDING_BACK_LEFT,
      STANDING_BACK_RIGHT
    );


  if (
    output.safetyStopped
  ) {

    balance.setEnabled(
      false
    );


    safetyStopped =
      true;


    commandStandingPosition();


    return;
  }


  commandFootTargets(
    output.frontLeft,
    output.frontRight,
    output.backLeft,
    output.backRight
  );
}
// #include "InverseKinematics.h"
// #include "SitToStand.h"


// InverseKinematics frontLeftLeg(17, 19, 18, 0);
// InverseKinematics backLeftLeg(23, 21, 22, 1);
// InverseKinematics frontRightLeg(13, 27, 14, 2);
// InverseKinematics backRightLeg(32, 26, 25, 3);


// const double SIT_X = 0.0;
// const double SIT_Y = 8.5;
// const double SIT_Z = 0.0;


// const double STANDING_X = 0.0;
// const double STANDING_Y = 20.0;
// const double STANDING_Z = 0.0;


// /*
//   IMPORTANT:

//   This must be the distance from the
//   FRONT hip pivot axis to the
//   REAR hip pivot axis.

//   Do not use foot spacing here.
// */

// const double BODY_LENGTH = 20.0;


// const double TEST_PITCH = 20.0;


// const unsigned long TRANSITION_TIME_MS = 2000;
// const unsigned long HOLD_TIME_MS = 5000;


// SitToStand frontLeftStand(
//   frontLeftLeg,
//   SIT_X,
//   SIT_Y,
//   SIT_Z,
//   STANDING_X,
//   STANDING_Y,
//   STANDING_Z
// );


// SitToStand frontRightStand(
//   frontRightLeg,
//   SIT_X,
//   SIT_Y,
//   SIT_Z,
//   STANDING_X,
//   STANDING_Y,
//   STANDING_Z
// );


// SitToStand backLeftStand(
//   backLeftLeg,
//   SIT_X,
//   SIT_Y,
//   SIT_Z,
//   STANDING_X,
//   STANDING_Y,
//   STANDING_Z
// );


// SitToStand backRightStand(
//   backRightLeg,
//   SIT_X,
//   SIT_Y,
//   SIT_Z,
//   STANDING_X,
//   STANDING_Y,
//   STANDING_Z
// );


// bool standingComplete = false;


// double smoothstep(
//   double t
// ) {

//   if (t < 0.0) {
//     t = 0.0;
//   }

//   if (t > 1.0) {
//     t = 1.0;
//   }


//   return
//     t *
//     t *
//     (
//       3.0 -
//       2.0 *
//       t
//     );
// }


// void commandPitchAngle(
//   double pitchDegrees
// ) {

//   double pitch =
//     radians(
//       pitchDegrees
//     );


//   double halfBodyLength =
//     BODY_LENGTH /
//     2.0;


//   double height =
//     STANDING_Y;


//   /*
//     These are Sean's pitch equations,
//     simplified algebraically.

//     BODY:
//       +X = forward
//       +Y = downward

//     LOCAL IK:
//       +X = backward

//     Therefore local X is the negative
//     of body X.
//   */


//   double frontBodyX =
//     height *
//     sin(pitch)
//     +
//     halfBodyLength *
//     (
//       cos(pitch) -
//       1.0
//     );


//   double frontY =
//     height *
//     cos(pitch)
//     -
//     halfBodyLength *
//     sin(pitch);


//   double backBodyX =
//     height *
//     sin(pitch)
//     -
//     halfBodyLength *
//     (
//       cos(pitch) -
//       1.0
//     );


//   double backY =
//     height *
//     cos(pitch)
//     +
//     halfBodyLength *
//     sin(pitch);


//   double frontLocalX =
//     -frontBodyX;


//   double backLocalX =
//     -backBodyX;


//   /*
//     Both front legs get EXACTLY
//     the same pitch target.

//     Both rear legs get EXACTLY
//     the same pitch target.
//   */

//   frontLeftLeg.updateTarget(
//     frontLocalX,
//     frontY,
//     0.0
//   );


//   frontRightLeg.updateTarget(
//     frontLocalX,
//     frontY,
//     0.0
//   );


//   backLeftLeg.updateTarget(
//     backLocalX,
//     backY,
//     0.0
//   );


//   backRightLeg.updateTarget(
//     backLocalX,
//     backY,
//     0.0
//   );
// }


// void movePitch(
//   double startPitch,
//   double endPitch
// ) {

//   unsigned long startTime =
//     millis();


//   while (
//     millis() -
//     startTime <
//     TRANSITION_TIME_MS
//   ) {

//     double progress =
//       (double)(
//         millis() -
//         startTime
//       )
//       /
//       (double)TRANSITION_TIME_MS;


//     double r =
//       smoothstep(
//         progress
//       );


//     double pitch =
//       startPitch +
//       (
//         endPitch -
//         startPitch
//       )
//       *
//       r;


//     commandPitchAngle(
//       pitch
//     );


//     delay(20);
//   }


//   commandPitchAngle(
//     endPitch
//   );
// }


// void holdPitch(
//   double pitch
// ) {

//   commandPitchAngle(
//     pitch
//   );


//   delay(
//     HOLD_TIME_MS
//   );
// }


// void updateSitToStand() {

//   frontLeftStand.updateTransition();

//   frontRightStand.updateTransition();

//   backLeftStand.updateTransition();

//   backRightStand.updateTransition();


//   if (
//     frontLeftStand.isFinished() &&
//     frontRightStand.isFinished() &&
//     backLeftStand.isFinished() &&
//     backRightStand.isFinished()
//   ) {

//     standingComplete =
//       true;


//     commandPitchAngle(
//       0.0
//     );


//     delay(3000);
//   }
// }


// void setup() {

//   frontLeftLeg.begin();

//   backLeftLeg.begin();

//   frontRightLeg.begin();

//   backRightLeg.begin();
// }


// void loop() {

//   if (!standingComplete) {

//     updateSitToStand();

//     return;
//   }


//   movePitch(
//     0.0,
//     TEST_PITCH
//   );


//   holdPitch(
//     TEST_PITCH
//   );


//   movePitch(
//     TEST_PITCH,
//     0.0
//   );


//   holdPitch(
//     0.0
//   );


//   movePitch(
//     0.0,
//     -TEST_PITCH
//   );


//   holdPitch(
//     -TEST_PITCH
//   );


//   movePitch(
//     -TEST_PITCH,
//     0.0
//   );


//   holdPitch(
//     0.0
//   );
// }