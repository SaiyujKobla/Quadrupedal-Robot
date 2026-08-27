#include "InverseKinematics.h"
#include "SitToStand.h"


InverseKinematics frontLeftLeg(17, 19, 18, 0);
InverseKinematics backLeftLeg(23, 21, 22, 1);
InverseKinematics frontRightLeg(13, 27, 14, 2);
InverseKinematics backRightLeg(32, 26, 25, 3);


const float SIT_X = 0.0f;
const float SIT_Y = 8.5f;
const float SIT_Z = 0.0f;

const float STANDING_X = 0.0f;
const float STANDING_Y = 20.0f;
const float STANDING_Z = 0.0f;


const float LEFT_TEST_Y = 20 + 1.5f;
const float RIGHT_TEST_Y = 20 - 1.5f;


const unsigned long WAIT_BEFORE_TEST_MS = 3000;
const unsigned long TEST_DURATION_MS = 4000;


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


enum TestState {
  STANDING_UP,
  WAITING,
  TESTING,
  FINISHED
};


TestState state = STANDING_UP;

unsigned long stateStartTime = 0;


void commandLegs(
  float frontLeftY,
  float frontRightY,
  float backLeftY,
  float backRightY
) {
  frontLeftLeg.updateTarget(
    0.0f,
    frontLeftY,
    0.0f
  );

  frontRightLeg.updateTarget(
    0.0f,
    frontRightY,
    0.0f
  );

  backLeftLeg.updateTarget(
    0.0f,
    backLeftY,
    0.0f
  );

  backRightLeg.updateTarget(
    0.0f,
    backRightY,
    0.0f
  );
}


void commandStandingPosition() {
  commandLegs(
    STANDING_Y,
    STANDING_Y,
    STANDING_Y,
    STANDING_Y
  );
}


void commandTestPosition() {
  commandLegs(
    LEFT_TEST_Y,
    RIGHT_TEST_Y,
    LEFT_TEST_Y,
    RIGHT_TEST_Y
  );
}


bool sitToStandFinished() {
  return
    frontLeftStand.isFinished() &&
    frontRightStand.isFinished() &&
    backLeftStand.isFinished() &&
    backRightStand.isFinished();
}


void updateSitToStand() {
  frontLeftStand.updateTransition();
  frontRightStand.updateTransition();
  backLeftStand.updateTransition();
  backRightStand.updateTransition();
}


void setup() {
  frontLeftLeg.begin();
  backLeftLeg.begin();
  frontRightLeg.begin();
  backRightLeg.begin();
}


void loop() {

  switch (state) {

    case STANDING_UP:

      updateSitToStand();

      if (sitToStandFinished()) {
        commandStandingPosition();

        stateStartTime = millis();
        state = WAITING;
      }

      break;


    case WAITING:

      commandStandingPosition();

      if (
        millis() - stateStartTime >=
        WAIT_BEFORE_TEST_MS
      ) {
        stateStartTime = millis();
        state = TESTING;
      }

      break;


    case TESTING:

      commandTestPosition();

      if (
        millis() - stateStartTime >=
        TEST_DURATION_MS
      ) {
        commandStandingPosition();

        state = FINISHED;
      }

      break;


    case FINISHED:

      commandStandingPosition();

      break;
  }
}