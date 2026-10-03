#ifndef DANCE_CONTROLLER_H
#define DANCE_CONTROLLER_H

#include <Arduino.h>
#include "BalanceController.h"


// ============================================================
// DANCE POSE
// ============================================================
//
// forwardCm:
//   + = body moves forward
//   - = body moves backward
//
// rightCm:
//   + = body moves right
//   - = body moves left
//
// heightCm:
//   + = body rises
//   - = body lowers
//
// rollDeg:
//   + = left side lowers
//   - = right side lowers
//
// pitchDeg:
//   + = nose lowers
//   - = nose rises
//
// durationMs:
//   time used to smoothly move INTO this pose
// ============================================================

struct DancePose {
  float forwardCm;
  float rightCm;
  float heightCm;

  float rollDeg;
  float pitchDeg;

  unsigned long durationMs;
};


// ============================================================
// DANCE OUTPUT
// ============================================================

struct DanceOutput {
  FootTarget frontLeft;
  FootTarget frontRight;
  FootTarget backLeft;
  FootTarget backRight;
};


// ============================================================
// DANCE CONTROLLER
// ============================================================

class DanceController {
private:

  float bodyLength;
  float bodyWidth;

  float rotationCenterOffsetX;
  float rotationCenterOffsetZ;


  // ----------------------------------------------------------
  // BALANCED BASELINE
  // ----------------------------------------------------------

  FootTarget baseFrontLeft;
  FootTarget baseFrontRight;
  FootTarget baseBackLeft;
  FootTarget baseBackRight;


  // ----------------------------------------------------------
  // MOTION STATE
  // ----------------------------------------------------------

  DancePose fromPose;
  DancePose currentPose;
  DancePose stopFromPose;

  size_t frameIndex;

  unsigned long frameStartTime;
  unsigned long stopStartTime;

  bool initialized;
  bool running;
  bool stopping;


  // Time used to smoothly return to neutral after pressing S.
  static const unsigned long STOP_RETURN_TIME_MS = 700;


  // ----------------------------------------------------------
  // INTERNAL HELPERS
  // ----------------------------------------------------------

  DancePose neutralPose() const;

  DancePose interpolatePose(
    const DancePose &startPose,
    const DancePose &endPose,
    float weight
  ) const;


  float smoothProgress(
    float progress
  ) const;


  FootTarget transformTarget(
    const FootTarget &baseTarget,

    float hipX,
    float hipZ,

    bool isLeftLeg,

    const DancePose &pose
  ) const;


  DanceOutput buildOutput(
    const DancePose &pose
  ) const;


public:

  DanceController(
    float bodyLengthCm,
    float bodyWidthCm,

    float rotationCenterOffsetXcm,
    float rotationCenterOffsetZcm
  );


  void begin(
    const FootTarget &frontLeft,
    const FootTarget &frontRight,
    const FootTarget &backLeft,
    const FootTarget &backRight
  );


  void start();

  void stop();

  void forceNeutral();


  bool isActive() const;


  DanceOutput update();
};


#endif