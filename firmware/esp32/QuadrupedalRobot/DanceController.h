#ifndef DANCE_CONTROLLER_H
#define DANCE_CONTROLLER_H

#include <Arduino.h>
#include "BalanceController.h"

// A commanded body pose relative to the robot's fixed neutral stance
// captured after the first successful active balance.
//
// Coordinate/sign convention matches the existing project:
//
// forwardCm: + = body forward, - = body backward
// rightCm:   + = body right,   - = body left
// heightCm:  + = body rises,   - = body lowers
// rollDeg:   + = left side lowers, - = right side lowers
// pitchDeg:  + = nose lowers,      - = nose rises
//
// durationMs is the time used to move INTO this pose.
struct DancePose {
  float forwardCm;
  float rightCm;
  float heightCm;
  float rollDeg;
  float pitchDeg;
  unsigned long durationMs;
};

struct DanceOutput {
  FootTarget frontLeft;
  FootTarget frontRight;
  FootTarget backLeft;
  FootTarget backRight;
};

class DanceController {
private:
  float bodyLength;
  float bodyWidth;

  float rotationCenterOffsetX;
  float rotationCenterOffsetZ;

  FootTarget baseFrontLeft;
  FootTarget baseFrontRight;
  FootTarget baseBackLeft;
  FootTarget baseBackRight;

  DancePose fromPose;
  DancePose currentPose;

  const DancePose *activeSequence;
  size_t activeFrameCount;
  size_t frameIndex;

  unsigned long frameStartTime;

  bool initialized;
  bool running;
  bool finished;

  DancePose neutralPose() const;

  float smoothProgress(float progress) const;

  DancePose interpolatePose(
    const DancePose &startPose,
    const DancePose &endPose,
    float weight
  ) const;

  // This uses the SAME inverse body-rotation equations as the
  // existing BalanceController::calculateCorrectedFootTarget().
  FootTarget rotateTargetUsingBalanceMath(
    const FootTarget &baseTarget,
    float hipX,
    float hipZ,
    bool isLeftLeg,
    float rollDeg,
    float pitchDeg
  ) const;

  // Rotation is calculated first with the balance-controller math.
  // The requested body translation is then applied in the same
  // leg-local coordinate convention already used by the robot.
  FootTarget transformTarget(
    const FootTarget &baseTarget,
    float hipX,
    float hipZ,
    bool isLeftLeg,
    const DancePose &pose
  ) const;

  DanceOutput buildOutput(const DancePose &pose) const;

  void startSequence(
    const DancePose *sequence,
    size_t frameCount
  );

public:
  DanceController(
    float bodyLengthCm,
    float bodyWidthCm,
    float rotationCenterOffsetXcm,
    float rotationCenterOffsetZcm
  );

  // Set/reset the fixed base targets used by a scripted section.
  // The main sketch always supplies the SAME initially balanced pose,
  // preventing neutral-position drift across repeated routines.
  void begin(
    const FootTarget &frontLeft,
    const FootTarget &frontRight,
    const FootTarget &backLeft,
    const FootTarget &backRight
  );

  void startPitchRollSequence();
  void startSquareSequence();

  void forceNeutral();

  bool isActive() const;
  bool isFinished() const;

  DanceOutput update();
};

#endif
