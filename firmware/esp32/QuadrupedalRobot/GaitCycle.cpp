#include "GaitCycle.h"
#include <math.h>


// Creates a smooth transition with zero velocity and acceleration at both endpoints.
static double smoothStep(double value) {
  return 10.0 * pow(value, 3) - 15.0 * pow(value, 4) + 6.0 * pow(value, 5);
}


// Stores the gait geometry, timing, phase offset, and walking direction for one leg.
GaitCycle::GaitCycle(double x0, double y0, double s, double h, double start, double cycle, double swing, double comp, double dir) {
  initialX = x0;
  initialY = y0;

  strideLength = s;
  height = h;

  swingStart = start;
  cycleTime = cycle;
  swingFraction = swing;
  stanceFraction = 1.0 - swingFraction;
  compression = comp;

  direction = (dir >= 0.0) ? 1.0 : -1.0;

  startupRampTime = cycleTime;

  startTime = 0;
}


// Resets the gait using the same shared timestamp as the other legs.
void GaitCycle::resetCycle(unsigned long sharedStartTime) {
  startTime = sharedStartTime;
}


// Calculates the current phase of this leg from 0.0 to 1.0.
double GaitCycle::getPhase(unsigned long currentTime) {
  double elapsed = (double)(currentTime - startTime);

  double phase = fmod((elapsed / cycleTime) - swingStart, 1.0);

  if (phase < 0.0) {
    phase += 1.0;
  }

  return phase;
}


// Calculates the current Cartesian foot target for this leg.
void GaitCycle::getTarget(unsigned long currentTime, double &targetX, double &targetY, double &targetZ) {

  // Calculates how much time has passed since the gait began.
  unsigned long elapsedMilliseconds = currentTime - startTime;
  double elapsed = (double)elapsedMilliseconds;


  // Gradually increases the gait amplitude during the first cycle to prevent a sudden movement.
  double ramp = 1.0;

  if (startupRampTime > 0.0 && elapsed < startupRampTime) {
    ramp = smoothStep(elapsed / startupRampTime);
  }


  // Applies the startup ramp to the stride length, swing height, and stance compression.
  double activeStride = strideLength * ramp;
  double activeHeight = height * ramp;
  double activeCompression = compression * ramp;

  double halfStride = activeStride / 2.0;


  // Gets the current phase of this leg.
  double phase = getPhase(currentTime);


  // Swing phase: lifts the foot and moves it forward to the next step position.
  if (phase < swingFraction) {
    double swingProgress = phase / swingFraction;
    double r = smoothStep(swingProgress);

    targetX = initialX + direction * (halfStride - activeStride * r);
    targetY = initialY - activeHeight * sin(PI * swingProgress);
  }


  // Stance phase: moves the planted foot backward relative to the body.
  else {
    double stanceProgress = (phase - swingFraction) / stanceFraction;
    double r = smoothStep(stanceProgress);

    targetX = initialX + direction * (-halfStride + activeStride * r);
    targetY = initialY + activeCompression * sin(PI * stanceProgress);
  }


  // Planned lateral body movement is added later in the main controller.
  targetZ = 0.0;
}


// Determines how much roll and pitch balance correction should be applied to this leg.
double GaitCycle::getStanceBlend(unsigned long currentTime) {
  double phase = getPhase(currentTime);


  // Applies full balance correction whenever the foot is in stance.
  if (phase >= swingFraction) {
    return 1.0;
  }


  // Converts the swing phase into progress from 0.0 to 1.0.
  double swingProgress = phase / swingFraction;


  // Uses the first and last 20 percent of swing to smoothly change balance authority.
  const double transitionFraction = 0.20;


  // Smoothly removes balance correction immediately after liftoff.
  if (swingProgress < transitionFraction) {
    return 1.0 - smoothStep(swingProgress / transitionFraction);
  }


  // Smoothly restores balance correction immediately before touchdown.
  if (swingProgress > 1.0 - transitionFraction) {
    return smoothStep((swingProgress - (1.0 - transitionFraction)) / transitionFraction);
  }


  // The swing trajectory has complete control during the middle of swing.
  return 0.0;
}