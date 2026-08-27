#ifndef GAIT_CYCLE_H
#define GAIT_CYCLE_H


#include <Arduino.h>


class GaitCycle {
 private:
   double initialX;
   double initialY;


   double strideLength;
   double height;


   double swingStart;
   double cycleTime;
   double swingFraction;
   double stanceFraction;
   double compression;
   double direction;


   double startupRampTime;


   unsigned long startTime;


   double getPhase(unsigned long currentTime);


 public:
   GaitCycle(double x0, double y0, double s, double h, double start, double cycle, double swing, double comp, double dir);
   void resetCycle(unsigned long sharedStartTime);
   void getTarget(unsigned long currentTime, double &targetX, double &targetY, double &targetZ);
   double getStanceBlend(unsigned long currentTime);
};


#endif