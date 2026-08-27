#ifndef SIT_TO_STAND_H
#define SIT_TO_STAND_H

#include "InverseKinematics.h"

class SitToStand {
private:
 InverseKinematics &leg;

 const double transitionTime = 2000; // total sit-to-stand time (ms)
 const double delayTime = 10;        // delay per update (ms)

 double sitX;
 double sitY;
 double sitZ;

 double standX;
 double standY;
 double standZ;

 double progress;
 double r;

 double timer;
 int count = 0;
 bool finished = false;

public:
 SitToStand(InverseKinematics &l,
            double sx, double sy, double sz,
            double tx, double ty, double tz);

 void updateTransition();
 bool isFinished();
};

#endif