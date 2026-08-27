#include "SitToStand.h"
#include <Arduino.h>
#include <math.h>


SitToStand::SitToStand(InverseKinematics &l,
                      double sx, double sy, double sz,
                      double tx, double ty, double tz)
 : leg(l) {
 sitX = sx;
 sitY = sy;
 sitZ = sz;

 standX = tx;
 standY = ty;
 standZ = tz;
}


void SitToStand::updateTransition() {
 if (finished) {
   leg.updateTarget(standX, standY, standZ);
   return;
 }

 timer = millis() - count * delayTime;

 progress = timer / transitionTime;

 if (progress >= 1.0) {
   progress = 1.0;
   finished = true;
 }

 // cubic smoothstep:
 // r(t) = 3t^2 - 2t^3
 r = progress * progress * (3.0 - 2.0 * progress);

 double targetX = sitX + (standX - sitX) * r;
 double targetY = sitY + (standY - sitY) * r;
 double targetZ = sitZ + (standZ - sitZ) * r;

 leg.updateTarget(targetX, targetY, targetZ);

 count++;
 delay(delayTime);
}

bool SitToStand::isFinished() {
 return finished;
}