#include <Servo.h>
#include <math.h>


Servo upperMotor;
Servo lowerMotor;


double targetX;
double targetY;


double targetYPrime;
double targetZ;


const double upperLeg = 12;
const double lowerLeg = 13.5;


double lowerAngle;
double upperAngle;


void setup() {
 upperMotor.attach(4);
 lowerMotor.attach(3);
}


void loop() {
 targetX = -7;
 targetY = 10;

 targetYPrime = targetY;


 lowerAngle = acos((pow(targetX, 2) + pow(targetYPrime, 2) - pow(upperLeg, 2) - pow(lowerLeg, 2)) / (2 * upperLeg * lowerLeg));
 upperAngle = atan2(targetYPrime, targetX) - atan2((lowerLeg * sin(lowerAngle)), (upperLeg + lowerLeg * cos(lowerAngle)));


 upperMotor.write((90 - degrees(upperAngle)) * 3 / 4);

 lowerMotor.write(degrees(lowerAngle) * 3 / 4);
}
