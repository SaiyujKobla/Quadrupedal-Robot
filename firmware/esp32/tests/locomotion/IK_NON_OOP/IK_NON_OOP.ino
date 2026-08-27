#include <Servo.h>
#include <math.h>


Servo upperMotor;
Servo lowerMotor;
Servo sideMotor;


double targetX;
double targetY;
double targetYPrime;
double targetZ;


const double upperLeg = 12;
const double lowerLeg = 13.5;


double lowerAngle;
double upperAngle;
double sideAngle;


void setup() {
 upperMotor.attach(3);
 lowerMotor.attach(5);
 sideMotor.attach(6);
}


void loop() {
 targetX = -2;
 targetY = 15;
 targetZ = 0;


 sideAngle = asin(-targetZ / lowerLeg);


 targetYPrime = targetY + lowerLeg * (1 - cos(sideAngle));


 lowerAngle = acos((pow(targetX, 2) + pow(targetYPrime, 2) - pow(upperLeg, 2) - pow(lowerLeg, 2)) / (2 * upperLeg * lowerLeg));
 upperAngle = atan2(targetYPrime, targetX) - atan2((lowerLeg * sin(lowerAngle)), (upperLeg + lowerLeg * cos(lowerAngle)));


 sideMotor.write((90 - degrees(sideAngle)) * 3 / 4);
 delay(2000);


 upperMotor.write((90 - degrees(upperAngle)) * 3 / 4);
 delay(2000);


 lowerMotor.write(degrees(lowerAngle) * 3 / 4);
 delay(2000);
}
