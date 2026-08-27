#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H


#include <ESP32Servo.h> // #include <Servo.h> on Arduino Uno
#include <math.h>
#include <Arduino.h>


class InverseKinematics {
 private:
   Servo upperMotor;
   Servo lowerMotor;
   Servo sideMotor;


   const double upperLeg = 12;
   const double lowerLeg = 13.5;
   const double sideOffset = 4.121;


   int upperPin;
   int lowerPin;
   int sidePin;


   int orientation;


 public:
   InverseKinematics(int up, int low, int side, int o);
   void begin();
   void updateTarget(double targetX, double targetY, double targetZ);
   double calibrateUpperAngle(double angle);
   double calibrateSideAngle(double angle);
   double calibrateLowerAngle(double angle);
};


#endif