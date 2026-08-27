// #include <ESP32Servo.h> // #include <Servo.h> on Arduino Uno
#include <Servo.h>
#include <math.h>
#include <Arduino.h>


Servo sideMotor0;
Servo sideMotor1;
Servo sideMotor2;
Servo sideMotor3;

int sidePin0 = 3;
// int sidePin1 = 22;
// int sidePin2 = 3;
// int sidePin3 = 3;


float calibrateSide0(float angle) {

    angle = constrain(angle, 0.0, 180.0);

    float corrected = -0.000246914 * angle * angle + 0.988889 * angle;

    return corrected * 0.75;
}

float calibrateSide1(float angle) {

    angle = constrain(angle, 0, 180);

    float corrected = 0.00030864 * angle * angle + 0.833333 * angle + 7.5;

    return corrected * 0.75;
}

float calibrateSide2(float angle) {

    angle = constrain(angle, 0.0, 180.0);

    float corrected = -0.0000617284 * angle * angle + 0.95 * angle + 13.0;

    return corrected * 0.75;
}

float calibrateSide3(float angle) {

    angle = constrain(angle, 0.0, 180.0);

    float corrected = -0.000185185 * angle * angle + 0.983333333 * angle + 3.0;

    return corrected * 0.75;
}


void setup() {
 // put your setup code here, to run once:
//  sideMotor0.attach(sidePin0, 500, 2500);
//  sideMotor1.attach(sidePin1, 500, 2500);
 sideMotor2.attach(sidePin0);

//  delay(2000);
//  sideMotor0.write((0 + 0) * 0.75);
//  delay(2000);
//  sideMotor0.write((90 - 3) * 0.75);
//  delay(2000);
//  sideMotor0.write((180 - 10) * 0.75);
//  delay(2000);



 sideMotor2.write(calibrateSide2(0));
 delay(2000);
 sideMotor2.write(calibrateSide2(90));
 delay(2000);
 sideMotor2.write(calibrateSide2(180));
 delay(2000);
 sideMotor2.write(calibrateSide2(90));
 delay(2000);


}


void loop() {
 // put your main code here, to run repeatedly:


}
