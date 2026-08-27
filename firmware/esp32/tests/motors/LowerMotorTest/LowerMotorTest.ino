// #include <ESP32Servo.h> // #include <Servo.h> on Arduino Uno
#include <Servo.h>
#include <math.h>
#include <Arduino.h>


Servo lowerMotor0;
Servo lowerMotor1;
Servo lowerMotor2;
Servo lowerMotor3;

int lowerPin0 = 3;
int lowerPin1 = 3;
int lowerPin2 = 3;
int lowerPin3 = 3;

float calibrateLower0(float angle) {

    angle = constrain(angle, 0.0, 180.0);

    float corrected =
        0.0 * angle * angle +
        0.955555556 * angle +
        0.0;

    return corrected * 0.75;
}

float calibrateLower1(float angle) {

    angle = constrain(angle, 0.0, 180.0);

    float corrected =
        0.000246914 * angle * angle +
        0.900000000 * angle +
        10.0;

    return corrected * 0.75;
}

float calibrateLower2(float angle) {

    angle = constrain(angle, 90.0, 270.0);

    float corrected =
        0.944444444 * angle - 27.0;

    return corrected * 0.75;
}

float calibrateLower3(float angle) {

    angle = constrain(angle, 90.0, 270.0);

    float corrected =
        0.001666667 * angle * angle +
        0.494444444 * angle +
        15.0;

    return corrected * 0.75;
}

void setup() {
 // put your setup code here, to run once:
 lowerMotor2.attach(3);

//  lowerMotor2.write((0 + 0) * 0.75);
//  delay(2000);
//  lowerMotor2.write((90 - 32) * 0.75);
//  delay(2000);
//  lowerMotor2.write((180 - 37) * 0.75);
//  delay(2000);
//  lowerMotor2.write((270 - 42) * 0.75);
//  delay(2000);

 lowerMotor2.write(calibrateLower2(90));
 delay(2000);
 lowerMotor2.write(calibrateLower2(180));
 delay(2000);
 lowerMotor2.write(calibrateLower2(270));
 delay(2000);
 lowerMotor2.write(calibrateLower2(135));
 delay(2000);
 lowerMotor2.write(calibrateLower2(225));


}


void loop() {
 // put your main code here, to run repeatedly:


}
