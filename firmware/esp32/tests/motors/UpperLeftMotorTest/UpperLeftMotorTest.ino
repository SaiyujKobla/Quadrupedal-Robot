#include <Servo.h> // #include <Servo.h> on Arduino Uno
#include <math.h>
#include <Arduino.h>


Servo upperMotor;



int upperPin = 3;



int orientation;

// Back right Upper Motor
float calibrateUpper(float angle) {

    angle = constrain(angle, 0.0, 180.0);

    float corrected =
        0.000154321 * angle * angle +
        0.930555556 * angle +
        13.0;

    return corrected * 0.75;
}

void setup() {
 // put your setup code here, to run once:
 upperMotor.attach(upperPin);




 // left
//  upperMotor.write((180 - 15) * 3.0 / 4.0);
//  delay(2000);
//  upperMotor.write((90 - 6) * 3.0 / 4.0);
//  delay(2000);
//  upperMotor.write((6) * 3.0 / 4.0);

 upperMotor.write(calibrateUpper(0));
 delay(2000);
 upperMotor.write(calibrateUpper(90));
 delay(2000);
 upperMotor.write(calibrateUpper(180));
 delay(2000);
 upperMotor.write(calibrateUpper(45));
 delay(2000);
 upperMotor.write(calibrateUpper(135));
 delay(2000);
 upperMotor.write(calibrateUpper(90));
 delay(2000);

//  // right
//  upperMotor.write((0 + 13) * 0.75);
//  delay(2000);
//  upperMotor.write((90 + 8) * 0.75);
//  delay(2000);
//  upperMotor.write((180 + 5.5) * 0.75);
//  delay(2000);
}


void loop() {
 // put your main code here, to run repeatedly:


}
