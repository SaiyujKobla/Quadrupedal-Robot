#include "IMUOrientation.h"

IMUOrientation orientation(
    33,
    16
);

const float ACCEL_X_OFFSET = -0.11622900f;
const float ACCEL_Y_OFFSET =  0.08589700f;
const float ACCEL_Z_OFFSET =  1.30524301f;

const float ACCEL_X_SCALE = 0.99895497f;
const float ACCEL_Y_SCALE = 0.99418946f;
const float ACCEL_Z_SCALE = 0.99225240f;

const float GYRO_X_OFFSET = 0.00766f;
const float GYRO_Y_OFFSET = 0.01859f;
const float GYRO_Z_OFFSET = 0.00223f;

unsigned long previousPrintTime = 0;

const unsigned long PRINT_PERIOD_MS =
    100;

void setup() {
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println(
        "RAW + CALIBRATED IMU DIAGNOSTIC"
    );

    Serial.println();

    orientation.setAccelerometerCalibration(
        ACCEL_X_OFFSET,
        ACCEL_Y_OFFSET,
        ACCEL_Z_OFFSET,
        ACCEL_X_SCALE,
        ACCEL_Y_SCALE,
        ACCEL_Z_SCALE
    );

    orientation.setGyroscopeCalibration(
        GYRO_X_OFFSET,
        GYRO_Y_OFFSET,
        GYRO_Z_OFFSET
    );

    orientation.setComplementaryAlpha(
        0.98f
    );

    if (!orientation.begin()) {
        Serial.println(
            "ERROR: Could not initialize LSM9DS1."
        );

        while (true) {
            delay(100);
        }
    }

    Serial.println(
        "IMU initialized."
    );

    Serial.println();
    Serial.println(
        "First hold the IMU completely still with +Z UP."
    );

    Serial.println(
        "Copy about 10-20 lines."
    );

    Serial.println();
    Serial.println(
        "Then flip it so -Z is UP."
    );

    Serial.println(
        "Again copy about 10-20 lines."
    );

    Serial.println();
}

void loop() {
    bool valid =
        orientation.update();

    if (!valid) {
        return;
    }

    if (
        millis() -
        previousPrintTime <
        PRINT_PERIOD_MS
    ) {
        return;
    }

    previousPrintTime =
        millis();

    Serial.print("RawAX:");
    Serial.print(
        orientation.getRawAccelX(),
        3
    );

    Serial.print(", RawAY:");
    Serial.print(
        orientation.getRawAccelY(),
        3
    );

    Serial.print(", RawAZ:");
    Serial.print(
        orientation.getRawAccelZ(),
        3
    );

    Serial.print(" | CalAX:");
    Serial.print(
        orientation.getAccelX(),
        3
    );

    Serial.print(", CalAY:");
    Serial.print(
        orientation.getAccelY(),
        3
    );

    Serial.print(", CalAZ:");
    Serial.print(
        orientation.getAccelZ(),
        3
    );

    Serial.print(", |A|:");
    Serial.print(
        orientation.getAccelerationMagnitude(),
        3
    );

    Serial.print(" | RawGX:");
    Serial.print(
        orientation.getRawGyroX(),
        4
    );

    Serial.print(", RawGY:");
    Serial.print(
        orientation.getRawGyroY(),
        4
    );

    Serial.print(", RawGZ:");
    Serial.print(
        orientation.getRawGyroZ(),
        4
    );

    Serial.print(" | CalGX:");
    Serial.print(
        orientation.getGyroX(),
        4
    );

    Serial.print(", CalGY:");
    Serial.print(
        orientation.getGyroY(),
        4
    );

    Serial.print(", CalGZ:");
    Serial.print(
        orientation.getGyroZ(),
        4
    );

    Serial.print(" | AccelRoll:");
    Serial.print(
        orientation.getAccelRoll(),
        2
    );

    Serial.print(", AccelPitch:");
    Serial.print(
        orientation.getAccelPitch(),
        2
    );

    Serial.print(" | Roll:");
    Serial.print(
        orientation.getRoll(),
        2
    );

    Serial.print(", Pitch:");
    Serial.print(
        orientation.getPitch(),
        2
    );

    Serial.print(", Yaw:");
    Serial.print(
        orientation.getYaw(),
        2
    );

    Serial.print(" | RollRate:");
    Serial.print(
        orientation.getRollRate(),
        2
    );

    Serial.print(", PitchRate:");
    Serial.print(
        orientation.getPitchRate(),
        2
    );

    Serial.print(", YawRate:");
    Serial.println(
        orientation.getYawRate(),
        2
    );
}