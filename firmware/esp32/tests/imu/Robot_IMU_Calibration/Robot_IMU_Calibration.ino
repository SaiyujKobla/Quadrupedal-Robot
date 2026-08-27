#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define SDA_PIN 33
#define SCL_PIN 16

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

const float G = 9.80665f;

const int ACCEL_SAMPLES = 400;
const int GYRO_SAMPLES = 800;

const int SAMPLE_DELAY_MS = 15;
const int SETTLE_DELAY_MS = 1500;

const float MAX_ACCEL_STDDEV = 0.20f;
const float MAX_GYRO_STDDEV = 0.03f;

const float MIN_EXPECTED_AXIS = 0.70f * G;
const float MAX_CROSS_AXIS = 0.60f * G;

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Measurement {
  Vec3 mean;
  Vec3 stdDev;
  Vec3 minimum;
  Vec3 maximum;
};

struct AccelCalibration {
  Vec3 offset;
  Vec3 scale;
};

void clearSerialInput() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void waitForEnter() {
  clearSerialInput();

  while (Serial.available() == 0) {
    delay(10);
  }

  clearSerialInput();
}

void printSeparator() {
  Serial.println();
  Serial.println("============================================================");
}

void printVec(const char* label, Vec3 v, int decimals = 6) {
  Serial.print(label);

  Serial.print(" X: ");
  Serial.print(v.x, decimals);

  Serial.print("  Y: ");
  Serial.print(v.y, decimals);

  Serial.print("  Z: ");
  Serial.println(v.z, decimals);
}

float magnitude(Vec3 v) {
  return sqrt(
    v.x * v.x +
    v.y * v.y +
    v.z * v.z
  );
}

void printProgress(int i, int total) {
  if (i == total / 4) {
    Serial.println("25% complete");
  }

  if (i == total / 2) {
    Serial.println("50% complete");
  }

  if (i == (3 * total) / 4) {
    Serial.println("75% complete");
  }
}

Vec3 readAccelOnce() {
  sensors_event_t accelEvent;
  sensors_event_t magEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  lsm.getEvent(
    &accelEvent,
    &magEvent,
    &gyroEvent,
    &tempEvent
  );

  Vec3 v;

  v.x = accelEvent.acceleration.x;
  v.y = accelEvent.acceleration.y;
  v.z = accelEvent.acceleration.z;

  return v;
}

Vec3 readGyroOnce() {
  sensors_event_t accelEvent;
  sensors_event_t magEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  lsm.getEvent(
    &accelEvent,
    &magEvent,
    &gyroEvent,
    &tempEvent
  );

  Vec3 v;

  v.x = gyroEvent.gyro.x;
  v.y = gyroEvent.gyro.y;
  v.z = gyroEvent.gyro.z;

  return v;
}

Measurement readAccelMeasurement() {
  Measurement result;

  Vec3 sum = {
    0.0f,
    0.0f,
    0.0f
  };

  result.minimum = {
    99999.0f,
    99999.0f,
    99999.0f
  };

  result.maximum = {
    -99999.0f,
    -99999.0f,
    -99999.0f
  };

  Serial.println("Pass 1 of 2: calculating average...");

  for (int i = 0; i < ACCEL_SAMPLES; i++) {
    Vec3 v = readAccelOnce();

    sum.x += v.x;
    sum.y += v.y;
    sum.z += v.z;

    if (v.x < result.minimum.x) result.minimum.x = v.x;
    if (v.y < result.minimum.y) result.minimum.y = v.y;
    if (v.z < result.minimum.z) result.minimum.z = v.z;

    if (v.x > result.maximum.x) result.maximum.x = v.x;
    if (v.y > result.maximum.y) result.maximum.y = v.y;
    if (v.z > result.maximum.z) result.maximum.z = v.z;

    printProgress(
      i,
      ACCEL_SAMPLES
    );

    delay(SAMPLE_DELAY_MS);
  }

  Serial.println("100% complete");

  result.mean.x =
    sum.x / ACCEL_SAMPLES;

  result.mean.y =
    sum.y / ACCEL_SAMPLES;

  result.mean.z =
    sum.z / ACCEL_SAMPLES;

  Serial.println();
  Serial.println("Pass 2 of 2: calculating variation...");
  Serial.println("KEEP THE ROBOT COMPLETELY STILL.");

  float squaredErrorX = 0.0f;
  float squaredErrorY = 0.0f;
  float squaredErrorZ = 0.0f;

  for (int i = 0; i < ACCEL_SAMPLES; i++) {
    Vec3 v = readAccelOnce();

    float dx =
      v.x - result.mean.x;

    float dy =
      v.y - result.mean.y;

    float dz =
      v.z - result.mean.z;

    squaredErrorX +=
      dx * dx;

    squaredErrorY +=
      dy * dy;

    squaredErrorZ +=
      dz * dz;

    printProgress(
      i,
      ACCEL_SAMPLES
    );

    delay(SAMPLE_DELAY_MS);
  }

  Serial.println("100% complete");

  result.stdDev.x =
    sqrt(
      squaredErrorX /
      ACCEL_SAMPLES
    );

  result.stdDev.y =
    sqrt(
      squaredErrorY /
      ACCEL_SAMPLES
    );

  result.stdDev.z =
    sqrt(
      squaredErrorZ /
      ACCEL_SAMPLES
    );

  return result;
}

Measurement readGyroMeasurement() {
  Measurement result;

  Vec3 sum = {
    0.0f,
    0.0f,
    0.0f
  };

  result.minimum = {
    99999.0f,
    99999.0f,
    99999.0f
  };

  result.maximum = {
    -99999.0f,
    -99999.0f,
    -99999.0f
  };

  Serial.println("Pass 1 of 2: calculating average...");

  for (int i = 0; i < GYRO_SAMPLES; i++) {
    Vec3 v = readGyroOnce();

    sum.x += v.x;
    sum.y += v.y;
    sum.z += v.z;

    if (v.x < result.minimum.x) result.minimum.x = v.x;
    if (v.y < result.minimum.y) result.minimum.y = v.y;
    if (v.z < result.minimum.z) result.minimum.z = v.z;

    if (v.x > result.maximum.x) result.maximum.x = v.x;
    if (v.y > result.maximum.y) result.maximum.y = v.y;
    if (v.z > result.maximum.z) result.maximum.z = v.z;

    printProgress(
      i,
      GYRO_SAMPLES
    );

    delay(SAMPLE_DELAY_MS);
  }

  Serial.println("100% complete");

  result.mean.x =
    sum.x / GYRO_SAMPLES;

  result.mean.y =
    sum.y / GYRO_SAMPLES;

  result.mean.z =
    sum.z / GYRO_SAMPLES;

  Serial.println();
  Serial.println("Pass 2 of 2: calculating variation...");
  Serial.println("KEEP THE ROBOT COMPLETELY STILL.");

  float squaredErrorX = 0.0f;
  float squaredErrorY = 0.0f;
  float squaredErrorZ = 0.0f;

  for (int i = 0; i < GYRO_SAMPLES; i++) {
    Vec3 v = readGyroOnce();

    float dx =
      v.x - result.mean.x;

    float dy =
      v.y - result.mean.y;

    float dz =
      v.z - result.mean.z;

    squaredErrorX +=
      dx * dx;

    squaredErrorY +=
      dy * dy;

    squaredErrorZ +=
      dz * dz;

    printProgress(
      i,
      GYRO_SAMPLES
    );

    delay(SAMPLE_DELAY_MS);
  }

  Serial.println("100% complete");

  result.stdDev.x =
    sqrt(
      squaredErrorX /
      GYRO_SAMPLES
    );

  result.stdDev.y =
    sqrt(
      squaredErrorY /
      GYRO_SAMPLES
    );

  result.stdDev.z =
    sqrt(
      squaredErrorZ /
      GYRO_SAMPLES
    );

  return result;
}

bool accelStable(Measurement m) {
  return (
    m.stdDev.x <= MAX_ACCEL_STDDEV &&
    m.stdDev.y <= MAX_ACCEL_STDDEV &&
    m.stdDev.z <= MAX_ACCEL_STDDEV
  );
}

bool gyroStable(Measurement m) {
  return (
    m.stdDev.x <= MAX_GYRO_STDDEV &&
    m.stdDev.y <= MAX_GYRO_STDDEV &&
    m.stdDev.z <= MAX_GYRO_STDDEV
  );
}

bool orientationCorrect(
  Vec3 v,
  char axis,
  int expectedSign
) {
  float mainAxis;
  float cross1;
  float cross2;

  if (axis == 'X') {
    mainAxis = v.x;
    cross1 = v.y;
    cross2 = v.z;
  }
  else if (axis == 'Y') {
    mainAxis = v.y;
    cross1 = v.x;
    cross2 = v.z;
  }
  else {
    mainAxis = v.z;
    cross1 = v.x;
    cross2 = v.y;
  }

  if (expectedSign > 0) {
    if (mainAxis < MIN_EXPECTED_AXIS) {
      return false;
    }
  }
  else {
    if (mainAxis > -MIN_EXPECTED_AXIS) {
      return false;
    }
  }

  if (fabs(cross1) > MAX_CROSS_AXIS) {
    return false;
  }

  if (fabs(cross2) > MAX_CROSS_AXIS) {
    return false;
  }

  return true;
}

void showMeasurementData(
  Measurement m
) {
  Serial.println();

  printVec(
    "Mean:    ",
    m.mean
  );

  printVec(
    "Std dev: ",
    m.stdDev
  );

  printVec(
    "Minimum: ",
    m.minimum
  );

  printVec(
    "Maximum: ",
    m.maximum
  );

  Serial.print("Magnitude of mean: ");
  Serial.print(
    magnitude(m.mean),
    6
  );
  Serial.println(" m/s^2");
}

void showPrompt(
  int step,
  int total,
  const char* title,
  const char* instructions
) {
  printSeparator();

  Serial.print("STEP ");
  Serial.print(step);
  Serial.print(" OF ");
  Serial.println(total);

  Serial.println();
  Serial.println(title);

  Serial.println();
  Serial.println(instructions);

  Serial.println();
  Serial.println("Make sure the robot is completely still.");
  Serial.println("Press ENTER when ready.");

  printSeparator();

  waitForEnter();

  Serial.println();
  Serial.println("Waiting for the robot to settle...");

  delay(SETTLE_DELAY_MS);

  Serial.println();
  Serial.println("MEASURING NOW.");
  Serial.println("DO NOT MOVE THE ROBOT.");
  Serial.println();
}

void accepted(
  const char* next
) {
  Serial.println();
  Serial.println("------------------------------------------------------------");
  Serial.println("MEASUREMENT ACCEPTED");
  Serial.println("------------------------------------------------------------");

  Serial.println();
  Serial.println("You may move the robot now.");

  if (next != nullptr) {
    Serial.println();
    Serial.println("NEXT:");
    Serial.println(next);
  }
}

void rejected(
  const char* reason
) {
  Serial.println();
  Serial.println("------------------------------------------------------------");
  Serial.println("MEASUREMENT REJECTED");
  Serial.println("------------------------------------------------------------");

  Serial.println();
  Serial.println(reason);

  Serial.println();
  Serial.println("This measurement was NOT used.");
  Serial.println("Repeat this same orientation.");
}

Measurement calibrateGyro(
  int step,
  int total,
  const char* next
) {
  while (true) {
    showPrompt(
      step,
      total,
      "GYROSCOPE CALIBRATION",
      "The robot may be in any orientation.\n"
      "It must be completely motionless."
    );

    Measurement m =
      readGyroMeasurement();

    showMeasurementData(m);

    if (!gyroStable(m)) {
      rejected(
        "The gyroscope readings varied too much."
      );

      continue;
    }

    accepted(next);

    return m;
  }
}

Measurement calibrateAccelFace(
  int step,
  int total,
  const char* title,
  const char* instructions,
  char axis,
  int sign,
  const char* next
) {
  while (true) {
    showPrompt(
      step,
      total,
      title,
      instructions
    );

    Measurement m =
      readAccelMeasurement();

    showMeasurementData(m);

    if (!accelStable(m)) {
      rejected(
        "The accelerometer readings varied too much."
      );

      continue;
    }

    if (!orientationCorrect(
      m.mean,
      axis,
      sign
    )) {
      rejected(
        "The requested axis does not appear to be pointing upward."
      );

      continue;
    }

    accepted(next);

    return m;
  }
}

Vec3 applyCalibration(
  Vec3 raw,
  AccelCalibration c
) {
  Vec3 corrected;

  corrected.x =
    (raw.x - c.offset.x) *
    c.scale.x;

  corrected.y =
    (raw.y - c.offset.y) *
    c.scale.y;

  corrected.z =
    (raw.z - c.offset.z) *
    c.scale.z;

  return corrected;
}

Measurement getVerificationMeasurement(
  const char* title,
  const char* instruction
) {
  while (true) {
    printSeparator();

    Serial.println(title);

    Serial.println();
    Serial.println(instruction);

    Serial.println();
    Serial.println("Press ENTER when the robot is completely still.");

    printSeparator();

    waitForEnter();

    Serial.println();
    Serial.println("Waiting for robot to settle...");

    delay(SETTLE_DELAY_MS);

    Serial.println();
    Serial.println("MEASURING NOW.");
    Serial.println("DO NOT MOVE THE ROBOT.");
    Serial.println();

    Measurement m =
      readAccelMeasurement();

    showMeasurementData(m);

    if (!accelStable(m)) {
      Serial.println();
      Serial.println(
        "Too much variation detected."
      );

      Serial.println(
        "Repeat this verification orientation."
      );

      continue;
    }

    return m;
  }
}

void printCorrectedVerification(
  const char* label,
  Measurement raw,
  AccelCalibration c,
  Vec3 expected
) {
  Vec3 corrected =
    applyCalibration(
      raw.mean,
      c
    );

  Serial.println();
  Serial.println(label);

  printVec(
    "Raw mean:       ",
    raw.mean
  );

  printVec(
    "Corrected mean: ",
    corrected
  );

  printVec(
    "Expected:       ",
    expected
  );

  Vec3 error;

  error.x =
    corrected.x - expected.x;

  error.y =
    corrected.y - expected.y;

  error.z =
    corrected.z - expected.z;

  printVec(
    "Error:          ",
    error
  );

  Serial.print(
    "Corrected magnitude: "
  );

  Serial.print(
    magnitude(corrected),
    6
  );

  Serial.println(" m/s^2");
}

void printCalibrationConstants(
  AccelCalibration c,
  Vec3 gyroOffset
) {
  printSeparator();

  Serial.println(
    "FINAL CALIBRATION CONSTANTS"
  );

  printSeparator();

  Serial.println();

  Serial.print(
    "const float ACCEL_X_OFFSET = "
  );
  Serial.print(
    c.offset.x,
    8
  );
  Serial.println("f;");

  Serial.print(
    "const float ACCEL_Y_OFFSET = "
  );
  Serial.print(
    c.offset.y,
    8
  );
  Serial.println("f;");

  Serial.print(
    "const float ACCEL_Z_OFFSET = "
  );
  Serial.print(
    c.offset.z,
    8
  );
  Serial.println("f;");

  Serial.println();

  Serial.print(
    "const float ACCEL_X_SCALE = "
  );
  Serial.print(
    c.scale.x,
    8
  );
  Serial.println("f;");

  Serial.print(
    "const float ACCEL_Y_SCALE = "
  );
  Serial.print(
    c.scale.y,
    8
  );
  Serial.println("f;");

  Serial.print(
    "const float ACCEL_Z_SCALE = "
  );
  Serial.print(
    c.scale.z,
    8
  );
  Serial.println("f;");

  Serial.println();

  Serial.print(
    "const float GYRO_X_OFFSET = "
  );
  Serial.print(
    gyroOffset.x,
    8
  );
  Serial.println("f;");

  Serial.print(
    "const float GYRO_Y_OFFSET = "
  );
  Serial.print(
    gyroOffset.y,
    8
  );
  Serial.println("f;");

  Serial.print(
    "const float GYRO_Z_OFFSET = "
  );
  Serial.print(
    gyroOffset.z,
    8
  );
  Serial.println("f;");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  printSeparator();

  Serial.println(
    "LSM9DS1 ACCELEROMETER + GYROSCOPE CALIBRATION"
  );

  printSeparator();

  Serial.println();
  Serial.println(
    "This version uses two-pass sampling without large arrays."
  );

  Serial.println(
    "It should not cause the ESP32 stack overflow seen previously."
  );

  Serial.println();

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(400000);

  if (!lsm.begin()) {
    Serial.println(
      "ERROR: Could not detect LSM9DS1."
    );

    Serial.println(
      "Check wiring, power, SDA, and SCL."
    );

    while (true) {
      delay(100);
    }
  }

  lsm.setupAccel(
    lsm.LSM9DS1_ACCELRANGE_2G
  );

  lsm.setupGyro(
    lsm.LSM9DS1_GYROSCALE_245DPS
  );

  lsm.setupMag(
    lsm.LSM9DS1_MAGGAIN_4GAUSS
  );

  Serial.println(
    "Sensor detected successfully."
  );

  Serial.println();
  Serial.println(
    "Warming up sensor for 4 seconds..."
  );

  delay(4000);

  for (int i = 0; i < 100; i++) {
    readAccelOnce();
    delay(10);
  }

  Serial.println(
    "Sensor warm-up complete."
  );

  Serial.println();
  Serial.println(
    "STAGE 1:"
  );

  Serial.println(
    "1 gyro measurement"
  );

  Serial.println(
    "6 accelerometer face measurements"
  );

  Serial.println();
  Serial.println(
    "STAGE 2:"
  );

  Serial.println(
    "Verify all 6 faces after applying calibration."
  );

  Serial.println();
  Serial.println(
    "IMPORTANT:"
  );

  Serial.println(
    "During each measurement there are TWO passes."
  );

  Serial.println(
    "Do not move the robot until both passes are complete."
  );

  Serial.println();
  Serial.println(
    "Press ENTER to begin."
  );

  waitForEnter();

  const int CALIBRATION_STEPS = 7;

  Measurement gyroCalibration =
    calibrateGyro(
      1,
      CALIBRATION_STEPS,
      "Next: place +X upward."
    );

  Measurement xPlus =
    calibrateAccelFace(
      2,
      CALIBRATION_STEPS,
      "+X CALIBRATION",
      "Place the robot so the IMU's +X direction points straight toward the ceiling.",
      'X',
      +1,
      "Next: place -X upward."
    );

  Measurement xMinus =
    calibrateAccelFace(
      3,
      CALIBRATION_STEPS,
      "-X CALIBRATION",
      "Flip the robot so the IMU's -X direction points straight toward the ceiling.",
      'X',
      -1,
      "Next: place +Y upward."
    );

  Measurement yPlus =
    calibrateAccelFace(
      4,
      CALIBRATION_STEPS,
      "+Y CALIBRATION",
      "Place the robot so the IMU's +Y direction points straight toward the ceiling.",
      'Y',
      +1,
      "Next: place -Y upward."
    );

  Measurement yMinus =
    calibrateAccelFace(
      5,
      CALIBRATION_STEPS,
      "-Y CALIBRATION",
      "Flip the robot so the IMU's -Y direction points straight toward the ceiling.",
      'Y',
      -1,
      "Next: place +Z upward."
    );

  Measurement zPlus =
    calibrateAccelFace(
      6,
      CALIBRATION_STEPS,
      "+Z CALIBRATION",
      "Place the robot so the IMU's +Z direction points straight toward the ceiling.",
      'Z',
      +1,
      "Next: place -Z upward."
    );

  Measurement zMinus =
    calibrateAccelFace(
      7,
      CALIBRATION_STEPS,
      "-Z CALIBRATION",
      "Flip the robot so the IMU's -Z direction points straight toward the ceiling.",
      'Z',
      -1,
      nullptr
    );

  AccelCalibration calibration;

  calibration.offset.x =
    (
      xPlus.mean.x +
      xMinus.mean.x
    ) / 2.0f;

  calibration.offset.y =
    (
      yPlus.mean.y +
      yMinus.mean.y
    ) / 2.0f;

  calibration.offset.z =
    (
      zPlus.mean.z +
      zMinus.mean.z
    ) / 2.0f;

  float xSpan =
    xPlus.mean.x -
    xMinus.mean.x;

  float ySpan =
    yPlus.mean.y -
    yMinus.mean.y;

  float zSpan =
    zPlus.mean.z -
    zMinus.mean.z;

  calibration.scale.x =
    (2.0f * G) /
    xSpan;

  calibration.scale.y =
    (2.0f * G) /
    ySpan;

  calibration.scale.z =
    (2.0f * G) /
    zSpan;

  printCalibrationConstants(
    calibration,
    gyroCalibration.mean
  );

  printSeparator();

  Serial.println(
    "STAGE 2: SIX-FACE VERIFICATION"
  );

  printSeparator();

  Serial.println();
  Serial.println(
    "We will now repeat all six orientations."
  );

  Serial.println(
    "The new calibration will be applied to each reading."
  );

  Serial.println();
  Serial.println(
    "Press ENTER when ready to start verification."
  );

  waitForEnter();

  Measurement verifyXPlus =
    getVerificationMeasurement(
      "VERIFY +X",
      "Place the IMU so +X points straight upward."
    );

  printCorrectedVerification(
    "+X verification",
    verifyXPlus,
    calibration,
    {G, 0.0f, 0.0f}
  );

  Measurement verifyXMinus =
    getVerificationMeasurement(
      "VERIFY -X",
      "Place the IMU so -X points straight upward."
    );

  printCorrectedVerification(
    "-X verification",
    verifyXMinus,
    calibration,
    {-G, 0.0f, 0.0f}
  );

  Measurement verifyYPlus =
    getVerificationMeasurement(
      "VERIFY +Y",
      "Place the IMU so +Y points straight upward."
    );

  printCorrectedVerification(
    "+Y verification",
    verifyYPlus,
    calibration,
    {0.0f, G, 0.0f}
  );

  Measurement verifyYMinus =
    getVerificationMeasurement(
      "VERIFY -Y",
      "Place the IMU so -Y points straight upward."
    );

  printCorrectedVerification(
    "-Y verification",
    verifyYMinus,
    calibration,
    {0.0f, -G, 0.0f}
  );

  Measurement verifyZPlus =
    getVerificationMeasurement(
      "VERIFY +Z",
      "Place the IMU so +Z points straight upward."
    );

  printCorrectedVerification(
    "+Z verification",
    verifyZPlus,
    calibration,
    {0.0f, 0.0f, G}
  );

  Measurement verifyZMinus =
    getVerificationMeasurement(
      "VERIFY -Z",
      "Place the IMU so -Z points straight upward."
    );

  printCorrectedVerification(
    "-Z verification",
    verifyZMinus,
    calibration,
    {0.0f, 0.0f, -G}
  );

  printSeparator();

  Serial.println(
    "CALIBRATION AND VERIFICATION COMPLETE"
  );

  printSeparator();

  Serial.println();
  Serial.println(
    "Send me the final Serial Monitor output."
  );

  Serial.println();
  Serial.println(
    "Please include:"
  );

  Serial.println(
    "- all 6 calibration means"
  );

  Serial.println(
    "- all standard deviations"
  );

  Serial.println(
    "- min/max values"
  );

  Serial.println(
    "- final offsets and scales"
  );

  Serial.println(
    "- gyro offsets"
  );

  Serial.println(
    "- all 6 verification results"
  );
}

void loop() {
}