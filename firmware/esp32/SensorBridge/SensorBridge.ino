#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>
#include <string.h>

#include "secrets.h"

// ============================================================
// USER CONFIGURATION
// ============================================================

// Mac Wi-Fi IP
IPAddress MAC_IP(192, 168, 1, 223);

const uint16_t SENSOR_UDP_PORT = 5005;
const uint16_t COMMAND_UDP_PORT = 5006;
const uint16_t SENSOR_LOCAL_UDP_PORT = 5007;

const bool ENABLE_LIDAR = true;
const bool ENABLE_TOF = true;
const bool ENABLE_ULTRASONIC = true;

// ============================================================
// RPLIDAR A1
// ============================================================

// LiDAR TX -> ESP32 GPIO 15
// LiDAR RX -> ESP32 GPIO 4
const int LIDAR_RX_PIN = 15;
const int LIDAR_TX_PIN = 4;

// LiDAR MOTOCTL -> ESP32 GPIO 2
const int LIDAR_MOTOR_PIN = 2;

const uint32_t LIDAR_BAUD = 115200;

// ============================================================
// VL53L0X TOF
// ============================================================

const int TOF_SDA_PIN = 33;
const int TOF_SCL_PIN = 16;

// ============================================================
// ULTRASONIC
// ============================================================

const int ULTRASONIC_TRIG_PIN = 5;
const int ULTRASONIC_ECHO_PIN = 34;

// ============================================================
// PYTHON NETWORK PROTOCOL -- MUST MATCH NetworkProtocol.py
// ============================================================

const uint8_t PROTOCOL_VERSION = 1;

const uint8_t PACKET_RANGE = 1;
const uint8_t PACKET_LIDAR_CHUNK = 2;
const uint8_t PACKET_COMMAND = 1;

const uint8_t RANGE_FLAG_ULTRASONIC_VALID = 1 << 0;
const uint8_t RANGE_FLAG_TOF_VALID = 1 << 1;

const uint8_t COMMAND_STOP = 0;
const uint8_t COMMAND_FORWARD = 1;
const uint8_t COMMAND_TURN_LEFT = 2;
const uint8_t COMMAND_TURN_RIGHT = 3;

const size_t MAX_LIDAR_POINTS_PER_CHUNK = 200;
const size_t MAX_LIDAR_POINTS_PER_SCAN = 2500;

struct __attribute__((packed)) RangePacket {
  char magic[2];
  uint8_t version;
  uint8_t packetType;
  uint32_t sequence;
  uint32_t senderMillis;
  float ultrasonicMm;
  float tofDownMm;
  uint8_t flags;
  uint8_t padding[3];
};

struct __attribute__((packed)) LidarPointPacket {
  uint16_t angleCdeg;
  uint16_t distanceMm;
  uint8_t quality;
  uint8_t reserved;
};

struct __attribute__((packed)) LidarChunkPrefix {
  char magic[2];
  uint8_t version;
  uint8_t packetType;
  uint32_t sequence;
  uint32_t senderMillis;
  uint32_t scanId;
  uint16_t chunkIndex;
  uint16_t chunkCount;
  uint16_t pointCount;
  uint8_t padding[2];
};

struct __attribute__((packed)) CommandPacket {
  char magic[2];
  uint8_t version;
  uint8_t packetType;
  uint32_t sequence;
  uint32_t senderMillis;
  uint8_t commandCode;
  uint8_t padding[3];
  float headingDeg;
  float speedScale;
};

static_assert(sizeof(RangePacket) == 24, "RangePacket must be 24 bytes");
static_assert(sizeof(LidarPointPacket) == 6, "LidarPointPacket must be 6 bytes");
static_assert(sizeof(LidarChunkPrefix) == 24, "LidarChunkPrefix must be 24 bytes");
static_assert(sizeof(CommandPacket) == 24, "CommandPacket must be 24 bytes");

// ============================================================
// GLOBAL OBJECTS / STATE
// ============================================================

WiFiUDP sensorUdp;
WiFiUDP commandUdp;
HardwareSerial LidarSerial(2);
Adafruit_VL53L0X tof;

bool tofAvailable = false;
bool lidarScanStreaming = false;

uint32_t sensorSequence = 0;
uint32_t lidarScanId = 0;

LidarPointPacket lidarScanPoints[MAX_LIDAR_POINTS_PER_SCAN];
size_t lidarPointCount = 0;
bool lidarScanStarted = false;
bool lidarScanOverflowed = false;

uint8_t lidarNodeBuffer[5];
size_t lidarNodeBytes = 0;

unsigned long lastRangeSendMs = 0;
unsigned long lastWiFiReconnectAttemptMs = 0;
unsigned long lastLidarStartAttemptMs = 0;
unsigned long lastLidarValidNodeMs = 0;
unsigned long lastStatusPrintMs = 0;

const unsigned long RANGE_INTERVAL_MS = 100;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 3000;
const unsigned long LIDAR_RESTART_INTERVAL_MS = 2000;
const unsigned long STATUS_PRINT_INTERVAL_MS = 3000;

uint8_t lastCommandCode = 255;
float lastHeadingDeg = 9999.0f;
float lastSpeedScale = -1.0f;
unsigned long lastCommandReceivedMs = 0;

// ============================================================
// SMALL HELPERS
// ============================================================

const char* commandName(uint8_t code) {
  switch (code) {
    case COMMAND_STOP:
      return "STOP";

    case COMMAND_FORWARD:
      return "FORWARD";

    case COMMAND_TURN_LEFT:
      return "TURN LEFT";

    case COMMAND_TURN_RIGHT:
      return "TURN RIGHT";

    default:
      return "UNKNOWN";
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 20000
  ) {
    Serial.print(".");
    delay(250);
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected");

    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());

    Serial.print("ESP32 MAC: ");
    Serial.println(WiFi.macAddress());

    Serial.print("Mac sensor destination: ");
    Serial.print(MAC_IP);
    Serial.print(":");
    Serial.println(SENSOR_UDP_PORT);

    Serial.print("Listening for Python commands on UDP port ");
    Serial.println(COMMAND_UDP_PORT);
  } else {
    Serial.println(
      "Wi-Fi connection failed. The loop will keep retrying."
    );
  }
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();

  if (
    now - lastWiFiReconnectAttemptMs <
    WIFI_RECONNECT_INTERVAL_MS
  ) {
    return;
  }

  lastWiFiReconnectAttemptMs = now;

  Serial.println("Wi-Fi disconnected; reconnecting...");

  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ============================================================
// ULTRASONIC
// ============================================================

bool readUltrasonicMm(float& distanceMm) {
  if (!ENABLE_ULTRASONIC) {
    return false;
  }

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  const unsigned long echoTimeoutUs = 25000;

  unsigned long durationUs = pulseIn(
    ULTRASONIC_ECHO_PIN,
    HIGH,
    echoTimeoutUs
  );

  if (durationUs == 0) {
    return false;
  }

  distanceMm = durationUs * 0.343f / 2.0f;

  if (
    !isfinite(distanceMm) ||
    distanceMm <= 0.0f ||
    distanceMm > 5000.0f
  ) {
    return false;
  }

  return true;
}

// ============================================================
// DOWNWARD TOF
// ============================================================

bool readTofMm(float& distanceMm) {
  if (!ENABLE_TOF || !tofAvailable) {
    return false;
  }

  VL53L0X_RangingMeasurementData_t measurement;

  // tof.rangingTest(&measurement, false);

  // if (measurement.RangeStatus != 0) {
  //   return false;
  // }
  tof.rangingTest(&measurement, false);

  static unsigned long lastTofDebugMs = 0;

  if (millis() - lastTofDebugMs >= 1000) {
      lastTofDebugMs = millis();

      Serial.print("ToF RangeStatus: ");
      Serial.print(measurement.RangeStatus);

      Serial.print("   Distance: ");
      Serial.print(measurement.RangeMilliMeter);

      Serial.println(" mm");
  }

  if (measurement.RangeStatus != 0) {
      return false;
  }

  distanceMm =
    static_cast<float>(measurement.RangeMilliMeter);

  if (
    !isfinite(distanceMm) ||
    distanceMm <= 0.0f
  ) {
    return false;
  }

  return true;
}

// ============================================================
// RANGE PACKET: ESP32 -> PYTHON
// ============================================================

void sendRangePacket() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  float ultrasonicMm = 0.0f;
  float tofMm = 0.0f;

  bool ultrasonicValid =
    readUltrasonicMm(ultrasonicMm);

  bool tofValid =
    readTofMm(tofMm);

  RangePacket packet{};

  packet.magic[0] = 'Q';
  packet.magic[1] = 'R';

  packet.version = PROTOCOL_VERSION;
  packet.packetType = PACKET_RANGE;

  packet.sequence = sensorSequence++;
  packet.senderMillis = millis();

  packet.ultrasonicMm = ultrasonicMm;
  packet.tofDownMm = tofMm;

  packet.flags = 0;

  if (ultrasonicValid) {
    packet.flags |=
      RANGE_FLAG_ULTRASONIC_VALID;
  }

  if (tofValid) {
    packet.flags |=
      RANGE_FLAG_TOF_VALID;
  }

  sensorUdp.beginPacket(
    MAC_IP,
    SENSOR_UDP_PORT
  );

  sensorUdp.write(
    reinterpret_cast<const uint8_t*>(&packet),
    sizeof(packet)
  );

  sensorUdp.endPacket();
}

// ============================================================
// RPLIDAR A1 LOW-LEVEL UART
// ============================================================

void flushLidarInput() {
  while (LidarSerial.available() > 0) {
    LidarSerial.read();
  }
}

void sendLidarCommand(uint8_t command) {
  const uint8_t request[2] = {
    0xA5,
    command
  };

  LidarSerial.write(
    request,
    sizeof(request)
  );

  LidarSerial.flush();
}

bool waitForLidarDescriptor(
  unsigned long timeoutMs
) {
  uint8_t descriptor[7];

  size_t count = 0;

  unsigned long start = millis();

  while (
    millis() - start < timeoutMs
  ) {
    while (
      LidarSerial.available() > 0
    ) {
      uint8_t value =
        static_cast<uint8_t>(
          LidarSerial.read()
        );

      if (count == 0) {
        if (value == 0xA5) {
          descriptor[count++] = value;
        }

        continue;
      }

      if (count == 1) {
        if (value == 0x5A) {
          descriptor[count++] = value;
        } else if (value == 0xA5) {
          descriptor[0] = 0xA5;
          count = 1;
        } else {
          count = 0;
        }

        continue;
      }

      descriptor[count++] = value;

      if (
        count == sizeof(descriptor)
      ) {
        Serial.print(
          "RPLIDAR descriptor received; answer type 0x"
        );

        Serial.println(
          descriptor[6],
          HEX
        );

        return true;
      }
    }

    handleCommandPackets();

    delay(1);
  }

  return false;
}

bool startLidarScan() {
  if (!ENABLE_LIDAR) {
    return false;
  }

  Serial.println(
    "Starting RPLIDAR standard scan..."
  );

  // Stop any previous scan.
  sendLidarCommand(0x25);

  delay(50);

  flushLidarInput();

  // Start standard scan.
  sendLidarCommand(0x20);

  if (
    !waitForLidarDescriptor(1200)
  ) {
    Serial.println(
      "ERROR: No RPLIDAR scan descriptor received."
    );

    lidarScanStreaming = false;

    return false;
  }

  lidarNodeBytes = 0;
  lidarPointCount = 0;

  lidarScanStarted = false;
  lidarScanOverflowed = false;

  lidarScanStreaming = true;

  lastLidarValidNodeMs = millis();

  Serial.println(
    "RPLIDAR scan stream started."
  );

  return true;
}

bool lidarNodeIsValid(
  const uint8_t node[5]
) {
  bool syncBit =
    (node[0] & 0x01) != 0;

  bool inverseSyncBit =
    (node[0] & 0x02) != 0;

  if (
    syncBit == inverseSyncBit
  ) {
    return false;
  }

  if (
    (node[1] & 0x01) == 0
  ) {
    return false;
  }

  return true;
}

void addLidarNode(
  const uint8_t node[5]
) {
  lastLidarValidNodeMs = millis();

  bool newScan =
    (node[0] & 0x01) != 0;

  if (newScan) {
    if (
      lidarScanStarted &&
      lidarPointCount > 0
    ) {
      sendCompletedLidarScan();
    }

    lidarPointCount = 0;

    lidarScanOverflowed = false;

    lidarScanStarted = true;
  }

  if (!lidarScanStarted) {
    return;
  }

  uint8_t quality =
    node[0] >> 2;

  uint16_t angleQ6Check =
    static_cast<uint16_t>(node[1]) |
    (
      static_cast<uint16_t>(node[2])
      << 8
    );

  uint16_t angleQ6 =
    angleQ6Check >> 1;

  float angleDeg =
    static_cast<float>(angleQ6) /
    64.0f;

  uint16_t distanceQ2 =
    static_cast<uint16_t>(node[3]) |
    (
      static_cast<uint16_t>(node[4])
      << 8
    );

  float distanceMmFloat =
    static_cast<float>(distanceQ2) /
    4.0f;

  if (
    !isfinite(angleDeg) ||
    !isfinite(distanceMmFloat)
  ) {
    return;
  }

  if (
    distanceMmFloat <= 0.0f ||
    distanceMmFloat > 65535.0f
  ) {
    return;
  }

  if (
    lidarPointCount >=
    MAX_LIDAR_POINTS_PER_SCAN
  ) {
    lidarScanOverflowed = true;
    return;
  }

  uint32_t angleCdegLong =
    lroundf(angleDeg * 100.0f);

  if (
    angleCdegLong > 35999U
  ) {
    angleCdegLong %= 36000U;
  }

  uint32_t distanceMmLong =
    lroundf(distanceMmFloat);

  if (
    distanceMmLong > 65535U
  ) {
    return;
  }

  LidarPointPacket& point =
    lidarScanPoints[
      lidarPointCount++
    ];

  point.angleCdeg =
    static_cast<uint16_t>(
      angleCdegLong
    );

  point.distanceMm =
    static_cast<uint16_t>(
      distanceMmLong
    );

  point.quality = quality;

  point.reserved = 0;
}

void processLidarSerial() {
  if (
    !ENABLE_LIDAR ||
    !lidarScanStreaming
  ) {
    return;
  }

  while (
    LidarSerial.available() > 0
  ) {
    uint8_t value =
      static_cast<uint8_t>(
        LidarSerial.read()
      );

    if (
      lidarNodeBytes <
      sizeof(lidarNodeBuffer)
    ) {
      lidarNodeBuffer[
        lidarNodeBytes++
      ] = value;
    }

    if (
      lidarNodeBytes <
      sizeof(lidarNodeBuffer)
    ) {
      continue;
    }

    if (
      lidarNodeIsValid(
        lidarNodeBuffer
      )
    ) {
      addLidarNode(
        lidarNodeBuffer
      );

      lidarNodeBytes = 0;
    } else {
      memmove(
        lidarNodeBuffer,
        lidarNodeBuffer + 1,
        sizeof(lidarNodeBuffer) - 1
      );

      lidarNodeBytes =
        sizeof(lidarNodeBuffer) - 1;
    }
  }
}

// ============================================================
// LIDAR CHUNKS: ESP32 -> PYTHON
// ============================================================

void sendCompletedLidarScan() {
  if (lidarPointCount == 0) {
    return;
  }

  if (
    WiFi.status() != WL_CONNECTED
  ) {
    lidarScanId++;
    return;
  }

  const uint16_t chunkCount =
    static_cast<uint16_t>(
      (
        lidarPointCount +
        MAX_LIDAR_POINTS_PER_CHUNK -
        1
      ) /
      MAX_LIDAR_POINTS_PER_CHUNK
    );

  static uint8_t packetBuffer[
    sizeof(LidarChunkPrefix) +
    MAX_LIDAR_POINTS_PER_CHUNK *
    sizeof(LidarPointPacket)
  ];

  size_t sentPointCount = 0;

  for (
    uint16_t chunkIndex = 0;
    chunkIndex < chunkCount;
    chunkIndex++
  ) {
    size_t remaining =
      lidarPointCount -
      sentPointCount;

    uint16_t pointCount =
      static_cast<uint16_t>(
        remaining >
        MAX_LIDAR_POINTS_PER_CHUNK
          ? MAX_LIDAR_POINTS_PER_CHUNK
          : remaining
      );

    LidarChunkPrefix prefix{};

    prefix.magic[0] = 'Q';
    prefix.magic[1] = 'R';

    prefix.version =
      PROTOCOL_VERSION;

    prefix.packetType =
      PACKET_LIDAR_CHUNK;

    prefix.sequence =
      sensorSequence++;

    prefix.senderMillis =
      millis();

    prefix.scanId =
      lidarScanId;

    prefix.chunkIndex =
      chunkIndex;

    prefix.chunkCount =
      chunkCount;

    prefix.pointCount =
      pointCount;

    memcpy(
      packetBuffer,
      &prefix,
      sizeof(prefix)
    );

    memcpy(
      packetBuffer +
      sizeof(prefix),

      &lidarScanPoints[
        sentPointCount
      ],

      pointCount *
      sizeof(LidarPointPacket)
    );

    size_t packetSize =
      sizeof(prefix) +
      pointCount *
      sizeof(LidarPointPacket);

    sensorUdp.beginPacket(
      MAC_IP,
      SENSOR_UDP_PORT
    );

    sensorUdp.write(
      packetBuffer,
      packetSize
    );

    sensorUdp.endPacket();

    sentPointCount +=
      pointCount;

    yield();
  }

  if (lidarScanOverflowed) {
    Serial.println(
      "WARNING: RPLIDAR scan exceeded local point buffer."
    );
  }

  lidarScanId++;
}

// ============================================================
// COMMAND PACKET: PYTHON -> ESP32
// ============================================================

void handleCommandPackets() {
  int packetLength =
    commandUdp.parsePacket();

  while (packetLength > 0) {
    if (
      packetLength ==
      static_cast<int>(
        sizeof(CommandPacket)
      )
    ) {
      CommandPacket packet{};

      int bytesRead =
        commandUdp.read(
          reinterpret_cast<uint8_t*>(
            &packet
          ),
          sizeof(packet)
        );

      if (
        bytesRead ==
          static_cast<int>(
            sizeof(packet)
          ) &&
        packet.magic[0] == 'Q' &&
        packet.magic[1] == 'C' &&
        packet.version ==
          PROTOCOL_VERSION &&
        packet.packetType ==
          PACKET_COMMAND &&
        packet.commandCode <=
          COMMAND_TURN_RIGHT
      ) {
        float speedScale =
          packet.speedScale;

        if (speedScale < 0.0f) {
          speedScale = 0.0f;
        }

        if (speedScale > 1.0f) {
          speedScale = 1.0f;
        }

        bool changed =
          packet.commandCode !=
            lastCommandCode ||

          fabsf(
            packet.headingDeg -
            lastHeadingDeg
          ) > 0.5f ||

          fabsf(
            speedScale -
            lastSpeedScale
          ) > 0.02f;

        lastCommandReceivedMs =
          millis();

        if (changed) {
          lastCommandCode =
            packet.commandCode;

          lastHeadingDeg =
            packet.headingDeg;

          lastSpeedScale =
            speedScale;

          Serial.println();

          Serial.println(
            "===== PYTHON AVOIDANCE DECISION ====="
          );

          Serial.print(
            "Command: "
          );

          Serial.println(
            commandName(
              packet.commandCode
            )
          );

          Serial.print(
            "Heading: "
          );

          Serial.print(
            packet.headingDeg,
            1
          );

          Serial.println(
            " deg"
          );

          Serial.print(
            "Speed scale: "
          );

          Serial.println(
            speedScale,
            2
          );

          Serial.print(
            "Sequence: "
          );

          Serial.println(
            packet.sequence
          );

          Serial.println(
            "====================================="
          );
        }
      }
    } else {
      while (
        commandUdp.available() > 0
      ) {
        commandUdp.read();
      }
    }

    packetLength =
      commandUdp.parsePacket();
  }
}

// ============================================================
// STATUS OUTPUT
// ============================================================

void printStatus() {
  unsigned long now = millis();

  if (
    now - lastStatusPrintMs <
    STATUS_PRINT_INTERVAL_MS
  ) {
    return;
  }

  lastStatusPrintMs = now;

  Serial.println();

  Serial.println(
    "----- SENSOR HUB STATUS -----"
  );

  Serial.print("Wi-Fi: ");

  Serial.println(
    WiFi.status() ==
      WL_CONNECTED
        ? "CONNECTED"
        : "DISCONNECTED"
  );

  if (
    WiFi.status() ==
    WL_CONNECTED
  ) {
    Serial.print(
      "ESP32 IP: "
    );

    Serial.println(
      WiFi.localIP()
    );

    Serial.print(
      "RSSI: "
    );

    Serial.print(
      WiFi.RSSI()
    );

    Serial.println(
      " dBm"
    );
  }

  Serial.print(
    "LiDAR stream: "
  );

  Serial.println(
    lidarScanStreaming
      ? "RUNNING"
      : "NOT RUNNING"
  );

  Serial.print(
    "Last local LiDAR scan point count: "
  );

  Serial.println(
    lidarPointCount
  );

  Serial.print(
    "ToF initialized: "
  );

  Serial.println(
    tofAvailable
      ? "YES"
      : "NO"
  );

  Serial.print(
    "Ultrasonic enabled: "
  );

  Serial.println(
    ENABLE_ULTRASONIC
      ? "YES"
      : "NO"
  );

  Serial.print(
    "Last Python command age: "
  );

  if (
    lastCommandReceivedMs == 0
  ) {
    Serial.println(
      "none received yet"
    );
  } else {
    Serial.print(
      now -
      lastCommandReceivedMs
    );

    Serial.println(
      " ms"
    );
  }

  Serial.println(
    "-----------------------------"
  );
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  delay(1000);

  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "QUADRUPED ESP32 SENSOR <-> MAC UDP TEST"
  );

  Serial.println(
    "========================================"
  );

  // ----------------------------------------------------------
  // ULTRASONIC
  // ----------------------------------------------------------

  if (ENABLE_ULTRASONIC) {
    pinMode(
      ULTRASONIC_TRIG_PIN,
      OUTPUT
    );

    pinMode(
      ULTRASONIC_ECHO_PIN,
      INPUT
    );

    digitalWrite(
      ULTRASONIC_TRIG_PIN,
      LOW
    );
  }

  // ----------------------------------------------------------
  // TOF
  // ----------------------------------------------------------

  if (ENABLE_TOF) {
    Wire.begin(
      TOF_SDA_PIN,
      TOF_SCL_PIN
    );

    tofAvailable =
      tof.begin();

    if (tofAvailable) {
      Serial.println(
        "VL53L0X ToF initialized."
      );
    } else {
      Serial.println(
        "ERROR: VL53L0X ToF initialization failed."
      );
    }
  }

  // ----------------------------------------------------------
  // RPLIDAR MOTOR
  // ----------------------------------------------------------

  if (ENABLE_LIDAR) {
    pinMode(
      LIDAR_MOTOR_PIN,
      OUTPUT
    );

    digitalWrite(
      LIDAR_MOTOR_PIN,
      HIGH
    );

    Serial.println(
      "RPLIDAR motor enabled on GPIO 2."
    );

    delay(500);
  }

  // ----------------------------------------------------------
  // RPLIDAR UART
  // ----------------------------------------------------------

  if (ENABLE_LIDAR) {
    LidarSerial.setRxBufferSize(
      8192
    );

    LidarSerial.begin(
      LIDAR_BAUD,
      SERIAL_8N1,
      LIDAR_RX_PIN,
      LIDAR_TX_PIN
    );

    Serial.println(
      "RPLIDAR UART initialized."
    );
  }

  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  connectWiFi();

  // ----------------------------------------------------------
  // SENSOR UDP
  // ----------------------------------------------------------

  if (
    !sensorUdp.begin(
      SENSOR_LOCAL_UDP_PORT
    )
  ) {
    Serial.println(
      "ERROR: Could not open sensor UDP source port."
    );
  } else {
    Serial.println(
      "Sensor UDP sender socket started."
    );
  }

  // ----------------------------------------------------------
  // COMMAND UDP
  // ----------------------------------------------------------

  if (
    !commandUdp.begin(
      COMMAND_UDP_PORT
    )
  ) {
    Serial.println(
      "ERROR: Could not open command UDP port."
    );
  } else {
    Serial.println(
      "Command UDP listener started."
    );
  }

  // ----------------------------------------------------------
  // START RPLIDAR SCAN
  // ----------------------------------------------------------

  if (ENABLE_LIDAR) {
    lastLidarStartAttemptMs =
      millis();

    startLidarScan();
  }
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  maintainWiFi();

  handleCommandPackets();

  // ----------------------------------------------------------
  // RPLIDAR
  // ----------------------------------------------------------

  if (ENABLE_LIDAR) {
    processLidarSerial();

    unsigned long lidarNow =
      millis();

    if (
      lidarScanStreaming &&
      lidarNow -
      lastLidarValidNodeMs >
      1500
    ) {
      Serial.println(
        "WARNING: RPLIDAR data timed out; restarting scan stream."
      );

      lidarScanStreaming =
        false;
    }

    if (!lidarScanStreaming) {
      unsigned long now =
        millis();

      if (
        now -
        lastLidarStartAttemptMs >=
        LIDAR_RESTART_INTERVAL_MS
      ) {
        lastLidarStartAttemptMs =
          now;

        startLidarScan();
      }
    }
  }

  // ----------------------------------------------------------
  // ULTRASONIC + TOF PACKET
  // ----------------------------------------------------------

  unsigned long now =
    millis();

  if (
    now -
    lastRangeSendMs >=
    RANGE_INTERVAL_MS
  ) {
    lastRangeSendMs =
      now;

    sendRangePacket();
  }

  handleCommandPackets();

  printStatus();
}