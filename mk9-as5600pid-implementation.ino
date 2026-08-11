// mk9-as5600-shard.cpp
// ESP32-S3 | Cosmic3D MK9 | AS5600 Magnetic Encoder Edition

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <ArduinoMqttClient.h>
#include <MD5Builder.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>

// --- AS5600 & TCA9548A ---
// All four AS5600s share address 0x36. TCA9548A at 0x70 switches between them.
#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D
#define AS5600_ANGLE_H 0x0E
#define AS5600_ANGLE_L 0x0F
#define AS5600_STATUS 0x0B
#define AS5600_AGC 0x1A
#define AS5600_MAGNITUDE_H 0x1B
#define AS5600_MAGNITUDE_L 0x1C
#define TCA9548A_ADDRESS 0x70
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_TIMEOUT_MS 30

// --- Gear Ratio ---
// 15 magnet rotations = 1 motor rotation = 15 × 4096 = 61440 raw AS5600 counts.
// encoderCount[] is in motor-rotation-scaled units. Setpoints must match.
#define GEAR_RATIO 1

// --- Cartesian -> Delta IK / Encoder conversion ---
// Tower anchor points in millimeters.
const float TOWER_A_X = 0.000f;
const float TOWER_A_Y = 157.631f;
const float TOWER_A_Z = 735.00f;

const float TOWER_B_X = -136.513f;
const float TOWER_B_Y = -78.816f;
const float TOWER_B_Z = 735.00f;

const float TOWER_C_X = 136.513f;
const float TOWER_C_Y = -78.816f;
const float TOWER_C_Z = 735.00f;

// Configure these to match your mechanics.
const float PULLEY_RADIUS_MM = 9.5f;
const float HOME_X = 0.0f;
const float HOME_Y = 0.0f;
const float HOME_Z = 0.0f;
const float STEPS_PER_MM_E = 100.0f;
const long MOTION_TOLERANCE_COUNTS = 5;

const float AS5600_COUNTS_PER_ROTATION = 4096.0f;
const float OUTPUT_COUNTS_PER_ROTATION = AS5600_COUNTS_PER_ROTATION;
const float MANUAL_JOG_MM = 10.0f;
const float COUNTS_PER_MM =
    OUTPUT_COUNTS_PER_ROTATION / (2.0f * PI * PULLEY_RADIUS_MM);

long mmToEncoderCounts(float mm) { return (long)lroundf(mm * COUNTS_PER_MM); }

float homeCableLength[3] = {0.0f, 0.0f, 0.0f};
bool homeCableLengthReady = false;

// --- SD Card ---
#define REASSIGN_PINS
int sck = 12;
int miso = 13;
int mosi = 11;
int cs = 10;

// --- Heater & Thermistor ---
#define THERMISTOR_PIN 2
#define HEATER_PIN 21
#define MAX_TEMP 280
#define HYSTERESIS 2.0f
#define SETPOINT 0 // Updated at runtime via MQTT: hotendtemp/VALUE
#define SERIES_RESISTOR 4700.0f
#define BETA 3950.0f

#define BED_THERMISTOR_PIN 1
#define BED_HEATER_PIN 47
#define BED_MAX_TEMP 130
#define BED_HYSTERESIS 2.0f
#define BED_SETPOINT 0 // Updated at runtime via MQTT: bedtemp/VALUE
#define BED_SERIES_RESISTOR 4700.0f
#define BED_BETA 3950.0f

#define ADC_MAX 4095.0f
#define T0 25.0f
#define T0_K (T0 + 273.15f)
#define HEATER_POLL_MS 500

bool hotendHeaterOn = false;
bool bedHeaterOn = false;
volatile float hotendSetpoint = SETPOINT;
volatile float bedSetpoint = BED_SETPOINT;

// --- WiFi & MQTT ---
const char *MQTT_BROKER = "test.mosquitto.org";
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";

#define TOPIC_START_file_start_stop "file_transfer/start"
#define TOPIC_STOP "file_transfer/data"
#define TOPIC_xyz_move "motor/xyz_move"
#define TOPIC_ACK "file_transfer/ack" // outbound only
#define MOTOR_START "motor/start"
#define MOTOR_STOP "motor/stop"

bool messageReceived = false;

// --- Cartesian State ---
float currentX = HOME_X;
float currentY = HOME_Y;
float currentZ = HOME_Z;
String topicBuffer = "";
String payloadBuffer = "";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// --- NVS ---
Preferences prefs;
long lastReceivedChunk = -1;
unsigned long lastAttemptTime = 0;
const unsigned long reconnectDelay = 5000;
bool printedLostMsg = false;
bool fileReceiving = false;
String fileName = "";
int totalChunks = 0;
String expectedChecksum = "";

// --- Motor Pins ---
#define motorPinA1 14
#define motorPinA2 7
#define motorPinB1 15
#define motorPinB2 16
#define motorPinc1 5
#define motorPinc2 6
#define motorPind1 4
#define motorPind2 3

// --- Limit Switches ---
#define LIMIT1_PIN 46
#define LIMIT2_PIN 43
#define LIMIT3_PIN 48

// --- Encoder & Motion ---
volatile long encoderCount[4] = {0, 0, 0, 0};
volatile int rotation[4] = {0, 0, 0, 0};
volatile long setpoint[4] = {0, 0, 0, 0};
long lastAngle[4] = {0, 0, 0, 0};
long totalRotations[4] = {0, 0, 0, 0};
static long rawAccumulator[4] = {0, 0, 0, 0};

// --- PID ---
float Kp[4] = {2.0f, 2.0f, 2.0f, 2.0f};
float Ki[4] = {0.01f, 0.01f, 0.01f, 0.01f};
float Kd[4] = {0.5f, 0.5f, 0.5f, 0.5f};
long prevError[4] = {0, 0, 0, 0};
float integral[4] = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastTimePID = 0;

// --- SD File Handle ---
File file;
int totalLines = 0;
int myconut = 0;

String myBit0 = "0", myBit1 = "0", myBit2 = "0", myBit3 = "0";
String myBit4 = "0", myBit5 = "0", myBit6 = "0", myBit7 = "0";

const uint8_t motorToChannel[4] = {0, 1, 2, 3};
bool as5600Ready = false;
bool mk9CoreReady = false;

// --- State Machine ---
enum SystemState {
  STATE_IDLE,
  STATE_HOMING_SEEK,
  STATE_HOMING_ZERO,
  STATE_HOMING_STANDOFF,
  STATE_MOTION_OPEN,
  STATE_MOTION_READ_LINE,
  STATE_MOTION_PID,
  STATE_MOTION_DWELL,
  STATE_MOTION_DONE,
  STATE_MOTION_PAUSED,
  STATE_MANUAL_XYZ
};

volatile SystemState sysState = STATE_IDLE;
bool m1_home = false, m2_home = false, m3_home = false;
long setpointHome[4] = {100, 100, 100, 100};
unsigned long dwellStart = 0;

bool manualMovePending = false;
float manualMoveDeltaX = 0.0f;
float manualMoveDeltaY = 0.0f;
float manualMoveDeltaZ = 0.0f;
long manualTarget[4] = {0, 0, 0, 0};
SystemState pausedPrintState = STATE_IDLE;

bool isActivePrintState(int state) {
  return state == STATE_MOTION_OPEN || state == STATE_MOTION_READ_LINE ||
         state == STATE_MOTION_PID || state == STATE_MOTION_DWELL;
}

void resetManualXYZTarget() {
  for (int i = 0; i < 4; i++) {
    manualTarget[i] = encoderCount[i];
  }
  lastTimePID = millis();
  for (int i = 0; i < 4; i++) {
    integral[i] = 0.0f;
    prevError[i] = 0;
  }
}

void queueManualMove(float dx, float dy, float dz) {
  manualMoveDeltaX = dx;
  manualMoveDeltaY = dy;
  manualMoveDeltaZ = dz;
  manualMovePending = true;
}

void applyPendingManualMove() {
  if (!manualMovePending)
    return;

  float nextX = currentX + manualMoveDeltaX;
  float nextY = currentY + manualMoveDeltaY;
  float nextZ = currentZ + manualMoveDeltaZ;

  long targetA = 0, targetB = 0, targetC = 0;
  if (convertXYZToEncoderTargets(nextX, nextY, nextZ, targetA, targetB,
                                 targetC)) {
    currentX = nextX;
    currentY = nextY;
    currentZ = nextZ;
    manualTarget[0] = targetA;
    manualTarget[1] = targetB;
    manualTarget[2] = targetC;

    Serial.printf("XYZ %.2f %.2f %.2f\n", currentX, currentY, currentZ);
    Serial.printf("Targets %ld %ld %ld\n", targetA, targetB, targetC);

    lastTimePID = millis();
    for (int i = 0; i < 4; i++) {
      integral[i] = 0.0f;
      prevError[i] = 0;
    }
  } else {
    Serial.println("[MANUAL] IK calculation failed, move discarded.");
  }

  manualMovePending = false;
  manualMoveDeltaX = 0.0f;
  manualMoveDeltaY = 0.0f;
  manualMoveDeltaZ = 0.0f;
}

// =============================================================================
// NVS
// =============================================================================
void saveState() {
  prefs.putString("fileName", fileName);
  prefs.putInt("totalChunks", totalChunks);
  prefs.putInt("lastChunk", lastReceivedChunk);
  prefs.putString("checksum", expectedChecksum);
  Serial.println("[NVS] State saved.");
}

void restoreState() {
  fileName = prefs.getString("fileName", "");
  totalChunks = prefs.getInt("totalChunks", 0);
  lastReceivedChunk = prefs.getInt("lastChunk", -1);
  expectedChecksum = prefs.getString("checksum", "");
  Serial.println("[NVS] State restored.");
  Serial.print("[NVS]   fileName: ");
  Serial.println(fileName);
  Serial.print("[NVS]   totalChunks: ");
  Serial.println(totalChunks);
  Serial.print("[NVS]   lastChunk: ");
  Serial.println(lastReceivedChunk);
}

void loadState() {
  lastReceivedChunk = prefs.getLong("lastChunk", -1);
  Serial.print("[NVS] Last chunk: ");
  Serial.println(lastReceivedChunk);
}

// =============================================================================
// SD
// =============================================================================
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("[SD] Listing: %s\n", dirname);
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) {
    Serial.println("[SD] Failed.");
    return;
  }
  File f = root.openNextFile();
  while (f) {
    if (f.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(f.name());
      if (levels)
        listDir(fs, f.name(), levels - 1);
    } else {
      Serial.print("  FILE: ");
      Serial.print(f.name());
      Serial.print("  SIZE: ");
      Serial.println(f.size());
    }
    f = root.openNextFile();
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File f = fs.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("[SD] Write open failed.");
    return;
  }
  f.print(message) ? Serial.println("[SD] Written.")
                   : Serial.println("[SD] Write failed.");
  f.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File f = fs.open(path, FILE_APPEND);
  if (!f) {
    Serial.println("[SD] Append open failed.");
    return;
  }
  f.print(message) ? Serial.println("[SD] Appended ✅")
                   : Serial.println("[SD] Append failed.");
  f.close();
}

int countLinesInFile(fs::FS &fs, const char *path) {
  File tempFile = fs.open(path);
  if (!tempFile) {
    Serial.println("[SD] Line count open failed.");
    return -1;
  }
  int lineCount = 0;
  while (tempFile.available()) {
    if (tempFile.read() == '\n')
      lineCount++;
  }
  if (tempFile.size() > 0 && tempFile.peek() != '\n')
    lineCount++;
  tempFile.close();
  Serial.printf("[SD] Lines in %s: %d\n", path, lineCount);
  return lineCount;
}

String readNextLine() {
  if (file && file.available())
    return file.readStringUntil('\n');
  return "";
}

bool calculateIK(float nozzleX, float nozzleY, float nozzleZ, float &cableA,
                 float &cableB, float &cableC) {
  const float EFFECTOR_RADIUS = 22.92f;
  const float NOZZLE_TO_ATTACHMENT = 124.403f;

  const float PIVOT_X = 0.0f;
  const float PIVOT_Y = 0.0f;
  const float PIVOT_Z = 735.0f;

  // ------------------------------------------------
  // Rod direction (Pivot -> Nozzle)
  // ------------------------------------------------

  float dx = nozzleX - PIVOT_X;
  float dy = nozzleY - PIVOT_Y;
  float dz = nozzleZ - PIVOT_Z;

  float mag = sqrtf(dx * dx + dy * dy + dz * dz);

  if (mag < 0.0001f)
    return false;

  float ux = dx / mag;
  float uy = dy / mag;
  float uz = dz / mag;

  // ------------------------------------------------
  // Cable attachment ring center
  // ------------------------------------------------

  float centerX = nozzleX - ux * NOZZLE_TO_ATTACHMENT;
  float centerY = nozzleY - uy * NOZZLE_TO_ATTACHMENT;
  float centerZ = nozzleZ - uz * NOZZLE_TO_ATTACHMENT;

  // ------------------------------------------------
  // Build local frame around rod
  // ------------------------------------------------

  float rx = 0.0f;
  float ry = 0.0f;
  float rz = 1.0f;

  if (fabsf(uz) > 0.99f) {
    rx = 1.0f;
    ry = 0.0f;
    rz = 0.0f;
  }

  // u = reference × rodDir

  float u1x = ry * uz - rz * uy;
  float u1y = rz * ux - rx * uz;
  float u1z = rx * uy - ry * ux;

  float u1mag = sqrtf(u1x * u1x + u1y * u1y + u1z * u1z);

  if (u1mag < 0.0001f)
    return false;

  u1x /= u1mag;
  u1y /= u1mag;
  u1z /= u1mag;

  // u2 = rodDir × u1

  float u2x = uy * u1z - uz * u1y;
  float u2y = uz * u1x - ux * u1z;
  float u2z = ux * u1y - uy * u1x;

  // ------------------------------------------------
  // Attachment points
  // ------------------------------------------------

  const float COS120 = -0.5f;
  const float SIN120 = 0.8660254f;

  float ax = centerX + EFFECTOR_RADIUS * u1x;

  float ay = centerY + EFFECTOR_RADIUS * u1y;

  float az = centerZ + EFFECTOR_RADIUS * u1z;

  float bx = centerX + EFFECTOR_RADIUS * (COS120 * u1x + SIN120 * u2x);

  float by = centerY + EFFECTOR_RADIUS * (COS120 * u1y + SIN120 * u2y);

  float bz = centerZ + EFFECTOR_RADIUS * (COS120 * u1z + SIN120 * u2z);

  float cx = centerX + EFFECTOR_RADIUS * (COS120 * u1x - SIN120 * u2x);

  float cy = centerY + EFFECTOR_RADIUS * (COS120 * u1y - SIN120 * u2y);

  float cz = centerZ + EFFECTOR_RADIUS * (COS120 * u1z - SIN120 * u2z);

  // ------------------------------------------------
  // Cable lengths
  // ------------------------------------------------

  cableA = sqrtf((ax - TOWER_A_X) * (ax - TOWER_A_X) +
                 (ay - TOWER_A_Y) * (ay - TOWER_A_Y) +
                 (az - TOWER_A_Z) * (az - TOWER_A_Z));

  cableB = sqrtf((bx - TOWER_B_X) * (bx - TOWER_B_X) +
                 (by - TOWER_B_Y) * (by - TOWER_B_Y) +
                 (bz - TOWER_B_Z) * (bz - TOWER_B_Z));

  cableC = sqrtf((cx - TOWER_C_X) * (cx - TOWER_C_X) +
                 (cy - TOWER_C_Y) * (cy - TOWER_C_Y) +
                 (cz - TOWER_C_Z) * (cz - TOWER_C_Z));

  return isfinite(cableA) && isfinite(cableB) && isfinite(cableC);
}

void initializeHomeCableLengths() {
  float l1 = 0.0f, l2 = 0.0f, l3 = 0.0f;
  if (!calculateIK(HOME_X, HOME_Y, HOME_Z, l1, l2, l3)) {
    Serial.println("[IK] Failed to compute home cable lengths.");
    homeCableLengthReady = false;
    return;
  }

  homeCableLength[0] = l1;
  homeCableLength[1] = l2;
  homeCableLength[2] = l3;
  homeCableLengthReady = true;

  Serial.printf("[IK] Home lengths -> A:%.3f  B:%.3f  C:%.3f\n",
                homeCableLength[0], homeCableLength[1], homeCableLength[2]);
}

bool convertXYZToEncoderTargets(float targetX, float targetY, float targetZ,
                                long &targetA, long &targetB, long &targetC) {
  if (!homeCableLengthReady)
    initializeHomeCableLengths();
  if (!homeCableLengthReady)
    return false;

  if (PULLEY_RADIUS_MM <= 0.0f) {
    Serial.println("[IK] Invalid PULLEY_RADIUS_MM. Must be > 0.");
    return false;
  }

  float l1 = 0.0f, l2 = 0.0f, l3 = 0.0f;
  if (!calculateIK(targetX, targetY, targetZ, l1, l2, l3)) {
    Serial.println("[IK] Failed to compute target cable lengths.");
    return false;
  }

  float circumference = 2.0f * PI * PULLEY_RADIUS_MM;

  float deltaL1 = l1 - homeCableLength[0];
  float deltaL2 = l2 - homeCableLength[1];
  float deltaL3 = l3 - homeCableLength[2];

  float rotationsA = deltaL1 / circumference;
  float rotationsB = deltaL2 / circumference;
  float rotationsC = deltaL3 / circumference;

  targetA = (long)lroundf(rotationsA * OUTPUT_COUNTS_PER_ROTATION);
  targetB = (long)lroundf(rotationsB * OUTPUT_COUNTS_PER_ROTATION);
  targetC = (long)lroundf(rotationsC * OUTPUT_COUNTS_PER_ROTATION);

  return true;
}

bool parseCartesianLine(String line, float &x, float &y, float &z, float &e) {
  line.trim();
  if (line.length() == 0)
    return false;

  line.replace(",", " ");
  line.replace(";", " ");

  float val1, val2, val3, val4, val5;
  int parsedCount =
      sscanf(line.c_str(), "%f %f %f %f %f", &val1, &val2, &val3, &val4, &val5);

  if (parsedCount == 5) {
    x = val1;
    y = val2;
    z = val3;
    e = val5; // rot (val4) is ignored in 3-axis motion kinematics
    return true;
  } else if (parsedCount == 4) {
    x = val1;
    y = val2;
    z = val3;
    e = val4;
    return true;
  }
  return false;
}

// =============================================================================
// THERMISTOR — Steinhart-Hart B-parameter. Returns °C, -1 on fault.
// =============================================================================
float readThermistorC(int pin, float seriesResistor, float beta) {
  long sum = 0;
  const int samples = 32;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  float adcValue = (float)sum / samples;

  if (adcValue <= 5.0f || adcValue >= 4090.0f)
    return -1.0f;
  float resistance = seriesResistor * (ADC_MAX / adcValue - 1.0f);
  float steinhart = log(resistance / 100000.0f);
  steinhart /= beta;
  steinhart += 1.0f / (T0 + 273.15f);
  return (1.0f / steinhart) - 273.15f;
}

// =============================================================================
// I2C / AS5600 LOW-LEVEL
// =============================================================================
bool selectI2CChannel(uint8_t channel) {
  if (channel > 7)
    return false;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("[I2C] TCA9548A select ch %u failed (err=%u)\n", channel,
                  err);
    return false;
  }
  return true;
}

void disableAllI2CChannels() {
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
}

uint16_t readAS5600Register16(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return 0;

  size_t n =
      Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2, (uint8_t) true);
  if (n >= 2 && Wire.available() >= 2) {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

uint8_t readAS5600Register8(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return 0;

  size_t n =
      Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)1, (uint8_t) true);
  if (n >= 1 && Wire.available())
    return Wire.read();
  return 0;
}

uint16_t readRawAngle() { return readAS5600Register16(AS5600_RAW_ANGLE_H); }
uint16_t readAngle() { return readAS5600Register16(AS5600_ANGLE_H); }
uint8_t readStatus() { return readAS5600Register8(AS5600_STATUS); }
uint8_t readAGC() { return readAS5600Register8(AS5600_AGC); }
uint16_t readMagnitude() { return readAS5600Register16(AS5600_MAGNITUDE_H); }

bool probeI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool isMagnetDetected() {
  uint8_t status = readStatus();
  return (status & 0x20) && !(status & 0x18);
}

// =============================================================================
// ENCODER UPDATE
// Accumulates raw delta, divides by GEAR_RATIO into encoderCount[].
// Remainder preserved in rawAccumulator[] between calls.
// =============================================================================
void updateEncoderFromAS5600(int motorIndex) {
  if (!as5600Ready)
    return;
  if (!selectI2CChannel(motorToChannel[motorIndex]))
    return;
  delay(1);

  uint16_t currentAngle = readRawAngle();
  long diff = (long)currentAngle - (long)lastAngle[motorIndex];

  if (diff > 2048) {
    diff -= 4096;
    totalRotations[motorIndex]--;
  }
  if (diff < -2048) {
    diff += 4096;
    totalRotations[motorIndex]++;
  }

  if (abs(diff) > 2048) {
    Serial.printf("[ENC] Motor %d: Spurious jump (%ld), ignored.\n", motorIndex,
                  diff);
    return;
  }

  rawAccumulator[motorIndex] += diff;
  long motorCounts = rawAccumulator[motorIndex] / GEAR_RATIO;

  if (motorCounts != 0) {
    long prevCount = encoderCount[motorIndex];
    encoderCount[motorIndex] += motorCounts;
    rawAccumulator[motorIndex] -= motorCounts * GEAR_RATIO;

    Serial.printf("[ENC] Motor %d | Ch %d | RawAngle: %u | RawDiff: %ld | "
                  "RawAcc: %ld | MotorDelta: %ld | Count: %ld → %ld | "
                  "TotalRots: %ld | Setpoint: %ld\n",
                  motorIndex, motorToChannel[motorIndex], currentAngle, diff,
                  rawAccumulator[motorIndex], motorCounts, prevCount,
                  encoderCount[motorIndex], totalRotations[motorIndex],
                  setpoint[motorIndex]);
  }

  lastAngle[motorIndex] = currentAngle;
  disableAllI2CChannels();
}

// =============================================================================
// AS5600 INIT
// =============================================================================
void initAS5600() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  Serial.println("[AS5600] Initializing...");

  if (!probeI2CDevice(TCA9548A_ADDRESS)) {
    Serial.println(
        "[AS5600] TCA9548A not detected at 0x70. Encoder subsystem disabled.");
    as5600Ready = false;
    return;
  }

  as5600Ready = true;

  Serial.println("[I2C SCAN] Scanning TCA9548A channels for AS5600 (0x36)...");
  for (uint8_t ch = 0; ch < 8; ch++) {
    if (selectI2CChannel(ch)) {
      delay(5);
      if (probeI2CDevice(AS5600_ADDRESS)) {
        Serial.printf("[I2C SCAN] Found AS5600 encoder on channel %u\n", ch);
      }
    }
  }
  disableAllI2CChannels();
  Serial.println("[I2C SCAN] Scan complete.");

  for (int i = 0; i < 4; i++) {
    if (!selectI2CChannel(motorToChannel[i])) {
      as5600Ready = false;
      Serial.printf("[AS5600] Channel %d unavailable.\n", motorToChannel[i]);
      continue;
    }
    delay(10);

    uint16_t angle = readRawAngle();
    uint8_t status = readStatus();
    uint16_t magnitude = readMagnitude();
    uint8_t agc = readAGC();
    bool magOK = isMagnetDetected();

    Serial.printf("[AS5600] Motor %d (Ch %d): Angle=%u  Status=0x%02X  "
                  "Magnitude=%u  AGC=%u  Magnet=%s\n",
                  i, motorToChannel[i], angle, status, magnitude, agc,
                  magOK ? "✓ OK" : "✗ MISSING!");

    lastAngle[i] = angle;
    encoderCount[i] = 0;
    rawAccumulator[i] = 0;
    totalRotations[i] = 0;
    delay(1);
  }

  disableAllI2CChannels();
  Serial.println(as5600Ready ? "[AS5600] Init complete."
                             : "[AS5600] Init degraded (offline channels).");
}

// =============================================================================
// ENCODER DIAGNOSTICS
// =============================================================================
void checkAllEncoders() {
  if (!as5600Ready) {
    Serial.println("[DIAG] AS5600 unavailable (TCA9548A/encoder offline).");
    return;
  }

  Serial.println("\n[DIAG] ===== Encoder Status =====");
  for (int i = 0; i < 4; i++) {
    if (!selectI2CChannel(motorToChannel[i])) {
      Serial.printf("[DIAG] Motor %d | Channel select failed\n", i);
      continue;
    }
    delay(1);

    uint16_t angle = readRawAngle();
    uint8_t status = readStatus();
    uint16_t magnitude = readMagnitude();
    uint8_t agc = readAGC();
    bool magOK = isMagnetDetected();

    Serial.printf("[DIAG] Motor %d | Angle=%u | Count=%ld | RawAcc=%ld | "
                  "Setpoint=%ld | Mag=%u | AGC=%u | Status=0x%02X | %s\n",
                  i, angle, encoderCount[i], rawAccumulator[i], setpoint[i],
                  magnitude, agc, status, magOK ? "✓ OK" : "✗ MISSING!");
    delay(1);
  }
  disableAllI2CChannels();
  Serial.println("[DIAG] ==============================\n");
}

// =============================================================================
// MQTT RECONNECT
// =============================================================================
void mqttReconnect() {
  Serial.print("[MQTT] Reconnecting... ");
  if (mqttClient.connect(MQTT_BROKER, 1883)) {
    Serial.println("connected ✅");
    mqttClient.subscribe(TOPIC_START_file_start_stop);
    mqttClient.subscribe(TOPIC_STOP);
    mqttClient.subscribe(TOPIC_xyz_move);
    mqttClient.subscribe(MOTOR_START);
    mqttClient.subscribe(MOTOR_STOP);
  } else {
    Serial.print("[MQTT] Failed, error=");
    Serial.println(mqttClient.connectError());
  }
}

void sendmessage(String publishtopic, String messagetosend) {
  if (!mqttClient.connected())
    return;
  mqttClient.beginMessage(publishtopic);
  mqttClient.print(messagetosend);
  mqttClient.endMessage();
}

// =============================================================================
// HEATER CONTROL — called every HEATER_POLL_MS via millis() in loop().
// Logs on state transitions only.
// =============================================================================
void runHeaterControl() {
  float hotendTemp = readThermistorC(THERMISTOR_PIN, SERIES_RESISTOR, BETA);

  if (hotendTemp < 0 || hotendTemp > MAX_TEMP) {
    if (hotendHeaterOn) {
      digitalWrite(HEATER_PIN, LOW);
      hotendHeaterOn = false;
      Serial.printf("[HEAT] HOTEND FAULT (%.1f°C) → OFF ❌\n", hotendTemp);
    }
  } else {
    if (hotendTemp < (hotendSetpoint - HYSTERESIS) && !hotendHeaterOn) {
      digitalWrite(HEATER_PIN, HIGH);
      hotendHeaterOn = true;
      Serial.printf("[HEAT] Hotend %.1f°C → ON 🔥  (sp %.1f°C)\n", hotendTemp,
                    hotendSetpoint);
    } else if (hotendTemp > (hotendSetpoint + HYSTERESIS) && hotendHeaterOn) {
      digitalWrite(HEATER_PIN, LOW);
      hotendHeaterOn = false;
      Serial.printf("[HEAT] Hotend %.1f°C → OFF  (sp %.1f°C)\n", hotendTemp,
                    hotendSetpoint);
    }
  }

  float bedTemp =
      readThermistorC(BED_THERMISTOR_PIN, BED_SERIES_RESISTOR, BED_BETA);

  if (bedTemp < 0 || bedTemp > BED_MAX_TEMP) {
    if (bedHeaterOn) {
      digitalWrite(BED_HEATER_PIN, LOW);
      bedHeaterOn = false;
      Serial.printf("[HEAT] BED FAULT (%.1f°C) → OFF ❌\n", bedTemp);
    }
  } else {
    if (bedTemp < (bedSetpoint - BED_HYSTERESIS) && !bedHeaterOn) {
      digitalWrite(BED_HEATER_PIN, HIGH);
      bedHeaterOn = true;
      Serial.printf("[HEAT] Bed %.1f°C → ON 🛏  (sp %.1f°C)\n", bedTemp,
                    bedSetpoint);
    } else if (bedTemp > (bedSetpoint + BED_HYSTERESIS) && bedHeaterOn) {
      digitalWrite(BED_HEATER_PIN, LOW);
      bedHeaterOn = false;
      Serial.printf("[HEAT] Bed %.1f°C → OFF  (sp %.1f°C)\n", bedTemp,
                    bedSetpoint);
    }
  }

  Serial.printf(
      "[HEAT] Hotend: %.1f°C [%s / sp %.1f°C] | Bed: %.1f°C [%s / sp %.1f°C]\n",
      hotendTemp, hotendHeaterOn ? "ON" : "OFF", hotendSetpoint, bedTemp,
      bedHeaterOn ? "ON" : "OFF", bedSetpoint);
}

// =============================================================================
// PID HELPER
// Returns signed float: positive = forward, negative = reverse.
// Updates integral[i] and prevError[i] in-place.
// =============================================================================
float calculatePID(int i, long targetPosition, float dt) {
  long error = targetPosition - encoderCount[i];

  float P = Kp[i] * (float)error;

  integral[i] += (float)error * dt;
  integral[i] = constrain(integral[i], -1000.0f, 1000.0f);
  float I = Ki[i] * integral[i];

  float D = Kd[i] * ((float)(error - prevError[i]) / dt);

  prevError[i] = error;
  float output = P + I + D;

  Serial.printf("[PID] Motor %d | err=%ld  P=%.2f  I=%.2f  D=%.2f  "
                "out=%.2f  count=%ld  sp=%ld\n",
                i, error, P, I, D, output, encoderCount[i], targetPosition);
  return output;
}

// =============================================================================
// MOTOR DRIVE
// PWM_MIN_FLOOR: motors stall below ~50 PWM due to static friction.
// Any non-zero output below this is bumped up to guarantee shaft movement.
// Adjust if your motor/driver needs a different threshold.
// =============================================================================
#define PWM_MIN_FLOOR 50
#define PWM_MIN_FLOOR_M4 190

static int lastDirection[4] = {-1, -1, -1, -1};

void driveMotor(int i, float output) {
  int pwm = (int)abs(output);
  int floor = (i == 3) ? PWM_MIN_FLOOR_M4 : PWM_MIN_FLOOR;

  if (pwm > 0 && pwm < floor) {
    Serial.printf("[MOTOR] Motor %d stiction floor: %d → %d\n", i, pwm, floor);
    pwm = floor;
  }
  if (pwm > 255)
    pwm = 255;

  int newDir = (output > 0) ? 0 : (output < 0) ? 1 : 2;

  if (i == 0) {
    if (output > 0) {
      analogWrite(motorPinA1, 0);
      analogWrite(motorPinA2, pwm);
    } else if (output < 0) {
      analogWrite(motorPinA1, pwm);
      analogWrite(motorPinA2, 0);
    } else {
      analogWrite(motorPinA1, 0);
      analogWrite(motorPinA2, 0);
    }
  } else if (i == 1) {
    if (output > 0) {
      analogWrite(motorPinB1, 0);
      analogWrite(motorPinB2, pwm);
    } else if (output < 0) {
      analogWrite(motorPinB1, pwm);
      analogWrite(motorPinB2, 0);
    } else {
      analogWrite(motorPinB1, 0);
      analogWrite(motorPinB2, 0);
    }
  } else if (i == 2) {
    if (output > 0) {
      analogWrite(motorPinc1, 0);
      analogWrite(motorPinc2, pwm);
    } else if (output < 0) {
      analogWrite(motorPinc1, pwm);
      analogWrite(motorPinc2, 0);
    } else {
      analogWrite(motorPinc1, 0);
      analogWrite(motorPinc2, 0);
    }
  } else {
    if (output > 0) {
      analogWrite(motorPind1, 0);
      analogWrite(motorPind2, pwm);
    } else if (output < 0) {
      analogWrite(motorPind1, pwm);
      analogWrite(motorPind2, 0);
    } else {
      analogWrite(motorPind1, 0);
      analogWrite(motorPind2, 0);
    }
  }

  if (newDir != lastDirection[i]) {
    const char *dirStr = (newDir == 0) ? "FWD" : (newDir == 1) ? "REV" : "STOP";
    Serial.printf("[MOTOR] Motor %d %s  out=%.2f  pwm=%d  count=%ld  sp=%ld\n",
                  i, dirStr, output, pwm, encoderCount[i], setpoint[i]);
    lastDirection[i] = newDir;
    rotation[i] = (newDir == 1) ? 1 : 0;
  }
}

// =============================================================================
// STOP ALL MOTORS
// =============================================================================
void stopAllMotors() {
  analogWrite(motorPinA1, 0);
  Serial.println("stop A1");
  analogWrite(motorPinA2, 0);
  Serial.println("stop A2");
  analogWrite(motorPinB1, 0);
  Serial.println("stop B1");
  analogWrite(motorPinB2, 0);
  Serial.println("stop B2");
  analogWrite(motorPinc1, 0);
  Serial.println("stop C1");
  analogWrite(motorPinc2, 0);
  Serial.println("stop C2");
  analogWrite(motorPind1, 0);
  Serial.println("stop D1");
  analogWrite(motorPind2, 0);
  Serial.println("stop D2");
  for (int i = 0; i < 4; i++)
    lastDirection[i] = 2;
  Serial.println("[MOTOR] All stopped.");
}

// =============================================================================
// MK9 SETUP (called by WiFi/LED sketch after STA is connected)
// =============================================================================
void mk9Setup() {
  mk9CoreReady = false;

  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
  pinMode(motorPinc1, OUTPUT);
  pinMode(motorPinc2, OUTPUT);
  pinMode(motorPind1, OUTPUT);
  pinMode(motorPind2, OUTPUT);
  stopAllMotors();
  delay(1);

  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(LIMIT3_PIN, INPUT_PULLUP);
  delay(1);

  if (HEATER_PIN == motorPinA1 || HEATER_PIN == motorPinA2 ||
      HEATER_PIN == motorPinB1 || HEATER_PIN == motorPinB2 ||
      HEATER_PIN == motorPinc1 || HEATER_PIN == motorPinc2 ||
      HEATER_PIN == motorPind1 || HEATER_PIN == motorPind2) {
    Serial.printf("[PIN] WARNING: HEATER_PIN (%d) conflicts with motor pin.\n",
                  HEATER_PIN);
  }
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  pinMode(BED_HEATER_PIN, OUTPUT);
  digitalWrite(BED_HEATER_PIN, LOW);
  delay(1);

  initAS5600();
  delay(1);

  if (!as5600Ready) {
    Serial.println("[SETUP] AS5600/TCA9548A offline. Entering safe mode (WiFi "
                   "active, motion disabled).");
    return;
  }

#ifdef REASSIGN_PINS
  SPI.begin(sck, miso, mosi, cs);
#endif
  delay(1);
  if (!SD.begin(cs)) {
    Serial.println("[SD] Init FAILED!");
  } else {
    Serial.printf("[SD] Init OK. %lluMB\n", SD.cardSize() / (1024 * 1024));
  }
  delay(1);

  prefs.begin("file-transfer", false);
  restoreState();
  delay(1);

  mqttClient.setId("cosmic3d-mk9-as5600");
  if (strlen(MQTT_USER) > 0)
    mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASSWORD);

  mqttClient.onMessage([](int messageSize) {
    topicBuffer = mqttClient.messageTopic();
    payloadBuffer = "";
    while (mqttClient.available())
      payloadBuffer += (char)mqttClient.read();
    messageReceived = true;
    Serial.printf("[MQTT] ← %s  (%d bytes)\n", topicBuffer.c_str(),
                  (int)payloadBuffer.length());
  });

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[MQTT] Connecting to ");
    Serial.println(MQTT_BROKER);
    if (!mqttClient.connect(MQTT_BROKER, 1883)) {
      Serial.print("[MQTT] Failed, error=");
      Serial.println(mqttClient.connectError());
    } else {
      Serial.println("[MQTT] Connected ✅");
      mqttClient.subscribe(TOPIC_START_file_start_stop);
      mqttClient.subscribe(TOPIC_STOP);
      mqttClient.subscribe(TOPIC_xyz_move);
      mqttClient.subscribe(MOTOR_START);
      mqttClient.subscribe(MOTOR_STOP);
    }
  } else {
    Serial.println("[MQTT] WiFi unavailable during init, deferred.");
  }

  Serial.println("[SETUP] Ready.");
  initializeHomeCableLengths();
  // Skip immediate full diagnostics at startup to avoid long I2C burst right
  // after WiFi attach.
  mk9CoreReady = true;
}

bool isPrinterActive() {
  return sysState != STATE_IDLE && sysState != STATE_MOTION_PAUSED;
}

// =============================================================================
// LOOP — Cooperative state machine. No blocking while-loops.
// =============================================================================
void mk9Loop() {
  if (!mk9CoreReady) {
    delay(1);
    return;
  }

  // --- Always serviced regardless of state ---
  if (!mqttClient.connected()) {
    if (WiFi.status() == WL_CONNECTED && !isPrinterActive()) {
      unsigned long now = millis();
      if (now - lastAttemptTime > reconnectDelay) {
        lastAttemptTime = now;
        mqttReconnect();
      }
    }
  } else {
    mqttClient.poll();
  }

  static unsigned long lastHeaterPoll = 0;
  if (millis() - lastHeaterPoll >= HEATER_POLL_MS) {
    lastHeaterPoll = millis();
    runHeaterControl();
  }

  static unsigned long lastDiagnostic = 0;
  if (millis() - lastDiagnostic > 30000) {
    lastDiagnostic = millis();
    checkAllEncoders();
  }

  // --- Process MQTT messages ---
  if (messageReceived) {
    messageReceived = false;
    Serial.printf("[LOOP] Topic: %s\n", topicBuffer.c_str());

    // Emergency stop / pause — works from ANY state
    if (topicBuffer == MOTOR_STOP) {
      if (payloadBuffer == "STOP") {
        Serial.println("[MOTION] Stop.");
        stopAllMotors();
        if (file)
          file.close();
        sysState = STATE_IDLE;
        pausedPrintState = STATE_IDLE;
        manualMovePending = false;
      } else if (payloadBuffer == "PAUSE") {
        if (isActivePrintState(sysState)) {
          Serial.println("[MOTION] Pause requested. Pausing print safely...");
          pausedPrintState = sysState;
          sysState = STATE_MOTION_PAUSED;
        } else {
          Serial.println(
              "[MOTION] Pause requested but no active print running.");
        }
      }
    }

    // -------------------------------------------------------------------------
    // file_transfer/start
    // '|' = file init | '/' = command with value | plain = bare command
    // -------------------------------------------------------------------------
    else if (topicBuffer == TOPIC_START_file_start_stop) {

      if (payloadBuffer.indexOf('|') != -1) {
        int pipe1 = payloadBuffer.indexOf('|');
        int pipe2 = payloadBuffer.indexOf('|', pipe1 + 1);
        int pipe3 = payloadBuffer.indexOf('|', pipe2 + 1);

        if (pipe1 != -1 && pipe2 != -1) {
          if (pipe1 == 0 && pipe3 != -1) {
            // Format: |filename|total_chunks|checksum
            fileName = payloadBuffer.substring(pipe1 + 1, pipe2);
            totalChunks = payloadBuffer.substring(pipe2 + 1, pipe3).toInt();
            expectedChecksum = payloadBuffer.substring(pipe3 + 1);
          } else {
            // Format: filename|total_chunks|checksum
            fileName = payloadBuffer.substring(0, pipe1);
            totalChunks = payloadBuffer.substring(pipe1 + 1, pipe2).toInt();
            expectedChecksum = payloadBuffer.substring(pipe2 + 1);
          }
          fileName.trim();
          expectedChecksum.trim();
          if (!fileName.startsWith("/")) {
            fileName = "/" + fileName;
          }

          lastReceivedChunk = -1;
          fileReceiving = true;
          if (SD.exists(fileName.c_str()))
            SD.remove(fileName.c_str());
          saveState();
          Serial.printf("[XFER] Start: file=%s  chunks=%d  checksum=%s\n",
                        fileName.c_str(), totalChunks,
                        expectedChecksum.c_str());
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("READY");
          mqttClient.endMessage();
        }
      }

      else if (payloadBuffer.indexOf('/') != -1) {
        int sep = payloadBuffer.indexOf('/');
        String command = payloadBuffer.substring(0, sep);
        int value = payloadBuffer.substring(sep + 1).toInt();

        if (command == "hotendtemp") {
          hotendSetpoint = (float)value;
          Serial.printf("[HEAT] Hotend setpoint → %.1f°C\n", hotendSetpoint);
        } else if (command == "bedtemp") {
          bedSetpoint = (float)value;
          Serial.printf("[HEAT] Bed setpoint → %.1f°C\n", bedSetpoint);
        }
      }

      else {
        String command = payloadBuffer;
        command.trim();

        if (command == "home" && sysState == STATE_IDLE) {
          Serial.println("[HOME] Homing all axes...");
          m1_home = false;
          m2_home = false;
          m3_home = false;
          sysState = STATE_HOMING_SEEK;
        } else if (command == "stop") {
          Serial.println("[CMD] Emergency stop.");
          stopAllMotors();
          if (file)
            file.close();
          sysState = STATE_IDLE;
        } else if (command == "end_of_file_transfer") {
          Serial.println("[CMD] End of file transfer command received, but no "
                         "file transfer in progress.");
          MD5Builder md5;
          File f = SD.open(fileName.c_str());
          if (f) {
            md5.begin();
            md5.addStream(f, f.size());
            md5.calculate();
            String computed = md5.toString();
            f.close();
            Serial.printf("[XFER] Expected: %s\n", expectedChecksum.c_str());
            Serial.printf("[XFER] Computed: %s\n", computed.c_str());
            if (computed.equalsIgnoreCase(expectedChecksum)) {
              Serial.println("[XFER] ✅ Verified.");
              mqttClient.beginMessage(TOPIC_ACK);
              mqttClient.print("SUCCESS");
              mqttClient.endMessage();
            } else {
              Serial.println("[XFER] ❌ Checksum MISMATCH.");
              mqttClient.beginMessage(TOPIC_ACK);
              mqttClient.print("FAIL_CHECKSUM");
              mqttClient.endMessage();
            }
          } else {
            Serial.println("[XFER] Could not open file for MD5.");
          }
          fileReceiving = false;
        }
      }
    }

    // -------------------------------------------------------------------------
    // file_transfer/data — payload: chunkID|data
    // -------------------------------------------------------------------------
    else if (topicBuffer == TOPIC_STOP && fileReceiving) {
      int pipe1 = payloadBuffer.indexOf('|');
      int pipe2 = payloadBuffer.indexOf('|', pipe1 + 1);

      if (pipe1 != -1 && pipe2 != -1) {
        int chunkID = payloadBuffer.substring(pipe1 + 1, pipe2).toInt();

        if (chunkID <= lastReceivedChunk) {
          Serial.printf("[XFER] Duplicate chunk %d, ignored.\n", chunkID);
        } else if (chunkID != lastReceivedChunk + 1) {
          Serial.printf("[XFER] Gap! Expected %ld got %d — resend.\n",
                        lastReceivedChunk + 1, chunkID);
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("RESEND|");
          mqttClient.print(lastReceivedChunk + 1);
          mqttClient.endMessage();
        } else {
          appendFile(SD, fileName.c_str(),
                     payloadBuffer.substring(pipe2 + 1).c_str());
          lastReceivedChunk = chunkID;
          saveState();
          Serial.printf("[XFER] Chunk %d / %d written.\n", chunkID,
                        totalChunks);
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("ACK|");
          mqttClient.print(chunkID);
          mqttClient.endMessage();
        }
      }
    }

    // -------------------------------------------------------------------------
    // manual XYZ jogging — separate manual state flow
    // -------------------------------------------------------------------------
    else if (topicBuffer == TOPIC_xyz_move) {
      String cmd = payloadBuffer;
      cmd.trim();
      cmd.toUpperCase();

      char firstChar = cmd.length() > 0 ? cmd.charAt(0) : '\0';
      float dx = 0.0f, dy = 0.0f, dz = 0.0f;
      bool valid = false;

      if (firstChar == 'X' || firstChar == 'Y' || firstChar == 'Z') {
        if (cmd.length() >= 2) {
          char axis = firstChar;
          char sign = cmd.charAt(1);
          if (sign == '+' || sign == '-') {
            float amount = cmd.substring(2).toFloat();
            if (amount == 0.0f)
              amount = 1.0f;
            if (sign == '-')
              amount = -amount;

            if (axis == 'X')
              dx = amount;
            else if (axis == 'Y')
              dy = amount;
            else if (axis == 'Z')
              dz = amount;
            valid = true;
          }
        }
      } else {
        // Legacy/alternate format support (e.g. x+, y-, z+)
        if (cmd == "X+") {
          dx = MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "X-") {
          dx = -MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Y+") {
          dy = MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Y-") {
          dy = -MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Z+") {
          dz = MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Z-") {
          dz = -MANUAL_JOG_MM;
          valid = true;
        }
      }

      if (valid) {
        Serial.printf("[CMD] Received XYZ move command: %s  → delta (dx:%.2f "
                      "dy:%.2f dz:%.2f)\n",
                      cmd.c_str(), dx, dy, dz);
        if (isActivePrintState(sysState)) {
          Serial.println("[CMD] Manual XYZ move requested during active print. "
                         "Pausing print first.");
          queueManualMove(dx, dy, dz);
          pausedPrintState = sysState;
          sysState = STATE_MOTION_PAUSED;
        } else if (sysState == STATE_MOTION_PAUSED ||
                   sysState == STATE_MANUAL_XYZ || sysState == STATE_IDLE) {
          if (sysState != STATE_MANUAL_XYZ) {
            resetManualXYZTarget();
            sysState = STATE_MANUAL_XYZ;
          }
          queueManualMove(dx, dy, dz);
        } else {
          Serial.println("[CMD] Manual XYZ move blocked. Printer busy.");
        }
      } else {
        Serial.printf("[CMD] Unknown XYZ jog payload: %s\n",
                      payloadBuffer.c_str());
      }
    }

    // -------------------------------------------------------------------------
    // motor/start — begin/resume motion
    // -------------------------------------------------------------------------
    else if (topicBuffer == MOTOR_START) {
      if (payloadBuffer == "RESUME" &&
          (sysState == STATE_MOTION_PAUSED || sysState == STATE_MANUAL_XYZ)) {
        if (pausedPrintState != STATE_IDLE) {
          Serial.println("[MOTION] Resuming paused print.");
          sysState = pausedPrintState;
          pausedPrintState = STATE_IDLE;
          manualMovePending = false;
          manualMoveDeltaX = 0.0f;
          manualMoveDeltaY = 0.0f;
          manualMoveDeltaZ = 0.0f;
        } else {
          Serial.println("[MOTION] Resume requested but no paused print "
                         "available. Returning to IDLE.");
          stopAllMotors();
          sysState = STATE_IDLE;
        }
      } else if (sysState == STATE_IDLE) {
        Serial.println("[MOTION] Start.");
        for (int i = 0; i < 4; i++) {
          updateEncoderFromAS5600(i);
          totalRotations[i] = 0;
        }
        sysState = STATE_MOTION_OPEN;
      }
    }
  }

  // Feed scheduler/WiFi task watchdog in high-load loops.
  delay(1);

  // ===========================================================================
  // STATE MACHINE — one small step per loop() iteration, never blocks
  // ===========================================================================
  switch (sysState) {

  case STATE_IDLE:
    break;

  // --- MOTION: paused safely, waiting for manual input or resume ---
  case STATE_MOTION_PAUSED: {
    stopAllMotors();
    if (manualMovePending) {
      resetManualXYZTarget();
      applyPendingManualMove();
      sysState = STATE_MANUAL_XYZ;
    }
    break;
  }

  // --- MANUAL XYZ: dedicated repositioning state ---
  case STATE_MANUAL_XYZ: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    for (int i = 0; i < 4; i++)
      updateEncoderFromAS5600(i);
    if (manualMovePending)
      applyPendingManualMove();

    bool allTargetReached = true;
    for (int i = 0; i < 4; i++) {
      if (abs(manualTarget[i] - encoderCount[i]) > MOTION_TOLERANCE_COUNTS) {
        allTargetReached = false;
        break;
      }
    }

    if (allTargetReached) {
      stopAllMotors();
    } else {
      for (int i = 0; i < 4; i++) {
        driveMotor(i, calculatePID(i, manualTarget[i], dt));
      }
    }
    break;
  }

  // --- HOMING: drive toward limit switches ---
  case STATE_HOMING_SEEK: {
    if (!m1_home) {
      analogWrite(motorPinA1, 0);
      analogWrite(motorPinA2, 255);
      if (digitalRead(LIMIT1_PIN) == LOW) {
        analogWrite(motorPinA2, 0);
        m1_home = true;
        encoderCount[0] = 0;
        rawAccumulator[0] = 0;
        Serial.println("[HOME] Axis 0 limit reached.");
      }
    }
    if (!m2_home) {
      analogWrite(motorPinB1, 0);
      analogWrite(motorPinB2, 255);
      if (digitalRead(LIMIT2_PIN) == LOW) {
        analogWrite(motorPinB2, 0);
        m2_home = true;
        encoderCount[1] = 0;
        rawAccumulator[1] = 0;
        Serial.println("[HOME] Axis 1 limit reached.");
      }
    }
    if (!m3_home) {
      analogWrite(motorPinc1, 0);
      analogWrite(motorPinc2, 255);
      if (digitalRead(LIMIT3_PIN) == LOW) {
        analogWrite(motorPinc2, 0);
        m3_home = true;
        encoderCount[2] = 0;
        rawAccumulator[2] = 0;
        Serial.println("[HOME] Axis 2 limit reached.");
      }
    }
    if (m1_home && m2_home && m3_home) {
      sysState = STATE_HOMING_ZERO;
    }
    break;
  }

  // --- HOMING: zero all encoders ---
  case STATE_HOMING_ZERO: {
    for (int i = 0; i < 4; i++) {
      if (as5600Ready && selectI2CChannel(motorToChannel[i])) {
        delay(5);
        lastAngle[i] = readRawAngle();
      } else {
        lastAngle[i] = 0;
      }
      encoderCount[i] = 0;
      rawAccumulator[i] = 0;
      totalRotations[i] = 0;
      Serial.printf("[HOME] Motor %d zeroed. BaseAngle=%ld\n", i, lastAngle[i]);
      delay(1);
    }
    disableAllI2CChannels();

    setpointHome[0] = -13400;
    setpointHome[1] = -13400;
    setpointHome[2] = -13400;
    setpointHome[3] = 100;

    lastTimePID = millis();
    for (int i = 0; i < 4; i++) {
      integral[i] = 0.0f;
      prevError[i] = 0;
    }
    Serial.println("[HOME] Moving to standoff (625.28)...");
    sysState = STATE_HOMING_STANDOFF;
    break;
  }

    // --- HOMING: PID to standoff position ---
  // --- HOMING: Sequential PID standoff to final position ---
  case STATE_HOMING_STANDOFF: {

    // ============================================================
    // SEQUENTIAL HOMING CONSTANTS
    // ============================================================
    const long COUNTS_PER_ROTATION = 4096;
    const long TOTAL_STANDOFF_COUNTS = 13400;

    const long FULL_ROTATIONS =
        TOTAL_STANDOFF_COUNTS / COUNTS_PER_ROTATION; // 3

    const long REMAINING_COUNTS =
        TOTAL_STANDOFF_COUNTS % COUNTS_PER_ROTATION; // 1112

    // ============================================================
    // PERSISTENT SEQUENTIAL STATE
    // ============================================================
    static bool seqInitialized = false;
    static bool motor3Complete = false;
    static bool sequenceComplete = false;

    // 0 = Motor A
    // 1 = Motor B
    // 2 = Motor C
    static int activeMotor = 0;

    // 0,1,2 = full-rotation cycles
    // 3     = final partial cycle
    static int rotationCycle = 0;

    static long sequentialTarget = 0;

    // ============================================================
    // PID TIMING
    // ============================================================
    unsigned long currentTime = millis();

    float dt = (currentTime - lastTimePID) / 1000.0f;

    if (dt <= 0.0f)
      dt = 0.001f;

    lastTimePID = currentTime;

    // ============================================================
    // UPDATE ALL ENCODERS
    // ============================================================
    for (int i = 0; i < 4; i++) {
      updateEncoderFromAS5600(i);
    }

    // ============================================================
    // INITIALIZE SEQUENTIAL HOMING ON FIRST ENTRY
    // ============================================================
    if (!seqInitialized) {

      seqInitialized = true;
      motor3Complete = false;
      sequenceComplete = false;

      activeMotor = 0;
      rotationCycle = 0;
      sequentialTarget = 0;

      // Reset PID state for all motors
      for (int i = 0; i < 4; i++) {
        integral[i] = 0.0f;
        prevError[i] = 0;
      }

      Serial.println();
      Serial.println("========================================");
      Serial.println("[HOME-SEQ] SEQUENTIAL STANDOFF START");
      Serial.printf("[HOME-SEQ] Total counts      : %ld\n",
                    TOTAL_STANDOFF_COUNTS);
      Serial.printf("[HOME-SEQ] Counts / rotation : %ld\n",
                    COUNTS_PER_ROTATION);
      Serial.printf("[HOME-SEQ] Full rotations    : %ld\n", FULL_ROTATIONS);
      Serial.printf("[HOME-SEQ] Remaining counts  : %ld\n", REMAINING_COUNTS);
      Serial.println("========================================");

      // ==========================================================
      // MOTOR 3
      // ==========================================================
      // Preserve the existing Motor 3 homing target.
      // Motor 3 is NOT part of the A/B/C sequential rotation test.
      setpointHome[3] = 100;

      integral[3] = 0.0f;
      prevError[3] = 0;

      Serial.printf("[HOME-SEQ] Motor 3 START | Target=%ld\n", setpointHome[3]);
    }

    // ============================================================
    // MOTOR 3 EXISTING STANDOFF
    // ============================================================
    //
    // Motor 3 must still reach its existing target of 100.
    // Once it reaches 100, it is stopped and never touched again
    // during the A/B/C sequential diagnostic.
    //
    if (!motor3Complete) {

      long error3 = setpointHome[3] - encoderCount[3];

      if (abs(error3) <= MOTION_TOLERANCE_COUNTS) {

        // Stop Motor 3
        driveMotor(3, 0.0f);

        motor3Complete = true;

        Serial.printf(
            "[HOME-SEQ] Motor 3 STOP | Final=%ld | Target=%ld | Error=%ld\n",
            encoderCount[3], setpointHome[3], error3);

        // --------------------------------------------------------
        // Start Motor A
        // --------------------------------------------------------
        activeMotor = 0;
        rotationCycle = 0;

        sequentialTarget = -COUNTS_PER_ROTATION;

        integral[0] = 0.0f;
        prevError[0] = 0;

        Serial.printf(
            "[HOME-SEQ] Cycle 1/%ld | Motor 0 (A) START | Target=%ld\n",
            FULL_ROTATIONS, sequentialTarget);

      } else {

        // Existing Motor 3 PID behavior
        float output3 = calculatePID(3, setpointHome[3], dt);

        driveMotor(3, output3);
      }

      break;
    }

    // ============================================================
    // A/B/C SEQUENTIAL MOVEMENT
    // ============================================================
    //
    // ONLY ONE OF MOTOR 0, 1, 2 IS ALLOWED TO MOVE HERE.
    //
    // Sequence:
    //
    // A -> -4096  STOP
    // B -> -4096  STOP
    // C -> -4096  STOP
    //
    // A -> -8192  STOP
    // B -> -8192  STOP
    // C -> -8192  STOP
    //
    // A -> -12288 STOP
    // B -> -12288 STOP
    // C -> -12288 STOP
    //
    // A -> -13400 STOP
    // B -> -13400 STOP
    // C -> -13400 STOP
    //
    // ============================================================

    long currentError = sequentialTarget - encoderCount[activeMotor];

    // ============================================================
    // ACTIVE MOTOR REACHED TARGET
    // ============================================================
    if (abs(currentError) <= MOTION_TOLERANCE_COUNTS) {

      // Stop ONLY the active motor.
      driveMotor(activeMotor, 0.0f);

      Serial.printf(
          "[HOME-SEQ] Motor %d STOP | Final=%ld | Target=%ld | Error=%ld\n",
          activeMotor, encoderCount[activeMotor], sequentialTarget,
          currentError);

      // ----------------------------------------------------------
      // MOVE TO NEXT MOTOR
      // ----------------------------------------------------------
      activeMotor++;

      // ==========================================================
      // A -> B -> C COMPLETED
      // ==========================================================
      if (activeMotor >= 3) {

        activeMotor = 0;
        rotationCycle++;

        // ========================================================
        // THREE FULL ROTATIONS COMPLETED
        // ========================================================
        if (rotationCycle >= FULL_ROTATIONS) {

          // ------------------------------------------------------
          // FINAL PARTIAL MOVEMENT
          // -12288 -> -13400
          // ------------------------------------------------------

          if (REMAINING_COUNTS > 0) {

            sequentialTarget = -TOTAL_STANDOFF_COUNTS;

            integral[0] = 0.0f;
            prevError[0] = 0;

            Serial.println();
            Serial.println("[HOME-SEQ] FINAL PARTIAL CYCLE");

            Serial.printf(
                "[HOME-SEQ] Motor 0 (A) START | Target=%ld | Remaining=%ld\n",
                sequentialTarget, REMAINING_COUNTS);

          } else {

            // No remainder.
            sequenceComplete = true;
          }

        } else {

          // ------------------------------------------------------
          // NEXT FULL ROTATION
          // ------------------------------------------------------

          sequentialTarget = -(COUNTS_PER_ROTATION * (rotationCycle + 1));

          integral[0] = 0.0f;
          prevError[0] = 0;

          Serial.printf(
              "[HOME-SEQ] Cycle %d/%ld | Motor 0 (A) START | Target=%ld\n",
              rotationCycle + 1, FULL_ROTATIONS, sequentialTarget);
        }

      } else {

        // ========================================================
        // NEXT MOTOR IN SAME ROTATION
        // ========================================================

        if (rotationCycle < FULL_ROTATIONS) {

          // Same target for B/C in this cycle.
          sequentialTarget = -(COUNTS_PER_ROTATION * (rotationCycle + 1));

        } else {

          // Final partial cycle.
          sequentialTarget = -TOTAL_STANDOFF_COUNTS;
        }

        integral[activeMotor] = 0.0f;
        prevError[activeMotor] = 0;

        Serial.printf("[HOME-SEQ] Cycle %d/%ld | Motor %d START | Target=%ld\n",
                      rotationCycle + 1, FULL_ROTATIONS, activeMotor,
                      sequentialTarget);
      }
    }

    // ============================================================
    // SEQUENCE COMPLETE CHECK
    // ============================================================
    //
    // This occurs after Motor C completes the final -13400 target.
    //
    if (rotationCycle >= FULL_ROTATIONS && activeMotor == 0 &&
        sequentialTarget == -TOTAL_STANDOFF_COUNTS) {

      // If Motor C was the last motor to complete, the code above
      // advances activeMotor back to 0. Verify all three motors.
      bool allABCComplete = true;

      for (int i = 0; i < 3; i++) {

        long finalError = setpointHome[i] - encoderCount[i];

        if (abs(finalError) > MOTION_TOLERANCE_COUNTS) {
          allABCComplete = false;
        }
      }

      if (allABCComplete) {
        sequenceComplete = true;
      }
    }

    // ============================================================
    // FINAL HOMING COMPLETION
    // ============================================================
    if (sequenceComplete) {

      // Stop all motors once everything has reached its target.
      stopAllMotors();

      Serial.println();
      Serial.println("========================================");
      Serial.println("[HOME-SEQ] SEQUENTIAL STANDOFF COMPLETE");
      Serial.printf("[HOME-SEQ] Motor 0 final = %ld\n", encoderCount[0]);
      Serial.printf("[HOME-SEQ] Motor 1 final = %ld\n", encoderCount[1]);
      Serial.printf("[HOME-SEQ] Motor 2 final = %ld\n", encoderCount[2]);
      Serial.printf("[HOME-SEQ] Motor 3 final = %ld\n", encoderCount[3]);
      Serial.println("========================================");

      checkAllEncoders();

      Serial.println("[HOME] Complete. Axes at standoff (100).");

      currentX = HOME_X;
      currentY = HOME_Y;
      currentZ = HOME_Z;

      // Reset the sequence state so that if homing is started again,
      // it initializes cleanly.
      seqInitialized = false;
      motor3Complete = false;
      sequenceComplete = false;
      activeMotor = 0;
      rotationCycle = 0;
      sequentialTarget = 0;

      sysState = STATE_IDLE;

      break;
    }

    // ============================================================
    // RUN PID FOR ONLY THE CURRENT A/B/C MOTOR
    // ============================================================
    //
    // IMPORTANT:
    // No PID command is sent to the other A/B/C motors.
    //
    // Therefore:
    //
    // Motor 0 MOVING -> Motors 1 and 2 STOPPED
    // Motor 1 MOVING -> Motors 0 and 2 STOPPED
    // Motor 2 MOVING -> Motors 0 and 1 STOPPED
    //
    if (activeMotor >= 0 && activeMotor <= 2) {

      float output = calculatePID(activeMotor, sequentialTarget, dt);

      driveMotor(activeMotor, output);
    }

    break;
  }

  // --- MOTION: open file ---
  case STATE_MOTION_OPEN: {
    listDir(SD, "/", 0);
    file = SD.open(fileName.c_str());
    if (!file) {
      Serial.printf("[MOTION] Failed to open: %s\n", fileName.c_str());
      sysState = STATE_IDLE;
      break;
    }
    totalLines = countLinesInFile(SD, fileName.c_str());
    myconut = 0;
    dwellStart = millis();
    sysState = STATE_MOTION_DWELL;
    break;
  }

  // --- MOTION: inter-line dwell (replaces blocking delay) ---
  case STATE_MOTION_DWELL: {
    if (millis() - dwellStart >= 1000) {
      if (myconut >= totalLines) {
        sysState = STATE_MOTION_DONE;
      } else {
        sysState = STATE_MOTION_READ_LINE;
      }
    }
    break;
  }

  // --- MOTION: read & parse next line ---
  case STATE_MOTION_READ_LINE: {
    Serial.printf("[MOTION] Line %d / %d\n", myconut + 1, totalLines);

    String motionLine = readNextLine();
    motionLine.trim();
    Serial.printf("[MOTION] Raw line: %s\n", motionLine.c_str());

    float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f, targetE = 0.0f;
    if (!parseCartesianLine(motionLine, targetX, targetY, targetZ, targetE)) {
      Serial.println(
          "[MOTION] Invalid line format, expected X Y Z E floats. Skipping.");
      myconut++;
      dwellStart = millis();
      sysState = STATE_MOTION_DWELL;
      break;
    }

    long targetA = 0, targetB = 0, targetC = 0;
    if (!convertXYZToEncoderTargets(targetX, targetY, targetZ, targetA, targetB,
                                    targetC)) {
      Serial.println("[MOTION] IK/encoder conversion failed. Stopping motion.");
      stopAllMotors();
      if (file)
        file.close();
      sysState = STATE_IDLE;
      break;
    }

    setpoint[0] = targetA;
    setpoint[1] = targetB;
    setpoint[2] = targetC;
    setpoint[3] = (long)lroundf(targetE * STEPS_PER_MM_E);

    currentX = targetX;
    currentY = targetY;
    currentZ = targetZ;

    Serial.printf("[MOTION] XYZE: %.3f %.3f %.3f %.3f\n", targetX, targetY,
                  targetZ, targetE);
    Serial.printf("[MOTION] Setpoints(counts) -> A:%ld  B:%ld  C:%ld  E:%ld\n",
                  setpoint[0], setpoint[1], setpoint[2], setpoint[3]);

    lastTimePID = millis();
    for (int i = 0; i < 4; i++) {
      integral[i] = 0.0f;
      prevError[i] = 0;
    }
    sysState = STATE_MOTION_PID;
    break;
  }

  // --- MOTION: PID drive toward current setpoint ---
  case STATE_MOTION_PID: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    for (int i = 0; i < 4; i++)
      updateEncoderFromAS5600(i);

    if (abs(setpoint[0] - encoderCount[0]) <= MOTION_TOLERANCE_COUNTS &&
        abs(setpoint[1] - encoderCount[1]) <= MOTION_TOLERANCE_COUNTS &&
        abs(setpoint[2] - encoderCount[2]) <= MOTION_TOLERANCE_COUNTS &&
        abs(setpoint[3] - encoderCount[3]) <= MOTION_TOLERANCE_COUNTS) {
      Serial.println("[MOTION] Target reached ✅");
      myconut++;
      dwellStart = millis();
      sysState = STATE_MOTION_DWELL;
    } else {
      for (int i = 0; i < 4; i++)
        driveMotor(i, calculatePID(i, setpoint[i], dt));
    }
    break;
  }

  // --- MOTION: cleanup ---
  case STATE_MOTION_DONE: {
    stopAllMotors();
    file.close();
    Serial.println("[MOTION] Complete.");
    checkAllEncoders();
    sysState = STATE_IDLE;
    break;
  }
  }
}