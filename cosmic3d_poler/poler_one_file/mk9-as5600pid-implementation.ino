// mk9-as5600-shard.cpp
// ESP32-S3 | Cosmic3D MK9 | Cosmic Polar 400 Edition (R / Theta / Z)

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <ArduinoMqttClient.h>
#include <ESP32Servo.h>
#include <MD5Builder.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>

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
#define GEAR_RATIO 1

// --- Cosmic Polar 400 Mechanical Configuration ---
const float R_MOTOR_PULLEY_DIAMETER_MM = 19.0f;
const float R_BED_DIAMETER_MM = 401.0f;
const float R_AS5600_COUNTS_PER_MOTOR_REV = 4096.0f;

const float RADIAL_MIN_MM = 0.0f;
const float RADIAL_MAX_RADIUS_MM = 200.0f;

const int SERVO_MIN_ANGLE_DEG = 0;
const int SERVO_MAX_ANGLE_DEG = 180;

const float Z_COUNTS_PER_MM = 100.0f;
const float INTERPOLATION_SEGMENT_MM = 2.0f;

const float HOME_X = 0.0f;
const float HOME_Y = 0.0f;
const float HOME_Z = 0.0f;
const float STEPS_PER_MM_E = 100.0f;
const long MOTION_TOLERANCE_COUNTS = 5;

const float MANUAL_JOG_MM = 10.0f;

// --- Pin Allocations ---
#define THETA_SERVO_PIN 18
#define THETA_HOME_SENSOR_PIN 46 // Bed optical/hall home reference sensor
#define Z_LIMIT_PIN 43           // Vertical Z limit switch

Servo servoTheta;
int currentServoAngle = SERVO_MIN_ANGLE_DEG;

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
// Motor 0: R (Bed Rotation) DC Motor
// Motor 1: Z (Vertical Height) DC Motor
// Motor 2: Reserved for MG945 PWM Servo (no DC H-Bridge driver)
// Motor 3: E (Extruder) DC/Stepper Driver
#define motorPinA1 14 // R Motor Pin 1
#define motorPinA2 7  // R Motor Pin 2
#define motorPinB1 15 // Z Motor Pin 1
#define motorPinB2 16 // Z Motor Pin 2
#define motorPinc1 5  // E Motor Pin 1
#define motorPinc2 6  // E Motor Pin 2
#define motorPind1 4  // Auxiliary Pin 1
#define motorPind2 3  // Auxiliary Pin 2

// --- Encoder & Motion State ---
// Index 0: R (Bed), Index 1: Z (Height), Index 2: Theta (Servo), Index 3: E
// (Extruder)
volatile long encoderCount[4] = {0, 0, 0, 0};
volatile int rotation[4] = {0, 0, 0, 0};
volatile long setpoint[4] = {0, 0, 0, 0};
long lastAngle[4] = {0, 0, 0, 0};
long totalRotations[4] = {0, 0, 0, 0};
static long rawAccumulator[4] = {0, 0, 0, 0};

// --- PID Tuning Parameters ---
// Motor 0 (R Bed): Closed-loop DC PID
// Motor 1 (Z Axis): Closed-loop DC PID
// Motor 2 (Theta Servo): Direct PWM control (no PID)
// Motor 3 (Extruder E): Closed-loop DC/Stepper PID
float Kp[4] = {2.0f, 2.5f, 0.0f, 1.5f};
float Ki[4] = {0.01f, 0.01f, 0.0f, 0.01f};
float Kd[4] = {0.5f, 0.6f, 0.0f, 0.3f};
long prevError[4] = {0, 0, 0, 0};
float integral[4] = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastTimePID = 0;

// --- SD File Handle ---
File file;
int totalLines = 0;
int myconut = 0;

// AS5600 Multiplexer Channel Mapping:
// Channel 0: R Motor Encoder
// Channel 1: Z Motor Encoder
// Channel 2: Unused / Spare
// Channel 3: E Motor Encoder
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
bool r_home = false, z_home = false, theta_home = false;
long setpointHome[4] = {0, 1000, 0, 0};
unsigned long dwellStart = 0;

bool manualMovePending = false;
float manualMoveDeltaX = 0.0f;
float manualMoveDeltaY = 0.0f;
float manualMoveDeltaZ = 0.0f;
long manualTarget[4] = {0, 0, 0, 0};
int manualServoTarget = SERVO_MIN_ANGLE_DEG;
SystemState pausedPrintState = STATE_IDLE;

// --- Cartesian Segmenter Struct ---
struct CartesianSegment {
  float x;
  float y;
  float z;
  float e;
};

#define MAX_INTERPOLATION_SEGMENTS 32
CartesianSegment segmentBuffer[MAX_INTERPOLATION_SEGMENTS];
int currentSegmentIndex = 0;
int totalSegmentsCount = 0;

bool isActivePrintState(int state) {
  return state == STATE_MOTION_OPEN || state == STATE_MOTION_READ_LINE ||
         state == STATE_MOTION_PID || state == STATE_MOTION_DWELL;
}

bool isPrinterActive() {
  return isActivePrintState(sysState);
}


// =============================================================================
// POLAR CONVERSION KINEMATICS & HELPERS
// =============================================================================

long bedAngleToMotorCounts(float bedAngleDeg) {
  double motorRevs =
      ((double)bedAngleDeg / 360.0) *
      ((double)R_BED_DIAMETER_MM / (double)R_MOTOR_PULLEY_DIAMETER_MM);
  return (long)lround(motorRevs * (double)R_AS5600_COUNTS_PER_MOTOR_REV);
}

int radialMmToServoAngle(float radialMm) {
  radialMm = constrain(radialMm, RADIAL_MIN_MM, RADIAL_MAX_RADIUS_MM);
  float scale = (float)(SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG) /
                (RADIAL_MAX_RADIUS_MM - RADIAL_MIN_MM);
  int angle =
      SERVO_MIN_ANGLE_DEG + (int)lroundf((radialMm - RADIAL_MIN_MM) * scale);
  return constrain(angle, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
}

long zMmToEncoderCounts(float zMm) {
  return (long)lroundf(zMm * Z_COUNTS_PER_MM);
}

void setThetaServoAngle(int angle) {
  currentServoAngle =
      constrain(angle, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
  servoTheta.write(currentServoAngle);
}

static float lastBedAngleDeg = 0.0f;
static float accumulatedBedAngleDeg = 0.0f;

bool calculatePolarIK(float nozzleX, float nozzleY, float nozzleZ,
                      long &targetR_counts, int &targetTheta_servoAngle,
                      long &targetZ_counts) {

  float radialR = sqrtf(nozzleX * nozzleX + nozzleY * nozzleY);
  if (radialR > RADIAL_MAX_RADIUS_MM) {
    Serial.printf("[KINEMATICS] Radial distance %.2f mm exceeds maximum radius "
                  "%.2f mm.\n",
                  radialR, RADIAL_MAX_RADIUS_MM);
    return false;
  }

  float rawBedAngleDeg = atan2f(nozzleY, nozzleX) * 180.0f / PI;
  if (rawBedAngleDeg < 0.0f) {
    rawBedAngleDeg += 360.0f;
  }

  // Continuous multi-turn angle accumulation via shortest angular step
  float deltaAngle = rawBedAngleDeg - lastBedAngleDeg;
  while (deltaAngle > 180.0f)
    deltaAngle -= 360.0f;
  while (deltaAngle < -180.0f)
    deltaAngle += 360.0f;

  accumulatedBedAngleDeg += deltaAngle;
  lastBedAngleDeg = rawBedAngleDeg;

  targetR_counts = bedAngleToMotorCounts(accumulatedBedAngleDeg);
  targetTheta_servoAngle = radialMmToServoAngle(radialR);
  targetZ_counts = zMmToEncoderCounts(nozzleZ);

  return isfinite(radialR) && isfinite(rawBedAngleDeg) && isfinite(nozzleZ);
}

int generateCartesianSegments(float startX, float startY, float startZ,
                              float startE, float endX, float endY, float endZ,
                              float endE, CartesianSegment segments[],
                              int maxSegments) {
  float dx = endX - startX;
  float dy = endY - startY;
  float dz = endZ - startZ;
  float de = endE - startE;

  float distance = sqrtf(dx * dx + dy * dy + dz * dz);
  int numSegments = (int)ceilf(distance / INTERPOLATION_SEGMENT_MM);
  if (numSegments < 1)
    numSegments = 1;
  if (numSegments > maxSegments)
    numSegments = maxSegments;

  for (int i = 1; i <= numSegments; i++) {
    float t = (float)i / (float)numSegments;
    segments[i - 1].x = startX + t * dx;
    segments[i - 1].y = startY + t * dy;
    segments[i - 1].z = startZ + t * dz;
    segments[i - 1].e = startE + t * de;
  }
  return numSegments;
}

void resetManualXYZTarget() {
  manualTarget[0] = encoderCount[0];   // R DC motor
  manualTarget[1] = encoderCount[1];   // Z DC motor
  manualTarget[2] = currentServoAngle; // Theta Servo
  manualTarget[3] = encoderCount[3];   // E motor
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

  long targetR_counts = 0, targetZ_counts = 0;
  int targetTheta_servoAngle = SERVO_MIN_ANGLE_DEG;

  if (calculatePolarIK(nextX, nextY, nextZ, targetR_counts,
                       targetTheta_servoAngle, targetZ_counts)) {
    currentX = nextX;
    currentY = nextY;
    currentZ = nextZ;
    manualTarget[0] = targetR_counts;
    manualTarget[1] = targetZ_counts;
    manualServoTarget = targetTheta_servoAngle;

    Serial.printf("[MANUAL] XYZ: %.2f %.2f %.2f → Targets (R-counts:%ld "
                  "Theta-deg:%d Z-counts:%ld)\n",
                  currentX, currentY, currentZ, targetR_counts,
                  targetTheta_servoAngle, targetZ_counts);

    lastTimePID = millis();
    for (int i = 0; i < 4; i++) {
      integral[i] = 0.0f;
      prevError[i] = 0;
    }
  } else {
    Serial.println(
        "[MANUAL] Polar kinematics calculation failed, move discarded.");
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
    e = val5;
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
  if (motorIndex == 2)
    return; // Motor 2 is Theta MG945 Servo (no AS5600)
  if (!as5600Ready)
    return;
  if (!selectI2CChannel(motorToChannel[motorIndex]))
    return;
  delayMicroseconds(100);

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
    Serial.printf("[ENC] Axis %d: Spurious jump (%ld), ignored.\n", motorIndex,
                  diff);
    return;
  }

  rawAccumulator[motorIndex] += diff;
  long motorCounts = rawAccumulator[motorIndex] / GEAR_RATIO;

  if (motorCounts != 0) {
    long prevCount = encoderCount[motorIndex];
    encoderCount[motorIndex] += motorCounts;
    rawAccumulator[motorIndex] -= motorCounts * GEAR_RATIO;
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
  Serial.println("[AS5600] Initializing TCA9548A & AS5600 Encoders...");

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

  for (int i = 0; i < 4; i++) {
    if (i == 2)
      continue; // Skip Motor 2 (Theta MG945 Servo)
    if (!selectI2CChannel(motorToChannel[i])) {
      as5600Ready = false;
      Serial.printf("[AS5600] Axis %d (Ch %d) unavailable.\n", i,
                    motorToChannel[i]);
      continue;
    }
    delay(5);

    uint16_t angle = readRawAngle();
    uint8_t status = readStatus();
    uint16_t magnitude = readMagnitude();
    uint8_t agc = readAGC();
    bool magOK = isMagnetDetected();

    Serial.printf("[AS5600] Axis %d (Ch %d): Angle=%u Status=0x%02X Mag=%u "
                  "AGC=%u Magnet=%s\n",
                  i, motorToChannel[i], angle, status, magnitude, agc,
                  magOK ? "✓ OK" : "✗ MISSING!");

    lastAngle[i] = angle;
    encoderCount[i] = 0;
    rawAccumulator[i] = 0;
    totalRotations[i] = 0;
  }

  disableAllI2CChannels();
  Serial.println(as5600Ready ? "[AS5600] Init complete."
                             : "[AS5600] Init degraded.");
}

// =============================================================================
// ENCODER DIAGNOSTICS
// =============================================================================
void checkAllEncoders() {
  if (!as5600Ready) {
    Serial.println("[DIAG] AS5600 unavailable (TCA9548A/encoder offline).");
    return;
  }

  Serial.println("\n[DIAG] ===== Cosmic Polar 400 Diagnostics =====");
  for (int i = 0; i < 4; i++) {
    if (i == 2) {
      Serial.printf("[DIAG] Theta Axis (Servo) | Angle: %d deg | Pin: %d\n",
                    currentServoAngle, THETA_SERVO_PIN);
      continue;
    }
    if (!selectI2CChannel(motorToChannel[i])) {
      Serial.printf("[DIAG] Axis %d | Channel select failed\n", i);
      continue;
    }
    delayMicroseconds(500);

    uint16_t angle = readRawAngle();
    uint8_t status = readStatus();
    uint16_t magnitude = readMagnitude();
    uint8_t agc = readAGC();
    bool magOK = isMagnetDetected();

    Serial.printf("[DIAG] Axis %d | Angle=%u | Count=%ld | Setpoint=%ld | "
                  "Mag=%u | AGC=%u | %s\n",
                  i, angle, encoderCount[i], setpoint[i], magnitude, agc,
                  magOK ? "✓ OK" : "✗ MISSING!");
  }
  disableAllI2CChannels();
  Serial.println("[DIAG] ========================================\n");
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
// HEATER CONTROL
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
      Serial.printf("[HEAT] Hotend %.1f°C → ON 🔥 (sp %.1f°C)\n", hotendTemp,
                    hotendSetpoint);
    } else if (hotendTemp > (hotendSetpoint + HYSTERESIS) && hotendHeaterOn) {
      digitalWrite(HEATER_PIN, LOW);
      hotendHeaterOn = false;
      Serial.printf("[HEAT] Hotend %.1f°C → OFF (sp %.1f°C)\n", hotendTemp,
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
      Serial.printf("[HEAT] Bed %.1f°C → ON 🛏 (sp %.1f°C)\n", bedTemp,
                    bedSetpoint);
    } else if (bedTemp > (bedSetpoint + BED_HYSTERESIS) && bedHeaterOn) {
      digitalWrite(BED_HEATER_PIN, LOW);
      bedHeaterOn = false;
      Serial.printf("[HEAT] Bed %.1f°C → OFF (sp %.1f°C)\n", bedTemp,
                    bedSetpoint);
    }
  }
}

// =============================================================================
// PID HELPER
// =============================================================================
float calculatePID(int i, long targetPosition, float dt) {
  if (i == 2)
    return 0.0f; // Motor 2 is MG945 Servo (no DC PID)

  long error = targetPosition - encoderCount[i];
  float P = Kp[i] * (float)error;

  integral[i] += (float)error * dt;
  integral[i] = constrain(integral[i], -1000.0f, 1000.0f);
  float I = Ki[i] * integral[i];

  float D = Kd[i] * ((float)(error - prevError[i]) / dt);

  prevError[i] = error;
  float output = P + I + D;
  return output;
}

// =============================================================================
// MOTOR DRIVE
// =============================================================================
#define PWM_MIN_FLOOR 50
#define PWM_MIN_FLOOR_M4 190

void driveMotor(int i, float output) {
  if (i == 2) {
    // Theta MG945 Servo position control handled separately via
    // setThetaServoAngle()
    return;
  }

  int pwm = (int)abs(output);
  int floor = (i == 3) ? PWM_MIN_FLOOR_M4 : PWM_MIN_FLOOR;

  if (pwm > 0 && pwm < floor) {
    pwm = floor;
  }
  if (pwm > 255)
    pwm = 255;

  int pin1 = 0, pin2 = 0;
  if (i == 0) {
    pin1 = motorPinA1;
    pin2 = motorPinA2;
  } else if (i == 1) {
    pin1 = motorPinB1;
    pin2 = motorPinB2;
  } else if (i == 3) {
    pin1 = motorPinc1;
    pin2 = motorPinc2;
  }

  if (output > 0) {
    analogWrite(pin1, pwm);
    analogWrite(pin2, 0);
  } else if (output < 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, pwm);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

void stopAllMotors() {
  analogWrite(motorPinA1, 0);
  analogWrite(motorPinA2, 0);
  analogWrite(motorPinB1, 0);
  analogWrite(motorPinB2, 0);
  analogWrite(motorPinc1, 0);
  analogWrite(motorPinc2, 0);
  analogWrite(motorPind1, 0);
  analogWrite(motorPind2, 0);
  Serial.println("[MOTOR] All DC motors stopped.");
}

// =============================================================================
// SETUP & LOOP INTEGRATION
// =============================================================================
void mk9Setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== Cosmic Polar 400 Firmware Controller Starting ===");

  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
  pinMode(motorPinc1, OUTPUT);
  pinMode(motorPinc2, OUTPUT);
  pinMode(motorPind1, OUTPUT);
  pinMode(motorPind2, OUTPUT);
  stopAllMotors();

  pinMode(THETA_HOME_SENSOR_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  pinMode(BED_HEATER_PIN, OUTPUT);
  digitalWrite(BED_HEATER_PIN, LOW);

  // Initialize MG945 Servo for Theta Axis
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoTheta.setPeriodHertz(50);
  servoTheta.attach(THETA_SERVO_PIN, 500, 2400);
  setThetaServoAngle(SERVO_MIN_ANGLE_DEG);
  Serial.printf("[SERVO] MG945 Theta Servo attached to GPIO %d (Min: %d deg)\n",
                THETA_SERVO_PIN, SERVO_MIN_ANGLE_DEG);

  if (!SD.begin(cs)) {
    Serial.println("[SD] SD Card Mount Failed.");
  } else {
    Serial.println("[SD] SD Card initialized.");
  }

  prefs.begin("mk9_state", false);
  loadState();

  initAS5600();

  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASSWORD);
  if (WiFi.status() == WL_CONNECTED) {
    if (mqttClient.connect(MQTT_BROKER, 1883)) {
      Serial.println("[MQTT] Initial connection successful ✅");
      mqttClient.subscribe(TOPIC_START_file_start_stop);
      mqttClient.subscribe(TOPIC_STOP);
      mqttClient.subscribe(TOPIC_xyz_move);
      mqttClient.subscribe(MOTOR_START);
      mqttClient.subscribe(MOTOR_STOP);
    }
  }

  Serial.println("[SETUP] Cosmic Polar 400 Core Ready.");
  mk9CoreReady = true;
}

void mk9Loop() {
  if (!mk9CoreReady) {
    delay(10);
    return;
  }

  // --- MQTT Maintenance ---
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

  // --- Heater Control ---
  static unsigned long lastHeaterPoll = 0;
  if (millis() - lastHeaterPoll >= HEATER_POLL_MS) {
    lastHeaterPoll = millis();
    runHeaterControl();
  }

  // --- Encoder Diagnostics Poll ---
  static unsigned long lastDiagnostic = 0;
  if (millis() - lastDiagnostic > 30000) {
    lastDiagnostic = millis();
    checkAllEncoders();
  }

  // --- Process Interactive Serial Commands (for Axis Testing) ---
  if (Serial.available() > 0) {
    String serCmd = Serial.readStringUntil('\n');
    serCmd.trim();
    serCmd.toUpperCase();

    if (serCmd == "HOME") {
      Serial.println("[CMD] Homing Cosmic Polar 400...");
      r_home = false;
      z_home = false;
      theta_home = false;
      sysState = STATE_HOMING_SEEK;
    } else if (serCmd.startsWith("R ")) {
      float bedDeg = serCmd.substring(2).toFloat();
      long rCounts = bedAngleToMotorCounts(bedDeg);
      setpoint[0] = rCounts;
      Serial.printf("[TEST] R Bed Target: %.2f deg → %ld counts\n", bedDeg,
                    rCounts);
      sysState = STATE_MOTION_PID;
    } else if (serCmd.startsWith("THETA ")) {
      float radialMm = serCmd.substring(6).toFloat();
      int sAngle = radialMmToServoAngle(radialMm);
      setThetaServoAngle(sAngle);
      Serial.printf("[TEST] Theta Target: %.2f mm → Servo %d deg\n", radialMm,
                    sAngle);
    } else if (serCmd.startsWith("Z ")) {
      float zMm = serCmd.substring(2).toFloat();
      long zCounts = zMmToEncoderCounts(zMm);
      setpoint[1] = zCounts;
      Serial.printf("[TEST] Z Target: %.2f mm → %ld counts\n", zMm, zCounts);
      sysState = STATE_MOTION_PID;
    } else if (serCmd == "POS?") {
      checkAllEncoders();
    }
  }

  // --- Process MQTT Messages ---
  if (messageReceived) {
    messageReceived = false;
    Serial.printf("[LOOP] Topic: %s\n", topicBuffer.c_str());

    if (topicBuffer == MOTOR_STOP) {
      if (payloadBuffer == "STOP") {
        Serial.println("[MOTION] Emergency Stop.");
        stopAllMotors();
        if (file)
          file.close();
        sysState = STATE_IDLE;
        pausedPrintState = STATE_IDLE;
        manualMovePending = false;
      } else if (payloadBuffer == "PAUSE") {
        if (isActivePrintState(sysState)) {
          Serial.println("[MOTION] Pausing print safely...");
          pausedPrintState = sysState;
          sysState = STATE_MOTION_PAUSED;
        }
      }
    } else if (topicBuffer == TOPIC_START_file_start_stop) {
      if (payloadBuffer.indexOf('|') != -1) {
        int pipe1 = payloadBuffer.indexOf('|');
        int pipe2 = payloadBuffer.indexOf('|', pipe1 + 1);
        int pipe3 = payloadBuffer.indexOf('|', pipe2 + 1);

        if (pipe1 != -1 && pipe2 != -1) {
          if (pipe1 == 0 && pipe3 != -1) {
            fileName = payloadBuffer.substring(pipe1 + 1, pipe2);
            totalChunks = payloadBuffer.substring(pipe2 + 1, pipe3).toInt();
            expectedChecksum = payloadBuffer.substring(pipe3 + 1);
          } else {
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
          Serial.printf("[XFER] Start: file=%s chunks=%d checksum=%s\n",
                        fileName.c_str(), totalChunks,
                        expectedChecksum.c_str());
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("READY");
          mqttClient.endMessage();
        }
      } else if (payloadBuffer.indexOf('/') != -1) {
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
      } else {
        String command = payloadBuffer;
        command.trim();

        if (command == "home" && sysState == STATE_IDLE) {
          Serial.println("[HOME] Polar Homing all axes...");
          r_home = false;
          z_home = false;
          theta_home = false;
          sysState = STATE_HOMING_SEEK;
        } else if (command == "stop") {
          Serial.println("[CMD] Stop.");
          stopAllMotors();
          if (file)
            file.close();
          sysState = STATE_IDLE;
        }
      }
    } else if (topicBuffer == TOPIC_STOP && fileReceiving) {
      int pipe1 = payloadBuffer.indexOf('|');
      int pipe2 = payloadBuffer.indexOf('|', pipe1 + 1);

      if (pipe1 != -1 && pipe2 != -1) {
        int chunkID = payloadBuffer.substring(pipe1 + 1, pipe2).toInt();
        if (chunkID <= lastReceivedChunk) {
          Serial.printf("[XFER] Duplicate chunk %d, ignored.\n", chunkID);
        } else if (chunkID != lastReceivedChunk + 1) {
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("RESEND|");
          mqttClient.print(lastReceivedChunk + 1);
          mqttClient.endMessage();
        } else {
          appendFile(SD, fileName.c_str(),
                     payloadBuffer.substring(pipe2 + 1).c_str());
          lastReceivedChunk = chunkID;
          saveState();
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("ACK|");
          mqttClient.print(chunkID);
          mqttClient.endMessage();
        }
      }
    } else if (topicBuffer == TOPIC_xyz_move) {
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
        if (isActivePrintState(sysState)) {
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
        }
      }
    } else if (topicBuffer == MOTOR_START) {
      if (payloadBuffer == "RESUME" &&
          (sysState == STATE_MOTION_PAUSED || sysState == STATE_MANUAL_XYZ)) {
        if (pausedPrintState != STATE_IDLE) {
          sysState = pausedPrintState;
          pausedPrintState = STATE_IDLE;
        } else {
          stopAllMotors();
          sysState = STATE_IDLE;
        }
      } else if (sysState == STATE_IDLE) {
        for (int i = 0; i < 4; i++) {
          updateEncoderFromAS5600(i);
          totalRotations[i] = 0;
        }
        sysState = STATE_MOTION_OPEN;
      }
    }
  }

  delay(1);

  // ===========================================================================
  // STATE MACHINE — NON-BLOCKING COOPERATIVE EXECUTION
  // ===========================================================================
  switch (sysState) {

  case STATE_IDLE:
    break;

  case STATE_MOTION_PAUSED: {
    stopAllMotors();
    if (manualMovePending) {
      resetManualXYZTarget();
      applyPendingManualMove();
      sysState = STATE_MANUAL_XYZ;
    }
    break;
  }

  case STATE_MANUAL_XYZ: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    updateEncoderFromAS5600(0); // R DC Motor
    updateEncoderFromAS5600(1); // Z DC Motor
    updateEncoderFromAS5600(3); // E Motor

    if (manualMovePending)
      applyPendingManualMove();

    setThetaServoAngle(manualServoTarget);

    bool rReached =
        abs(manualTarget[0] - encoderCount[0]) <= MOTION_TOLERANCE_COUNTS;
    bool zReached =
        abs(manualTarget[1] - encoderCount[1]) <= MOTION_TOLERANCE_COUNTS;

    if (rReached && zReached) {
      stopAllMotors();
    } else {
      driveMotor(0, calculatePID(0, manualTarget[0], dt));
      driveMotor(1, calculatePID(1, manualTarget[1], dt));
    }
    break;
  }

  // --- POLAR HOMING ARCHITECTURE ---
  case STATE_HOMING_SEEK: {
    // R Bed Rotation Homing
    if (!r_home) {
      analogWrite(motorPinA1, 60);
      analogWrite(motorPinA2, 0);
      if (digitalRead(THETA_HOME_SENSOR_PIN) == LOW) {
        analogWrite(motorPinA1, 0);
        r_home = true;
        encoderCount[0] = 0;
        rawAccumulator[0] = 0;
        lastBedAngleDeg = 0.0f;
        accumulatedBedAngleDeg = 0.0f;
        Serial.println("[HOME] R Bed reference sensor reached.");
      }
    }
    // Z Height Homing
    if (!z_home) {
      analogWrite(motorPinB1, 0);
      analogWrite(motorPinB2, 100);
      if (digitalRead(Z_LIMIT_PIN) == LOW) {
        analogWrite(motorPinB2, 0);
        z_home = true;
        encoderCount[1] = 0;
        rawAccumulator[1] = 0;
        Serial.println("[HOME] Z Limit switch reached.");
      }
    }
    // Theta Servo Homing
    if (!theta_home) {
      setThetaServoAngle(SERVO_MIN_ANGLE_DEG);
      theta_home = true;
      Serial.println("[HOME] Theta MG945 Servo homed to 0 deg.");
    }

    if (r_home && z_home && theta_home) {
      sysState = STATE_HOMING_ZERO;
    }
    break;
  }

  case STATE_HOMING_ZERO: {
    for (int i = 0; i < 4; i++) {
      if (i == 2)
        continue;
      if (as5600Ready && selectI2CChannel(motorToChannel[i])) {
        delay(5);
        lastAngle[i] = readRawAngle();
      } else {
        lastAngle[i] = 0;
      }
      encoderCount[i] = 0;
      rawAccumulator[i] = 0;
      totalRotations[i] = 0;
      delay(1);
    }
    disableAllI2CChannels();

    setpointHome[0] = 0;
    setpointHome[1] = zMmToEncoderCounts(10.0f); // 10 mm standoff
    setpointHome[2] = SERVO_MIN_ANGLE_DEG;

    lastTimePID = millis();
    for (int i = 0; i < 4; i++) {
      integral[i] = 0.0f;
      prevError[i] = 0;
    }
    Serial.println("[HOME] Moving Z to standoff (10 mm)...");
    sysState = STATE_HOMING_STANDOFF;
    break;
  }

  case STATE_HOMING_STANDOFF: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    updateEncoderFromAS5600(0);
    updateEncoderFromAS5600(1);

    long zError = setpointHome[1] - encoderCount[1];
    if (abs(zError) <= MOTION_TOLERANCE_COUNTS) {
      stopAllMotors();
      Serial.println("[HOME] Polar homing sequence complete ✅");
      currentX = HOME_X;
      currentY = HOME_Y;
      currentZ = 10.0f;
      sysState = STATE_IDLE;
    } else {
      driveMotor(1, calculatePID(1, setpointHome[1], dt));
    }
    break;
  }

  case STATE_MOTION_OPEN: {
    listDir(SD, "/", 0);
    file = SD.open(fileName.c_str());
    if (!file) {
      Serial.printf("[MOTION] Failed to open file: %s\n", fileName.c_str());
      sysState = STATE_IDLE;
      break;
    }
    totalLines = countLinesInFile(SD, fileName.c_str());
    myconut = 0;
    dwellStart = millis();
    sysState = STATE_MOTION_DWELL;
    break;
  }

  case STATE_MOTION_READ_LINE: {
    String motionLine = readNextLine();
    motionLine.trim();

    float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f, targetE = 0.0f;
    if (!parseCartesianLine(motionLine, targetX, targetY, targetZ, targetE)) {
      myconut++;
      dwellStart = millis();
      sysState = STATE_MOTION_DWELL;
      break;
    }

    // Interpolate line into Cartesian segments
    totalSegmentsCount = generateCartesianSegments(
        currentX, currentY, currentZ, 0.0f, targetX, targetY, targetZ, targetE,
        segmentBuffer, MAX_INTERPOLATION_SEGMENTS);
    currentSegmentIndex = 0;
    myconut++;

    dwellStart = millis();
    sysState = STATE_MOTION_DWELL;
    break;
  }

  case STATE_MOTION_DWELL: {
    if (millis() - dwellStart >= 50) {
      if (currentSegmentIndex < totalSegmentsCount) {
        // Execute next segment of current line
        CartesianSegment seg = segmentBuffer[currentSegmentIndex++];
        long targetR = 0, targetZ = 0;
        int targetTheta = SERVO_MIN_ANGLE_DEG;

        if (!calculatePolarIK(seg.x, seg.y, seg.z, targetR, targetTheta,
                              targetZ)) {
          Serial.println("[MOTION] Segment Polar IK failed. Stopping.");
          stopAllMotors();
          if (file)
            file.close();
          sysState = STATE_IDLE;
          break;
        }

        setpoint[0] = targetR;
        setpoint[1] = targetZ;
        setpoint[3] = (long)lroundf(seg.e * STEPS_PER_MM_E);
        setThetaServoAngle(targetTheta);

        currentX = seg.x;
        currentY = seg.y;
        currentZ = seg.z;

        lastTimePID = millis();
        for (int i = 0; i < 4; i++) {
          integral[i] = 0.0f;
          prevError[i] = 0;
        }
        sysState = STATE_MOTION_PID;
      } else {
        if (myconut >= totalLines) {
          sysState = STATE_MOTION_DONE;
        } else {
          sysState = STATE_MOTION_READ_LINE;
        }
      }
    }
    break;
  }

  case STATE_MOTION_PID: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    updateEncoderFromAS5600(0); // R Motor
    updateEncoderFromAS5600(1); // Z Motor
    updateEncoderFromAS5600(3); // E Motor

    bool rReached =
        abs(setpoint[0] - encoderCount[0]) <= MOTION_TOLERANCE_COUNTS;
    bool zReached =
        abs(setpoint[1] - encoderCount[1]) <= MOTION_TOLERANCE_COUNTS;
    bool eReached =
        abs(setpoint[3] - encoderCount[3]) <= MOTION_TOLERANCE_COUNTS;

    if (rReached && zReached && eReached) {
      dwellStart = millis();
      sysState = STATE_MOTION_DWELL;
    } else {
      driveMotor(0, calculatePID(0, setpoint[0], dt));
      driveMotor(1, calculatePID(1, setpoint[1], dt));
      driveMotor(3, calculatePID(3, setpoint[3], dt));
    }
    break;
  }

  case STATE_MOTION_DONE: {
    stopAllMotors();
    if (file)
      file.close();
    Serial.println("[MOTION] Print complete.");
    checkAllEncoders();
    sysState = STATE_IDLE;
    break;
  }
  }
}