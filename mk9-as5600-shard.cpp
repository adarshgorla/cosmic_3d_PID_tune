// mk9-as5600-shard.cpp
// ESP32-S3 | Cosmic3D MK9 | AS5600 Magnetic Encoder Edition (Serial Monitored & SD Logged)

#include <WiFi.h> // Kept only if main .ino file relies on it for background tasks
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Preferences.h>
#include <Wire.h>

// --- AS5600 & TCA9548A ---
#define AS5600_ADDRESS      0x36
#define AS5600_RAW_ANGLE_H  0x0C
#define AS5600_RAW_ANGLE_L  0x0D
#define AS5600_ANGLE_H      0x0E
#define AS5600_ANGLE_L      0x0F
#define AS5600_STATUS       0x0B
#define AS5600_AGC          0x1A
#define AS5600_MAGNITUDE_H  0x1B
#define AS5600_MAGNITUDE_L  0x1C
#define TCA9548A_ADDRESS    0x70
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_TIMEOUT_MS 30

// --- Gear Ratio ---
#define GEAR_RATIO 1

// --- Cartesian -> Delta IK / Encoder conversion ---
const float TOWER_A_X = 0.000f;
const float TOWER_A_Y = 157.631f;
const float TOWER_A_Z = 735.00f;

const float TOWER_B_X = -136.513f;
const float TOWER_B_Y = -78.816f;
const float TOWER_B_Z = 735.00f;

const float TOWER_C_X = 136.513f;
const float TOWER_C_Y = -78.816f;
const float TOWER_C_Z = 735.00f;

const float PULLEY_RADIUS_MM = 9.5f;
const float HOME_X = 0.0f;
const float HOME_Y = 0.0f;
const float HOME_Z = 0.0f;
const float STEPS_PER_MM_E = 100.0f;
const long MOTION_TOLERANCE_COUNTS = 5;

const float AS5600_COUNTS_PER_ROTATION = 4096.0f;
const float OUTPUT_COUNTS_PER_ROTATION = AS5600_COUNTS_PER_ROTATION;
const float MANUAL_JOG_MM = 10.0f;
const float COUNTS_PER_MM = OUTPUT_COUNTS_PER_ROTATION / (2.0f * PI * PULLEY_RADIUS_MM);

long mmToEncoderCounts(float mm) {
  return (long)lroundf(mm * COUNTS_PER_MM);
}

float homeCableLength[3] = {0.0f, 0.0f, 0.0f};
bool homeCableLengthReady = false;

// --- SD Card ---
#define REASSIGN_PINS
int sck  = 12;
int miso = 13;
int mosi = 11;
int cs   = 10;

// --- Heater & Thermistor ---
#define THERMISTOR_PIN      2
#define HEATER_PIN          21
#define MAX_TEMP           280
#define HYSTERESIS           2.0f
#define SETPOINT             0      
#define SERIES_RESISTOR   4700.0f
#define BETA              3950.0f

#define BED_THERMISTOR_PIN  1
#define BED_HEATER_PIN      47
#define BED_MAX_TEMP       130
#define BED_HYSTERESIS       2.0f
#define BED_SETPOINT         0      
#define BED_SERIES_RESISTOR  4700.0f
#define BED_BETA             3950.0f

#define ADC_MAX  4095.0f
#define T0       25.0f
#define T0_K     (T0 + 273.15f)
#define HEATER_POLL_MS 500

bool hotendHeaterOn = false;
bool bedHeaterOn    = false;
volatile float hotendSetpoint = SETPOINT;
volatile float bedSetpoint    = BED_SETPOINT;

// --- Cartesian State ---
float currentX = HOME_X;
float currentY = HOME_Y;
float currentZ = HOME_Z;

// --- NVS ---
Preferences prefs;
String fileName = "";

// --- Motor Pins & LEDC PWM ---
#define motorPinA1  14
#define motorPinA2  7
#define motorPinB1  15
#define motorPinB2  16
#define motorPinc1  5
#define motorPinc2  6
#define motorPind1  4
#define motorPind2  3

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define MIN_PWM 20

// --- Limit Switches ---
#define LIMIT1_PIN 46
#define LIMIT2_PIN 43
#define LIMIT3_PIN 48

// --- Encoder & Motion --- 
volatile long encoderCount[4]  = {0, 0, 0, 0};
volatile int  rotation[4]      = {0, 0, 0, 0};
volatile long setpoint[4]      = {0, 0, 0, 0};
long lastAngle[4]              = {0, 0, 0, 0};
long totalRotations[4]         = {0, 0, 0, 0};
static long rawAccumulator[4]  = {0, 0, 0, 0};

// --- PID (Pure P-Control) ---
float Kp[4] = {1.0f,  1.0f,  1.0f,  1.0f};
float Ki[4] = {0.0f,  0.0f,  0.0f,  0.0f};
float Kd[4] = {0.0f,  0.0f,  0.0f,  0.0f};
long  prevError[4]  = {0, 0, 0, 0};
float integral[4]   = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastTimePID = 0;

// --- SD File Handle ---
File file;
int  totalLines = 0;
int  myconut    = 0;

const uint8_t motorToChannel[4] = {0, 1, 2, 3};
const int encoderDirection[4] = {1, 1, 1, 1}; // FIX 5: Encoder Configuration (Tower A fixed to positive 1)
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

#define SWITCH_DEBOUNCE_REQUIRED 3
uint8_t switchDebounceCounter[3] = {0, 0, 0};

// =============================================================================
// SD BLACK BOX LOGGING
// =============================================================================
void logToSD(String message) {
  String logEntry = String(millis()) + " ms | " + message + "\n";
  File f = SD.open("/blackbox_log.txt", FILE_APPEND);  
  if (f) { f.print(logEntry.c_str()); f.close(); }
  Serial.print("[LOG] " + logEntry);
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================
bool checkDebouncedLimitSwitch(int towerIdx, int pin) {
  if (digitalRead(pin) == LOW) {
    switchDebounceCounter[towerIdx]++;
    if (switchDebounceCounter[towerIdx] >= SWITCH_DEBOUNCE_REQUIRED) {
      return true;
    }
  } else {
    switchDebounceCounter[towerIdx] = 0;
  }
  return false;
}

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
  for (int i = 0; i < 4; i++) manualTarget[i] = encoderCount[i];
  lastTimePID = millis();
  for (int i = 0; i < 4; i++) { integral[i] = 0.0f; prevError[i] = 0; }
}

void queueManualMove(float dx, float dy, float dz) {
  manualMoveDeltaX = dx; manualMoveDeltaY = dy; manualMoveDeltaZ = dz;
  manualMovePending = true;
}

void applyPendingManualMove() {
  if (!manualMovePending) return;

  float nextX = currentX + manualMoveDeltaX;
  float nextY = currentY + manualMoveDeltaY;
  float nextZ = currentZ + manualMoveDeltaZ;

  long targetA = 0, targetB = 0, targetC = 0;
  if (convertXYZToEncoderTargets(nextX, nextY, nextZ, targetA, targetB, targetC)) {
    currentX = nextX; currentY = nextY; currentZ = nextZ;
    manualTarget[0] = targetA; manualTarget[1] = targetB; manualTarget[2] = targetC;
    
    lastTimePID = millis();
    for (int i = 0; i < 4; i++) { integral[i] = 0.0f; prevError[i] = 0; }
  } else {
    logToSD("IK calculation failed, manual move discarded.");
  }
  manualMovePending = false;
  manualMoveDeltaX = 0.0f; manualMoveDeltaY = 0.0f; manualMoveDeltaZ = 0.0f;
}

// =============================================================================
// SD
// =============================================================================
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("[SD] Listing: %s\n", dirname);
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) return;
  File f = root.openNextFile();
  while (f) {
    if (f.isDirectory()) {
      Serial.print("  DIR : "); Serial.println(f.name());
      if (levels) listDir(fs, f.name(), levels - 1);
    } else {
      Serial.print("  FILE: "); Serial.print(f.name());
      Serial.print("  SIZE: "); Serial.println(f.size());
    }
    f = root.openNextFile();
  }
}

int countLinesInFile(fs::FS &fs, const char *path) {
  File tempFile = fs.open(path);
  if (!tempFile) return -1;
  int lineCount = 0;
  while (tempFile.available()) { if (tempFile.read() == '\n') lineCount++; }
  if (tempFile.size() > 0 && tempFile.peek() != '\n') lineCount++;
  tempFile.close();
  return lineCount;
}

String readNextLine() {
  if (file && file.available()) return file.readStringUntil('\n');
  return "";
}

// =============================================================================
// INVERSE KINEMATICS
// =============================================================================
bool calculateIK(float nozzleX, float nozzleY, float nozzleZ, float &cableA, float &cableB, float &cableC) {
    const float EFFECTOR_RADIUS = 22.92f;
    const float NOZZLE_TO_ATTACHMENT = 124.403f;
    const float PIVOT_X = 0.0f; const float PIVOT_Y = 0.0f; const float PIVOT_Z = 735.0f;

    float dx = nozzleX - PIVOT_X; float dy = nozzleY - PIVOT_Y; float dz = nozzleZ - PIVOT_Z;
    float mag = sqrtf(dx*dx + dy*dy + dz*dz);
    if (mag < 0.0001f) return false;

    float ux = dx / mag; float uy = dy / mag; float uz = dz / mag;
    float centerX = nozzleX - ux * NOZZLE_TO_ATTACHMENT;
    float centerY = nozzleY - uy * NOZZLE_TO_ATTACHMENT;
    float centerZ = nozzleZ - uz * NOZZLE_TO_ATTACHMENT;

    float rx = 0.0f; float ry = 0.0f; float rz = 1.0f;
    if (fabsf(uz) > 0.99f) { rx = 1.0f; ry = 0.0f; rz = 0.0f; }

    float u1x = ry*uz - rz*uy; float u1y = rz*ux - rx*uz; float u1z = rx*uy - ry*ux;
    float u1mag = sqrtf(u1x*u1x + u1y*u1y + u1z*u1z);
    if (u1mag < 0.0001f) return false;
    u1x /= u1mag; u1y /= u1mag; u1z /= u1mag;

    float u2x = uy*u1z - uz*u1y; float u2y = uz*u1x - ux*u1z; float u2z = ux*u1y - uy*u1x;

    const float COS120 = -0.5f; const float SIN120 = 0.8660254f;

    float ax = centerX + EFFECTOR_RADIUS * u1x;
    float ay = centerY + EFFECTOR_RADIUS * u1y;
    float az = centerZ + EFFECTOR_RADIUS * u1z;
    float bx = centerX + EFFECTOR_RADIUS * (COS120*u1x + SIN120*u2x);
    float by = centerY + EFFECTOR_RADIUS * (COS120*u1y + SIN120*u2y);
    float bz = centerZ + EFFECTOR_RADIUS * (COS120*u1z + SIN120*u2z);
    float cx = centerX + EFFECTOR_RADIUS * (COS120*u1x - SIN120*u2x);
    float cy = centerY + EFFECTOR_RADIUS * (COS120*u1y - SIN120*u2y);                              
    float cz = centerZ + EFFECTOR_RADIUS * (COS120*u1z - SIN120*u2z);

    cableA = sqrtf((ax - TOWER_A_X)*(ax - TOWER_A_X) + (ay - TOWER_A_Y)*(ay - TOWER_A_Y) + (az - TOWER_A_Z)*(az - TOWER_A_Z));
    cableB = sqrtf((bx - TOWER_B_X)*(bx - TOWER_B_X) + (by - TOWER_B_Y)*(by - TOWER_B_Y) + (bz - TOWER_B_Z)*(bz - TOWER_B_Z));
    cableC = sqrtf((cx - TOWER_C_X)*(cx - TOWER_C_X) + (cy - TOWER_C_Y)*(cy - TOWER_C_Y) + (cz - TOWER_C_Z)*(cz - TOWER_C_Z));

    return isfinite(cableA) && isfinite(cableB) && isfinite(cableC);
}

void initializeHomeCableLengths() {
  float l1 = 0.0f, l2 = 0.0f, l3 = 0.0f;
  if (!calculateIK(HOME_X, HOME_Y, HOME_Z, l1, l2, l3)) {
    homeCableLengthReady = false;
    return;
  }
  homeCableLength[0] = l1; homeCableLength[1] = l2; homeCableLength[2] = l3;
  homeCableLengthReady = true;
}

bool convertXYZToEncoderTargets(float targetX, float targetY, float targetZ, long &targetA, long &targetB, long &targetC) {
  if (!homeCableLengthReady) initializeHomeCableLengths();
  if (!homeCableLengthReady) return false;
  if (PULLEY_RADIUS_MM <= 0.0f) return false;

  float l1 = 0.0f, l2 = 0.0f, l3 = 0.0f;
  if (!calculateIK(targetX, targetY, targetZ, l1, l2, l3)) return false;

  float circumference = 2.0f * PI * PULLEY_RADIUS_MM;
  float rotationsA = (l1 - homeCableLength[0]) / circumference;
  float rotationsB = (l2 - homeCableLength[1]) / circumference;
  float rotationsC = (l3 - homeCableLength[2]) / circumference;

  targetA = (long)lroundf(rotationsA * OUTPUT_COUNTS_PER_ROTATION);
  targetB = (long)lroundf(rotationsB * OUTPUT_COUNTS_PER_ROTATION);
  targetC = (long)lroundf(rotationsC * OUTPUT_COUNTS_PER_ROTATION);
  return true;  
}

bool parseCartesianLine(String line, float &x, float &y, float &z, float &e) {
  line.trim();
  if (line.length() == 0) return false;
  line.replace(",", " "); line.replace(";", " ");

  float val1, val2, val3, val4, val5;
  int parsedCount = sscanf(line.c_str(), "%f %f %f %f %f", &val1, &val2, &val3, &val4, &val5);

  if (parsedCount == 5) { x = val1; y = val2; z = val3; e = val5; return true; } 
  else if (parsedCount == 4) { x = val1; y = val2; z = val3; e = val4; return true; }
  return false;
}

// =============================================================================
// THERMISTOR
// =============================================================================
float readThermistorC(int pin, float seriesResistor, float beta) {
  long sum = 0;
  const int samples = 32;
  for (int i = 0; i < samples; i++) { sum += analogRead(pin); delayMicroseconds(100); }
  float adcValue = (float)sum / samples;

  if (adcValue <= 5.0f || adcValue >= 4090.0f) return -1.0f;
  float resistance = seriesResistor * (ADC_MAX / adcValue - 1.0f);
  float steinhart  = log(resistance / 100000.0f);
  steinhart       /= beta;
  steinhart       += 1.0f / (T0 + 273.15f);
  return (1.0f / steinhart) - 273.15f;
}

// =============================================================================
// I2C / AS5600 LOW-LEVEL
// =============================================================================
bool selectI2CChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);
  return (Wire.endTransmission() == 0);
}

void disableAllI2CChannels() {
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
}

uint16_t readAS5600Register16(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDRESS); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  size_t n = Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2, (uint8_t)true);
  if (n >= 2 && Wire.available() >= 2) {
    uint16_t high = Wire.read(); uint16_t low  = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

uint8_t readAS5600Register8(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDRESS); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  size_t n = Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)1, (uint8_t)true);
  if (n >= 1 && Wire.available()) return Wire.read();
  return 0;
}

uint16_t readRawAngle()  { return readAS5600Register16(AS5600_RAW_ANGLE_H); }
uint16_t readAngle()     { return readAS5600Register16(AS5600_ANGLE_H); }
uint8_t  readStatus()    { return readAS5600Register8(AS5600_STATUS); }
uint8_t  readAGC()       { return readAS5600Register8(AS5600_AGC); }
uint16_t readMagnitude() { return readAS5600Register16(AS5600_MAGNITUDE_H); }

bool probeI2CDevice(uint8_t address) {
  Wire.beginTransmission(address); return Wire.endTransmission() == 0;
}
bool isMagnetDetected() {
  uint8_t status = readStatus(); return (status & 0x20) && !(status & 0x18);
}

// =============================================================================
// ENCODER UPDATE
// =============================================================================
void updateEncoderFromAS5600(int motorIndex) {
  if (!as5600Ready) return;
  if (!selectI2CChannel(motorToChannel[motorIndex])) return;
  delay(1);

  uint16_t currentAngle = readRawAngle();
  
  // FIX 6: Ignore Invalid Encoder Reads
  if (currentAngle == 0) {
      disableAllI2CChannels();
      return;
  }

  // FIX 5: Multiplied by unified EncoderDirection
  long diff = ((long)currentAngle - (long)lastAngle[motorIndex]) * encoderDirection[motorIndex];

  if (diff >  2048) { diff -= 4096; totalRotations[motorIndex]--; }
  if (diff < -2048) { diff += 4096; totalRotations[motorIndex]++; }

  if (abs(diff) > 2048) {
    disableAllI2CChannels();
    return;
  }

  rawAccumulator[motorIndex] += diff;
  long motorCounts = rawAccumulator[motorIndex] / GEAR_RATIO;

  if (motorCounts != 0) {
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
  Wire.setClock(400000); Wire.setTimeOut(I2C_TIMEOUT_MS);
  if (!probeI2CDevice(TCA9548A_ADDRESS)) { as5600Ready = false; return; }
  as5600Ready = true;

  for (int i = 0; i < 4; i++) {
    if (!selectI2CChannel(motorToChannel[i])) { as5600Ready = false; continue; }
    delay(10);
    
    // FIX 7: Encoder Startup Verification
    uint16_t raw = readRawAngle();
    Serial.printf("[ENCODER] Motor %d Raw Angle = %u\n", i, raw);
    
    lastAngle[i] = raw;
    encoderCount[i] = 0; rawAccumulator[i] = 0; totalRotations[i] = 0;
    delay(1);
  }
  disableAllI2CChannels();
}

void checkAllEncoders() {
  if (!as5600Ready) return;
  for (int i = 0; i < 4; i++) {
    if (!selectI2CChannel(motorToChannel[i])) continue;
    delay(1);
    uint16_t angle = readRawAngle(); uint8_t status = readStatus();
    bool magOK = isMagnetDetected();
    delay(1);
  }
  disableAllI2CChannels();
}

// =============================================================================
// HEATER CONTROL
// =============================================================================
void runHeaterControl() {
  float hotendTemp = readThermistorC(THERMISTOR_PIN, SERIES_RESISTOR, BETA);
  if (hotendTemp < 0 || hotendTemp > MAX_TEMP) {
    if (hotendHeaterOn) { digitalWrite(HEATER_PIN, LOW); hotendHeaterOn = false; }
  } else {
    if (hotendTemp < (hotendSetpoint - HYSTERESIS) && !hotendHeaterOn) { digitalWrite(HEATER_PIN, HIGH); hotendHeaterOn = true; } 
    else if (hotendTemp > (hotendSetpoint + HYSTERESIS) && hotendHeaterOn) { digitalWrite(HEATER_PIN, LOW); hotendHeaterOn = false; }
  }

  float bedTemp = readThermistorC(BED_THERMISTOR_PIN, BED_SERIES_RESISTOR, BED_BETA);
  if (bedTemp < 0 || bedTemp > BED_MAX_TEMP) {
    if (bedHeaterOn) { digitalWrite(BED_HEATER_PIN, LOW); bedHeaterOn = false; }
  } else {
    if (bedTemp < (bedSetpoint - BED_HYSTERESIS) && !bedHeaterOn) { digitalWrite(BED_HEATER_PIN, HIGH); bedHeaterOn = true; } 
    else if (bedTemp > (bedSetpoint + BED_HYSTERESIS) && bedHeaterOn) { digitalWrite(BED_HEATER_PIN, LOW); bedHeaterOn = false; }
  }
}

// =============================================================================
// PID HELPER
// =============================================================================
float calculatePID(int i, long targetPosition, float dt) {
  long error = targetPosition - encoderCount[i];

  float P = Kp[i] * (float)error;
  integral[i] += (float)error * dt; integral[i] = constrain(integral[i], -1000.0f, 1000.0f);
  float I = Ki[i] * integral[i];
  float D = Kd[i] * ((float)(error - prevError[i]) / dt);

  prevError[i] = error;
  float output = P + I + D;
  return output;
}

// =============================================================================
// MOTOR DRIVE (LEDC IMPLEMENTED)
// =============================================================================
static int lastDirection[4] = {-1, -1, -1, -1};

void driveMotor(int i, float output) {
  // FIX 2: Explicit dead-stop for zero output
  if (output == 0.0f) {
    if (i == 0) { ledcWrite(motorPinA1, 0); ledcWrite(motorPinA2, 0); }
    else if (i == 1) { ledcWrite(motorPinB1, 0); ledcWrite(motorPinB2, 0); }
    else if (i == 2) { ledcWrite(motorPinc1, 0); ledcWrite(motorPinc2, 0); }
    else if (i == 3) { ledcWrite(motorPind1, 0); ledcWrite(motorPind2, 0); }
    return;
  }

  int pwm = (int)abs(output);
  int floor = MIN_PWM; // FIX 8: Lower stiction floor

  if (pwm > 0 && pwm < floor) { pwm = floor; }
  if (pwm > 255) pwm = 255;

  int newDir = (output > 0) ? 0 : (output < 0) ? 1 : 2;

  // FIX 3: Replaced all analogWrite with ledcWrite
  if (i == 0) {
    if (output > 0)      { ledcWrite(motorPinA1, 0);   ledcWrite(motorPinA2, pwm); }
    else if (output < 0) { ledcWrite(motorPinA1, pwm); ledcWrite(motorPinA2, 0);   }
  } else if (i == 1) {
    if (output > 0)      { ledcWrite(motorPinB1, 0);   ledcWrite(motorPinB2, pwm); }
    else if (output < 0) { ledcWrite(motorPinB1, pwm); ledcWrite(motorPinB2, 0);   }
  } else if (i == 2) {
    if (output > 0)      { ledcWrite(motorPinc1, 0);   ledcWrite(motorPinc2, pwm); }
    else if (output < 0) { ledcWrite(motorPinc1, pwm); ledcWrite(motorPinc2, 0);   }
  } else {
    if (output > 0)      { ledcWrite(motorPind1, 0);   ledcWrite(motorPind2, pwm); }
    else if (output < 0) { ledcWrite(motorPind1, pwm); ledcWrite(motorPind2, 0);   }
  }

  if (newDir != lastDirection[i]) {
    lastDirection[i] = newDir; rotation[i] = (newDir == 1) ? 1 : 0;
  }
}

// =============================================================================
// STOP ALL MOTORS
// =============================================================================
void stopAllMotors() {
  ledcWrite(motorPinA1, 0); ledcWrite(motorPinA2, 0);
  ledcWrite(motorPinB1, 0); ledcWrite(motorPinB2, 0);
  ledcWrite(motorPinc1, 0); ledcWrite(motorPinc2, 0);
  ledcWrite(motorPind1, 0); ledcWrite(motorPind2, 0);
  for (int i = 0; i < 4; i++) lastDirection[i] = 2;
}

// =============================================================================
// MK9 SETUP 
// =============================================================================
void mk9Setup() {
  mk9CoreReady = false;

  // FIX 3: Initialize LEDC Channels instead of standard output pins
  ledcAttach(motorPinA1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motorPinA2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motorPinB1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motorPinB2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motorPinc1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motorPinc2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motorPind1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motorPind2, PWM_FREQ, PWM_RESOLUTION);

  stopAllMotors();
  delay(1);

  pinMode(LIMIT1_PIN, INPUT_PULLUP); pinMode(LIMIT2_PIN, INPUT_PULLUP); pinMode(LIMIT3_PIN, INPUT_PULLUP);
  delay(1);

  pinMode(HEATER_PIN, OUTPUT); digitalWrite(HEATER_PIN, LOW);
  pinMode(BED_HEATER_PIN, OUTPUT); digitalWrite(BED_HEATER_PIN, LOW);
  delay(1);

  initAS5600();
  delay(1);

  if (!as5600Ready) return;

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

  initializeHomeCableLengths();
  mk9CoreReady = true;
  Serial.println("[SETUP] Printer Ready. Awaiting Serial Commands...");
}

bool isPrinterActive() { return sysState != STATE_IDLE && sysState != STATE_MOTION_PAUSED; }

// =============================================================================
// LOOP 
// =============================================================================
void mk9Loop() {
  if (!mk9CoreReady) { delay(1); return; }

  static unsigned long lastHeaterPoll = 0;
  if (millis() - lastHeaterPoll >= HEATER_POLL_MS) { lastHeaterPoll = millis(); runHeaterControl(); }

  // FIX 10: PID Debug Output Every 100ms
  static unsigned long lastPIDDebugMs = 0;
  if (millis() - lastPIDDebugMs >= 100) {
    lastPIDDebugMs = millis();
    if (isActivePrintState(sysState) || sysState == STATE_MANUAL_XYZ || sysState == STATE_HOMING_STANDOFF) {
      long target0 = isActivePrintState(sysState) ? setpoint[0] : (sysState == STATE_MANUAL_XYZ ? manualTarget[0] : setpointHome[0]);
      long target1 = isActivePrintState(sysState) ? setpoint[1] : (sysState == STATE_MANUAL_XYZ ? manualTarget[1] : setpointHome[1]);
      long target2 = isActivePrintState(sysState) ? setpoint[2] : (sysState == STATE_MANUAL_XYZ ? manualTarget[2] : setpointHome[2]);
      
      Serial.printf(
        "A Target=%ld Enc=%ld Err=%ld | "
        "B Target=%ld Enc=%ld Err=%ld | "
        "C Target=%ld Enc=%ld Err=%ld\n",
        target0, encoderCount[0], target0 - encoderCount[0],
        target1, encoderCount[1], target1 - encoderCount[1],
        target2, encoderCount[2], target2 - encoderCount[2]
      );
    }
  }

  // ===========================================================================
  // SERIAL COMMAND LISTENER (Replaces MQTT)
  // ===========================================================================
  if (Serial.available() > 0) {
    String payloadBuffer = Serial.readStringUntil('\n');
    payloadBuffer.trim();

    if (payloadBuffer.length() > 0) {
      logToSD("USER COMMAND: " + payloadBuffer); 
      String cmd = payloadBuffer;
      cmd.toUpperCase(); 

      if (cmd == "STOP") {
        stopAllMotors(); 
        if (file) file.close();
        sysState = STATE_IDLE; pausedPrintState = STATE_IDLE; manualMovePending = false;
        logToSD("System Halted by User.");
      } 
      else if (cmd == "PAUSE") {
        if (isActivePrintState(sysState)) { 
          pausedPrintState = sysState; sysState = STATE_MOTION_PAUSED; logToSD("Print Paused.");
        }
      }
      else if (cmd == "RESUME") {
        if (sysState == STATE_MOTION_PAUSED || sysState == STATE_MANUAL_XYZ) {
          if (pausedPrintState != STATE_IDLE) {
            sysState = pausedPrintState; pausedPrintState = STATE_IDLE; manualMovePending = false;
            logToSD("Print Resumed.");
          } else { stopAllMotors(); sysState = STATE_IDLE; }
        }
      }
      else if (cmd == "HOME" && sysState == STATE_IDLE) {
        m1_home = false; m2_home = false; m3_home = false;
        switchDebounceCounter[0] = 0; switchDebounceCounter[1] = 0; switchDebounceCounter[2] = 0;
        sysState = STATE_HOMING_SEEK;
        logToSD("Homing Sequence Initiated.");
      }
      else if (cmd.startsWith("START ")) {
        if (sysState == STATE_IDLE) {
          fileName = payloadBuffer.substring(6); 
          fileName.trim();
          if (!fileName.startsWith("/")) fileName = "/" + fileName; 
          
          for (int i = 0; i < 4; i++) { updateEncoderFromAS5600(i); totalRotations[i] = 0; }
          sysState = STATE_MOTION_OPEN;
          logToSD("Starting print file: " + fileName);
        } else { Serial.println("Cannot start print. Printer is not IDLE."); }
      }
      else if (cmd.startsWith("HOTEND ")) {
        hotendSetpoint = cmd.substring(7).toFloat(); logToSD("Hotend set to: " + String(hotendSetpoint));
      }
      else if (cmd.startsWith("BED ")) {
        bedSetpoint = cmd.substring(4).toFloat(); logToSD("Bed set to: " + String(bedSetpoint));
      }
      else if (cmd == "X+" || cmd == "X-" || cmd == "Y+" || cmd == "Y-" || cmd == "Z+" || cmd == "Z-") {
        float dx = 0.0f, dy = 0.0f, dz = 0.0f;
        if (cmd == "X+") dx = MANUAL_JOG_MM; else if (cmd == "X-") dx = -MANUAL_JOG_MM;
        else if (cmd == "Y+") dy = MANUAL_JOG_MM; else if (cmd == "Y-") dy = -MANUAL_JOG_MM;
        else if (cmd == "Z+") dz = MANUAL_JOG_MM; else if (cmd == "Z-") dz = -MANUAL_JOG_MM;

        if (isActivePrintState(sysState)) {
          queueManualMove(dx, dy, dz); pausedPrintState = sysState; sysState = STATE_MOTION_PAUSED;
        } else if (sysState == STATE_MOTION_PAUSED || sysState == STATE_MANUAL_XYZ || sysState == STATE_IDLE) {
          if (sysState != STATE_MANUAL_XYZ) { resetManualXYZTarget(); sysState = STATE_MANUAL_XYZ; }
          queueManualMove(dx, dy, dz);
        }
      } 
      else {
        Serial.println("Unknown Command. Available: HOME, STOP, PAUSE, RESUME, START /file, HOTEND 200, BED 60, X+, X-, Y+, Y-, Z+, Z-");
      }
    }
  }

  delay(1);

  // ===========================================================================
  // STATE MACHINE
  // ===========================================================================
  switch (sysState) {
    case STATE_IDLE: break;

    case STATE_MOTION_PAUSED: {
      stopAllMotors();
      if (manualMovePending) { resetManualXYZTarget(); applyPendingManualMove(); sysState = STATE_MANUAL_XYZ; }
      break;
    }

    case STATE_MANUAL_XYZ: {
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTimePID) / 1000.0f;
      if (dt <= 0.0f) dt = 0.001f;
      lastTimePID = currentTime;

      for (int i = 0; i < 4; i++) updateEncoderFromAS5600(i);
      if (manualMovePending) applyPendingManualMove();

      // FIX 9: All targets reached logic
      bool allReached = true;
      for (int i = 0; i < 4; i++) {
        if (abs(manualTarget[i] - encoderCount[i]) > MOTION_TOLERANCE_COUNTS) { allReached = false; break; }
      }

      if (allReached) { stopAllMotors(); Serial.println("[MOTION] ALL TARGETS REACHED"); } 
      else { for (int i = 0; i < 4; i++) driveMotor(i, calculatePID(i, manualTarget[i], dt)); }
      break;
    }

    case STATE_HOMING_SEEK: {
      // Replaced analogWrite with ledcWrite for homing sequence
      if (!m1_home) {
        ledcWrite(motorPinA1, 0); ledcWrite(motorPinA2, 255);
        if (checkDebouncedLimitSwitch(0, LIMIT1_PIN)) {
          ledcWrite(motorPinA2,0); m1_home = true; encoderCount[0] = 0; rawAccumulator[0] = 0;
        }
      }
      if (!m2_home) {
        ledcWrite(motorPinB1, 0); ledcWrite(motorPinB2, 255);
        if (checkDebouncedLimitSwitch(1, LIMIT2_PIN)) {
          ledcWrite(motorPinB2, 0); m2_home = true; encoderCount[1] = 0; rawAccumulator[1] = 0;
        }
      }
      if (!m3_home) {
        ledcWrite(motorPinc1, 0); ledcWrite(motorPinc2, 255);
        if (checkDebouncedLimitSwitch(2, LIMIT3_PIN)) {
          ledcWrite(motorPinc2, 0); m3_home = true; encoderCount[2] = 0; rawAccumulator[2] = 0;
        }
      }
      if (m1_home && m2_home && m3_home) sysState = STATE_HOMING_ZERO;
      break;
    }

    case STATE_HOMING_ZERO: {
      // FIX 1: Explicitly select correct channel before zero-read
      for (int i = 0; i < 4; i++) {
        if (as5600Ready && selectI2CChannel(motorToChannel[i])) {
          delay(1); 
          lastAngle[i] = readRawAngle();
          disableAllI2CChannels();
        } else {
          lastAngle[i] = 0;
        }
        encoderCount[i] = 0; rawAccumulator[i] = 0; totalRotations[i] = 0;
        delay(1);
      }
      disableAllI2CChannels();

      setpointHome[0] = -13400; setpointHome[1] = -13400; setpointHome[2] = -13400; setpointHome[3] = 100; 
      lastTimePID = millis();
      for (int i = 0; i < 4; i++) { integral[i] = 0.0f; prevError[i] = 0; }
      sysState = STATE_HOMING_STANDOFF;
      break;
    }

    case STATE_HOMING_STANDOFF: {
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTimePID) / 1000.0f;
      if (dt <= 0.0f) dt = 0.001f;
      lastTimePID = currentTime;

      for (int i = 0; i < 4; i++) updateEncoderFromAS5600(i);

      bool allReached = true;
      for (int i = 0; i < 4; i++) {
        if (abs(setpointHome[i] - encoderCount[i]) > MOTION_TOLERANCE_COUNTS) { allReached = false; break; }
      }

      if (allReached) {
        stopAllMotors();
        Serial.println("[MOTION] ALL TARGETS REACHED");
        currentX = HOME_X; currentY = HOME_Y; currentZ = HOME_Z;
        sysState = STATE_IDLE;
      } else {
        for (int i = 0; i < 4; i++) driveMotor(i, calculatePID(i, setpointHome[i], dt));
      }
      break;
    }

    case STATE_MOTION_OPEN: {
      listDir(SD, "/", 0);
      file = SD.open(fileName.c_str());
      if (!file) { sysState = STATE_IDLE; break; }
      totalLines = countLinesInFile(SD, fileName.c_str());
      myconut = 0; dwellStart = millis(); sysState = STATE_MOTION_DWELL;
      break;
    }

    case STATE_MOTION_DWELL: {
      if (millis() - dwellStart >= 1000) {
        if (myconut >= totalLines) sysState = STATE_MOTION_DONE;
        else sysState = STATE_MOTION_READ_LINE;
      }
      break;
    }

    case STATE_MOTION_READ_LINE: {
      String motionLine = readNextLine(); motionLine.trim();
      float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f, targetE = 0.0f;
      if (!parseCartesianLine(motionLine, targetX, targetY, targetZ, targetE)) {
        myconut++; dwellStart = millis(); sysState = STATE_MOTION_DWELL; break;
      }

      long targetA = 0, targetB = 0, targetC = 0;
      if (!convertXYZToEncoderTargets(targetX, targetY, targetZ, targetA, targetB, targetC)) {
        stopAllMotors(); if (file) file.close(); sysState = STATE_IDLE; break;
      }

      setpoint[0] = targetA; setpoint[1] = targetB; setpoint[2] = targetC;
      setpoint[3] = (long)lroundf(targetE * STEPS_PER_MM_E);
      currentX = targetX; currentY = targetY; currentZ = targetZ;

      lastTimePID = millis();
      for (int i = 0; i < 4; i++) { integral[i] = 0.0f; prevError[i] = 0; }
      sysState = STATE_MOTION_PID;
      break;
    }

    case STATE_MOTION_PID: {
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTimePID) / 1000.0f;
      if (dt <= 0.0f) dt = 0.001f;
      lastTimePID = currentTime;

      for (int i = 0; i < 4; i++) updateEncoderFromAS5600(i);

      // FIX 9: Safe target reached condition 
      bool allReached = true;
      for (int i = 0; i < 4; i++) {
        if (abs(setpoint[i] - encoderCount[i]) > MOTION_TOLERANCE_COUNTS) {
          allReached = false;
          break;
        }
      }

      if (allReached) {
        stopAllMotors();
        Serial.println("[MOTION] ALL TARGETS REACHED");
        logToSD("Motion targets reached line: " + String(myconut)); 
        myconut++;
        dwellStart = millis();
        sysState = STATE_MOTION_DWELL;
      } else {
        for (int i = 0; i < 4; i++) driveMotor(i, calculatePID(i, setpoint[i], dt));
      }
      break;
    } 

    case STATE_MOTION_DONE: {
      stopAllMotors(); file.close(); checkAllEncoders(); sysState = STATE_IDLE; break;
    }
  }
}
