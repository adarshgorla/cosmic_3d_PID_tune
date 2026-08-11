// =============================================================================
// Cosmic3D MK9 - Individual Motor & Encoder Comprehensive Diagnostic Suite
// ESP32-S3 | 12V DC Motors + L293D H-Bridge | AS5600 Encoders | TCA9548A I2C Mux
// =============================================================================
// Purpose & Story of the Experiment:
// Tests each motor (A, B, C, Extruder) INDIVIDUALLY in isolation to get the full story:
// 1. AS5600 Magnet & I2C Health (Magnet detection, AGC level, angle status)
// 2. Directional Mapping & Sign Integrity (Does +PWM increase encoder count?)
// 3. Stiction & Friction Threshold (Finds min PWM required for movement)
// 4. Full PWM Speed Curve (Ramps PWM from 60 to 255 FWD & REV)
// 5. Automated Stall / Glitch Detection
//
// Saves comprehensive logs to SD Card (/motor_individual_diagnostic_log.csv)
// and prints live telemetry to Serial Monitor (115200 baud).
// =============================================================================

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "pid_config.h"
#include <Wire.h>

#define DIAG_LOG_FILE_PATH "/motor_individual_diagnostic_log.csv"
#define LOG_SAMPLE_INTERVAL_MS 20  // High resolution telemetry logging (20ms)

// PWM Ramp Steps for diagnostic
const int PWM_RAMP_STEPS[] = {60, 100, 140, 180, 220, 255};
const int NUM_PWM_STEPS = sizeof(PWM_RAMP_STEPS) / sizeof(PWM_RAMP_STEPS[0]);
const unsigned long STEP_HOLD_MS = 1500;  // Run each PWM level for 1.5 seconds
const unsigned long COAST_DWELL_MS = 1000; // Pause 1.0 sec between direction changes

// Motor Metadata
const char* MOTOR_NAMES[MOTOR_COUNT] = {"Tower_A", "Tower_B", "Tower_C", "Extruder"};
const int MOTOR_PINS[MOTOR_COUNT][2] = {
  {MOTOR_0_PIN_1, MOTOR_0_PIN_2}, // Tower A (14, 7)
  {MOTOR_1_PIN_1, MOTOR_1_PIN_2}, // Tower B (15, 16)
  {MOTOR_2_PIN_1, MOTOR_2_PIN_2}, // Tower C (5, 6)
  {MOTOR_3_PIN_1, MOTOR_3_PIN_2}  // Extruder (4, 3)
};
const int LIMIT_PINS[3] = {LIMIT1_PIN, LIMIT2_PIN, LIMIT3_PIN};

// State Tracking per motor
volatile long encoderCount[MOTOR_COUNT] = {0, 0, 0, 0};
long lastRawAngle[MOTOR_COUNT] = {0, 0, 0, 0};
long totalRotations[MOTOR_COUNT] = {0, 0, 0, 0};
static long rawAccumulator[MOTOR_COUNT] = {0, 0, 0, 0};

// Telemetry state
long prevSampleCount = 0;
unsigned long prevSampleTimeMs = 0;
float currentRPM = 0.0f;
float currentSpeedMMs = 0.0f;

// Test Execution State Machine
enum DiagnosticTestState {
  DIAG_STATE_IDLE,
  DIAG_STATE_START_MOTOR,
  DIAG_STATE_FWD_RAMP,
  DIAG_STATE_FWD_COAST,
  DIAG_STATE_REV_RAMP,
  DIAG_STATE_REV_COAST,
  DIAG_STATE_MOTOR_SUMMARY,
  DIAG_STATE_ALL_FINISHED
};

DiagnosticTestState diagState = DIAG_STATE_IDLE;
int currentTestingMotor = 0;
int currentPwmStepIndex = 0;
unsigned long stepStartTimeMs = 0;
unsigned long diagSessionStartTimeMs = 0;

bool as5600Ready = false;
bool sdReady = false;
SPIClass sdSPI(HSPI);

// Motor Summary Performance Metrics
struct MotorHealthSummary {
  bool i2cConnected;
  bool magnetDetected;
  uint8_t agcLevel;
  uint16_t initialRawAngle;
  int minFwdPwmMoved;
  int minRevPwmMoved;
  float maxFwdRPM;
  float maxRevRPM;
  long totalCountsFwd;
  long totalCountsRev;
  bool directionMatched; // True if +PWM produced +Counts
  bool stallDetectedAtMaxPWM;
};

MotorHealthSummary motorReport[MOTOR_COUNT];

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void stopAllMotors();
void driveMotorRaw(int motorIdx, int pwmSigned);
bool selectI2CChannel(uint8_t channel);
void disableAllI2CChannels();
uint16_t readRawAngle();
uint8_t readStatus();
uint8_t readAGC();
uint16_t readMagnitude();
bool isMagnetDetected();
void updateEncoder(int motorIdx);
void initHardware();
void initSDLogHeader();
void logTelemetryRow(int motorIdx, const char* phase, int pwm, const char* note);
void printMotorSummaryReport(int motorIdx);
void printFullSuiteFinalReport();

// =============================================================================
// I2C & ENCODER LOW LEVEL
// =============================================================================
bool selectI2CChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

void disableAllI2CChannels() {
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
}

uint16_t readAS5600Register16(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;

  size_t n = Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2, (uint8_t)true);
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
  if (Wire.endTransmission(false) != 0) return 0;

  size_t n = Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)1, (uint8_t)true);
  if (n >= 1 && Wire.available()) return Wire.read();
  return 0;
}

uint16_t readRawAngle() { return readAS5600Register16(AS5600_RAW_ANGLE_H); }
uint8_t readStatus() { return readAS5600Register8(AS5600_STATUS); }
uint8_t readAGC() { return readAS5600Register8(AS5600_AGC); }
uint16_t readMagnitude() { return readAS5600Register16(AS5600_MAGNITUDE_H); }

bool isMagnetDetected() {
  uint8_t status = readStatus();
  return (status & 0x20) && !(status & 0x18);
}

void updateEncoder(int motorIdx) {
  if (!selectI2CChannel(MOTOR_TO_I2C_CHANNEL[motorIdx])) return;
  delayMicroseconds(50);

  uint16_t currentAngle = readRawAngle();
  long diff = (long)currentAngle - (long)lastRawAngle[motorIdx];

  if (diff > 2048) {
    diff -= 4096;
    totalRotations[motorIdx]--;
  } else if (diff < -2048) {
    diff += 4096;
    totalRotations[motorIdx]++;
  }

  if (abs(diff) <= 2048) {
    rawAccumulator[motorIdx] += diff;
    long motorCounts = rawAccumulator[motorIdx] / GEAR_RATIO;
    if (motorCounts != 0) {
      encoderCount[motorIdx] += motorCounts;
      rawAccumulator[motorIdx] -= motorCounts * GEAR_RATIO;
    }
  }

  lastRawAngle[motorIdx] = currentAngle;
  disableAllI2CChannels();
}

// =============================================================================
// MOTOR DRIVER LOW LEVEL
// =============================================================================
void driveMotorRaw(int motorIdx, int pwmSigned) {
  int pin1 = MOTOR_PINS[motorIdx][0];
  int pin2 = MOTOR_PINS[motorIdx][1];

  int pwmAbs = constrain(abs(pwmSigned), 0, 255);

  if (pwmSigned > 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, pwmAbs);
  } else if (pwmSigned < 0) {
    analogWrite(pin1, pwmAbs);
    analogWrite(pin2, 0);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

void stopAllMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    driveMotorRaw(i, 0);
  }
}

// =============================================================================
// SD CARD LOGGING ENGINE
// =============================================================================
void initSDLogHeader() {
  sdSPI.begin(SD_PIN_SCK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
  if (!SD.begin(SD_PIN_CS, sdSPI)) {
    Serial.println("[SD LOG] ❌ SD Card initialization failed! Telemetry output to Serial only.");
    sdReady = false;
    return;
  }
  sdReady = true;
  Serial.println("[SD LOG] ✅ SD Card initialized successfully.");

  File logFile = SD.open(DIAG_LOG_FILE_PATH, FILE_APPEND);
  if (logFile) {
    if (logFile.size() == 0) {
      logFile.println("TimestampMs,SessionElapsedMs,MotorIndex,MotorName,TestPhase,CommandedPWM,Pin1,Pin2,"
                      "RawAngle,EncoderCounts,DeltaCounts,TotalRotations,InstantRPM,SpeedMMs,MagnetStatus,AGC,Magnitude,"
                      "LimitSwitchState,DiagnosticNote");
    }
    logFile.println("--- NEW INDIVIDUAL MOTOR DIAGNOSTIC SESSION ---");
    logFile.close();
  }
}

void logTelemetryRow(int motorIdx, const char* phase, int pwmCommand, const char* note) {
  unsigned long now = millis();
  unsigned long sessionElapsed = now - diagSessionStartTimeMs;

  uint16_t angle = 0;
  uint8_t status = 0;
  uint8_t agc = 0;
  uint16_t magnitude = 0;
  bool magOK = false;

  if (selectI2CChannel(MOTOR_TO_I2C_CHANNEL[motorIdx])) {
    angle = readRawAngle();
    status = readStatus();
    agc = readAGC();
    magnitude = readMagnitude();
    magOK = isMagnetDetected();
    disableAllI2CChannels();
  }

  // Calculate speed over sampling window
  float dtSec = (now - prevSampleTimeMs) / 1000.0f;
  long dCounts = encoderCount[motorIdx] - prevSampleCount;
  if (dtSec >= 0.019f) {
    float dRotations = (float)dCounts / AS5600_COUNTS_PER_REV;
    currentRPM = (dRotations / dtSec) * 60.0f;
    currentSpeedMMs = (currentRPM * 2.0f * PI * PULLEY_RADIUS_MM) / 60.0f;
    prevSampleCount = encoderCount[motorIdx];
    prevSampleTimeMs = now;
  }

  // Limit Switch state
  bool limitClicked = false;
  if (motorIdx < 3) {
    limitClicked = (digitalRead(LIMIT_PINS[motorIdx]) == LOW);
  }

  int p1Val = (pwmCommand > 0) ? 0 : (pwmCommand < 0) ? abs(pwmCommand) : 0;
  int p2Val = (pwmCommand > 0) ? abs(pwmCommand) : (pwmCommand < 0) ? 0 : 0;

  float rots = (float)encoderCount[motorIdx] / AS5600_COUNTS_PER_REV;

  // Print Live Telemetry to Serial Monitor
  Serial.printf("[%6lu ms] Motor %d (%-8s) | Phase: %-10s | PWM: %4d (P1:%3d P2:%3d) | "
                "Angle: %4u | Count: %7ld (Δ:%4ld) | Rots: %6.3f | RPM: %6.1f | Speed: %5.1f mm/s | "
                "Mag: %s (AGC:%3u Mag:%4u) | Switch: %s | Note: %s\n",
                sessionElapsed, motorIdx, MOTOR_NAMES[motorIdx], phase, pwmCommand, p1Val, p2Val,
                angle, encoderCount[motorIdx], dCounts, rots, currentRPM, currentSpeedMMs,
                magOK ? "OK" : "NO", agc, magnitude, limitClicked ? "CLICKED" : "OPEN", note);

  // Write detailed row to SD CSV
  if (sdReady) {
    File logFile = SD.open(DIAG_LOG_FILE_PATH, FILE_APPEND);
    if (logFile) {
      logFile.printf("%lu,%lu,%d,%s,%s,%d,%d,%d,%u,%ld,%ld,%.4f,%.2f,%.2f,%s,%u,%u,%s,%s\n",
                     now, sessionElapsed, motorIdx, MOTOR_NAMES[motorIdx], phase, pwmCommand, p1Val, p2Val,
                     angle, encoderCount[motorIdx], dCounts, rots, currentRPM, currentSpeedMMs,
                     magOK ? "OK" : "FAULT", agc, magnitude, limitClicked ? "CLICKED" : "OPEN", note);
      logFile.close();
    }
  }
}

// =============================================================================
// INITIALIZATION & HARDWARE SETUP
// =============================================================================
void initHardware() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=========================================================================");
  Serial.println("  Cosmic3D MK9 - Individual Motor & Encoder Diagnostic Suite             ");
  Serial.println("=========================================================================");

  // Set pinModes for all motor driver pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
    motorReport[i] = {false, false, 0, 0, 0, 0, 0.0f, 0.0f, 0, 0, false, false};
  }
  stopAllMotors();

  // Set pinModes for limit switches
  for (int i = 0; i < 3; i++) {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
  }

  // Initialize Wire / I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  initSDLogHeader();

  Serial.println("\n[INIT] Initialized I2C, Motor Pins & Limit Switches.");
  Serial.println("-------------------------------------------------------------------------");
  Serial.println(" INSTRUCTIONS:");
  Serial.println(" 1. Send 'START' in Serial Monitor to run full diagnostic on all 4 motors.");
  Serial.println(" 2. Send 'MOTOR 0', 'MOTOR 1', 'MOTOR 2', or 'MOTOR 3' to test a single motor.");
  Serial.println(" 3. Send 'STOP' to immediately abort test.");
  Serial.println("-------------------------------------------------------------------------\n");

  diagState = DIAG_STATE_IDLE;
}

// =============================================================================
// DIAGNOSTIC SUITE RUNNER
// =============================================================================
void prepareMotorTest(int motorIdx) {
  currentTestingMotor = motorIdx;
  currentPwmStepIndex = 0;

  Serial.printf("\n=========================================================================\n");
  Serial.printf(" 🔬 PREPARING DIAGNOSTIC TEST FOR MOTOR %d (%s)\n", motorIdx, MOTOR_NAMES[motorIdx]);
  Serial.printf("=========================================================================\n");

  stopAllMotors();
  delay(100);

  // Check initial encoder & magnet status
  if (selectI2CChannel(MOTOR_TO_I2C_CHANNEL[motorIdx])) {
    delay(5);
    uint16_t angle = readRawAngle();
    uint8_t agc = readAGC();
    bool magOK = isMagnetDetected();

    motorReport[motorIdx].i2cConnected = true;
    motorReport[motorIdx].magnetDetected = magOK;
    motorReport[motorIdx].agcLevel = agc;
    motorReport[motorIdx].initialRawAngle = angle;

    lastRawAngle[motorIdx] = angle;
    encoderCount[motorIdx] = 0;
    rawAccumulator[motorIdx] = 0;
    totalRotations[motorIdx] = 0;
    prevSampleCount = 0;
    prevSampleTimeMs = millis();
    currentRPM = 0.0f;

    Serial.printf("[ENC SETUP] Motor %d (Ch %d): RawAngle=%u | AGC=%u | Magnet=%s\n",
                  motorIdx, MOTOR_TO_I2C_CHANNEL[motorIdx], angle, agc, magOK ? "✅ OK" : "❌ FAULT/MISSING");
    disableAllI2CChannels();
  } else {
    motorReport[motorIdx].i2cConnected = false;
    Serial.printf("[ENC SETUP] ❌ Motor %d: I2C Select Channel %d Failed!\n", motorIdx, MOTOR_TO_I2C_CHANNEL[motorIdx]);
  }

  stepStartTimeMs = millis();
  diagState = DIAG_STATE_FWD_RAMP;
}

void printMotorSummaryReport(int motorIdx) {
  MotorHealthSummary &rep = motorReport[motorIdx];

  Serial.println("\n-------------------------------------------------------------------------");
  Serial.printf(" 📊 MOTOR %d (%s) DIAGNOSTIC SUMMARY REPORT\n", motorIdx, MOTOR_NAMES[motorIdx]);
  Serial.println("-------------------------------------------------------------------------");
  Serial.printf(" I2C Communication      : %s\n", rep.i2cConnected ? "✅ PASS" : "❌ FAIL");
  Serial.printf(" AS5600 Magnet Health   : %s (AGC: %u)\n", rep.magnetDetected ? "✅ OK" : "❌ FAULT/MISSING", rep.agcLevel);
  Serial.printf(" Min PWM to Move (FWD)  : %d PWM\n", rep.minFwdPwmMoved);
  Serial.printf(" Min PWM to Move (REV)  : %d PWM\n", rep.minRevPwmMoved);
  Serial.printf(" Max Speed FWD (PWM 255): %.2f RPM | Net Counts: %ld\n", rep.maxFwdRPM, rep.totalCountsFwd);
  Serial.printf(" Max Speed REV (PWM 255): %.2f RPM | Net Counts: %ld\n", rep.maxRevRPM, rep.totalCountsRev);
  Serial.printf(" Direction Mapping      : %s\n", rep.directionMatched ? "✅ MATCHED (+PWM -> +Counts)" : "⚠️ INVERTED / UNCERTAIN");
  Serial.printf(" Stall at Max PWM (255) : %s\n", rep.stallDetectedAtMaxPWM ? "🚨 STALL DETECTED (0 RPM)" : "✅ NO STALL");
  Serial.println("-------------------------------------------------------------------------\n");

  if (sdReady) {
    File logFile = SD.open(DIAG_LOG_FILE_PATH, FILE_APPEND);
    if (logFile) {
      logFile.printf("Summary,Motor_%d,%s,I2C,%s,Magnet,%s,MinFwdPWM,%d,MinRevPWM,%d,MaxFwdRPM,%.2f,MaxRevRPM,%.2f,TotalFwdCounts,%ld,TotalRevCounts,%ld,StallAt255,%s\n",
                     motorIdx, MOTOR_NAMES[motorIdx], rep.i2cConnected ? "PASS" : "FAIL", rep.magnetDetected ? "OK" : "FAULT",
                     rep.minFwdPwmMoved, rep.minRevPwmMoved, rep.maxFwdRPM, rep.maxRevRPM, rep.totalCountsFwd, rep.totalCountsRev,
                     rep.stallDetectedAtMaxPWM ? "YES" : "NO");
      logFile.close();
    }
  }
}

void printFullSuiteFinalReport() {
  Serial.println("\n=========================================================================");
  Serial.println("        COMPREHENSIVE MULTI-MOTOR DIAGNOSTIC FINAL REPORT               ");
  Serial.println("=========================================================================");
  Serial.println(" Motor   | I2C Status | Magnet | Min Fwd PWM | Max Fwd RPM | Direction Sign | Stall at 255");
  Serial.println("---------+------------+--------+-------------+-------------+----------------+-------------");

  for (int i = 0; i < MOTOR_COUNT; i++) {
    MotorHealthSummary &r = motorReport[i];
    Serial.printf(" %-7s | %-10s | %-6s | %-11d | %-11.1f | %-14s | %-11s\n",
                  MOTOR_NAMES[i],
                  r.i2cConnected ? "PASS" : "FAIL",
                  r.magnetDetected ? "OK" : "MISSING",
                  r.minFwdPwmMoved,
                  r.maxFwdRPM,
                  r.directionMatched ? "OK (+)" : "INVERTED (-)",
                  r.stallDetectedAtMaxPWM ? "STALL (0)" : "HEALTHY");
  }
  Serial.println("=========================================================================\n");
}

// =============================================================================
// SETUP & MAIN LOOP
// =============================================================================
void setup() {
  initHardware();
}

void loop() {
  // Check for incoming user commands via Serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "START") {
      Serial.println("\n[CMD] Starting Full 4-Motor Diagnostic Sequence...");
      diagSessionStartTimeMs = millis();
      prepareMotorTest(0);
    } else if (cmd.startsWith("MOTOR")) {
      int idx = cmd.substring(5).toInt();
      if (idx >= 0 && idx < MOTOR_COUNT) {
        Serial.printf("\n[CMD] Testing Motor %d (%s) individually...\n", idx, MOTOR_NAMES[idx]);
        diagSessionStartTimeMs = millis();
        prepareMotorTest(idx);
      } else {
        Serial.println("[CMD] Invalid motor index. Use 0, 1, 2, or 3.");
      }
    } else if (cmd == "STOP") {
      Serial.println("\n[CMD] 🛑 EMERGENCY STOP REQUESTED! Stopping all motors.");
      stopAllMotors();
      diagState = DIAG_STATE_IDLE;
    }
  }

  // --- Diagnostic State Machine ---
  switch (diagState) {

  case DIAG_STATE_IDLE:
  case DIAG_STATE_ALL_FINISHED:
    delay(10);
    break;

  case DIAG_STATE_FWD_RAMP: {
    int currentPwm = PWM_RAMP_STEPS[currentPwmStepIndex];
    driveMotorRaw(currentTestingMotor, currentPwm);
    updateEncoder(currentTestingMotor);

    logTelemetryRow(currentTestingMotor, "FWD_RAMP", currentPwm, "Ramping FWD PWM");

    // Capture speed / movement milestones
    if (abs(currentRPM) > 0.5f && motorReport[currentTestingMotor].minFwdPwmMoved == 0) {
      motorReport[currentTestingMotor].minFwdPwmMoved = currentPwm;
    }
    if (currentRPM > motorReport[currentTestingMotor].maxFwdRPM) {
      motorReport[currentTestingMotor].maxFwdRPM = currentRPM;
    }

    if (millis() - stepStartTimeMs >= STEP_HOLD_MS) {
      currentPwmStepIndex++;
      stepStartTimeMs = millis();

      if (currentPwmStepIndex >= NUM_PWM_STEPS) {
        // FWD Ramp complete -> Coast
        driveMotorRaw(currentTestingMotor, 0);
        motorReport[currentTestingMotor].totalCountsFwd = encoderCount[currentTestingMotor];

        if (motorReport[currentTestingMotor].maxFwdRPM < 0.5f) {
          motorReport[currentTestingMotor].stallDetectedAtMaxPWM = true;
        }

        Serial.printf("[DIAG] Motor %d FWD Ramp Complete. Total FWD Counts: %ld. Coasting...\n",
                      currentTestingMotor, motorReport[currentTestingMotor].totalCountsFwd);
        diagState = DIAG_STATE_FWD_COAST;
      }
    }
    delay(LOG_SAMPLE_INTERVAL_MS);
    break;
  }

  case DIAG_STATE_FWD_COAST: {
    driveMotorRaw(currentTestingMotor, 0);
    updateEncoder(currentTestingMotor);
    logTelemetryRow(currentTestingMotor, "FWD_COAST", 0, "Coasting to Stop");

    if (millis() - stepStartTimeMs >= COAST_DWELL_MS) {
      currentPwmStepIndex = 0;
      stepStartTimeMs = millis();
      diagState = DIAG_STATE_REV_RAMP;
    }
    delay(LOG_SAMPLE_INTERVAL_MS);
    break;
  }

  case DIAG_STATE_REV_RAMP: {
    int currentPwm = -PWM_RAMP_STEPS[currentPwmStepIndex];
    driveMotorRaw(currentTestingMotor, currentPwm);
    updateEncoder(currentTestingMotor);

    logTelemetryRow(currentTestingMotor, "REV_RAMP", currentPwm, "Ramping REV PWM");

    if (abs(currentRPM) > 0.5f && motorReport[currentTestingMotor].minRevPwmMoved == 0) {
      motorReport[currentTestingMotor].minRevPwmMoved = abs(currentPwm);
    }
    if (abs(currentRPM) > motorReport[currentTestingMotor].maxRevRPM) {
      motorReport[currentTestingMotor].maxRevRPM = abs(currentRPM);
    }

    if (millis() - stepStartTimeMs >= STEP_HOLD_MS) {
      currentPwmStepIndex++;
      stepStartTimeMs = millis();

      if (currentPwmStepIndex >= NUM_PWM_STEPS) {
        driveMotorRaw(currentTestingMotor, 0);
        motorReport[currentTestingMotor].totalCountsRev = encoderCount[currentTestingMotor] - motorReport[currentTestingMotor].totalCountsFwd;

        // Check if direction sign matched expectations (+PWM gave positive counts)
        if (motorReport[currentTestingMotor].totalCountsFwd > 100) {
          motorReport[currentTestingMotor].directionMatched = true;
        } else if (motorReport[currentTestingMotor].totalCountsFwd < -100) {
          motorReport[currentTestingMotor].directionMatched = false; // Inverted
        }

        Serial.printf("[DIAG] Motor %d REV Ramp Complete. Coasting...\n", currentTestingMotor);
        diagState = DIAG_STATE_REV_COAST;
      }
    }
    delay(LOG_SAMPLE_INTERVAL_MS);
    break;
  }

  case DIAG_STATE_REV_COAST: {
    driveMotorRaw(currentTestingMotor, 0);
    updateEncoder(currentTestingMotor);
    logTelemetryRow(currentTestingMotor, "REV_COAST", 0, "Coasting to Stop");

    if (millis() - stepStartTimeMs >= COAST_DWELL_MS) {
      printMotorSummaryReport(currentTestingMotor);

      // Check if testing all motors in sequence, or just one
      if (currentTestingMotor + 1 < MOTOR_COUNT && Serial.available() == 0) {
        prepareMotorTest(currentTestingMotor + 1);
      } else {
        stopAllMotors();
        printFullSuiteFinalReport();
        diagState = DIAG_STATE_ALL_FINISHED;
      }
    }
    delay(LOG_SAMPLE_INTERVAL_MS);
    break;
  }
  }
}
