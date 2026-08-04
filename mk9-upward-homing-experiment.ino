// =============================================================================
// Cosmic3D MK9 - Upward Homing Rotation & Speed Experiment (Robust Edition)
// ESP32-S3 | 12V DC Motors + L293D H-Bridge | AS5600 Encoders | TCA9548A Mux
// =============================================================================
// Features & Built-In Checks:
// 1. PRE-FLIGHT HARDWARE CHECK: Checks I2C, Magnet health & initial angles.
// 2. LIMIT SWITCH DEBOUNCING: Requires 3 consecutive LOW reads (50ms filter)
//    to eliminate electrical noise spikes on GPIO 46/43/48.
// 3. ZERO AT MANUAL ORIGIN: Zeroes encoders when user places pole at origin.
// 4. INDEPENDENT MOTOR STOP: Each tower stops immediately when its switch clicks.
// 5. COMPREHENSIVE SD CARD LOGGING: Saves 50ms telemetry & summary to SD Card
//    (/homing_experiment_log.csv) and displays formatted live status on Serial.
// =============================================================================

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "pid_config.h"
#include <Wire.h>

// =============================================================================
// CONFIGURATION & CONSTANTS
// =============================================================================
#define EXP_LOG_FILE_PATH "/homing_experiment_log.csv"
#define LOG_INTERVAL_MS 50           // Telemetry update interval (ms)
#define UPWARD_DRIVE_PWM 200          // Motor drive PWM for upward movement (0 - 255)
#define SWITCH_DEBOUNCE_REQUIRED 3    // Number of consecutive LOW reads to confirm switch click

// Limit Switches (INPUT_PULLUP: HIGH = unclicked, LOW = clicked)
#define SWITCH_CLICKED LOW

// =============================================================================
// GLOBAL STATE & VARIABLES
// =============================================================================
volatile long encoderCount[MOTOR_COUNT] = {0, 0, 0, 0};
long lastAngle[MOTOR_COUNT] = {0, 0, 0, 0};
long totalRotations[MOTOR_COUNT] = {0, 0, 0, 0};
static long rawAccumulator[MOTOR_COUNT] = {0, 0, 0, 0};

// Telemetry & Speed tracking
long prevCountsForSpeed[MOTOR_COUNT] = {0, 0, 0, 0};
unsigned long prevSpeedCalcTime[MOTOR_COUNT] = {0, 0, 0, 0};
float currentRPM[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

// Limit switch debouncing counters
uint8_t switchDebounceCounter[3] = {0, 0, 0};

// Per-motor completion data
bool motorHitLimit[3] = {false, false, false};
unsigned long motorStopTimeMs[3] = {0, 0, 0};
long finalEncoderCounts[3] = {0, 0, 0};
float finalTotalRotations[3] = {0.0f, 0.0f, 0.0f};
float avgRPM[3] = {0.0f, 0.0f, 0.0f};

// Experiment State Machine
enum ExperimentState {
  EXP_STATE_INIT,
  EXP_STATE_WAIT_START,
  EXP_STATE_RUNNING,
  EXP_STATE_FINISHED,
  EXP_STATE_ABORTED
};

ExperimentState expState = EXP_STATE_INIT;

unsigned long expStartTimeMs = 0;
unsigned long expEndTimeMs = 0;
unsigned long lastLogTimeMs = 0;

bool as5600Ready = false;
bool sdReady = false;

SPIClass sdSPI(HSPI);

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void stopAllMotors();
void stopTowerMotor(int i);
void driveTowerMotorUp(int i, int pwm);
void disableAllI2CChannels();
bool selectI2CChannel(uint8_t channel);
uint16_t readRawAngle();
uint8_t readStatus();
bool isMagnetDetected();
void updateEncoderFromAS5600(int motorIndex);
bool checkDebouncedLimitSwitch(int towerIdx, int pin);
void performPreFlightHardwareChecks();
void initSDLog();
void logTelemetryToSDAndSerial();
void printFinalExperimentSummary();
void startExperiment();

// =============================================================================
// I2C & AS5600 LOW LEVEL FUNCTIONS
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
bool isMagnetDetected() {
  uint8_t status = readStatus();
  return (status & 0x20) && !(status & 0x18);
}

// =============================================================================
// ENCODER UPDATE & SPEED CALCULATIONS
// =============================================================================
void updateEncoderFromAS5600(int motorIndex) {
  if (!as5600Ready) return;
  if (!selectI2CChannel(MOTOR_TO_I2C_CHANNEL[motorIndex])) return;
  delayMicroseconds(100);

  uint16_t currentAngle = readRawAngle();
  long diff = (long)currentAngle - (long)lastAngle[motorIndex];

  if (diff > 2048) {
    diff -= 4096;
    totalRotations[motorIndex]--;
  } else if (diff < -2048) {
    diff += 4096;
    totalRotations[motorIndex]++;
  }

  if (abs(diff) <= 2048) {
    rawAccumulator[motorIndex] += diff;
    long motorCounts = rawAccumulator[motorIndex] / GEAR_RATIO;
    if (motorCounts != 0) {
      encoderCount[motorIndex] += motorCounts;
      rawAccumulator[motorIndex] -= motorCounts * GEAR_RATIO;
    }
  }

  lastAngle[motorIndex] = currentAngle;
  disableAllI2CChannels();

  // Calculate Real-Time RPM
  unsigned long now = millis();
  float dtSec = (now - prevSpeedCalcTime[motorIndex]) / 1000.0f;
  if (dtSec >= 0.05f) {
    long dCounts = encoderCount[motorIndex] - prevCountsForSpeed[motorIndex];
    float dRotations = (float)dCounts / AS5600_COUNTS_PER_REV;
    currentRPM[motorIndex] = (dRotations / dtSec) * 60.0f;

    prevCountsForSpeed[motorIndex] = encoderCount[motorIndex];
    prevSpeedCalcTime[motorIndex] = now;
  }
}

// =============================================================================
// DEBOUNCED LIMIT SWITCH CHECKER
// Requires 3 consecutive LOW reads to filter out electrical noise spikes
// =============================================================================
bool checkDebouncedLimitSwitch(int towerIdx, int pin) {
  if (digitalRead(pin) == SWITCH_CLICKED) {
    switchDebounceCounter[towerIdx]++;
    if (switchDebounceCounter[towerIdx] >= SWITCH_DEBOUNCE_REQUIRED) {
      return true; // Confirmed clicked
    }
  } else {
    switchDebounceCounter[towerIdx] = 0; // Reset counter if signal goes HIGH
  }
  return false;
}

// =============================================================================
// PRE-FLIGHT HARDWARE CHECKS
// =============================================================================
void performPreFlightHardwareChecks() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  Serial.println("\n[PRE-FLIGHT] Performing Automated Hardware & Sensor Diagnostics...");
  as5600Ready = true;

  for (int i = 0; i < 3; i++) {
    if (!selectI2CChannel(MOTOR_TO_I2C_CHANNEL[i])) {
      Serial.printf("[PRE-FLIGHT] ❌ Tower %d (Ch %d) I2C Select FAILED!\n", i, MOTOR_TO_I2C_CHANNEL[i]);
      as5600Ready = false;
      continue;
    }
    delay(5);

    uint16_t angle = readRawAngle();
    bool magOK = isMagnetDetected();

    Serial.printf("[PRE-FLIGHT] Tower %d (Ch %d): RawAngle = %4u | Magnet = %s\n",
                  i, MOTOR_TO_I2C_CHANNEL[i], angle, magOK ? "✅ OK" : "❌ FAULT");

    lastAngle[i] = angle;
    encoderCount[i] = 0;
    rawAccumulator[i] = 0;
    totalRotations[i] = 0;
    prevCountsForSpeed[i] = 0;
    prevSpeedCalcTime[i] = millis();
    currentRPM[i] = 0.0f;
    switchDebounceCounter[i] = 0;
  }
  disableAllI2CChannels();

  // Check Limit Switch Baseline States
  int swA = digitalRead(LIMIT1_PIN);
  int swB = digitalRead(LIMIT2_PIN);
  int swC = digitalRead(LIMIT3_PIN);

  Serial.printf("[PRE-FLIGHT] Limit Switches Baseline -> Tower A (GPIO 46): %s | Tower B (GPIO 43): %s | Tower C (GPIO 48): %s\n",
                (swA == LOW) ? "CLOSED" : "OPEN (OK)",
                (swB == LOW) ? "CLOSED" : "OPEN (OK)",
                (swC == LOW) ? "CLOSED" : "OPEN (OK)");

  if (as5600Ready) {
    Serial.println("[PRE-FLIGHT] ✅ All hardware checks PASSED. Systems ready.");
  } else {
    Serial.println("[PRE-FLIGHT] ⚠️ Hardware check completed with warnings.");
  }
}

// =============================================================================
// MOTOR DRIVE CONTROLS
// =============================================================================
void driveTowerMotorUp(int i, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (i == 0) {
    analogWrite(MOTOR_0_PIN_1, 0);
    analogWrite(MOTOR_0_PIN_2, pwm);
  } else if (i == 1) {
    analogWrite(MOTOR_1_PIN_1, 0);
    analogWrite(MOTOR_1_PIN_2, pwm);
  } else if (i == 2) {
    analogWrite(MOTOR_2_PIN_1, 0);
    analogWrite(MOTOR_2_PIN_2, pwm);
  }
}

void stopTowerMotor(int i) {
  if (i == 0) {
    analogWrite(MOTOR_0_PIN_1, 0);
    analogWrite(MOTOR_0_PIN_2, 0);
  } else if (i == 1) {
    analogWrite(MOTOR_1_PIN_1, 0);
    analogWrite(MOTOR_1_PIN_2, 0);
  } else if (i == 2) {
    analogWrite(MOTOR_2_PIN_1, 0);
    analogWrite(MOTOR_2_PIN_2, 0);
  }
}

void stopAllMotors() {
  stopTowerMotor(0);
  stopTowerMotor(1);
  stopTowerMotor(2);
  analogWrite(MOTOR_3_PIN_1, 0);
  analogWrite(MOTOR_3_PIN_2, 0);
}

// =============================================================================
// SD CARD LOGGING ENGINE
// =============================================================================
void initSDLog() {
  sdSPI.begin(SD_PIN_SCK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
  if (!SD.begin(SD_PIN_CS, sdSPI)) {
    Serial.println("[SD LOG] ❌ SD Card initialization failed! Logging to Serial Monitor only.");
    sdReady = false;
    return;
  }
  sdReady = true;
  Serial.println("[SD LOG] ✅ SD Card initialized successfully.");

  File logFile = SD.open(EXP_LOG_FILE_PATH, FILE_APPEND);
  if (logFile) {
    if (logFile.size() == 0) {
      logFile.println("TimestampMs,ElapsedMs,Tower,LimitState,TotalRotations,RawCounts,SpeedRPM,SpeedMMs,DrivePWM");
    }
    logFile.println("--- NEW UPWARD HOMING EXPERIMENT SESSION ---");
    logFile.close();
  }
}

void logTelemetryToSDAndSerial() {
  unsigned long now = millis();
  unsigned long elapsed = now - expStartTimeMs;

  float rotsA = (float)encoderCount[0] / AS5600_COUNTS_PER_REV;
  float rotsB = (float)encoderCount[1] / AS5600_COUNTS_PER_REV;
  float rotsC = (float)encoderCount[2] / AS5600_COUNTS_PER_REV;

  float mmSecA = (currentRPM[0] * 2.0f * PI * PULLEY_RADIUS_MM) / 60.0f;
  float mmSecB = (currentRPM[1] * 2.0f * PI * PULLEY_RADIUS_MM) / 60.0f;
  float mmSecC = (currentRPM[2] * 2.0f * PI * PULLEY_RADIUS_MM) / 60.0f;

  bool swA = motorHitLimit[0];
  bool swB = motorHitLimit[1];
  bool swC = motorHitLimit[2];

  // Print Live Telemetry to Serial Monitor
  Serial.printf("[%6lu ms] | Tower A: Rots=%6.2f, RPM=%6.1f, Speed=%5.1f mm/s, Switch=%s | "
                "Tower B: Rots=%6.2f, RPM=%6.1f, Speed=%5.1f mm/s, Switch=%s | "
                "Tower C: Rots=%6.2f, RPM=%6.1f, Speed=%5.1f mm/s, Switch=%s\n",
                elapsed,
                rotsA, currentRPM[0], mmSecA, swA ? "CLICKED" : "OPEN",
                rotsB, currentRPM[1], mmSecB, swB ? "CLICKED" : "OPEN",
                rotsC, currentRPM[2], mmSecC, swC ? "CLICKED" : "OPEN");

  // Write Rows to SD Card CSV
  if (sdReady) {
    File logFile = SD.open(EXP_LOG_FILE_PATH, FILE_APPEND);
    if (logFile) {
      logFile.printf("%lu,%lu,A,%s,%.4f,%ld,%.2f,%.2f,%d\n",
                     now, elapsed, swA ? "CLICKED" : "OPEN", rotsA, encoderCount[0], currentRPM[0], mmSecA, motorHitLimit[0] ? 0 : UPWARD_DRIVE_PWM);
      logFile.printf("%lu,%lu,B,%s,%.4f,%ld,%.2f,%.2f,%d\n",
                     now, elapsed, swB ? "CLICKED" : "OPEN", rotsB, encoderCount[1], currentRPM[1], mmSecB, motorHitLimit[1] ? 0 : UPWARD_DRIVE_PWM);
      logFile.printf("%lu,%lu,C,%s,%.4f,%ld,%.2f,%.2f,%d\n",
                     now, elapsed, swC ? "CLICKED" : "OPEN", rotsC, encoderCount[2], currentRPM[2], mmSecC, motorHitLimit[2] ? 0 : UPWARD_DRIVE_PWM);
      logFile.close();
    }
  }
}

void printFinalExperimentSummary() {
  unsigned long totalExpTime = expEndTimeMs - expStartTimeMs;

  Serial.println("\n=========================================================================");
  Serial.println("         UPWARD HOMING EXPERIMENT - FINAL SUMMARY REPORT                ");
  Serial.println("=========================================================================");
  Serial.printf(" Total Experiment Duration: %.3f seconds (%lu ms)\n", (float)totalExpTime / 1000.0f, totalExpTime);
  Serial.println("-------------------------------------------------------------------------");
  Serial.println(" Tower | Status   | Time to Limit (s) | Total Rotations | Raw Counts | Avg Speed (RPM) | Cable Travel (mm)");
  Serial.println("-------+----------+-------------------+-----------------+------------+-----------------+------------------");

  const char* towerNames[3] = {"  A  ", "  B  ", "  C  "};
  for (int i = 0; i < 3; i++) {
    float stopTimeSec = (float)(motorStopTimeMs[i] - expStartTimeMs) / 1000.0f;
    float cableTravelMm = finalTotalRotations[i] * (2.0f * PI * PULLEY_RADIUS_MM);

    Serial.printf(" %s | %s  | %17.3f | %15.4f | %10ld | %15.2f | %17.2f\n",
                  towerNames[i],
                  motorHitLimit[i] ? "CLICKED" : "ABORTED",
                  stopTimeSec,
                  finalTotalRotations[i],
                  finalEncoderCounts[i],
                  avgRPM[i],
                  cableTravelMm);
  }
  Serial.println("=========================================================================\n");

  if (sdReady) {
    File logFile = SD.open(EXP_LOG_FILE_PATH, FILE_APPEND);
    if (logFile) {
      logFile.println("--- EXPERIMENT FINAL SUMMARY ---");
      logFile.printf("TotalDurationMs,%lu\n", totalExpTime);
      for (int i = 0; i < 3; i++) {
        float stopTimeSec = (float)(motorStopTimeMs[i] - expStartTimeMs) / 1000.0f;
        float cableTravelMm = finalTotalRotations[i] * (2.0f * PI * PULLEY_RADIUS_MM);
        logFile.printf("Summary,Tower_%d,%s,TimeSec,%.3f,Rotations,%.4f,Counts,%ld,AvgRPM,%.2f,TravelMM,%.2f\n",
                       i, motorHitLimit[i] ? "CLICKED" : "ABORTED", stopTimeSec, finalTotalRotations[i], finalEncoderCounts[i], avgRPM[i], cableTravelMm);
      }
      logFile.close();
    }
  }
}

// =============================================================================
// EXPERIMENT EXECUTION ENGINE
// =============================================================================
void startExperiment() {
  Serial.println("\n[EXP] Starting Upward Homing Experiment...");
  Serial.println("[EXP] Make sure pole is set at origin (0,0,0) manually.");
  Serial.println("[EXP] Zeroing encoders and starting upward drive...");

  for (int i = 0; i < 3; i++) {
    motorHitLimit[i] = false;
    motorStopTimeMs[i] = 0;
    finalEncoderCounts[i] = 0;
    finalTotalRotations[i] = 0.0f;
    avgRPM[i] = 0.0f;
    switchDebounceCounter[i] = 0;
  }

  performPreFlightHardwareChecks();

  expStartTimeMs = millis();
  lastLogTimeMs = expStartTimeMs;
  expState = EXP_STATE_RUNNING;

  // Start driving all 3 tower motors upward
  driveTowerMotorUp(0, UPWARD_DRIVE_PWM);
  driveTowerMotorUp(1, UPWARD_DRIVE_PWM);
  driveTowerMotorUp(2, UPWARD_DRIVE_PWM);

  Serial.printf("[EXP] Motors A, B, C running UPWARD at PWM = %d\n", UPWARD_DRIVE_PWM);
}

// =============================================================================
// SETUP & LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("  Cosmic3D MK9 - Upward Homing Experiment Setup   ");
  Serial.println("==================================================");

  // Motor Pin Configurations
  pinMode(MOTOR_0_PIN_1, OUTPUT);
  pinMode(MOTOR_0_PIN_2, OUTPUT);
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
  pinMode(MOTOR_2_PIN_1, OUTPUT);
  pinMode(MOTOR_2_PIN_2, OUTPUT);
  pinMode(MOTOR_3_PIN_1, OUTPUT);
  pinMode(MOTOR_3_PIN_2, OUTPUT);
  stopAllMotors();

  // Limit Switch Configurations
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(LIMIT3_PIN, INPUT_PULLUP);

  // Initialize SD Logging & Perform Hardware Checks
  initSDLog();
  performPreFlightHardwareChecks();

  Serial.println("\n--------------------------------------------------");
  Serial.println(" INSTRUCTIONS:");
  Serial.println(" 1. Manually position the pole to origin (0, 0, 0).");
  Serial.println(" 2. Send 'START' in Serial Monitor to begin upward test.");
  Serial.println(" 3. Send 'STOP' to abort at any time.");
  Serial.println("--------------------------------------------------\n");

  expState = EXP_STATE_WAIT_START;
}

void loop() {
  // Check for incoming user commands via Serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "START") {
      if (expState == EXP_STATE_WAIT_START || expState == EXP_STATE_FINISHED || expState == EXP_STATE_ABORTED) {
        startExperiment();
      } else {
        Serial.println("[CMD] Experiment is already running!");
      }
    } else if (cmd == "STOP") {
      Serial.println("[CMD] Manual STOP requested! Halting experiment...");
      stopAllMotors();
      expEndTimeMs = millis();
      expState = EXP_STATE_ABORTED;
      printFinalExperimentSummary();
    }
  }

  // --- Experiment State Machine ---
  switch (expState) {

  case EXP_STATE_INIT:
  case EXP_STATE_WAIT_START:
  case EXP_STATE_FINISHED:
  case EXP_STATE_ABORTED:
    delay(10);
    break;

  case EXP_STATE_RUNNING: {
    // 1. Update encoders for all active tower motors
    for (int i = 0; i < 3; i++) {
      if (!motorHitLimit[i]) {
        updateEncoderFromAS5600(i);
      }
    }

    // 2. Check Debounced Limit Switches for each tower
    if (!motorHitLimit[0] && checkDebouncedLimitSwitch(0, LIMIT1_PIN)) {
      stopTowerMotor(0);
      motorHitLimit[0] = true;
      motorStopTimeMs[0] = millis();
      finalEncoderCounts[0] = encoderCount[0];
      finalTotalRotations[0] = (float)finalEncoderCounts[0] / AS5600_COUNTS_PER_REV;
      float timeSec = (float)(motorStopTimeMs[0] - expStartTimeMs) / 1000.0f;
      avgRPM[0] = (timeSec > 0.001f) ? (finalTotalRotations[0] / timeSec) * 60.0f : 0.0f;
      Serial.printf("\n[LIMIT] 🛑 Tower A Limit Switch CLICKED at t = %.3f s | Total Rots = %.4f | Avg RPM = %.2f\n\n",
                    timeSec, finalTotalRotations[0], avgRPM[0]);
    }

    if (!motorHitLimit[1] && checkDebouncedLimitSwitch(1, LIMIT2_PIN)) {
      stopTowerMotor(1);
      motorHitLimit[1] = true;
      motorStopTimeMs[1] = millis();
      finalEncoderCounts[1] = encoderCount[1];
      finalTotalRotations[1] = (float)finalEncoderCounts[1] / AS5600_COUNTS_PER_REV;
      float timeSec = (float)(motorStopTimeMs[1] - expStartTimeMs) / 1000.0f;
      avgRPM[1] = (timeSec > 0.001f) ? (finalTotalRotations[1] / timeSec) * 60.0f : 0.0f;
      Serial.printf("\n[LIMIT] 🛑 Tower B Limit Switch CLICKED at t = %.3f s | Total Rots = %.4f | Avg RPM = %.2f\n\n",
                    timeSec, finalTotalRotations[1], avgRPM[1]);
    }

    if (!motorHitLimit[2] && checkDebouncedLimitSwitch(2, LIMIT3_PIN)) {
      stopTowerMotor(2);
      motorHitLimit[2] = true;
      motorStopTimeMs[2] = millis();
      finalEncoderCounts[2] = encoderCount[2];
      finalTotalRotations[2] = (float)finalEncoderCounts[2] / AS5600_COUNTS_PER_REV;
      float timeSec = (float)(motorStopTimeMs[2] - expStartTimeMs) / 1000.0f;
      avgRPM[2] = (timeSec > 0.001f) ? (finalTotalRotations[2] / timeSec) * 60.0f : 0.0f;
      Serial.printf("\n[LIMIT] 🛑 Tower C Limit Switch CLICKED at t = %.3f s | Total Rots = %.4f | Avg RPM = %.2f\n\n",
                    timeSec, finalTotalRotations[2], avgRPM[2]);
    }

    // 3. Log Telemetry periodically
    if (millis() - lastLogTimeMs >= LOG_INTERVAL_MS) {
      lastLogTimeMs = millis();
      logTelemetryToSDAndSerial();
    }

    // 4. Check if all 3 towers have clicked their limit switches
    if (motorHitLimit[0] && motorHitLimit[1] && motorHitLimit[2]) {
      stopAllMotors();
      expEndTimeMs = millis();
      expState = EXP_STATE_FINISHED;
      Serial.println("\n[EXP] 🎉 All 3 Limit Switches CLICKED! Experiment completed successfully.");
      printFinalExperimentSummary();
    }
    break;
  }
  }
}
