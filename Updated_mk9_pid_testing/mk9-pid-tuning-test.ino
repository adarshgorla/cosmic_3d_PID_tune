// =============================================================================
// Cosmic3D MK9 - Step-by-Step PID Tuning & Diagnostic Test Suite
// ESP32-S3 | 12V 60RPM Motor | L293D H-Bridge | AS5600 Encoder
// =============================================================================

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "pid_config.h"
#include <Wire.h>

// =============================================================================
// GLOBAL STATE & VARIABLES
// =============================================================================
// Encoder & Motor variables
volatile long encoderCount[MOTOR_COUNT] = {0, 0, 0, 0};
volatile long setpoint[MOTOR_COUNT] = {0, 0, 0, 0};
long lastAngle[MOTOR_COUNT] = {0, 0, 0, 0};
long totalRotations[MOTOR_COUNT] = {0, 0, 0, 0};
static long rawAccumulator[MOTOR_COUNT] = {0, 0, 0, 0};

// Working PID Gains (Initialized from pid_config.h)
float Kp[MOTOR_COUNT] = {DEFAULT_KP[0], DEFAULT_KP[1], DEFAULT_KP[2],
                         DEFAULT_KP[3]};
float Ki[MOTOR_COUNT] = {DEFAULT_KI[0], DEFAULT_KI[1], DEFAULT_KI[2],
                         DEFAULT_KI[3]};
float Kd[MOTOR_COUNT] = {DEFAULT_KD[0], DEFAULT_KD[1], DEFAULT_KD[2],
                         DEFAULT_KD[3]};

long prevError[MOTOR_COUNT] = {0, 0, 0, 0};
float integral[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastTimePID = 0;

// Cartesian Position Tracking
float currentX = HOME_X;
float currentY = HOME_Y;
float currentZ = HOME_Z;

float homeCableLength[3] = {0.0f, 0.0f, 0.0f};
bool homeCableLengthReady = false;
bool as5600Ready = false;
bool sdReady = false;

// Test Engine State Machine
enum TestState {
  STATE_IDLE,
  STATE_TEST_X_OUT,
  STATE_TEST_X_RETURN,
  STATE_TEST_Y_OUT,
  STATE_TEST_Y_RETURN,
  STATE_TEST_Z_OUT,
  STATE_TEST_Z_RETURN,
  STATE_TEST_XY_OUT,
  STATE_TEST_XY_RETURN,
  STATE_TEST_YZ_OUT,
  STATE_TEST_YZ_RETURN,
  STATE_TEST_XZ_OUT,
  STATE_TEST_XZ_RETURN,
  STATE_TEST_XYZ_OUT,
  STATE_TEST_XYZ_RETURN,
  STATE_HOLD_DWELL
};

TestState testState = STATE_IDLE;
TestState nextStateAfterDwell = STATE_IDLE;
unsigned long dwellStartTime = 0;
unsigned long lastLogTime = 0;

SPIClass sdSPI(HSPI);

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void stopAllMotors();
void disableAllI2CChannels();
bool selectI2CChannel(uint8_t channel);
uint16_t readRawAngle();
void updateEncoderFromAS5600(int motorIndex);
float calculatePID(int i, long targetPosition, float dt);
void driveMotor(int i, float output);
bool convertXYZToEncoderTargets(float targetX, float targetY, float targetZ,
                                long &targetA, long &targetB, long &targetC);
void startTestStep(TestState newState, float targetX, float targetY,
                   float targetZ);
void logTelemetry(const char *statusMessage);
void logFault(const char *faultMessage);

// =============================================================================
// SD CARD LOGGING ENGINE
// =============================================================================
void initSDLog() {
  sdSPI.begin(SD_PIN_SCK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
  if (!SD.begin(SD_PIN_CS, sdSPI)) {
    Serial.println("[SD LOG] ❌ SD Card initialization failed! Telemetry will "
                   "be output to Serial only.");
    sdReady = false;
    return;
  }
  sdReady = true;
  Serial.println("[SD LOG] ✅ SD Card initialized successfully.");

  // Write CSV Header if creating a new file or appending
  File logFile = SD.open(LOG_FILE_PATH, FILE_APPEND);
  if (logFile) {
    if (logFile.size() == 0) {
      logFile.println(
          "TimestampMs,State,TargetX,TargetY,TargetZ,SetpointA,SetpointB,"
          "SetpointC,CountA,CountB,CountC,ErrA,ErrB,ErrC,Status");
    }
    logFile.println("--- NEW TEST SESSION STARTED ---");
    logFile.close();
  }
}

void logTelemetry(const char *statusMessage) {
  unsigned long now = millis();
  long errA = setpoint[0] - encoderCount[0];
  long errB = setpoint[1] - encoderCount[1];
  long errC = setpoint[2] - encoderCount[2];

  // Output to Serial Monitor
  Serial.printf(
      "[%lu ms] State:%d | TgtXYZ:(%.1f,%.1f,%.1f) | Setpoints:(%ld,%ld,%ld) | "
      "Counts:(%ld,%ld,%ld) | Errs:(%ld,%ld,%ld) | %s\n",
      now, (int)testState, currentX, currentY, currentZ, setpoint[0],
      setpoint[1], setpoint[2], encoderCount[0], encoderCount[1],
      encoderCount[2], errA, errB, errC, statusMessage);

  // Write to SD Log file
  if (sdReady) {
    File logFile = SD.open(LOG_FILE_PATH, FILE_APPEND);
    if (logFile) {
      logFile.printf(
          "%lu,%d,%.2f,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%s\n", now,
          (int)testState, currentX, currentY, currentZ, setpoint[0],
          setpoint[1], setpoint[2], encoderCount[0], encoderCount[1],
          encoderCount[2], errA, errB, errC, statusMessage);
      logFile.close();
    }
  }
}

void logFault(const char *faultMessage) {
  Serial.printf("[FAULT ALERT] ⚠️ %s\n", faultMessage);
  if (sdReady) {
    File logFile = SD.open(LOG_FILE_PATH, FILE_APPEND);
    if (logFile) {
      logFile.printf("%lu,%d,0,0,0,0,0,0,0,0,0,0,0,0,FAULT: %s\n", millis(),
                     (int)testState, faultMessage);
      logFile.close();
    }
  }
}

// =============================================================================
// I2C & AS5600 ENCODER SUBSYSTEM
// =============================================================================
bool selectI2CChannel(uint8_t channel) {
  if (channel > 7)
    return false;
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
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return 0;
  if (Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2, (uint8_t) true) >=
      2) {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

uint16_t readRawAngle() { return readAS5600Register16(AS5600_RAW_ANGLE_H); }

void updateEncoderFromAS5600(int motorIndex) {
  if (!as5600Ready)
    return;
  if (!selectI2CChannel(MOTOR_TO_I2C_CHANNEL[motorIndex]))
    return;
  delayMicroseconds(500);

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
    logFault("Spurious encoder jump detected!");
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

void initAS5600() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  Serial.println("[I2C] Scanning for TCA9548A Multiplexer at 0x70...");

  Wire.beginTransmission(TCA9548A_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println("[I2C] ❌ TCA9548A Multiplexer not found!");
    as5600Ready = false;
    return;
  }
  as5600Ready = true;
  Serial.println("[I2C] ✅ TCA9548A Multiplexer found.");

  for (int i = 0; i < MOTOR_COUNT; i++) {
    selectI2CChannel(MOTOR_TO_I2C_CHANNEL[i]);
    delay(5);
    lastAngle[i] = readRawAngle();
    encoderCount[i] = 0;
    rawAccumulator[i] = 0;
    totalRotations[i] = 0;
    Serial.printf("[AS5600] Motor %d (Ch %d) Initial Angle: %ld\n", i,
                  MOTOR_TO_I2C_CHANNEL[i], lastAngle[i]);
  }
  disableAllI2CChannels();
}

// =============================================================================
// INVERSE KINEMATICS (IK)
// =============================================================================
bool calculateIK(float nozzleX, float nozzleY, float nozzleZ, float &cableA,
                 float &cableB, float &cableC) {
  const float EFFECTOR_RADIUS = 22.92f;
  const float NOZZLE_TO_ATTACHMENT = 124.403f;
  const float PIVOT_X = 0.0f, PIVOT_Y = 0.0f, PIVOT_Z = 735.0f;

  float dx = nozzleX - PIVOT_X;
  float dy = nozzleY - PIVOT_Y;
  float dz = nozzleZ - PIVOT_Z;
  float mag = sqrtf(dx * dx + dy * dy + dz * dz);
  if (mag < 0.0001f)
    return false;

  float ux = dx / mag, uy = dy / mag, uz = dz / mag;
  float centerX = nozzleX - ux * NOZZLE_TO_ATTACHMENT;
  float centerY = nozzleY - uy * NOZZLE_TO_ATTACHMENT;
  float centerZ = nozzleZ - uz * NOZZLE_TO_ATTACHMENT;

  float ax = centerX + (TOWER_A_X /
                        sqrtf(TOWER_A_X * TOWER_A_X + TOWER_A_Y * TOWER_A_Y)) *
                           EFFECTOR_RADIUS;
  float ay = centerY + (TOWER_A_Y /
                        sqrtf(TOWER_A_X * TOWER_A_X + TOWER_A_Y * TOWER_A_Y)) *
                           EFFECTOR_RADIUS;
  float az = centerZ;

  float bx = centerX + (TOWER_B_X /
                        sqrtf(TOWER_B_X * TOWER_B_X + TOWER_B_Y * TOWER_B_Y)) *
                           EFFECTOR_RADIUS;
  float by = centerY + (TOWER_B_Y /
                        sqrtf(TOWER_B_X * TOWER_B_X + TOWER_B_Y * TOWER_B_Y)) *
                           EFFECTOR_RADIUS;
  float bz = centerZ;

  float cx = centerX + (TOWER_C_X /
                        sqrtf(TOWER_C_X * TOWER_C_X + TOWER_C_Y * TOWER_C_Y)) *
                           EFFECTOR_RADIUS;
  float cy = centerY + (TOWER_C_Y /
                        sqrtf(TOWER_C_X * TOWER_C_X + TOWER_C_Y * TOWER_C_Y)) *
                           EFFECTOR_RADIUS;
  float cz = centerZ;

  cableA = sqrtf((ax - TOWER_A_X) * (ax - TOWER_A_X) +
                 (ay - TOWER_A_Y) * (ay - TOWER_A_Y) +
                 (az - TOWER_A_Z) * (az - TOWER_A_Z));
  cableB = sqrtf((bx - TOWER_B_X) * (bx - TOWER_B_X) +
                 (by - TOWER_B_Y) * (by - TOWER_B_Y) +
                 (bz - TOWER_B_Z) * (bz - TOWER_B_Z));
  cableC = sqrtf((cx - TOWER_C_X) * (cx - TOWER_C_X) +
                 (cy - TOWER_C_Y) * (cy - TOWER_C_Y) +
                 (cz - TOWER_C_Z) * (cz - TOWER_C_Z));
  return true;
}

bool convertXYZToEncoderTargets(float targetX, float targetY, float targetZ,
                                long &targetA, long &targetB, long &targetC) {
  if (!homeCableLengthReady) {
    float hA = 0, hB = 0, hC = 0;
    if (!calculateIK(HOME_X, HOME_Y, HOME_Z, hA, hB, hC))
      return false;
    homeCableLength[0] = hA;
    homeCableLength[1] = hB;
    homeCableLength[2] = hC;
    homeCableLengthReady = true;
  }

  float cA = 0, cB = 0, cC = 0;
  if (!calculateIK(targetX, targetY, targetZ, cA, cB, cC))
    return false;

  float deltaA = cA - homeCableLength[0];
  float deltaB = cB - homeCableLength[1];
  float deltaC = cC - homeCableLength[2];

  targetA = (long)lroundf(deltaA * COUNTS_PER_MM);
  targetB = (long)lroundf(deltaB * COUNTS_PER_MM);
  targetC = (long)lroundf(deltaC * COUNTS_PER_MM);
  return true;
}

// =============================================================================
// PID & MOTOR DRIVING ENGINE
// =============================================================================
float calculatePID(int i, long targetPosition, float dt) {
  long error = targetPosition - encoderCount[i];
  float P = Kp[i] * (float)error;

  integral[i] += (float)error * dt;
  integral[i] = constrain(integral[i], -1000.0f, 1000.0f);
  float I = Ki[i] * integral[i];

  float D = Kd[i] * ((float)(error - prevError[i]) / dt);
  prevError[i] = error;

  return P + I + D;
}

void driveMotor(int i, float output) {
  int pwm = (int)abs(output);
  int floor = (i == 3) ? PWM_MIN_FLOOR_EXTRUDER : PWM_MIN_FLOOR;

  if (pwm > 0 && pwm < floor) {
    pwm = floor;
  }
  if (pwm > 255)
    pwm = 255;

  if (i == 0) {
    if (output > 0) {
      analogWrite(MOTOR_0_PIN_1, 0);
      analogWrite(MOTOR_0_PIN_2, pwm);
    } else if (output < 0) {
      analogWrite(MOTOR_0_PIN_1, pwm);
      analogWrite(MOTOR_0_PIN_2, 0);
    } else {
      analogWrite(MOTOR_0_PIN_1, 0);
      analogWrite(MOTOR_0_PIN_2, 0);
    }
  } else if (i == 1) {
    if (output > 0) {
      analogWrite(MOTOR_1_PIN_1, 0);
      analogWrite(MOTOR_1_PIN_2, pwm);
    } else if (output < 0) {
      analogWrite(MOTOR_1_PIN_1, pwm);
      analogWrite(MOTOR_1_PIN_2, 0);
    } else {
      analogWrite(MOTOR_1_PIN_1, 0);
      analogWrite(MOTOR_1_PIN_2, 0);
    }
  } else if (i == 2) {
    if (output > 0) {
      analogWrite(MOTOR_2_PIN_1, 0);
      analogWrite(MOTOR_2_PIN_2, pwm);
    } else if (output < 0) {
      analogWrite(MOTOR_2_PIN_1, pwm);
      analogWrite(MOTOR_2_PIN_2, 0);
    } else {
      analogWrite(MOTOR_2_PIN_1, 0);
      analogWrite(MOTOR_2_PIN_2, 0);
    }
  } else if (i == 3) {
    if (output > 0) {
      analogWrite(MOTOR_3_PIN_1, 0);
      analogWrite(MOTOR_3_PIN_2, pwm);
    } else if (output < 0) {
      analogWrite(MOTOR_3_PIN_1, pwm);
      analogWrite(MOTOR_3_PIN_2, 0);
    } else {
      analogWrite(MOTOR_3_PIN_1, 0);
      analogWrite(MOTOR_3_PIN_2, 0);
    }
  }
}

void stopAllMotors() {
  analogWrite(MOTOR_0_PIN_1, 0);
  analogWrite(MOTOR_0_PIN_2, 0);
  analogWrite(MOTOR_1_PIN_1, 0);
  analogWrite(MOTOR_1_PIN_2, 0);
  analogWrite(MOTOR_2_PIN_1, 0);
  analogWrite(MOTOR_2_PIN_2, 0);
  analogWrite(MOTOR_3_PIN_1, 0);
  analogWrite(MOTOR_3_PIN_2, 0);
}

// =============================================================================
// TEST STEP ENGINE
// =============================================================================
void startTestStep(TestState newState, float targetX, float targetY,
                   float targetZ) {
  long targetA = 0, targetB = 0, targetC = 0;
  if (!convertXYZToEncoderTargets(targetX, targetY, targetZ, targetA, targetB,
                                  targetC)) {
    logFault("IK calculation failed for test step target!");
    return;
  }

  currentX = targetX;
  currentY = targetY;
  currentZ = targetZ;

  setpoint[0] = targetA;
  setpoint[1] = targetB;
  setpoint[2] = targetC;
  setpoint[3] = 0; // Extruder stationary during axis tests

  lastTimePID = millis();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    integral[i] = 0.0f;
    prevError[i] = 0;
  }

  testState = newState;
  logTelemetry("START_STEP");
}

void handleSerialCommands() {
  if (!Serial.available())
    return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  input.toLowerCase();

  if (input == "1" || input == "x") {
    Serial.println("\n[TEST ROUTINE] Starting Test 1: X-Axis Only (+20mm)");
    nextStateAfterDwell = STATE_TEST_X_RETURN;
    startTestStep(STATE_TEST_X_OUT, HOME_X + TEST_MOVE_DIST_MM, HOME_Y, HOME_Z);
  } else if (input == "2" || input == "y") {
    Serial.println("\n[TEST ROUTINE] Starting Test 2: Y-Axis Only (+20mm)");
    nextStateAfterDwell = STATE_TEST_Y_RETURN;
    startTestStep(STATE_TEST_Y_OUT, HOME_X, HOME_Y + TEST_MOVE_DIST_MM, HOME_Z);
  } else if (input == "3" || input == "z") {
    Serial.println("\n[TEST ROUTINE] Starting Test 3: Z-Axis Only (+20mm)");
    nextStateAfterDwell = STATE_TEST_Z_RETURN;
    startTestStep(STATE_TEST_Z_OUT, HOME_X, HOME_Y, HOME_Z + TEST_MOVE_DIST_MM);
  } else if (input == "4" || input == "xy") {
    Serial.println("\n[TEST ROUTINE] Starting Test 4: X-Y Combined (+20mm)");
    nextStateAfterDwell = STATE_TEST_XY_RETURN;
    startTestStep(STATE_TEST_XY_OUT, HOME_X + TEST_MOVE_DIST_MM,
                  HOME_Y + TEST_MOVE_DIST_MM, HOME_Z);
  } else if (input == "5" || input == "yz") {
    Serial.println("\n[TEST ROUTINE] Starting Test 5: Y-Z Combined (+20mm)");
    nextStateAfterDwell = STATE_TEST_YZ_RETURN;
    startTestStep(STATE_TEST_YZ_OUT, HOME_X, HOME_Y + TEST_MOVE_DIST_MM,
                  HOME_Z + TEST_MOVE_DIST_MM);
  } else if (input == "6" || input == "xz") {
    Serial.println("\n[TEST ROUTINE] Starting Test 6: X-Z Combined (+20mm)");
    nextStateAfterDwell = STATE_TEST_XZ_RETURN;
    startTestStep(STATE_TEST_XZ_OUT, HOME_X + TEST_MOVE_DIST_MM, HOME_Y,
                  HOME_Z + TEST_MOVE_DIST_MM);
  } else if (input == "7" || input == "xyz") {
    Serial.println("\n[TEST ROUTINE] Starting Test 7: X-Y-Z Combined (+20mm)");
    nextStateAfterDwell = STATE_TEST_XYZ_RETURN;
    startTestStep(STATE_TEST_XYZ_OUT, HOME_X + TEST_MOVE_DIST_MM,
                  HOME_Y + TEST_MOVE_DIST_MM, HOME_Z + TEST_MOVE_DIST_MM);
  } else if (input == "stop") {
    stopAllMotors();
    testState = STATE_IDLE;
    Serial.println("[COMMAND] ⏹️ EMERGENCY STOP EXECUTED!");
    logTelemetry("EMERGENCY_STOP");
  } else if (input.startsWith("p") || input.startsWith("i") ||
             input.startsWith("d")) {
    // Dynamic PID Gain Adjustment (e.g. p0=0.8 or d1=0.04)
    char gainType = input.charAt(0);
    int motorIdx = input.substring(1, 2).toInt();
    int eqPos = input.indexOf('=');
    if (eqPos > 0 && motorIdx >= 0 && motorIdx < MOTOR_COUNT) {
      float val = input.substring(eqPos + 1).toFloat();
      if (gainType == 'p')
        Kp[motorIdx] = val;
      if (gainType == 'i')
        Ki[motorIdx] = val;
      if (gainType == 'd')
        Kd[motorIdx] = val;
      Serial.printf("[PID TUNE] Motor %d updated: Kp=%.3f Ki=%.3f Kd=%.3f\n",
                    motorIdx, Kp[motorIdx], Ki[motorIdx], Kd[motorIdx]);
    }
  } else {
    Serial.println("\n--- COMMAND HELP MENU ---");
    Serial.println("Type '1' or 'x'   : Run X-Axis Test");
    Serial.println("Type '2' or 'y'   : Run Y-Axis Test");
    Serial.println("Type '3' or 'z'   : Run Z-Axis Test");
    Serial.println("Type '4' or 'xy'  : Run X-Y Combined Test");
    Serial.println("Type '5' or 'yz'  : Run Y-Z Combined Test");
    Serial.println("Type '6' or 'xz'  : Run X-Z Combined Test");
    Serial.println("Type '7' or 'xyz' : Run X-Y-Z Combined Test");
    Serial.println("Type 'stop'       : Emergency Stop All Motors");
    Serial.println("Type 'p0=0.85'    : Adjust Motor 0 Kp to 0.85 live");
    Serial.println("-------------------------");
  }
}

// =============================================================================
// ARDUINO SETUP & LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=======================================================");
  Serial.println(" Cosmic3D MK9 - Step-by-Step PID Tuning Suite ");
  Serial.println("=======================================================");

  pinMode(MOTOR_0_PIN_1, OUTPUT);
  pinMode(MOTOR_0_PIN_2, OUTPUT);
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
  pinMode(MOTOR_2_PIN_1, OUTPUT);
  pinMode(MOTOR_2_PIN_2, OUTPUT);
  pinMode(MOTOR_3_PIN_1, OUTPUT);
  pinMode(MOTOR_3_PIN_2, OUTPUT);
  stopAllMotors();

  initSDLog();
  initAS5600();

  Serial.println("\nSystem Ready. Type '1' for X-test, '2' for Y-test, etc. "
                 "Send any key for menu.");
}

void loop() {
  handleSerialCommands();

  // Read Encoders continuously
  for (int i = 0; i < MOTOR_COUNT; i++) {
    updateEncoderFromAS5600(i);
  }

  // Periodic SD & Serial Telemetry Logging (Every 20ms during motion)
  if (testState != STATE_IDLE && millis() - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = millis();
    logTelemetry("RUNNING");
  }

  // State Machine for Motion & Dwell Steps
  switch (testState) {
  case STATE_IDLE:
    stopAllMotors();
    break;

  case STATE_TEST_X_OUT:
  case STATE_TEST_Y_OUT:
  case STATE_TEST_Z_OUT:
  case STATE_TEST_XY_OUT:
  case STATE_TEST_YZ_OUT:
  case STATE_TEST_XZ_OUT:
  case STATE_TEST_XYZ_OUT: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    bool targetReached = true;
    for (int i = 0; i < 3; i++) {
      if (abs(setpoint[i] - encoderCount[i]) > MOTION_TOLERANCE_COUNTS) {
        targetReached = false;
      }
    }

    if (targetReached) {
      logTelemetry("TARGET_REACHED_OUTBOUND");
      dwellStartTime = millis();
      testState = STATE_HOLD_DWELL;
    } else {
      for (int i = 0; i < 3; i++) {
        driveMotor(i, calculatePID(i, setpoint[i], dt));
      }
    }
    break;
  }

  case STATE_HOLD_DWELL: {
    // Actively hold position at target using PID for 1.5 seconds before returning to origin
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastTimePID = currentTime;

    for (int i = 0; i < 3; i++) {
      driveMotor(i, calculatePID(i, setpoint[i], dt));
    }

    if (millis() - dwellStartTime >= 1500) {
      Serial.println(
          "[TEST ROUTINE] Dwell finished. Returning to Home origin (0,0,0)...");
      TestState prevNext = nextStateAfterDwell;
      nextStateAfterDwell = STATE_IDLE; // Next return step goes back to IDLE
      startTestStep(prevNext, HOME_X, HOME_Y, HOME_Z);
    }
    break;
  }

  case STATE_TEST_X_RETURN:
  case STATE_TEST_Y_RETURN:
  case STATE_TEST_Z_RETURN:
  case STATE_TEST_XY_RETURN:
  case STATE_TEST_YZ_RETURN:
  case STATE_TEST_XZ_RETURN:
  case STATE_TEST_XYZ_RETURN: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    bool targetReached = true;
    for (int i = 0; i < 3; i++) {
      if (abs(setpoint[i] - encoderCount[i]) > MOTION_TOLERANCE_COUNTS) {
        targetReached = false;
      }
    }

    if (targetReached) {
      stopAllMotors();
      logTelemetry("TEST_COMPLETE_RETURNED_HOME");
      Serial.println(
          "[TEST ROUTINE] ✅ Test complete! Returned safely to origin.");
      testState = STATE_IDLE;
    } else {
      for (int i = 0; i < 3; i++) {
        driveMotor(i, calculatePID(i, setpoint[i], dt));
      }
    }
    break;
  }
  }

  delay(1);
}
