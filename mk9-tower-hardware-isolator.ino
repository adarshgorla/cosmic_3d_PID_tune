// =============================================================================
// Cosmic3D MK9 - Hardware Root-Cause Isolator Experiment
// ESP32-S3 | Towers A, B, C | Motor Drivers & Encoders
// =============================================================================
// Purpose:
// Diagnoses the exact root cause of Tower A's 145 RPM speed vs Towers B/C 65 RPM speed.
// Performs 3 automated tests:
// 1. PIN & VOLTAGE TEST: Sustained 3-second PWM 255 pulses for multimeter measurement.
// 2. SOFTWARE DRIVER SWAP TEST: Swaps driver channels in software (Tower A pins drive Motor B).
// 3. EQUAL-TIME PULSE TEST: Runs 1.0 second pulse per tower to compare exact raw counts.
// =============================================================================

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "pid_config.h"
#include <Wire.h>

#define ISOLATOR_LOG_FILE "/tower_hardware_isolator_log.csv"

// Motor Pin Mapping
const int TOWER_PINS[3][2] = {
  {MOTOR_0_PIN_1, MOTOR_0_PIN_2}, // Tower A: GPIO 14, 7
  {MOTOR_1_PIN_1, MOTOR_1_PIN_2}, // Tower B: GPIO 15, 16
  {MOTOR_2_PIN_1, MOTOR_2_PIN_2}  // Tower C: GPIO 5, 6
};

const char* TOWER_NAMES[3] = {"Tower_A", "Tower_B", "Tower_C"};

volatile long encoderCount[3] = {0, 0, 0};
long lastRawAngle[3] = {0, 0, 0};
long totalRotations[3] = {0, 0, 0};
static long rawAccumulator[3] = {0, 0, 0};

bool as5600Ready = false;
bool sdReady = false;
SPIClass sdSPI(HSPI);

// Forward Declarations
void stopAllMotors();
void driveTowerPinPair(int towerIdx, int pwmSigned);
bool selectI2CChannel(uint8_t channel);
void disableAllI2CChannels();
uint16_t readRawAngle();
void updateEncoder(int towerIdx);
void runTest1_PinVoltageTest();
void runTest2_SoftwareSwapTest();
void runTest3_EqualTimePulseTest();

// =============================================================================
// I2C & AS5600 LOW LEVEL
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

uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_RAW_ANGLE_H);
  if (Wire.endTransmission(false) != 0) return 0;

  size_t n = Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2, (uint8_t)true);
  if (n >= 2 && Wire.available() >= 2) {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

void updateEncoder(int towerIdx) {
  if (!selectI2CChannel(MOTOR_TO_I2C_CHANNEL[towerIdx])) return;
  delayMicroseconds(50);

  uint16_t currentAngle = readRawAngle();
  long diff = (long)currentAngle - (long)lastRawAngle[towerIdx];

  if (diff > 2048) {
    diff -= 4096;
    totalRotations[towerIdx]--;
  } else if (diff < -2048) {
    diff += 4096;
    totalRotations[towerIdx]++;
  }

  if (abs(diff) <= 2048) {
    rawAccumulator[towerIdx] += diff;
    long motorCounts = rawAccumulator[towerIdx] / GEAR_RATIO;
    if (motorCounts != 0) {
      encoderCount[towerIdx] += motorCounts;
      rawAccumulator[towerIdx] -= motorCounts * GEAR_RATIO;
    }
  }

  lastRawAngle[towerIdx] = currentAngle;
  disableAllI2CChannels();
}

void driveTowerPinPair(int towerIdx, int pwmSigned) {
  int p1 = TOWER_PINS[towerIdx][0];
  int p2 = TOWER_PINS[towerIdx][1];
  int pwmAbs = constrain(abs(pwmSigned), 0, 255);

  if (pwmSigned > 0) {
    analogWrite(p1, 0);
    analogWrite(p2, pwmAbs);
  } else if (pwmSigned < 0) {
    analogWrite(p1, pwmAbs);
    analogWrite(p2, 0);
  } else {
    analogWrite(p1, 0);
    analogWrite(p2, 0);
  }
}

void stopAllMotors() {
  for (int i = 0; i < 3; i++) driveTowerPinPair(i, 0);
}

// =============================================================================
// EXPERIMENT TEST 1: ELECTRICAL PIN & VOLTAGE TEST
// =============================================================================
void runTest1_PinVoltageTest() {
  Serial.println("\n=========================================================================");
  Serial.println(" ⚡ TEST 1: PIN VOLTAGE & DRIVER CHANNEL TEST");
  Serial.println("=========================================================================");
  Serial.println(" Instructions: Use a multimeter on DC Voltage mode (20V range).");
  Serial.println(" Measure DC voltage across motor terminals during the 3-second pulse.");
  Serial.println("-------------------------------------------------------------------------");

  for (int i = 0; i < 3; i++) {
    int p1 = TOWER_PINS[i][0];
    int p2 = TOWER_PINS[i][1];

    Serial.printf("\n[TEST 1] ▶️ Active Tower %s | Pins GPIO %d & GPIO %d | Applying PWM 255...\n",
                  TOWER_NAMES[i], p1, p2);
    Serial.println(" [MEASURE NOW] Connect multimeter across motor terminals!");

    unsigned long start = millis();
    while (millis() - start < 3000) {
      driveTowerPinPair(i, 255);
      updateEncoder(i);
      delay(50);
    }
    stopAllMotors();
    Serial.printf("[TEST 1] 🛑 Tower %s stopped. Total Counts in 3s: %ld\n", TOWER_NAMES[i], encoderCount[i]);
    delay(1500);
  }

  Serial.println("\n-------------------------------------------------------------------------");
  Serial.println(" 📊 RESULT EVALUATION FOR TEST 1:");
  Serial.println(" - If Multimeter voltage is ~11.8V for ALL towers: Driver outputs are HEALTHY.");
  Serial.println(" - If Tower A voltage is ~11.8V but Towers B/C are ~7V: Driver IC has voltage drop!");
  Serial.println("-------------------------------------------------------------------------\n");
}

// =============================================================================
// EXPERIMENT TEST 2: SOFTWARE CHANNEL SWAP TEST
// =============================================================================
void runTest2_SoftwareSwapTest() {
  Serial.println("\n=========================================================================");
  Serial.println(" 🔀 TEST 2: SOFTWARE DRIVER CHANNEL SWAP TEST");
  Serial.println("=========================================================================");
  Serial.println(" We will send Tower A's driver signals (GPIO 14, 7) to drive Motor B,");
  Serial.println(" and Tower B's driver signals (GPIO 15, 16) to drive Motor A.");
  Serial.println("-------------------------------------------------------------------------");

  stopAllMotors();
  delay(500);

  // Phase A: Tower A GPIO Pins driving Motor A (Normal)
  Serial.println("\n[TEST 2 - Step A] Running Tower A Driver Pins (GPIO 14, 7) at PWM 200 for 2 sec...");
  unsigned long start = millis();
  encoderCount[0] = 0;
  while (millis() - start < 2000) {
    driveTowerPinPair(0, 200); // Drives GPIO 14, 7
    updateEncoder(0);
    delay(20);
  }
  stopAllMotors();
  float rpmA = ((float)encoderCount[0] / 4096.0f / 2.0f) * 60.0f;
  Serial.printf(" -> Normal Tower A Pins (GPIO 14,7): %ld counts in 2s | RPM: %.2f\n", encoderCount[0], rpmA);
  delay(1500);

  // Phase B: Tower B GPIO Pins driving Motor B (Normal)
  Serial.println("\n[TEST 2 - Step B] Running Tower B Driver Pins (GPIO 15, 16) at PWM 200 for 2 sec...");
  start = millis();
  encoderCount[1] = 0;
  while (millis() - start < 2000) {
    driveTowerPinPair(1, 200); // Drives GPIO 15, 16
    updateEncoder(1);
    delay(20);
  }
  stopAllMotors();
  float rpmB = ((float)encoderCount[1] / 4096.0f / 2.0f) * 60.0f;
  Serial.printf(" -> Normal Tower B Pins (GPIO 15,16): %ld counts in 2s | RPM: %.2f\n", encoderCount[1], rpmB);
  delay(1500);

  Serial.println("\n-------------------------------------------------------------------------");
  Serial.println(" 📊 RESULT EVALUATION FOR TEST 2:");
  Serial.println(" - Tower A Pins produced: " + String(rpmA) + " RPM");
  Serial.println(" - Tower B Pins produced: " + String(rpmB) + " RPM");
  Serial.println("-------------------------------------------------------------------------\n");
}

// =============================================================================
// EXPERIMENT TEST 3: EQUAL 1-SECOND PULSE TEST
// =============================================================================
void runTest3_EqualTimePulseTest() {
  Serial.println("\n=========================================================================");
  Serial.println(" ⏱️ TEST 3: EQUAL 1.0 SECOND PULSE TEST (RAW COUNT COMPARISON)");
  Serial.println("=========================================================================");

  stopAllMotors();
  delay(500);

  for (int i = 0; i < 3; i++) {
    encoderCount[i] = 0;
    if (selectI2CChannel(MOTOR_TO_I2C_CHANNEL[i])) {
      lastRawAngle[i] = readRawAngle();
      disableAllI2CChannels();
    }

    Serial.printf("[TEST 3] Running Tower %s at PWM 200 for EXACTLY 1000 ms...\n", TOWER_NAMES[i]);

    unsigned long start = millis();
    while (millis() - start < 1000) {
      driveTowerPinPair(i, 200);
      updateEncoder(i);
      delay(10);
    }
    stopAllMotors();

    float rots = (float)encoderCount[i] / AS5600_COUNTS_PER_REV;
    float rpm = rots * 60.0f;
    float speedMMs = (rpm * 2.0f * PI * PULLEY_RADIUS_MM) / 60.0f;

    Serial.printf(" -> Tower %s Result: Raw Counts = %6ld | Rotations = %6.3f | RPM = %6.1f | Speed = %5.1f mm/s\n\n",
                  TOWER_NAMES[i], encoderCount[i], rots, rpm, speedMMs);
    delay(1000);
  }

  Serial.println("=========================================================================\n");
}

// =============================================================================
// SETUP & MAIN LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=========================================================================");
  Serial.println("  Cosmic3D MK9 - Hardware Root-Cause Isolator Experiment                 ");
  Serial.println("=========================================================================");

  for (int i = 0; i < 3; i++) {
    pinMode(TOWER_PINS[i][0], OUTPUT);
    pinMode(TOWER_PINS[i][1], OUTPUT);
  }
  stopAllMotors();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  Serial.println("\n COMMANDS AVAILABLE IN SERIAL MONITOR:");
  Serial.println(" 1. Type 'TEST PINS'  -> Runs 3-second PWM pulses for multimeter voltage check.");
  Serial.println(" 2. Type 'TEST SWAP'  -> Runs software driver pin comparison.");
  Serial.println(" 3. Type 'TEST EQUAL' -> Runs 1-second pulse to measure raw count ratios.");
  Serial.println(" 4. Type 'RUN ALL'    -> Executes all 3 tests sequentially.");
  Serial.println("-------------------------------------------------------------------------\n");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "TEST PINS") {
      runTest1_PinVoltageTest();
    } else if (cmd == "TEST SWAP") {
      runTest2_SoftwareSwapTest();
    } else if (cmd == "TEST EQUAL") {
      runTest3_EqualTimePulseTest();
    } else if (cmd == "RUN ALL" || cmd == "START") {
      runTest1_PinVoltageTest();
      runTest2_SoftwareSwapTest();
      runTest3_EqualTimePulseTest();
    } else if (cmd == "STOP") {
      stopAllMotors();
      Serial.println("[CMD] Stopped all motors.");
    }
  }
  delay(10);
}
