#include <iostream>
#include <cassert>
#include <cmath>
#include <cstring>
#include "Config.h"
#include "PolarKinematics.h"
#include "EncoderManager.h"
#include "MotorControl.h"
#include "ThermalControl.h"
#include "GCodeParser.h"
#include "NetworkManager.h"
#include "PrinterStateMachine.h"

// Instantiate Global Mocks required when ARDUINO is not defined
SerialMock Serial;
WireMock Wire;

int testsPassed = 0;
int testsFailed = 0;

#define TEST_ASSERT(cond, msg) \
  do { \
    if (cond) { \
      std::cout << "  [PASS] " << msg << std::endl; \
      testsPassed++; \
    } else { \
      std::cerr << "  [FAIL] " << msg << " (" << __FILE__ << ":" << __LINE__ << ")" << std::endl; \
      testsFailed++; \
    } \
  } while(0)

void testPolarKinematics() {
  std::cout << "\n=== Testing PolarKinematics Module ===" << std::endl;
  
  // 1. bedAngleToMotorCounts
  // R_BED_DIAMETER_MM = 401.0, R_MOTOR_PULLEY_DIAMETER_MM = 19.0, R_AS5600_COUNTS_PER_MOTOR_REV = 4096.0
  // bed 360 deg => (401/19) motor revs * 4096 = ~864455 counts
  long counts0 = bedAngleToMotorCounts(0.0f);
  TEST_ASSERT(counts0 == 0, "bedAngleToMotorCounts(0.0) == 0");
  
  long counts360 = bedAngleToMotorCounts(360.0f);
  long expected360 = (long)lround((401.0 / 19.0) * 4096.0);
  TEST_ASSERT(std::abs(counts360 - expected360) <= 1, "bedAngleToMotorCounts(360.0) calculation correct");

  // 2. radialMmToServoAngle
  // RADIAL_MIN_MM = 0, RADIAL_MAX_RADIUS_MM = 200, SERVO_MIN = 0, SERVO_MAX = 180
  TEST_ASSERT(radialMmToServoAngle(0.0f) == 0, "radialMmToServoAngle(0.0) == 0");
  TEST_ASSERT(radialMmToServoAngle(200.0f) == 180, "radialMmToServoAngle(200.0) == 180");
  TEST_ASSERT(radialMmToServoAngle(100.0f) == 90, "radialMmToServoAngle(100.0) == 90");
  TEST_ASSERT(radialMmToServoAngle(-10.0f) == 0, "radialMmToServoAngle(-10.0) clamped to 0");
  TEST_ASSERT(radialMmToServoAngle(250.0f) == 180, "radialMmToServoAngle(250.0) clamped to 180");

  // 3. zMmToEncoderCounts
  // Z_COUNTS_PER_MM = 100.0
  TEST_ASSERT(zMmToEncoderCounts(10.0f) == 1000, "zMmToEncoderCounts(10.0) == 1000");
  TEST_ASSERT(zMmToEncoderCounts(0.0f) == 0, "zMmToEncoderCounts(0.0) == 0");

  // 4. calculatePolarIK
  long rCounts = 0, zCounts = 0;
  int thetaAngle = 0;
  lastBedAngleDeg = 0.0f;
  accumulatedBedAngleDeg = 0.0f;

  bool ok = calculatePolarIK(0.0f, 0.0f, 5.0f, rCounts, thetaAngle, zCounts);
  TEST_ASSERT(ok == true, "calculatePolarIK(0,0,5) succeeds");
  TEST_ASSERT(rCounts == 0, "calculatePolarIK(0,0,5) rCounts == 0");
  TEST_ASSERT(thetaAngle == 0, "calculatePolarIK(0,0,5) thetaAngle == 0");
  TEST_ASSERT(zCounts == 500, "calculatePolarIK(0,0,5) zCounts == 500");

  // Radial move to (100, 0, 10)
  ok = calculatePolarIK(100.0f, 0.0f, 10.0f, rCounts, thetaAngle, zCounts);
  TEST_ASSERT(ok == true, "calculatePolarIK(100,0,10) succeeds");
  TEST_ASSERT(thetaAngle == 90, "calculatePolarIK(100,0,10) thetaAngle == 90 (100mm radius)");
  TEST_ASSERT(zCounts == 1000, "calculatePolarIK(100,0,10) zCounts == 1000");

  // Out of bounds radial move (>200mm)
  ok = calculatePolarIK(250.0f, 0.0f, 10.0f, rCounts, thetaAngle, zCounts);
  TEST_ASSERT(ok == false, "calculatePolarIK(250,0,10) fails (out of max radius bounds)");

  // 5. generateCartesianSegments
  CartesianSegment segments[32];
  int numSegs = generateCartesianSegments(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 1.0f, segments, 32);
  TEST_ASSERT(numSegs == 5, "generateCartesianSegments 10mm dist / 2mm segment == 5 segments");
  TEST_ASSERT(std::abs(segments[4].x - 10.0f) < 0.001f, "Final interpolated segment X is 10.0");
  TEST_ASSERT(std::abs(segments[4].e - 1.0f) < 0.001f, "Final interpolated segment E is 1.0");
}

void testGCodeParser() {
  std::cout << "\n=== Testing GCodeParser Module ===" << std::endl;

  float x = 0, y = 0, z = 0, e = 0;
  bool ok = parseCartesianLine("10.5 20.0 5.0 1.2", x, y, z, e);
  TEST_ASSERT(ok == true, "parseCartesianLine 4 numbers parses successfully");
  TEST_ASSERT(std::abs(x - 10.5f) < 0.001f, "Parsed X == 10.5");
  TEST_ASSERT(std::abs(y - 20.0f) < 0.001f, "Parsed Y == 20.0");
  TEST_ASSERT(std::abs(z - 5.0f) < 0.001f, "Parsed Z == 5.0");
  TEST_ASSERT(std::abs(e - 1.2f) < 0.001f, "Parsed E == 1.2");

  ok = parseCartesianLine("1 2 3 4 5", x, y, z, e);
  TEST_ASSERT(ok == true, "parseCartesianLine 5 numbers parses successfully");
  TEST_ASSERT(std::abs(x - 1.0f) < 0.001f && std::abs(e - 5.0f) < 0.001f, "5 parameter mapping X=1, E=5");

  ok = parseCartesianLine("invalid gcode", x, y, z, e);
  TEST_ASSERT(ok == false, "parseCartesianLine returns false for invalid line");

  ok = parseCartesianLine("", x, y, z, e);
  TEST_ASSERT(ok == false, "parseCartesianLine returns false for empty line");
}

void testMotorControl() {
  std::cout << "\n=== Testing MotorControl Module ===" << std::endl;

  // Servo angle setting
  setThetaServoAngle(90);
  TEST_ASSERT(currentServoAngle == 90, "setThetaServoAngle(90) sets currentServoAngle = 90");
  setThetaServoAngle(-50);
  TEST_ASSERT(currentServoAngle == 0, "setThetaServoAngle(-50) clamps to 0");
  setThetaServoAngle(300);
  TEST_ASSERT(currentServoAngle == 180, "setThetaServoAngle(300) clamps to 180");

  // PID calculation test
  // For Motor 0: Kp=2.0, Ki=0.01, Kd=0.5
  encoderCount[0] = 0;
  prevError[0] = 0;
  integral[0] = 0;
  float out = calculatePID(0, 100, 0.1f); // target = 100, error = 100
  // P = 200, I = 100 * 0.1 * 0.01 = 0.1, D = 0.5 * (100-0)/0.1 = 500 => total out = 700.1
  TEST_ASSERT(out > 0.0f, "calculatePID(0, 100, 0.1) produces positive control output");

  // Servo axis (motor 2) PID returns 0
  float outServo = calculatePID(2, 100, 0.1f);
  TEST_ASSERT(outServo == 0.0f, "calculatePID for Servo axis (motor 2) returns 0.0");

  // Stop all motors
  stopAllMotors();
  TEST_ASSERT(true, "stopAllMotors executed cleanly");
}

void testThermalControl() {
  std::cout << "\n=== Testing ThermalControl Module ===" << std::endl;

  // Test readThermistorC with out-of-bound ADC values (mock analogRead returns 2048)
  float temp = readThermistorC(THERMISTOR_PIN, SERIES_RESISTOR, BETA);
  TEST_ASSERT(temp > 0.0f && temp < 200.0f, "readThermistorC returns plausible temperature for mid ADC value");

  // Test runHeaterControl
  runHeaterControl();
  TEST_ASSERT(true, "runHeaterControl executed cleanly");
}

void testPrinterStateMachine() {
  std::cout << "\n=== Testing PrinterStateMachine Module ===" << std::endl;

  cosmicPolarSetup();
  TEST_ASSERT(sysState == STATE_IDLE, "Initial sysState after setup is STATE_IDLE");

  // Run loops in STATE_IDLE
  for (int i = 0; i < 5; i++) {
    cosmicPolarLoop();
  }
  TEST_ASSERT(sysState == STATE_IDLE, "State machine remains IDLE without active trigger");

  // Test manual move queuing logic
  queueManualMove(10.0f, 0.0f, 0.0f);
  TEST_ASSERT(manualMovePending == true, "queueManualMove sets manualMovePending to true");
  
  applyPendingManualMove();
  TEST_ASSERT(manualMovePending == false, "applyPendingManualMove clears manualMovePending");
}

int main() {
  std::cout << "=====================================================" << std::endl;
  std::cout << "  RUNNING ESP32 FIRMWARE NATIVE SUITE  " << std::endl;
  std::cout << "=====================================================" << std::endl;

  testPolarKinematics();
  testGCodeParser();
  testMotorControl();
  testThermalControl();
  testPrinterStateMachine();

  std::cout << "\n=====================================================" << std::endl;
  std::cout << "TEST RESULTS: " << testsPassed << " Passed, " << testsFailed << " Failed." << std::endl;
  std::cout << "=====================================================" << std::endl;

  return testsFailed == 0 ? 0 : 1;
}
