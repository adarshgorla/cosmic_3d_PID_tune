#include "EncoderManager.h"

volatile long encoderCount[4] = {0, 0, 0, 0};
volatile int rotation[4] = {0, 0, 0, 0};
volatile long setpoint[4] = {0, 0, 0, 0};
long lastAngle[4] = {0, 0, 0, 0};
long totalRotations[4] = {0, 0, 0, 0};
long rawAccumulator[4] = {0, 0, 0, 0};

const uint8_t motorToChannel[4] = {0, 1, 2, 3};
bool as5600Ready = false;

bool selectI2CChannel(uint8_t channel) {
  if (channel > 7)
    return false;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("[I2C] TCA9548A select ch %u failed (err=%u)\n", channel, err);
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
  if (Wire.endTransmission(false) != 0)
    return 0;

  size_t n = Wire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)1, (uint8_t)true);
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
    Serial.printf("[ENC] Axis %d: Spurious jump (%ld), ignored.\n", motorIndex, diff);
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
  Serial.println("[AS5600] Initializing TCA9548A & AS5600 Encoders...");

  if (!probeI2CDevice(TCA9548A_ADDRESS)) {
    Serial.println("[AS5600] TCA9548A not detected at 0x70. Encoder subsystem disabled.");
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
      Serial.printf("[AS5600] Axis %d (Ch %d) unavailable.\n", i, motorToChannel[i]);
      continue;
    }
    delay(5);

    uint16_t angle = readRawAngle();
    uint8_t status = readStatus();
    uint16_t magnitude = readMagnitude();
    uint8_t agc = readAGC();
    bool magOK = isMagnetDetected();

    Serial.printf("[AS5600] Axis %d (Ch %d): Angle=%u Status=0x%02X Mag=%u AGC=%u Magnet=%s\n",
                  i, motorToChannel[i], angle, status, magnitude, agc,
                  magOK ? "✓ OK" : "✗ MISSING!");

    lastAngle[i] = angle;
    encoderCount[i] = 0;
    rawAccumulator[i] = 0;
    totalRotations[i] = 0;
  }

  disableAllI2CChannels();
  Serial.println(as5600Ready ? "[AS5600] Init complete." : "[AS5600] Init degraded.");
}

void checkAllEncoders() {
  if (!as5600Ready) {
    Serial.println("[DIAG] AS5600 unavailable (TCA9548A/encoder offline).");
    return;
  }

  Serial.println("\n[DIAG] ===== Cosmic Polar 400 Diagnostics =====");
  for (int i = 0; i < 4; i++) {
    if (i == 2) {
      Serial.printf("[DIAG] Theta Axis (Servo) | Pin: %d\n", THETA_SERVO_PIN);
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

    Serial.printf("[DIAG] Axis %d | Angle=%u | Count=%ld | Setpoint=%ld | Mag=%u | AGC=%u | %s\n",
                  i, angle, encoderCount[i], setpoint[i], magnitude, agc,
                  magOK ? "✓ OK" : "✗ MISSING!");
  }
  disableAllI2CChannels();
  Serial.println("[DIAG] ========================================\n");
}
