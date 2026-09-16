#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include "Config.h"

#ifdef ARDUINO
#include <Wire.h>
#endif

// Global Encoder State Arrays (Axis 0=R, 1=Z, 2=Theta Servo, 3=E)
extern volatile long encoderCount[4];
extern volatile int rotation[4];
extern volatile long setpoint[4];
extern long lastAngle[4];
extern long totalRotations[4];
extern long rawAccumulator[4];
extern const uint8_t motorToChannel[4];
extern bool as5600Ready;

// I2C Multiplexer & Encoder Low-level Functions
bool selectI2CChannel(uint8_t channel);
void disableAllI2CChannels();
uint16_t readAS5600Register16(uint8_t reg);
uint8_t readAS5600Register8(uint8_t reg);
uint16_t readRawAngle();
uint16_t readAngle();
uint8_t readStatus();
uint8_t readAGC();
uint16_t readMagnitude();
bool probeI2CDevice(uint8_t address);
bool isMagnetDetected();

// High-level Subsystem Control
void updateEncoderFromAS5600(int motorIndex);
void initAS5600();
void checkAllEncoders();

#endif // ENCODER_MANAGER_H
