#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Config.h"
#include "EncoderManager.h"
#include "PolarKinematics.h"

#ifdef ARDUINO
#include <ESP32Servo.h>
extern Servo servoTheta;
#endif

extern int currentServoAngle;

extern float Kp[4];
extern float Ki[4];
extern float Kd[4];
extern long prevError[4];
extern float integral[4];
extern unsigned long lastTimePID;

// Manual move state variables
extern bool manualMovePending;
extern float manualMoveDeltaX;
extern float manualMoveDeltaY;
extern float manualMoveDeltaZ;
extern long manualTarget[4];
extern int manualServoTarget;

// Servo functions
void setThetaServoAngle(int angle);

// PID & Motor Driver functions
float calculatePID(int i, long targetPosition, float dt);
void driveMotor(int i, float output);
void stopAllMotors();

// Manual jog move functions
void resetManualXYZTarget();
void queueManualMove(float dx, float dy, float dz);
void applyPendingManualMove();

#endif // MOTOR_CONTROL_H
