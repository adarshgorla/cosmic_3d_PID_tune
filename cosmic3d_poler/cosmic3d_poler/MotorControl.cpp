#include "MotorControl.h"

#ifdef ARDUINO
Servo servoTheta;
#endif

int currentServoAngle = SERVO_MIN_ANGLE_DEG;

float Kp[4] = {2.0f, 2.5f, 0.0f, 1.5f};
float Ki[4] = {0.01f, 0.01f, 0.0f, 0.01f};
float Kd[4] = {0.5f, 0.6f, 0.0f, 0.3f};
long prevError[4] = {0, 0, 0, 0};
float integral[4] = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long lastTimePID = 0;

bool manualMovePending = false;
float manualMoveDeltaX = 0.0f;
float manualMoveDeltaY = 0.0f;
float manualMoveDeltaZ = 0.0f;
long manualTarget[4] = {0, 0, 0, 0};
int manualServoTarget = SERVO_MIN_ANGLE_DEG;

void setThetaServoAngle(int angle) {
  currentServoAngle = constrain(angle, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
#ifdef ARDUINO
  servoTheta.write(currentServoAngle);
#endif
}

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

#define PWM_MIN_FLOOR 50
#define PWM_MIN_FLOOR_M4 190

void driveMotor(int i, float output) {
  if (i == 2) {
    // Theta MG945 Servo position control handled separately via setThetaServoAngle()
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

  extern float currentX, currentY, currentZ;
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
