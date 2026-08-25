#include "PolarKinematics.h"

float lastBedAngleDeg = 0.0f;
float accumulatedBedAngleDeg = 0.0f;

long bedAngleToMotorCounts(float bedAngleDeg) {
  double motorRevs =
      ((double)bedAngleDeg / 360.0) *
      ((double)R_BED_DIAMETER_MM / (double)R_MOTOR_PULLEY_DIAMETER_MM);
  return (long)lround(motorRevs * (double)R_AS5600_COUNTS_PER_MOTOR_REV);
}

int radialMmToServoAngle(float radialMm) {
  radialMm = constrain(radialMm, RADIAL_MIN_MM, RADIAL_MAX_RADIUS_MM);
  float scale = (float)(SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG) /
                (RADIAL_MAX_RADIUS_MM - RADIAL_MIN_MM);
  int angle =
      SERVO_MIN_ANGLE_DEG + (int)lroundf((radialMm - RADIAL_MIN_MM) * scale);
  return constrain(angle, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
}

long zMmToEncoderCounts(float zMm) {
  return (long)lroundf(zMm * Z_COUNTS_PER_MM);
}

bool calculatePolarIK(float nozzleX, float nozzleY, float nozzleZ,
                      long &targetR_counts, int &targetTheta_servoAngle,
                      long &targetZ_counts) {

  float radialR = sqrtf(nozzleX * nozzleX + nozzleY * nozzleY);
  if (radialR > RADIAL_MAX_RADIUS_MM) {
    Serial.printf("[KINEMATICS] Radial distance %.2f mm exceeds maximum radius %.2f mm.\n",
                  radialR, RADIAL_MAX_RADIUS_MM);
    return false;
  }

  float rawBedAngleDeg = atan2f(nozzleY, nozzleX) * 180.0f / M_PI;
  if (rawBedAngleDeg < 0.0f) {
    rawBedAngleDeg += 360.0f;
  }

  // Continuous multi-turn angle accumulation via shortest angular step
  float deltaAngle = rawBedAngleDeg - lastBedAngleDeg;
  while (deltaAngle > 180.0f)
    deltaAngle -= 360.0f;
  while (deltaAngle < -180.0f)
    deltaAngle += 360.0f;

  accumulatedBedAngleDeg += deltaAngle;
  lastBedAngleDeg = rawBedAngleDeg;

  targetR_counts = bedAngleToMotorCounts(accumulatedBedAngleDeg);
  targetTheta_servoAngle = radialMmToServoAngle(radialR);
  targetZ_counts = zMmToEncoderCounts(nozzleZ);

  return isfinite(radialR) && isfinite(rawBedAngleDeg) && isfinite(nozzleZ);
}

int generateCartesianSegments(float startX, float startY, float startZ,
                              float startE, float endX, float endY, float endZ,
                              float endE, CartesianSegment segments[],
                              int maxSegments) {
  float dx = endX - startX;
  float dy = endY - startY;
  float dz = endZ - startZ;
  float de = endE - startE;

  float distance = sqrtf(dx * dx + dy * dy + dz * dz);
  int numSegments = (int)ceilf(distance / INTERPOLATION_SEGMENT_MM);
  if (numSegments < 1)
    numSegments = 1;
  if (numSegments > maxSegments)
    numSegments = maxSegments;

  for (int i = 1; i <= numSegments; i++) {
    float t = (float)i / (float)numSegments;
    segments[i - 1].x = startX + t * dx;
    segments[i - 1].y = startY + t * dy;
    segments[i - 1].z = startZ + t * dz;
    segments[i - 1].e = startE + t * de;
  }
  return numSegments;
}
