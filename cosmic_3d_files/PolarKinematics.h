#ifndef POLAR_KINEMATICS_H
#define POLAR_KINEMATICS_H

#include "Config.h"

// Cartesian Segment structure for linear movement interpolation
struct CartesianSegment {
  float x;
  float y;
  float z;
  float e;
};

// Global state for continuous bed angle tracking
extern float lastBedAngleDeg;
extern float accumulatedBedAngleDeg;

// Conversion functions
long bedAngleToMotorCounts(float bedAngleDeg);
int radialMmToServoAngle(float radialMm);
long zMmToEncoderCounts(float zMm);

// Inverse Kinematics calculation: Cartesian (X,Y,Z) -> Polar (R counts, Servo Angle, Z counts)
bool calculatePolarIK(float nozzleX, float nozzleY, float nozzleZ,
                      long &targetR_counts, int &targetTheta_servoAngle,
                      long &targetZ_counts);

// Segment interpolation helper
int generateCartesianSegments(float startX, float startY, float startZ,
                              float startE, float endX, float endY, float endZ,
                              float endE, CartesianSegment segments[],
                              int maxSegments);

#endif // POLAR_KINEMATICS_H
