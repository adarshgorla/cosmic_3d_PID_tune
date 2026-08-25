#ifndef PRINTER_STATE_MACHINE_H
#define PRINTER_STATE_MACHINE_H

#include "Config.h"
#include "EncoderManager.h"
#include "GCodeParser.h"
#include "MotorControl.h"
#include "NetworkManager.h"
#include "PolarKinematics.h"
#include "ThermalControl.h"

// Printer state variables
extern volatile SystemState sysState;
extern bool r_home, z_home, theta_home;
extern long setpointHome[4];
extern unsigned long dwellStart;
extern SystemState pausedPrintState;

// Cartesian interpolation segment buffer
#define MAX_INTERPOLATION_SEGMENTS 32
extern CartesianSegment segmentBuffer[MAX_INTERPOLATION_SEGMENTS];
extern int currentSegmentIndex;
extern int totalSegmentsCount;

// High-level controller functions
void cosmicPolarSetup();
void cosmicPolarLoop();

#endif // PRINTER_STATE_MACHINE_H
