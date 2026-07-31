#include "pid_config.h"

// =============================================================================
// MATHEMATICALLY DERIVED PID GAINS DEFINITION
// =============================================================================
float DEFAULT_KP[MOTOR_COUNT] = {0.60f, 0.60f, 0.60f, 0.60f};
float DEFAULT_KI[MOTOR_COUNT] = {0.05f, 0.05f, 0.05f, 0.05f};
float DEFAULT_KD[MOTOR_COUNT] = {0.35f, 0.35f, 0.35f, 0.35f};

// Channel Mapping (Motor Index -> TCA9548A Channel)
const uint8_t MOTOR_TO_I2C_CHANNEL[MOTOR_COUNT] = {0, 1, 2, 3};
