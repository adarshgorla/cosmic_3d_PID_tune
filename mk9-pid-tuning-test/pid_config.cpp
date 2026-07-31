#include "pid_config.h"

// =============================================================================
// MATHEMATICALLY DERIVED PID GAINS
// Calculated for 12V 60RPM Motor + L293D + AS5600 (Critically Damped zeta=1.0)
// =============================================================================
float DEFAULT_KP[MOTOR_COUNT] = {0.65f,  0.65f,  0.65f,  0.65f};
float DEFAULT_KI[MOTOR_COUNT] = {1.625f, 1.625f, 1.625f, 1.625f};
float DEFAULT_KD[MOTOR_COUNT] = {0.052f, 0.052f, 0.052f, 0.052f};

// Channel Mapping (Motor Index -> TCA9548A Channel)
const uint8_t MOTOR_TO_I2C_CHANNEL[MOTOR_COUNT] = {0, 1, 2, 3};
