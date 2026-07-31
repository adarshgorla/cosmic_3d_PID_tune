#ifndef PID_CONFIG_H
#define PID_CONFIG_H

#include <Arduino.h>

// =============================================================================
// MOTOR & DRIVER SPECIFICATIONS (12V 60RPM DC Motor + L293D H-Bridge)
// =============================================================================
#define MOTOR_VOLTAGE_SUPPLY   12.0f   // Volts
#define MOTOR_RATED_RPM        60.0f   // RPM
#define MOTOR_COUNT            4       // 3 Delta Towers + 1 Extruder

// L293D Darlington voltage drop offset (~2.8V).
// Motors stall below ~60 PWM due to L293D drop and static friction.
#define PWM_MIN_FLOOR          60
#define PWM_MIN_FLOOR_EXTRUDER 190

// Motor Driver Pin Assignments (ESP32-S3 GPIOs)
#define MOTOR_0_PIN_1 14  // Tower A Pin 1
#define MOTOR_0_PIN_2 7   // Tower A Pin 2

#define MOTOR_1_PIN_1 15  // Tower B Pin 1
#define MOTOR_1_PIN_2 16  // Tower B Pin 2

#define MOTOR_2_PIN_1 5   // Tower C Pin 1
#define MOTOR_2_PIN_2 6   // Tower C Pin 2

#define MOTOR_3_PIN_1 4   // Extruder Pin 1
#define MOTOR_3_PIN_2 3   // Extruder Pin 2

// Limit Switches
#define LIMIT1_PIN 46     // Tower A Limit
#define LIMIT2_PIN 43     // Tower B Limit
#define LIMIT3_PIN 48     // Tower C Limit

// =============================================================================
// MATHEMATICALLY DERIVED PID GAINS
// Calculated for 12V 60RPM Motor + L293D + AS5600 (Critically Damped zeta=1.0)
// =============================================================================
// Motor 0 (Tower A), Motor 1 (Tower B), Motor 2 (Tower C), Motor 3 (Extruder)
static float DEFAULT_KP[MOTOR_COUNT] = {0.65f,  0.65f,  0.65f,  0.65f};
static float DEFAULT_KI[MOTOR_COUNT] = {1.625f, 1.625f, 1.625f, 1.625f};
static float DEFAULT_KD[MOTOR_COUNT] = {0.052f, 0.052f, 0.052f, 0.052f};

// =============================================================================
// ENCODER & I2C MULTIPLEXER SPECIFICATIONS
// =============================================================================
#define AS5600_ADDRESS         0x36
#define AS5600_RAW_ANGLE_H     0x0C
#define AS5600_RAW_ANGLE_L     0x0D
#define AS5600_ANGLE_H         0x0E
#define AS5600_ANGLE_L         0x0F
#define AS5600_STATUS          0x0B
#define AS5600_AGC             0x1A
#define AS5600_MAGNITUDE_H     0x1B
#define AS5600_MAGNITUDE_L     0x1C

#define TCA9548A_ADDRESS       0x70
#define I2C_SDA                8
#define I2C_SCL                9
#define I2C_TIMEOUT_MS         30

#define GEAR_RATIO             1
#define AS5600_COUNTS_PER_REV  4096.0f

// Channel Mapping (Motor Index -> TCA9548A Channel)
static const uint8_t MOTOR_TO_I2C_CHANNEL[MOTOR_COUNT] = {0, 1, 2, 3};

// =============================================================================
// KINEMATICS & GEOMETRY SPECIFICATIONS
// =============================================================================
const float TOWER_A_X = 0.000f;
const float TOWER_A_Y = 157.631f;
const float TOWER_A_Z = 735.00f;

const float TOWER_B_X = -136.513f;
const float TOWER_B_Y = -78.816f;
const float TOWER_B_Z = 735.00f;

const float TOWER_C_X = 136.513f;
const float TOWER_C_Y = -78.816f;
const float TOWER_C_Z = 735.00f;

const float PULLEY_RADIUS_MM = 9.5f;
const float HOME_X = 0.0f;
const float HOME_Y = 0.0f;
const float HOME_Z = 0.0f;
const float STEPS_PER_MM_E = 100.0f;
const long  MOTION_TOLERANCE_COUNTS = 8;

const float COUNTS_PER_MM = AS5600_COUNTS_PER_REV / (2.0f * PI * PULLEY_RADIUS_MM);

// =============================================================================
// SD CARD & LOGGING SPECIFICATIONS
// =============================================================================
#define SD_PIN_SCK  12
#define SD_PIN_MISO 13
#define SD_PIN_MOSI 11
#define SD_PIN_CS   10

#define LOG_FILE_PATH "/pid_tune_log.csv"
#define LOG_INTERVAL_MS 20  // Log telemetry every 20ms during motion

// Test move magnitude for single/multi axis step testing (in millimeters)
#define TEST_MOVE_DIST_MM 20.0f

#endif // PID_CONFIG_H
