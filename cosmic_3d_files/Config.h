#ifndef CONFIG_H
#define CONFIG_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <algorithm>
#include <string>
#include <cstdio>

#define HIGH 1
#define LOW 0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template <typename T>
T constrain(T amt, T low, T high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

class String : public std::string {
public:
    String() : std::string() {}
    String(const char* s) : std::string(s) {}
    String(const std::string& s) : std::string(s) {}
    String(int v) : std::string(std::to_string(v)) {}
    String(float v) : std::string(std::to_string(v)) {}
    char charAt(int index) const { return (index >= 0 && index < (int)length()) ? (*this)[index] : '\0'; }
    void trim() {
        while (!empty() && (front() == ' ' || front() == '\t' || front() == '\r' || front() == '\n')) erase(begin());
        while (!empty() && (back() == ' ' || back() == '\t' || back() == '\r' || back() == '\n')) pop_back();
    }
    void toUpperCase() {
        for (auto& c : *this) c = toupper(c);
    }
    void replace(const char* from, const char* to) {
        size_t start_pos = 0;
        std::string f(from), t(to);
        while((start_pos = find(f, start_pos)) != std::string::npos) {
            std::string::replace(start_pos, f.length(), t);
            start_pos += t.length();
        }
    }
    int toInt() const { return atoi(c_str()); }
    float toFloat() const { return atof(c_str()); }
    bool startsWith(const char* prefix) const { return rfind(prefix, 0) == 0; }
    int indexOf(char ch, int start = 0) const {
        size_t pos = find(ch, start);
        return (pos == std::string::npos) ? -1 : (int)pos;
    }
    String substring(int start, int end = -1) const {
        if (end == -1) return substr(start);
        return substr(start, end - start);
    }
};

class SerialMock {
public:
    void begin(long) {}
    template<typename... Args>
    void printf(const char* fmt, Args... args) { ::printf(fmt, args...); }
    void println(const char* str = "") { ::printf("%s\n", str); }
    void println(const String& str) { ::printf("%s\n", str.c_str()); }
    void println(int val) { ::printf("%d\n", val); }
    void print(const char* str) { ::printf("%s", str); }
    void print(const String& str) { ::printf("%s", str.c_str()); }
    void print(int val) { ::printf("%d", val); }
    void print(double val) { ::printf("%.2f", val); }
    int available() { return 0; }
    String readStringUntil(char) { return ""; }
};
extern SerialMock Serial;

class WireMock {
public:
    void begin(int, int) {}
    void setClock(long) {}
    void setTimeOut(int) {}
    void beginTransmission(uint8_t) {}
    uint8_t endTransmission(bool = true) { return 0; }
    size_t write(uint8_t) { return 1; }
    size_t requestFrom(uint8_t, uint8_t, uint8_t) { return 0; }
    int available() { return 0; }
    uint8_t read() { return 0; }
};
extern WireMock Wire;

inline int analogRead(int) { return 2048; }
inline void analogWrite(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline void pinMode(int, int) {}
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
inline unsigned long millis() { return 1000; }
#endif

// =============================================================================
// I2C & AS5600 ENCODER DEFINITIONS
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

// =============================================================================
// COSMIC POLAR 400 MECHANICAL CONFIGURATION
// =============================================================================
const float R_MOTOR_PULLEY_DIAMETER_MM = 19.0f;
const float R_BED_DIAMETER_MM          = 401.0f;
const float R_AS5600_COUNTS_PER_MOTOR_REV = 4096.0f;

const float RADIAL_MIN_MM              = 0.0f;
const float RADIAL_MAX_RADIUS_MM       = 200.0f;

const int SERVO_MIN_ANGLE_DEG          = 0;
const int SERVO_MAX_ANGLE_DEG          = 180;

const float Z_COUNTS_PER_MM            = 100.0f;
const float INTERPOLATION_SEGMENT_MM   = 2.0f;

const float HOME_X                     = 0.0f;
const float HOME_Y                     = 0.0f;
const float HOME_Z                     = 0.0f;
const float STEPS_PER_MM_E             = 100.0f;
const long MOTION_TOLERANCE_COUNTS     = 5;

const float MANUAL_JOG_MM              = 10.0f;

// =============================================================================
// GPIO PIN ALLOCATIONS
// =============================================================================
#define THETA_SERVO_PIN        18
#define THETA_HOME_SENSOR_PIN  46 // Bed optical/hall home reference sensor
#define Z_LIMIT_PIN            43 // Vertical Z limit switch

// Motor H-Bridge Driver Pins
// Motor 0: R (Bed Rotation) DC Motor
// Motor 1: Z (Vertical Height) DC Motor
// Motor 2: Theta (MG945 Servo, no DC driver)
// Motor 3: E (Extruder) DC/Stepper Driver
#define motorPinA1             14 // R Motor Pin 1
#define motorPinA2             7  // R Motor Pin 2
#define motorPinB1             15 // Z Motor Pin 1
#define motorPinB2             16 // Z Motor Pin 2
#define motorPinc1             5  // E Motor Pin 1
#define motorPinc2             6  // E Motor Pin 2
#define motorPind1             4  // Auxiliary Pin 1
#define motorPind2             3  // Auxiliary Pin 2

// SD Card Pins
#define SD_SCK_PIN             12
#define SD_MISO_PIN            13
#define SD_MOSI_PIN            11
#define SD_CS_PIN              10

// Heater & Thermistor Pins
#define THERMISTOR_PIN         2
#define HEATER_PIN             21
#define MAX_TEMP               280
#define HYSTERESIS             2.0f
#define SETPOINT               0

#define BED_THERMISTOR_PIN     1
#define BED_HEATER_PIN         47
#define BED_MAX_TEMP           130
#define BED_HYSTERESIS         2.0f
#define BED_SETPOINT           0

#define SERIES_RESISTOR        4700.0f
#define BETA                   3950.0f
#define BED_SERIES_RESISTOR    4700.0f
#define BED_BETA               3950.0f

#define ADC_MAX                4095.0f
#define T0                     25.0f
#define HEATER_POLL_MS         500

// =============================================================================
// NETWORK & MQTT CONFIGURATION
// =============================================================================
extern const char *MQTT_BROKER;
extern const char *MQTT_USER;
extern const char *MQTT_PASSWORD;

#define TOPIC_START_file_start_stop "file_transfer/start"
#define TOPIC_STOP                  "file_transfer/data"
#define TOPIC_xyz_move              "motor/xyz_move"
#define TOPIC_ACK                   "file_transfer/ack"
#define MOTOR_START                 "motor/start"
#define MOTOR_STOP                  "motor/stop"

// =============================================================================
// SYSTEM STATE MACHINE ENUM
// =============================================================================
enum SystemState {
  STATE_IDLE,
  STATE_HOMING_SEEK,
  STATE_HOMING_ZERO,
  STATE_HOMING_STANDOFF,
  STATE_MOTION_OPEN,
  STATE_MOTION_READ_LINE,
  STATE_MOTION_PID,
  STATE_MOTION_DWELL,
  STATE_MOTION_DONE,
  STATE_MOTION_PAUSED,
  STATE_MANUAL_XYZ
};

#endif // CONFIG_H
