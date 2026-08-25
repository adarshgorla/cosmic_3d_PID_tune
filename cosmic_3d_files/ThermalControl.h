#ifndef THERMAL_CONTROL_H
#define THERMAL_CONTROL_H

#include "Config.h"

extern bool hotendHeaterOn;
extern bool bedHeaterOn;
extern volatile float hotendSetpoint;
extern volatile float bedSetpoint;

// Thermistor analog read with Steinhart-Hart B-parameter calculation (°C)
float readThermistorC(int pin, float seriesResistor, float beta);

// Non-blocking heater control loop (hysteresis & safety cutoff)
void runHeaterControl();

#endif // THERMAL_CONTROL_H
