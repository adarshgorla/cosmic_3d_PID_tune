#include "ThermalControl.h"

bool hotendHeaterOn = false;
bool bedHeaterOn = false;
volatile float hotendSetpoint = SETPOINT;
volatile float bedSetpoint = BED_SETPOINT;

float readThermistorC(int pin, float seriesResistor, float beta) {
  long sum = 0;
  const int samples = 32;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  float adcValue = (float)sum / samples;

  if (adcValue <= 5.0f || adcValue >= 4090.0f)
    return -1.0f;
  float resistance = seriesResistor * (ADC_MAX / adcValue - 1.0f);
  float steinhart = log(resistance / 100000.0f);
  steinhart /= beta;
  steinhart += 1.0f / (T0 + 273.15f);
  return (1.0f / steinhart) - 273.15f;
}

void runHeaterControl() {
  float hotendTemp = readThermistorC(THERMISTOR_PIN, SERIES_RESISTOR, BETA);

  if (hotendTemp < 0 || hotendTemp > MAX_TEMP) {
    if (hotendHeaterOn) {
      digitalWrite(HEATER_PIN, LOW);
      hotendHeaterOn = false;
      Serial.printf("[HEAT] HOTEND FAULT (%.1f°C) → OFF ❌\n", hotendTemp);
    }
  } else {
    if (hotendTemp < (hotendSetpoint - HYSTERESIS) && !hotendHeaterOn) {
      digitalWrite(HEATER_PIN, HIGH);
      hotendHeaterOn = true;
      Serial.printf("[HEAT] Hotend %.1f°C → ON 🔥 (sp %.1f°C)\n", hotendTemp,
                    hotendSetpoint);
    } else if (hotendTemp > (hotendSetpoint + HYSTERESIS) && hotendHeaterOn) {
      digitalWrite(HEATER_PIN, LOW);
      hotendHeaterOn = false;
      Serial.printf("[HEAT] Hotend %.1f°C → OFF (sp %.1f°C)\n", hotendTemp,
                    hotendSetpoint);
    }
  }

  float bedTemp =
      readThermistorC(BED_THERMISTOR_PIN, BED_SERIES_RESISTOR, BED_BETA);

  if (bedTemp < 0 || bedTemp > BED_MAX_TEMP) {
    if (bedHeaterOn) {
      digitalWrite(BED_HEATER_PIN, LOW);
      bedHeaterOn = false;
      Serial.printf("[HEAT] BED FAULT (%.1f°C) → OFF ❌\n", bedTemp);
    }
  } else {
    if (bedTemp < (bedSetpoint - BED_HYSTERESIS) && !bedHeaterOn) {
      digitalWrite(BED_HEATER_PIN, HIGH);
      bedHeaterOn = true;
      Serial.printf("[HEAT] Bed %.1f°C → ON 🛏 (sp %.1f°C)\n", bedTemp,
                    bedSetpoint);
    } else if (bedTemp > (bedSetpoint + BED_HYSTERESIS) && bedHeaterOn) {
      digitalWrite(BED_HEATER_PIN, LOW);
      bedHeaterOn = false;
      Serial.printf("[HEAT] Bed %.1f°C → OFF (sp %.1f°C)\n", bedTemp,
                    bedSetpoint);
    }
  }
}
