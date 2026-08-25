#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "Config.h"
#include "GCodeParser.h"
#include "MotorControl.h"
#include "ThermalControl.h"

#ifdef ARDUINO
#include <ArduinoMqttClient.h>
#include <MD5Builder.h>
#include <Preferences.h>
#include <WiFi.h>
extern WiFiClient wifiClient;
extern MqttClient mqttClient;
extern Preferences prefs;
#endif

extern const char *MQTT_BROKER;
extern const char *MQTT_USER;
extern const char *MQTT_PASSWORD;

extern bool messageReceived;
extern float currentX;
extern float currentY;
extern float currentZ;
extern String topicBuffer;
extern String payloadBuffer;

extern long lastReceivedChunk;
extern unsigned long lastAttemptTime;
extern const unsigned long reconnectDelay;
extern bool printedLostMsg;
extern bool fileReceiving;
extern String fileName;
extern int totalChunks;
extern String expectedChecksum;

// Utility functions
bool isActivePrintState(int state);

// NVS Persistent State functions
void saveState();
void restoreState();
void loadState();

// MQTT messaging functions
void mqttReconnect();
void sendmessage(String publishtopic, String messagetosend);

#endif // NETWORK_MANAGER_H
