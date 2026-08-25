#include "NetworkManager.h"

const char *MQTT_BROKER = "test.mosquitto.org";
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";

bool messageReceived = false;
float currentX = HOME_X;
float currentY = HOME_Y;
float currentZ = HOME_Z;
String topicBuffer = "";
String payloadBuffer = "";

#ifdef ARDUINO
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
Preferences prefs;
#endif

long lastReceivedChunk = -1;
unsigned long lastAttemptTime = 0;
const unsigned long reconnectDelay = 5000;
bool printedLostMsg = false;
bool fileReceiving = false;
String fileName = "";
int totalChunks = 0;
String expectedChecksum = "";

bool isActivePrintState(int state) {
  return state == STATE_MOTION_OPEN || state == STATE_MOTION_READ_LINE ||
         state == STATE_MOTION_PID || state == STATE_MOTION_DWELL;
}

void saveState() {
#ifdef ARDUINO
  prefs.putString("fileName", fileName);
  prefs.putInt("totalChunks", totalChunks);
  prefs.putInt("lastChunk", lastReceivedChunk);
  prefs.putString("checksum", expectedChecksum);
#endif
  Serial.println("[NVS] State saved.");
}

void restoreState() {
#ifdef ARDUINO
  fileName = prefs.getString("fileName", "");
  totalChunks = prefs.getInt("totalChunks", 0);
  lastReceivedChunk = prefs.getInt("lastChunk", -1);
  expectedChecksum = prefs.getString("checksum", "");
#endif
  Serial.println("[NVS] State restored.");
  Serial.print("[NVS]   fileName: ");
  Serial.println(fileName);
  Serial.print("[NVS]   totalChunks: ");
  Serial.println(totalChunks);
  Serial.print("[NVS]   lastChunk: ");
  Serial.println(lastReceivedChunk);
}

void loadState() {
#ifdef ARDUINO
  lastReceivedChunk = prefs.getLong("lastChunk", -1);
#endif
  Serial.print("[NVS] Last chunk: ");
  Serial.println(lastReceivedChunk);
}

void mqttReconnect() {
  Serial.print("[MQTT] Reconnecting... ");
#ifdef ARDUINO
  if (mqttClient.connect(MQTT_BROKER, 1883)) {
    Serial.println("connected ✅");
    mqttClient.subscribe(TOPIC_START_file_start_stop);
    mqttClient.subscribe(TOPIC_STOP);
    mqttClient.subscribe(TOPIC_xyz_move);
    mqttClient.subscribe(MOTOR_START);
    mqttClient.subscribe(MOTOR_STOP);
  } else {
    Serial.print("[MQTT] Failed, error=");
    Serial.println(mqttClient.connectError());
  }
#endif
}

void sendmessage(String publishtopic, String messagetosend) {
#ifdef ARDUINO
  if (!mqttClient.connected())
    return;
  mqttClient.beginMessage(publishtopic);
  mqttClient.print(messagetosend);
  mqttClient.endMessage();
#endif
}
