#include <WiFi.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>

#define RGB_BUILTIN 48
#define RGB_BRIGHTNESS 255

#define RESET_BUTTON_PIN 0
#define RESET_HOLD_TIME 5000

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;

const byte DNS_PORT = 53;

enum SystemMode { CONFIG_MODE, OPERATING_MODE, HYBRID_MODE };
SystemMode mode = CONFIG_MODE;

unsigned long buttonPressStart = 0;
bool buttonPressed = false;
bool portalStarted = false;
bool mk9Initialized = false;
bool pendingWiFiConnect = false;
unsigned long lastWiFiRetryMs = 0;
unsigned long lastRouterCheckMs = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;
const unsigned long ROUTER_CHECK_INTERVAL_MS = 20000;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 9000;

String storedSSID;
String storedPASS;

void mk9Setup();
void mk9Loop();
bool isPrinterActive();
void startSoftAP();

const char* wifiStatusToString(wl_status_t st) {
  switch (st) {
    case WL_NO_SHIELD:      return "WL_NO_SHIELD";
    case WL_IDLE_STATUS:    return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:  return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:      return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:   return "WL_DISCONNECTED";
    default:                return "WL_UNKNOWN";
  }
}

bool waitForWiFiConnectCooperative(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) return true;

    dnsServer.processNextRequest();
    server.handleClient();
    delay(25);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool tryConnectWiFi() {
  if (storedSSID.length() == 0) return false;

  storedSSID.trim();
  storedPASS.trim();

  Serial.println("[WIFI] Attempting STA connect to: " + storedSSID);
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) {
    Serial.println("[WIFI] Already connected.");
    return true;
  }

  // Keep AP active so captive portal remains reachable during every retry.
  startSoftAP();

  // Stage 1: AP+STA attempt.
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.disconnect(false, false);
  delay(150);
  WiFi.mode(WIFI_AP_STA);

  WiFi.begin(storedSSID.c_str(), storedPASS.c_str());
  bool connected = waitForWiFiConnectCooperative(WIFI_CONNECT_TIMEOUT_MS);
  st = WiFi.status();

  Serial.print("[WIFI] status (AP+STA) -> ");
  Serial.println(wifiStatusToString(st));

  if (connected && st == WL_CONNECTED) {
    Serial.println("[WIFI] Connected ✅");
    Serial.println("[WIFI] IP: " + WiFi.localIP().toString());
    Serial.print("[WIFI] RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    startSoftAP();
    return true;
  }

  Serial.print("[WIFI] Connect failed. Final status: ");
  Serial.println(wifiStatusToString(st));
  // Re-assert AP/DNS after failed attempt so captive portal stays reachable.
  startSoftAP();
  return false;
}

void startSoftAP() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ESP32-3DPrinter-Setup");
  
  Serial.println("AP Started");
  Serial.println("AP IP: " + WiFi.softAPIP().toString());
  
  // Start DNS server for captive portal
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
}

void stopSoftAP() {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
}

void startCaptivePortalServer() {
  if (portalStarted) return;

  // Captive portal detection endpoints
  server.on("/generate_204", HTTP_GET, []() {
    server.sendHeader("Location", "http://192.168.4.1", true);
    server.send(302, "text/plain", "");
  });
  
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.sendHeader("Location", "http://192.168.4.1", true);
    server.send(302, "text/plain", "");
  });

  // Main configuration page
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>"
      "<style>body{font-family:Arial;padding:20px;}"
      "input{width:100%;padding:10px;margin:10px 0;box-sizing:border-box;}"
      "button{background:#4CAF50;color:white;padding:14px;border:none;width:100%;cursor:pointer;}"
      "button:hover{background:#45a049;}</style></head><body>"
      "<h2>ESP32 3D Printer WiFi Setup</h2>"
      "<form method='POST' action='/save'>"
      "<label>WiFi Network:</label>"
      "<input name='ssid' placeholder='Enter SSID' required><br>"
      "<label>Password:</label>"
      "<input name='pass' type='password' placeholder='Enter Password'><br>"
      "<button type='submit'>Save & Connect</button>"
      "</form></body></html>";
    server.send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, []() {
    storedSSID = server.arg("ssid");
    storedPASS = server.arg("pass");
    
    preferences.putString("ssid", storedSSID);
    preferences.putString("pass", storedPASS);
    
    String html = "<!DOCTYPE html><html><head>"
      "<meta http-equiv='refresh' content='3;url=/' />"
      "</head><body><h2>Settings Saved!</h2>"
      "<p>Connecting to " + storedSSID + "...</p>"
      "<p>You will be redirected shortly.</p></body></html>";
    
    server.send(200, "text/html", html);

    pendingWiFiConnect = true;
    lastWiFiRetryMs = 0;
    Serial.println("[WIFI] Credentials saved. Scheduling immediate connect attempt...");
  });

  // Catch-all handler for captive portal
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  portalStarted = true;
  Serial.println("Web server started");
}

bool pingRouter() {
  // Many routers block TCP:80, which can cause false disconnect decisions.
  // Use link-layer status only for stability.
  return WiFi.status() == WL_CONNECTED;
}

void enterConfigMode() {
  mode = CONFIG_MODE;
  startSoftAP();
  startCaptivePortalServer();
}

void enterHybridMode() {
  mode = HYBRID_MODE;
  startSoftAP();
  startCaptivePortalServer();
  Serial.println("Entered hybrid mode (AP alive, STA reconnecting)");
}

void enterOperatingMode() {
  mode = OPERATING_MODE;
  startSoftAP();
  startCaptivePortalServer();
  Serial.println("Entered operating mode");
}

void checkResetButton() {
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressStart = millis();
    } else {
      if (millis() - buttonPressStart >= RESET_HOLD_TIME) {
        Serial.println("Factory reset triggered");
        preferences.clear();
        ESP.restart();
      }
    }
  } else {
    buttonPressed = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\nESP32 3D Printer Status Light");

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  preferences.begin("wifi", false);
  storedSSID = preferences.getString("ssid", "");
  storedPASS = preferences.getString("pass", "");

  // Start AP + captive portal immediately so it never appears late.
  enterConfigMode();

  // Try to connect to saved WiFi in the main loop retries.
  if (storedSSID.length() > 0) {
    pendingWiFiConnect = true;
    lastWiFiRetryMs = 0;
    Serial.println("Found saved credentials, scheduling immediate connection attempt...");
  } else {
    Serial.println("Starting configuration mode...");
  }
}

void loop() {
  checkResetButton();

  dnsServer.processNextRequest(); // Keep captive portal alive in every mode
  server.handleClient();

  bool printerActive = mk9Initialized && isPrinterActive();

  if (mode == CONFIG_MODE) {
    if (!printerActive && storedSSID.length() > 0) {
      bool retryWindow = millis() - lastWiFiRetryMs >= WIFI_RETRY_INTERVAL_MS;
      if (pendingWiFiConnect || retryWindow) {
        pendingWiFiConnect = false;
        lastWiFiRetryMs = millis();
        Serial.println("[WIFI] Config mode retry...");
        if (tryConnectWiFi()) {
          enterOperatingMode();
        } else {
          Serial.println("[WIFI] Retry failed; captive portal remains active.");
        }
      }
    }
    idleMode();
    if (mk9Initialized) {
      mk9Loop();
    }
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (mode != HYBRID_MODE) {
      Serial.println("WiFi disconnected, entering hybrid mode");
      enterHybridMode();
    }

    if (!printerActive && storedSSID.length() > 0 && millis() - lastWiFiRetryMs >= WIFI_RETRY_INTERVAL_MS) {
      lastWiFiRetryMs = millis();
      Serial.println("[WIFI] Hybrid mode reconnect attempt...");
      if (tryConnectWiFi()) {
        Serial.println("[WIFI] Reconnected, returning to operating mode");
        enterOperatingMode();
      }
    }

    idleMode();
  } else {
    if (mode == HYBRID_MODE) {
      enterOperatingMode();
    }

    if (millis() - lastRouterCheckMs >= ROUTER_CHECK_INTERVAL_MS) {
      lastRouterCheckMs = millis();
      bool routerOk = pingRouter();
      if (!routerOk && mode == OPERATING_MODE) {
        enterHybridMode();
      }
    }
  }

  if (!mk9Initialized) {
    mk9Setup();
    mk9Initialized = true;
  }

  mk9Loop();
}

// Your LED functions remain the same
void powerOnMode() { setColor(255,255,255); }
void sdPowerMode() { setColor(255,105,180); }
void wifiMode() { setColor(0,0,255); }
void mqttMode() { setColor(0,200,180); }
void homeMode() { setColor(0,255,255); }
void heatingBedMode() { pulseColor(255,100,0,400); }
void heatingHotendMode() { pulseColor(255,0,0,400); }
void motorStartMode() { setColor(255,255,0); }
void workingMode() {
  setColor(255,0,0); delay(400);
  setColor(255,255,0); delay(400);
  setColor(255,255,255); delay(400);
}
void pausedMode() { blinkColor(255,150,0,400); }
void resumeMode() { blinkColor(0,255,0,200); }
void printCompleteMode() { fadeColor(0,255,0,5); }
void coolingMode() { pulseColor(0,0,255,600); }
void errorMode() { blinkColor(255,0,0,150); }
void idleMode() { setColor(40,40,40); }

void setColor(uint8_t r,uint8_t g,uint8_t b) {
  rgbLedWrite(RGB_BUILTIN,r,g,b);
}

void blinkColor(uint8_t r,uint8_t g,uint8_t b,int d) {
  for(int i=0;i<3;i++){
    setColor(r,g,b); delay(d);
    setColor(0,0,0); delay(d);
  }
}

void pulseColor(uint8_t r,uint8_t g,uint8_t b,int d) {
  for(int i=50;i<=RGB_BRIGHTNESS;i+=25){
    rgbLedWrite(RGB_BUILTIN,(r*i)/255,(g*i)/255,(b*i)/255);
    delay(d/10);
  }
  for(int i=RGB_BRIGHTNESS;i>=50;i-=25){
    rgbLedWrite(RGB_BUILTIN,(r*i)/255,(g*i)/255,(b*i)/255);
    delay(d/10);
  }
}

void fadeColor(uint8_t r,uint8_t g,uint8_t b,int s) {
  for(int i=0;i<=255;i+=5){
    rgbLedWrite(RGB_BUILTIN,(r*i)/255,(g*i)/255,(b*i)/255);
    delay(s);
  }
  for(int i=255;i>=0;i-=5){
    rgbLedWrite(RGB_BUILTIN,(r*i)/255,(g*i)/255,(b*i)/255);
    delay(s);
  }
}