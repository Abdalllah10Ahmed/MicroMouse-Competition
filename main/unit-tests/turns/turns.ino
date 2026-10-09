#include <WiFi.h>

#include "rotation.h"

// Credentials: paste your own SSID/password when flashing this calibration
// sketch locally — the real values live only in main/secrets.h (git-ignored).
const char *WIFI_SSID = "your-wifi-ssid";
const char *WIFI_PASSWORD = "your-wifi-password";

constexpr uint16_t WIFI_SERIAL_PORT = 23;

WiFiServer wifiServer(WIFI_SERIAL_PORT);
WiFiClient wifiClient;

void logPrint(const String &text) {
  Serial.print(text);

  if (wifiClient && wifiClient.connected()) {
    wifiClient.print(text);
  }
}

void logPrintln(const String &text = "") {
  Serial.println(text);

  if (wifiClient && wifiClient.connected()) {
    wifiClient.println(text);
  }
}

// ============================================================
// WiFi
// ============================================================

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  logPrint("Connecting to WiFi: ");
  logPrintln(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - start > 15000) {
      Serial.println();
      Serial.println("WiFi connection timeout.");
      return;
    }
  }

  Serial.println();
  logPrintln("WiFi connected successfully.");

  logPrint("ESP32 IP address: ");
  logPrintln(WiFi.localIP().toString());

  wifiServer.begin();
  wifiServer.setNoDelay(true);

  logPrint("WiFi Serial port: ");
  logPrintln(String(WIFI_SERIAL_PORT));

  logPrintln("TCP serial server started.");
}

void serviceWiFiClient() {
  // Remove dead client.
  if (wifiClient && !wifiClient.connected()) {
    wifiClient.stop();
  }

  // Accept a new client.
  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient newClient = wifiServer.available();

    if (newClient) {
      wifiClient = newClient;
      wifiClient.setNoDelay(true);

      logPrintln();
      logPrintln("WiFi client connected.");
      logPrintln("Commands:");
      logPrintln("  l = simultaneous-wheel left 90 degrees");
      logPrintln("  r = simultaneous-wheel right 90 degrees");
      logPrintln("  d/x = stop");
    }
  }
}

int readCommand() {
  // USB Serial has priority.
  if (Serial.available()) {
    return Serial.read();
  }

  serviceWiFiClient();

  if (wifiClient &&
      wifiClient.connected() &&
      wifiClient.available()) {

    return wifiClient.read();
  }

  return -1;
}

bool abortRequested() {
  bool stop = false;

  // Check USB serial.
  while (Serial.available()) {
    char command = Serial.read();

    if (command == 'd' ||
        command == 'D' ||
        command == 'x' ||
        command == 'X') {
      stop = true;
    }
  }

  // Check WiFi serial.
  serviceWiFiClient();

  while (wifiClient &&
         wifiClient.connected() &&
         wifiClient.available()) {

    char command = wifiClient.read();

    if (command == 'd' ||
        command == 'D' ||
        command == 'x' ||
        command == 'X') {
      stop = true;
    }
  }

  return stop;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  beginRotation();
  setupWiFi();

  logPrintln("l: left 90 degrees, r: right 90 degrees.");
  logPrintln("d or x: stop. No movement starts automatically.");

  if (WiFi.status() == WL_CONNECTED) {
    logPrint("Connect using: ");
    logPrint(WiFi.localIP().toString());
    logPrint(":");
    logPrintln(String(WIFI_SERIAL_PORT));
  }
}

void loop() {
  serviceWiFiClient();
  int input = readCommand();

  if (input == 'l' || input == 'L') {
    turnDegrees(90);
  } else if (input == 'r' || input == 'R') {
    turnDegrees(-90);
  } else if (input == 'd' || input == 'D' ||
             input == 'x' || input == 'X') {
    stopRotation();
    logPrintln("STOP.");
  }
}
