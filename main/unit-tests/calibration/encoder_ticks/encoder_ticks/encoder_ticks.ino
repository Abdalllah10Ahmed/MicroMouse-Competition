#include <Arduino.h>
#include <WiFi.h>

// ============================================================
// WiFi
// ============================================================

// Credentials: paste your own SSID/password when flashing this calibration
// sketch locally — the real values live only in main/secrets.h (git-ignored).
const char *WIFI_SSID = "your-wifi-ssid";
const char *WIFI_PASSWORD = "your-wifi-password";

constexpr uint16_t WIFI_SERIAL_PORT = 23;

WiFiServer wifiServer(WIFI_SERIAL_PORT);
WiFiClient wifiClient;

// ============================================================
// Hardware
// ============================================================

constexpr int MOTOR_ENABLE = 23;

constexpr int MOTOR_PINS[] = {25, 26, 27, 14};

constexpr int LEFT_ENCODER = 33;
constexpr int RIGHT_ENCODER = 35;

constexpr int CELLS = 7;

constexpr float DISTANCE_MM = CELLS * 180.0f;

// ============================================================
// Encoder state
// ============================================================

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

volatile bool recording = false;

unsigned long lastPrint = 0;

// ============================================================
// Encoder interrupts
// ============================================================

void IRAM_ATTR onLeft() {
  if (recording) {
    ++leftTicks;
  }
}

void IRAM_ATTR onRight() {
  if (recording) {
    ++rightTicks;
  }
}

// ============================================================
// Logging
// ============================================================

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

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

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
  Serial.println("WiFi connected successfully.");

  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  wifiServer.begin();
  wifiServer.setNoDelay(true);

  Serial.print("WiFi Serial port: ");
  Serial.println(WIFI_SERIAL_PORT);

  Serial.println("TCP serial server started.");
}

void serviceWiFiClient() {
  if (wifiClient && !wifiClient.connected()) {
    wifiClient.stop();
  }

  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient newClient = wifiServer.available();

    if (newClient) {
      wifiClient = newClient;
      wifiClient.setNoDelay(true);

      logPrintln();
      logPrintln("WiFi client connected.");
      logPrintln("Commands:");
      logPrintln("  r = reset/start recording");
      logPrintln("  s = stop and calculate");
      logPrintln("  p = print current counts");
    }
  }
}

// ============================================================
// Command input
// ============================================================

int readCommand() {
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

// ============================================================
// Reporting
// ============================================================

void report(bool finalResult) {
  unsigned long left;
  unsigned long right;

  noInterrupts();

  left = leftTicks;
  right = rightTicks;

  interrupts();

  String line;

  line = "ticks L/R: ";
  line += String(left);
  line += "/";
  line += String(right);

  logPrintln(line);

  if (!finalResult) {
    return;
  }

  logPrintln(
      "Assuming you manually moved exactly 1260 mm without slipping:");

  logPrint("LEFT_TICKS_PER_CELL = ");
  logPrintln(String(left / (float)CELLS, 3));

  logPrint("RIGHT_TICKS_PER_CELL = ");
  logPrintln(String(right / (float)CELLS, 3));

  logPrint("Left ticks/mm = ");
  logPrintln(String(left / DISTANCE_MM, 6));

  logPrint("Right ticks/mm = ");
  logPrintln(String(right / DISTANCE_MM, 6));

  if (left == 0 || right == 0) {
    logPrintln(
        "INVALID: one or both encoders recorded zero ticks.");
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  // Keep motor driver completely disabled.
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW);

  for (int pin : MOTOR_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  Serial.begin(115200);

  // ----------------------------------------------------------
  // Encoders
  // ----------------------------------------------------------

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENCODER),
      onLeft,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENCODER),
      onRight,
      RISING);

  // ----------------------------------------------------------
  // WiFi
  // ----------------------------------------------------------

  setupWiFi();

  // ----------------------------------------------------------
  // Instructions
  // ----------------------------------------------------------

  logPrintln(
      "Manual encoder calibration: 7 cells = 1260 mm. "
      "Motor driver disabled.");

  logPrintln(
      "r: reset/start; s: stop and calculate; p: print counts.");

  logPrintln(
      "Start at your mark, send r, push straight to 1260 mm, "
      "stop, then send s.");

  if (WiFi.status() == WL_CONNECTED) {
    logPrint("Connect using: ");
    logPrint(WiFi.localIP().toString());
    logPrint(":");
    logPrintln(String(WIFI_SERIAL_PORT));
  }
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  serviceWiFiClient();

  int input = readCommand();

  if (input >= 0) {
    char command = (char)input;

    if (command == 'r' || command == 'R') {
      noInterrupts();

      recording = false;

      leftTicks = 0;
      rightTicks = 0;

      recording = true;

      interrupts();

      lastPrint = millis();

      logPrintln(
          "Recording. Move only forward; reverse movement also adds ticks.");
    }

    else if (command == 's' || command == 'S') {
      noInterrupts();

      recording = false;

      interrupts();

      report(true);
    }

    else if (command == 'p' || command == 'P') {
      report(false);
    }
  }

  // Print live counts every 250 ms.
  if (recording && millis() - lastPrint >= 250) {
    lastPrint = millis();
    report(false);
  }
}