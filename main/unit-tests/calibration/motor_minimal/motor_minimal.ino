// ============================================================
// motor_minimal.ino - motors + encoders, over USB Serial AND WiFi.
// ============================================================
// !! LIFT BOTH WHEELS OFF THE GROUND BEFORE POWERING/FLASHING !!
//
// Same motor/encoder test as before, but now you can also drive it over WiFi.
// How the WiFi part works:
//   1. The ESP32 joins your home router as a "station" (STA), using the
//      credentials in main/secrets.h.
//   2. The router gives it an IP address (DHCP), which we print on Serial.
//   3. The ESP32 runs a tiny TCP server on port 23 (the old "telnet" port).
//      Anything you type there arrives as the same single letters, and all
//      replies are sent back to BOTH USB Serial and the WiFi client.
//
// Pin mapping is the VERIFIED one from main/MoveForward.cpp:
//   physical LEFT  motor: IN1=27, IN2=14
//   physical RIGHT motor: IN1=25, IN2=26
//   enable = GPIO 23 (HIGH = enabled)
//   LEFT encoder = GPIO 33 (INPUT_PULLUP), RIGHT encoder = GPIO 35 (input-only)
// Single-channel encoders: they count how far a wheel turned, not direction.
//
// Commands (type in Serial Monitor or in the WiFi console):
//   l = left fwd   L = left back    r = right fwd   R = right back
//   b = both fwd   B = both back    s = stop        e = print counts
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "../../../secrets.h"   // WIFI_SSID / WIFI_PASSWORD (git-ignored)

constexpr uint16_t WIFI_PORT = 23;
WiFiServer wifiServer(WIFI_PORT);
WiFiClient wifiClient;
bool wifiReady = false;

constexpr int LEFT_IN1 = 27, LEFT_IN2 = 14;
constexpr int RIGHT_IN1 = 25, RIGHT_IN2 = 26;
constexpr int MOTOR_EN = 23;
constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;

constexpr int TEST_PWM = 255;          // gentle; raise if a wheel stalls
constexpr unsigned long RUN_MS = 2000;

constexpr int MOVE_NONE = 0;
constexpr int MOVE_LEFT_FWD = 1, MOVE_LEFT_BACK = 2;
constexpr int MOVE_RIGHT_FWD = 3, MOVE_RIGHT_BACK = 4;
constexpr int MOVE_BOTH_FWD = 5, MOVE_BOTH_BACK = 6;

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

void IRAM_ATTR countLeft() { ++leftTicks; }
void IRAM_ATTR countRight() { ++rightTicks; }

int active = MOVE_NONE;
unsigned long moveStart = 0;
unsigned long baseLeft = 0, baseRight = 0;

// ---------- output to USB Serial and (if connected) the WiFi client ----------

void send(const String &s) {
  Serial.print(s);
  if (wifiClient && wifiClient.connected()) wifiClient.print(s);
}

void sendln(const String &s = "") {
  Serial.println(s);
  if (wifiClient && wifiClient.connected()) wifiClient.println(s);
}

// ---------- WiFi ----------

constexpr const char *MDNS_NAME = "motor-test";  // reachable as motor-test.local

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_NAME);   // friendly name in the router's client list
  send("Connecting to WiFi: \""); send(WIFI_SSID); sendln("\"");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    send(".");
    if (millis() - start > 15000) {
      sendln();
      sendln("WiFi FAILED (wrong password? out of range?). USB Serial still works.");
      return;
    }
  }

  sendln();
  sendln("WiFi connected.");
  send("ESP32 IP address: "); sendln(WiFi.localIP().toString());
  send("TCP port: "); sendln(String(WIFI_PORT));

  wifiServer.begin();
  wifiServer.setNoDelay(true);
  wifiReady = true;

  // mDNS lets clients reach the board by name instead of a changing IP.
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("telnet", "tcp", WIFI_PORT);
    send("mDNS name: "); send(MDNS_NAME); sendln(".local");
  } else {
    sendln("mDNS failed; use the IP above.");
  }
}

// Accept a client / drop a dead one. Call this often.
void serviceWiFi() {
  if (!wifiReady) return;
  if (wifiClient && !wifiClient.connected()) wifiClient.stop();
  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient incoming = wifiServer.available();
    if (incoming) {
      wifiClient = incoming;
      wifiClient.setNoDelay(true);
      sendln();
      sendln("TCP client connected. Commands:");
      sendln("  l/L left fwd/back   r/R right fwd/back");
      sendln("  b/B both fwd/back   s stop   e counts");
    }
  }
}

// Read one command character from either USB Serial or the WiFi client.
int readCommand() {
  if (Serial.available()) return Serial.read();
  serviceWiFi();
  if (wifiClient && wifiClient.connected() && wifiClient.available()) {
    return wifiClient.read();
  }
  return -1;
}

// ---------- motors + encoders ----------

void snapshot(unsigned long &l, unsigned long &r) {
  noInterrupts();
  l = leftTicks;
  r = rightTicks;
  interrupts();
}

void brake() {
  analogWrite(LEFT_IN1, 255);  analogWrite(LEFT_IN2, 255);
  analogWrite(RIGHT_IN1, 255); analogWrite(RIGHT_IN2, 255);
}

void leftForward()   { analogWrite(LEFT_IN1, 0);         analogWrite(LEFT_IN2, TEST_PWM); }
void leftBackward()  { analogWrite(LEFT_IN1, TEST_PWM);  analogWrite(LEFT_IN2, 0); }
void rightForward()  { analogWrite(RIGHT_IN1, TEST_PWM); analogWrite(RIGHT_IN2, 0); }
void rightBackward() { analogWrite(RIGHT_IN1, 0);         analogWrite(RIGHT_IN2, TEST_PWM); }

void applyMove(int m) {
  brake();
  switch (m) {
    case MOVE_LEFT_FWD:   leftForward();                   break;
    case MOVE_LEFT_BACK:  leftBackward();                  break;
    case MOVE_RIGHT_FWD:  rightForward();                  break;
    case MOVE_RIGHT_BACK: rightBackward();                 break;
    case MOVE_BOTH_FWD:   leftForward();  rightForward();  break;
    case MOVE_BOTH_BACK:  leftBackward(); rightBackward(); break;
    default: break;
  }
}

void printTicks(const String &label) {
  unsigned long l, r;
  snapshot(l, r);
  send(label + ": delta L/R = " + String(l - baseLeft) + "/" + String(r - baseRight) +
       "   totals L/R = " + String(l) + "/" + String(r));
}

void startMove(int m, const String &name) {
  brake();
  snapshot(baseLeft, baseRight);
  moveStart = millis();
  active = m;
  applyMove(m);
  sendln(">> " + name + " at PWM " + String(TEST_PWM) + " for " + String(RUN_MS) + " ms...");
}

void stopNow() {
  brake();
  if (active != MOVE_NONE) printTicks("stopped");
  else                     sendln("stopped (was already idle)");
  active = MOVE_NONE;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  brake();

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), countRight, RISING);

  sendln("=== Motor + encoder test (USB + WiFi) ===");
  sendln("LIFT BOTH WHEELS OFF THE GROUND.");
  sendln("l=left fwd  L=left back  r=right fwd  R=right back");
  sendln("b=both fwd  B=both back  s=stop  e=print counts");

  setupWiFi();

  digitalWrite(MOTOR_EN, HIGH);
  snapshot(baseLeft, baseRight);
  sendln("Ready. Send a command.");
}

void loop() {
  serviceWiFi();

  if (active != MOVE_NONE && millis() - moveStart >= RUN_MS) {
    brake();
    printTicks("move finished");
    active = MOVE_NONE;
  }

  int c;
  while ((c = readCommand()) >= 0) {
    switch (c) {
      case 'l': startMove(MOVE_LEFT_FWD,   "LEFT forward");   break;
      case 'L': startMove(MOVE_LEFT_BACK,  "LEFT backward");  break;
      case 'r': startMove(MOVE_RIGHT_FWD,  "RIGHT forward");  break;
      case 'R': startMove(MOVE_RIGHT_BACK, "RIGHT backward"); break;
      case 'b': startMove(MOVE_BOTH_FWD,   "BOTH forward");   break;
      case 'B': startMove(MOVE_BOTH_BACK,  "BOTH backward");  break;
      case 's': stopNow(); break;
      case 'e': printTicks("counts"); break;
      default: break;
    }
  }
}
