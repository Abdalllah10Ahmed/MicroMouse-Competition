// ============================================================
// buttons.ino - LIVE monitor for the two physical push buttons.
// ============================================================
// Start button : GPIO 18  -> toggles the onboard LED on/off
// Clear button : GPIO 16  -> forces the onboard LED off
// Onboard LED  : GPIO 2 (classic ESP32 DevKit blue LED)
//
// This version PRINTS CONSTANTLY so you can see whether the pins are
// connected at all:
//   * "WIRING" lines at boot show each pin under pull-up / pull-down / no-pull.
//   * "CHANGE" lines print every time a pin's raw level moves (no filtering),
//     so even bounce or a single blip is visible.
//   * "ALIVE"  lines print once a second with the current levels, so even a
//     completely stuck/dead pin is still visible.
//
// WIRING: the default assumes each button connects its GPIO to GND and the
// pin's internal pull-up is enabled. Released -> HIGH, pressed -> LOW.
//   If you press and see NO "CHANGE" line, the button is probably wired to
//   3V3 instead: set ACTIVE_LOW = false and upload again.
//
// !! GPIO 16 is ALSO the front ToF's XSHUT pin in the real firmware
//    (main/MoveForward.cpp: FRONT_XSHUT = 16). Safe to run this test alone
//    (the ToF is not initialized here), but the two cannot coexist later.
// ============================================================

#include <Arduino.h>

constexpr int START_BUTTON_PIN = 18;
constexpr int CLEAR_BUTTON_PIN = 16;
constexpr int LED_PIN = 2;

constexpr bool ACTIVE_LOW = true;  // see note above
constexpr unsigned long DEBOUNCE_MS = 30;
constexpr unsigned long HEARTBEAT_MS = 1000;

const int IDLE_LEVEL = ACTIVE_LOW ? HIGH : LOW;
const int PRESSED_LEVEL = ACTIVE_LOW ? LOW : HIGH;

bool ledOn = false;
int lastStartRaw = -1;  // -1 forces the first CHANGE line to print
int lastClearRaw = -1;
unsigned long lastHeartbeatMs = 0;

struct Button {
  int lastRaw;
  int stable;
  unsigned long lastChangeMs;
};

Button startButton = {IDLE_LEVEL, IDLE_LEVEL, 0};
Button clearButton = {IDLE_LEVEL, IDLE_LEVEL, 0};

const char *levelName(int level) { return level == HIGH ? "HIGH" : "LOW"; }

void setLed(bool on) {
  ledOn = on;
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void configureInput(int pin) {
  pinMode(pin, ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
}

// Debounced press detector, used only for the LED behaviour.
bool pressedEdge(int pin, Button &b) {
  const int raw = digitalRead(pin);
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChangeMs = millis();
  }
  if (millis() - b.lastChangeMs >= DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    return b.stable == PRESSED_LEVEL;
  }
  return false;
}

// Read the pin under each internal pull mode. This shows whether the pin is
// being held by external hardware or is merely following our internal pull.
void reportWiring(const char *name, int pin) {
  pinMode(pin, INPUT_PULLUP);   delay(2);
  const int up = digitalRead(pin);
  pinMode(pin, INPUT_PULLDOWN); delay(2);
  const int down = digitalRead(pin);
  pinMode(pin, INPUT);          delay(2);
  const int floating = digitalRead(pin);

  Serial.print("WIRING ");
  Serial.print(name);
  Serial.print(" GPIO"); Serial.print(pin);
  Serial.print(": pull-up=");   Serial.print(levelName(up));
  Serial.print("  pull-down="); Serial.print(levelName(down));
  Serial.print("  no-pull=");   Serial.println(levelName(floating));
  Serial.print("     -> ");
  if (up == down) {
    Serial.println("held by external hardware (does NOT follow our pull)");
  } else {
    Serial.println("follows our internal pull (switch open, or nothing connected)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // let the Serial Monitor attach before printing

  setLed(false);

  Serial.println();
  Serial.println("=== Push-button LIVE monitor ===");
  Serial.print("Start=GPIO"); Serial.print(START_BUTTON_PIN);
  Serial.print("  Clear=GPIO"); Serial.print(CLEAR_BUTTON_PIN);
  Serial.print("  LED=GPIO"); Serial.println(LED_PIN);

  reportWiring("START", START_BUTTON_PIN);
  reportWiring("CLEAR", CLEAR_BUTTON_PIN);

  configureInput(START_BUTTON_PIN);
  configureInput(CLEAR_BUTTON_PIN);

  Serial.println();
  Serial.println(ACTIVE_LOW
      ? "Listening with INPUT_PULLUP (released=HIGH, pressed=LOW)."
      : "Listening with INPUT_PULLDOWN (released=LOW, pressed=HIGH).");
  Serial.println("Press each button and watch for CHANGE lines.");
  Serial.println("No CHANGE line while pressing? Try ACTIVE_LOW = false.");
  Serial.println("A pin LOW under BOTH pulls is shorted/stuck to GND.");
  Serial.println("-----------------------------------------------------------");
}

void loop() {
  const int startRaw = digitalRead(START_BUTTON_PIN);
  const int clearRaw = digitalRead(CLEAR_BUTTON_PIN);

  // 1. Report EVERY raw change immediately -- this is the "is it connected?"
  //    evidence. No debounce is applied here on purpose.
  if (startRaw != lastStartRaw || clearRaw != lastClearRaw) {
    Serial.print("CHANGE  ms="); Serial.print(millis());
    Serial.print("  START="); Serial.print(levelName(startRaw));
    Serial.print("  CLEAR="); Serial.println(levelName(clearRaw));
    lastStartRaw = startRaw;
    lastClearRaw = clearRaw;
  }

  // 2. Heartbeat: proves the sketch is running and reveals a stuck steady state.
  if (millis() - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = millis();
    Serial.print("ALIVE   ms="); Serial.print(millis());
    Serial.print("  START="); Serial.print(levelName(digitalRead(START_BUTTON_PIN)));
    Serial.print("  CLEAR="); Serial.print(levelName(digitalRead(CLEAR_BUTTON_PIN)));
    Serial.print("  LED="); Serial.println(ledOn ? "ON" : "OFF");
  }

  // LED behaviour on the debounced press edge.
  if (pressedEdge(START_BUTTON_PIN, startButton)) {
    setLed(!ledOn);
    Serial.print(">> START pressed -> LED ");
    Serial.println(ledOn ? "ON" : "OFF");
  }
  if (pressedEdge(CLEAR_BUTTON_PIN, clearButton)) {
    setLed(false);
    Serial.println(">> CLEAR pressed -> LED OFF");
  }
}
