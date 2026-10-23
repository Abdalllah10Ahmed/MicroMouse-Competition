// ============================================================
// buttons_scan.ino - find WHICH GPIO a push button is wired to.
// ============================================================
// Use this when the known pins (18 and 16) do not respond.
// It watches a list of safe GPIOs at once, each as an INPUT with its internal
// pull-up enabled. Nothing is ever DRIVEN, only read, so it is safe to run.
// Press one button at a time and watch which GPIO changes.
//
// The onboard BOOT button is GPIO 0 on most ESP32 devkits -- press it to
// confirm the scanner works (you should see GPIO 0 change).
//
// If you press and see NOTHING, the button may be wired to 3V3: set
// ACTIVE_LOW = false below (switches to pull-down) and upload again.
//
// Skipped on purpose: UART (1, 3), SPI flash (6-11), input-only (34-39),
// strapping pins (12, 15), and the LED (2).
// ============================================================

#include <Arduino.h>

constexpr bool ACTIVE_LOW = true;  // see note above
constexpr unsigned long HEARTBEAT_MS = 2000;

const int PINS[] = {0, 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
constexpr int PIN_COUNT = sizeof(PINS) / sizeof(PINS[0]);

int lastLevel[PIN_COUNT];
unsigned long lastHeartbeatMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Button pin scanner ===");
  Serial.print("Watching GPIOs:");
  for (int i = 0; i < PIN_COUNT; i++) { Serial.print(' '); Serial.print(PINS[i]); }
  Serial.println();
  Serial.println(ACTIVE_LOW
      ? "Mode: INPUT_PULLUP (a pressed button pulls its pin LOW)"
      : "Mode: INPUT_PULLDOWN (a pressed button pulls its pin HIGH)");
  Serial.println("Press ONE button. The GPIO that CHANGES is that button's pin.");
  Serial.println("Tip: the onboard BOOT button is GPIO 0.");
  Serial.println("-----------------------------------------------------------");

  for (int i = 0; i < PIN_COUNT; i++) {
    pinMode(PINS[i], ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    lastLevel[i] = digitalRead(PINS[i]);
  }

  Serial.print("Idle levels:");
  for (int i = 0; i < PIN_COUNT; i++) {
    Serial.print(" G"); Serial.print(PINS[i]); Serial.print('=');
    Serial.print(lastLevel[i] == HIGH ? "H" : "L");
  }
  Serial.println();
}

void loop() {
  for (int i = 0; i < PIN_COUNT; i++) {
    const int level = digitalRead(PINS[i]);
    if (level != lastLevel[i]) {
      Serial.print("CHANGE ms=");
      Serial.print(millis());
      Serial.print("  GPIO ");
      Serial.print(PINS[i]);
      Serial.print(level == HIGH ? " -> HIGH" : " -> LOW");
      Serial.println();
      lastLevel[i] = level;
    }
  }

  if (millis() - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = millis();
    Serial.print("ALIVE ms=");
    Serial.print(millis());
    Serial.println("  (press a button and watch for a CHANGE line)");
  }
}
