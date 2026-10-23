// ============================================================
// tof_minimal.ino - learn the three Time-of-Flight sensors alone.
// ============================================================
// Hardware on this robot:
//   Left  : VL6180X  on XSHUT pin 4   -> reassigned to I2C address 0x30
//   Right : VL6180X  on XSHUT pin 5   -> reassigned to I2C address 0x31
//   Front : VL53L1X  on XSHUT pin 16  -> reassigned to I2C address 0x32
//   All three share the I2C bus: SDA = 21, SCL = 22.
//
// THE ONE BIG IDEA:
//   Every one of these sensors powers up at the SAME I2C address (0x29).
//   You cannot talk to two devices at the same address on one bus. So we:
//     1. Hold ALL sensors in reset  (XSHUT = LOW),
//     2. Wake ONE sensor, give it a brand-new address,
//     3. Repeat for the next one.
//   XSHUT is just the sensor's "reset/power" pin: LOW = off, HIGH = on.
//
// Upload this like any sketch, open Serial Monitor at 115200 baud, and move a
// flat matte object in front of each sensor. Watch which label changes.
// ============================================================

#include <Wire.h>
#include <VL6180X.h>
#include <VL53L1X.h>

// ----- Pin and address map (kept in one place so it's easy to read) -----
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int LEFT_XSHUT = 4;
constexpr int RIGHT_XSHUT = 5;
constexpr int FRONT_XSHUT = 16;

constexpr uint8_t LEFT_ADDR = 0x30;
constexpr uint8_t RIGHT_ADDR = 0x31;
constexpr uint8_t FRONT_ADDR = 0x32;

// One object per physical sensor. The object is how we send commands to it.
VL6180X leftTof;
VL6180X rightTof;
VL53L1X frontTof;

// VL53L1X::Short = better for close range (this robot measures < ~1 m).
constexpr uint32_t FRONT_TIMING_BUDGET_US = 20000;  // 20 ms per measurement
constexpr uint32_t FRONT_PERIOD_MS = 25;             // continuous mode rate

// Reads one VL6180X. Returns distance in mm, or -1 if the reading is bad.
int readSide(VL6180X &sensor) {
  const uint16_t mm = sensor.readRangeSingleMillimeters();  // blocks until done
  const bool timedOut = sensor.timeoutOccurred();
  const uint8_t rangeStatus = sensor.readRangeStatus();     // 0 = no error
  if (timedOut || sensor.last_status != 0 || mm == 0 || mm >= 255) {
    return -1;  // treat timeout / I2C error / "no target" as invalid
  }
  // range_status 0 means the sensor is happy; non-zero means "can't trust mm".
  return (mm && sensor.last_status == 0) ? (int)mm : -1;
  (void)timedOut;
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // let the Serial Monitor attach before we print
  Serial.println();
  Serial.println("=== Three-ToF minimal test ===");

  // --- 1. Hold every sensor in reset BEFORE touching the bus. -----------
  pinMode(LEFT_XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);
  pinMode(FRONT_XSHUT, OUTPUT);
  digitalWrite(LEFT_XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);
  digitalWrite(FRONT_XSHUT, LOW);
  delay(10);  // let them fully power down

  Wire.begin(SDA_PIN, SCL_PIN);

  // ----- 3. Wake LEFT, give it 0x30 -----
  digitalWrite(LEFT_XSHUT, HIGH);   // now only LEFT is awake (still at 0x29)
  delay(50);                        // boot time
  leftTof.init();
  leftTof.configureDefault();
  leftTof.setAddress(LEFT_ADDR);    // move it off 0x29 so the next one can use it
  leftTof.setTimeout(200);

  // ----- 3. Wake RIGHT, give it 0x31 -----
  digitalWrite(RIGHT_XSHUT, HIGH);  // LEFT is now at 0x30, so no collision
  delay(50);
  rightTof.init();
  rightTof.configureDefault();
  rightTof.setAddress(RIGHT_ADDR);
  rightTof.setTimeout(200);

  // ----- 3. Wake FRONT, give it 0x32 -----
  digitalWrite(FRONT_XSHUT, HIGH);
  delay(50);
  frontTof.init();
  frontTof.setDistanceMode(VL53L1X::Short);
  frontTof.setMeasurementTimingBudget(FRONT_TIMING_BUDGET_US);
  frontTof.setAddress(FRONT_ADDR);
  frontTof.setTimeout(200);
  frontTof.startContinuous(FRONT_PERIOD_MS);  // background ranging

  Serial.println("Init done. Move an object in front of each sensor.");
  Serial.println("L / R / F are millimetres. 'bad' means the reading is not usable.");
}

void loop() {
  const int left = readSide(leftTof);
  const int right = readSide(rightTof);

  // Front runs in continuous mode, so read() just fetches the latest result.
  const uint16_t front = frontTof.read();
  const bool frontBad = frontTof.timeoutOccurred() ||
                        frontTof.ranging_data.range_status != VL53L1X::RangeValid;

  Serial.print("L=");
  left < 0 ? Serial.print("bad") : Serial.print(left);
  Serial.print("mm  R=");
  right < 0 ? Serial.print("bad") : Serial.print(right);
  Serial.print("mm  F=");
  frontBad ? Serial.print("bad") : Serial.print(front);
  Serial.print("mm");
  Serial.println();

  delay(200);  // 5 readings per second is plenty for eyeballing
}
