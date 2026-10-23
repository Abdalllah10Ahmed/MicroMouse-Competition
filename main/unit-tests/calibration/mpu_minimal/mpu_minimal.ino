// ============================================================
// mpu_minimal.ino - understand the MPU-6050 one register at a time.
// ============================================================
// The MPU-6050 is an I2C chip with a 3-axis accelerometer, a 3-axis gyro and a
// thermometer, all at address 0x68 on the shared bus (SDA=21, SCL=22).
//
// What the ROBOT actually uses: only the gyro Z rate. It integrates that into
// a heading (yaw). The accelerometer is not used by the robot at all, but we
// print it here so you can see the whole chip physically respond.
//
// A still gyro still reports a small nonzero rate (its "bias"). We measure it
// at boot while you hold the robot still, then subtract it -- exactly what the
// firmware's MpuYaw class does.
// ============================================================

#include <Wire.h>
#include <math.h>

constexpr uint8_t MPU_ADDR = 0x68;

// Register map (only the ones we touch).
constexpr uint8_t REG_CONFIG = 0x1A;        // digital low-pass filter
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;   // gyro full-scale range
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;   // gyro Z, high byte
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // start of the 14-byte data block
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;    // sleep / wake
constexpr uint8_t REG_WHO_AM_I = 0x75;      // identity

// Scale factors for the ranges we select.
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 deg/s
constexpr float ACCEL_LSB_PER_G = 16384.0f;  // +/-2 g

// The three ToF sensors share this bus; hold them in reset so they stay quiet.
constexpr int LEFT_XSHUT = 4, RIGHT_XSHUT = 5, FRONT_XSHUT = 16;

float gyroZBias = 0.0f;  // measured at boot, subtracted from every sample

unsigned long prevMs = 0;
unsigned long lastPrintMs = 0;
float yaw = 0.0f;

// ---------- Low-level I2C helpers ----------

bool readRegs(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

// Read just the gyro Z raw value (this is all the robot ever reads).
bool readGyroZraw(int16_t &raw) {
  uint8_t b[2];
  if (!readRegs(REG_GYRO_ZOUT_H, b, 2)) return false;
  raw = toInt16(b[0], b[1]);
  return true;
}

// Read the whole 14-byte block in one go so all values share one instant:
// raw[0..2]=accel XYZ, raw[3]=temperature, raw[4..6]=gyro XYZ.
bool readAllRaw(int16_t raw[7]) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) return false;
  raw[0] = toInt16(b[0], b[1]);
  raw[1] = toInt16(b[2], b[3]);
  raw[2] = toInt16(b[4], b[5]);
  raw[3] = toInt16(b[6], b[7]);
  raw[4] = toInt16(b[8], b[9]);
  raw[5] = toInt16(b[10], b[11]);
  raw[6] = toInt16(b[12], b[13]);
  return true;
}

bool configure() {
  uint8_t identity = 0;
  if (!readRegs(REG_WHO_AM_I, &identity, 1)) {
    Serial.println("I2C read failed. Check SDA=21, SCL=22 and MPU power.");
    return false;
  }
  if (identity != 0x68) {
    Serial.print("WHO_AM_I = 0x"); Serial.print(identity, HEX);
    Serial.println(" (expected 0x68). Wrong chip or bad wiring.");
    return false;
  }
  Serial.println("WHO_AM_I = 0x68  -> MPU-6050 detected.");

  if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;   // wake, gyro X PLL clock
  delay(100);
  if (!writeReg(REG_CONFIG, 0x06)) return false;       // DLPF ~5 Hz, as firmware
  if (!writeReg(REG_GYRO_CONFIG, 0x08)) return false;  // +/-500 deg/s, as firmware
  delay(100);

  uint8_t pwr = 0, gyroCfg = 0;
  readRegs(REG_PWR_MGMT_1, &pwr, 1);
  readRegs(REG_GYRO_CONFIG, &gyroCfg, 1);
  Serial.print("PWR_MGMT_1=0x"); Serial.print(pwr, HEX);
  Serial.print("  GYRO_CONFIG=0x"); Serial.println(gyroCfg, HEX);
  return pwr == 0x01 && gyroCfg == 0x08;
}

void calibrateGyroZ() {
  Serial.println("Calibrating gyro bias - keep the robot COMPLETELY still...");
  float sum = 0.0f;
  int good = 0;
  for (int i = 0; i < 200; ++i) {
    int16_t raw;
    if (readGyroZraw(raw)) { sum += raw / GYRO_LSB_PER_DPS; ++good; }
    delay(10);
  }
  gyroZBias = (good > 0) ? sum / good : 0.0f;
  Serial.print("Bias = ");
  Serial.print(gyroZBias, 3);
  Serial.println(" deg/s");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Hold the ToF sensors in reset so they stay off this shared bus.
  pinMode(LEFT_XSHUT, OUTPUT);  digitalWrite(LEFT_XSHUT, LOW);
  pinMode(RIGHT_XSHUT, OUTPUT); digitalWrite(RIGHT_XSHUT, LOW);
  pinMode(FRONT_XSHUT, OUTPUT); digitalWrite(FRONT_XSHUT, LOW);

  Wire.begin(21, 22);

  Serial.println();
  Serial.println("=== MPU-6050 minimal test ===");
  if (!configure()) {
    Serial.println("Initialization failed. Reset the board to retry.");
    while (true) delay(10);
  }
  calibrateGyroZ();

  Serial.println("Ready.");
  Serial.println("  Still          -> yaw stays near 0, accel Z ~ 1.0 g");
  Serial.println("  Rotate by hand -> yaw changes (positive = physical left)");
  Serial.println("  Tilt           -> accel X/Y/Z change");
  Serial.println("accel X/Y/Z [g] | temp [C] | gyro X/Y/Z [deg/s] | yaw [deg]");

  prevMs = millis();
  lastPrintMs = prevMs;
}

void loop() {
  const unsigned long now = millis();
  const float dt = (now - prevMs) / 1000.0f;
  prevMs = now;

  int16_t raw[7];
  if (!readAllRaw(raw)) {
    Serial.println("read failed");
    delay(50);
    return;
  }

  const float ax = raw[0] / ACCEL_LSB_PER_G;
  const float ay = raw[1] / ACCEL_LSB_PER_G;
  const float az = raw[2] / ACCEL_LSB_PER_G;
  const float tempC = raw[3] / 340.0f + 36.53f;
  const float gx = raw[4] / GYRO_LSB_PER_DPS;
  const float gy = raw[5] / GYRO_LSB_PER_DPS;
  const float gz = raw[6] / GYRO_LSB_PER_DPS;

  // The firmware's convention: negate so positive = physical left, then a
  // 0.5 deg/s deadband so a still robot does not slowly accumulate yaw.
  float rateLeft = -(gz - gyroZBias);
  if (fabsf(rateLeft) < 0.5f) rateLeft = 0.0f;
  yaw += rateLeft * dt;

  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;
    Serial.print(ax, 2); Serial.print('/');
    Serial.print(ay, 2); Serial.print('/');
    Serial.print(az, 2); Serial.print(" g | ");
    Serial.print(tempC, 1); Serial.print(" C | ");
    Serial.print(gx, 1); Serial.print('/');
    Serial.print(gy, 1); Serial.print('/');
    Serial.print(gz, 1); Serial.print(" dps | yaw=");
    Serial.print(yaw, 1); Serial.println(" deg");
  }

  delay(5);
}
