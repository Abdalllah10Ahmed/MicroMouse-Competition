#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <VL6180X.h>
#include <VL53L1X.h>
#include "MoveForward.h"

class MpuYaw {
 public:
  bool begin() {
    uint8_t identity;
    if (!readBytes(0x75, &identity, 1) || identity != 0x68) return false;
    if (!writeByte(0x6B, 0x01)) return false;
    delay(100);
    if (!writeByte(0x1A, 0x06) || !writeByte(0x1B, 0x08)) return false;
    delay(100);

    float sum = 0.0f;
    for (int i = 0; i < 200; ++i) {
      float rate;
      if (!readGyroZ(rate)) return false;
      sum += rate;
      delay(10);
    }
    gyroZBias_ = sum / 200.0f;
    ready_ = isfinite(gyroZBias_);
    reset();
    return ready_;
  }

  void reset() {
    yaw_ = 0.0f;
    rate_ = 0.0f;
    lastPollMs_ = millis();
    lastGoodMs_ = lastPollMs_;
  }

  void service() {
    if (!ready_) return;
    unsigned long now = millis();
    unsigned long elapsed = now - lastPollMs_;
    if (elapsed < 10) return;
    lastPollMs_ = now;

    float gyroZ;
    if (!readGyroZ(gyroZ)) {
      ready_ = false;
      return;
    }

    // Match rotation.cpp: physical right rotation is positive.
    rate_ = -(gyroZ - gyroZBias_);
    if (fabsf(rate_) < 0.5f) rate_ = 0.0f;

    // Do not integrate one sample across a long blocking sensor interval.
    if (elapsed <= 100) yaw_ += rate_ * (elapsed / 1000.0f);
    lastGoodMs_ = now;
  }

  bool healthy() const {
    return ready_ && (unsigned long)(millis() - lastGoodMs_) <= 250;
  }

  float yaw() const { return yaw_; }
  float rate() const { return rate_; }

 private:
  bool readBytes(uint8_t reg, uint8_t *data, uint8_t count) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom((uint8_t)0x68, count) != count) {
      return false;
    }
    for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
    return true;
  }

  bool writeByte(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  bool readGyroZ(float &rate) {
    uint8_t data[2];
    if (!readBytes(0x47, data, 2)) return false;
    int16_t raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    rate = raw / 65.5f;
    return true;
  }

  float gyroZBias_ = 0.0f;
  float yaw_ = 0.0f;
  float rate_ = 0.0f;
  bool ready_ = false;
  unsigned long lastPollMs_ = 0;
  unsigned long lastGoodMs_ = 0;
};

// ============================================================
// WiFi configuration
// ============================================================

// Credentials: paste your own SSID/password when flashing this calibration
// sketch locally — the real values live only in main/secrets.h (git-ignored).
const char* WIFI_SSID = "your-wifi-ssid";
const char* WIFI_PASSWORD = "your-wifi-password";

constexpr uint16_t WIFI_SERIAL_PORT = 23;

WiFiServer wifiSerialServer(WIFI_SERIAL_PORT);
WiFiClient wifiSerialClient;

// ============================================================
// Hardware configuration
// ============================================================

// Confirmed by isolated motor test: 27/14 drives physical LEFT, 25/26 RIGHT.
constexpr int LEFT_IN1 = 27, LEFT_IN2 = 14;
constexpr int RIGHT_IN1 = 25, RIGHT_IN2 = 26, MOTOR_EN = 23;

constexpr int LEFT_ENCODER = 33, RIGHT_ENCODER = 35;

constexpr int LEFT_XSHUT = 4, RIGHT_XSHUT = 5, FRONT_XSHUT = 16;

// Two hand-pushed trials over 7 cells (1260 mm): L/R
// 4363/4360, 4265/4283. Both trials were confirmed as 1260 mm.
constexpr float LEFT_TICKS_PER_CELL =
    ((4363.0f + 4265.0f) / 2.0f) / 7.0f;

constexpr float RIGHT_TICKS_PER_CELL =
    ((4360.0f + 4283.0f) / 2.0f) / 7.0f;

constexpr int CELL_LENGTH_MM = 180;
constexpr int FORWARD_CELLS = 7;
constexpr int FRONT_EMERGENCY_STOP_MM = 40;

constexpr unsigned long LEFT_TARGET_TICKS =
    (unsigned long)(LEFT_TICKS_PER_CELL * FORWARD_CELLS + 0.5f);

constexpr unsigned long RIGHT_TARGET_TICKS =
    (unsigned long)(RIGHT_TICKS_PER_CELL * FORWARD_CELLS + 0.5f);

static_assert(
    LEFT_TARGET_TICKS == 4314 &&
    RIGHT_TARGET_TICKS == 4322,
    "Check continuous seven-cell targets from calibration"
);

constexpr int FORWARD_TARGET_MM =
    CELL_LENGTH_MM * FORWARD_CELLS;

constexpr int FORWARD_SPEED = 140;         //////////// speeeed

constexpr unsigned long MOVE_TIMEOUT_MS =
    105000UL; // Whole seven-cell run; stall timeout remains 1500 ms.
    
constexpr unsigned long STALL_TIMEOUT_MS = 1500;

// Keep each factor on its original electrical output while fixing side labels.
constexpr float RIGHT_FACTOR = 1.0f;
constexpr float LEFT_FACTOR = 0.99f;

// Working main.ino behavior: asymmetric thresholds and slow the same named motor.
constexpr int WALL_SLOW_SPEED = 30;
constexpr int LEFT_WALL_THRESHOLD_MM = 40;
constexpr int RIGHT_TOF_INSET_MM = 10;
constexpr int RIGHT_WALL_THRESHOLD_MM = 40; // Chassis clearance: formerly 50 mm raw.
// A healthy VL6180X can report no valid target when the nearest wall is outside
// its useful range. Keep that distinct from an I2C/timeout fault and classify it
// as open space, safely above the 140 mm wall-retention threshold.
constexpr int SIDE_NO_TARGET_MM = 200;
// Case 1: center between two side walls using right-clearance minus left-clearance.
constexpr float TWO_WALL_KP = 7.3f;
constexpr float TWO_WALL_KI = 0.0f;
constexpr float TWO_WALL_KD = 5.7f;
constexpr float TWO_WALL_MAX_PWM = 40.0f;
constexpr float TWO_WALL_TOLERANCE_MM = 3.6f;
constexpr TwoWallPidSettings TWO_WALL_PID_SETTINGS = {
    TWO_WALL_TOLERANCE_MM, WALL_SLOW_SPEED,
    TWO_WALL_KP, TWO_WALL_KI, TWO_WALL_KD, TWO_WALL_MAX_PWM
};
TwoWallPidController twoWallPid;
// Distance PID changes forward speed only; preserve the working steering direction.
constexpr float APPROACH_SLOWDOWN_TICKS = 300.0f;
constexpr int MIN_APPROACH_PWM = 35;
constexpr CellApproachSettings APPROACH_SETTINGS = {
    0.24f, 0.01f, 0.01f, // Gentler approach response; tune against measured travel.
    APPROACH_SLOWDOWN_TICKS, // Begin approach about 87 mm before braking.
    FORWARD_SPEED, MIN_APPROACH_PWM
};
constexpr unsigned long BRAKE_LEAD_TICKS = 10; // Brake about 2.9 mm before the full-run endpoint.
static_assert(BRAKE_LEAD_TICKS < LEFT_TARGET_TICKS && BRAKE_LEAD_TICKS < RIGHT_TARGET_TICKS,
              "Brake lead must be smaller than the run target");
CellApproachController approachPid(APPROACH_SETTINGS);
// This installation reports negative raw yaw for the direction that must be
// corrected by slowing the left wheel. Reverse it so yaw and encoder steering
// request the same correction.
constexpr float MPU_YAW_SIGN = -1.0f;
constexpr float SINGLE_WALL_KP = 6.0f;
constexpr float SINGLE_WALL_KI = 0.0f;
constexpr float SINGLE_WALL_KD = 2.5f;
constexpr float SINGLE_WALL_MAX_PWM = 40.0f;
constexpr SingleWallSettings SINGLE_WALL_SETTINGS = {
    (float)LEFT_WALL_THRESHOLD_MM, (float)RIGHT_WALL_THRESHOLD_MM, 3.6f,
    WALL_SLOW_SPEED, // Minimum commanded PWM during correction.
    2.0f, 0.15f, 15.0f, // Stronger MPU trim inside the distance band.
    SINGLE_WALL_KP, SINGLE_WALL_KI, SINGLE_WALL_KD, SINGLE_WALL_MAX_PWM
};
SingleWallController singleWallPid;
// Case 3 uses the manual 7-cell calibration to compare travelled distance.
constexpr float NO_WALL_ENCODER_KP = 1.7f;
constexpr float NO_WALL_ENCODER_KI = 0.10f;
constexpr float NO_WALL_ENCODER_KD = 0.0f;
constexpr float NO_WALL_ENCODER_MAX_PWM = 30.0f;
constexpr float NO_WALL_MPU_HEADING_KP = 10.0f;
constexpr float NO_WALL_MPU_RATE_KD = 0.5f;
constexpr float NO_WALL_MPU_MAX_PWM = 25.0f;
constexpr float NO_WALL_MPU_WEIGHT = 0.80f;
// Initial rolling floors: the logged 35/30 commands stalled both encoders.
constexpr int NO_WALL_MIN_BASE_PWM = 70;
constexpr int NO_WALL_MIN_MOTOR_PWM = 60;
constexpr NoWallSettings NO_WALL_SETTINGS = {
    LEFT_TICKS_PER_CELL, RIGHT_TICKS_PER_CELL,
    NO_WALL_MIN_MOTOR_PWM,
    {NO_WALL_ENCODER_KP, NO_WALL_ENCODER_KI,
     NO_WALL_ENCODER_KD, NO_WALL_ENCODER_MAX_PWM}
};
NoWallController noWallPid;
MpuYaw mpuYaw;
WallModeDetector wallDetector;
bool motionActive=false;
bool motionStopRequested=false;
bool sideCommunicationFault=false;

// ============================================================
// Sensors / encoders
// ============================================================

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

VL6180X leftTof;
VL6180X rightTof;
VL53L1X frontTof;
constexpr int SIDE_CONVERGENCE_MS = 30;
constexpr unsigned long FRONT_TIMING_BUDGET_US = 20000;
constexpr int FRONT_PERIOD_MS = 25;
TofFilter frontFilter, leftFilter, rightFilter;
int tofFrontRaw = -1, tofLeftRaw = -1, tofFrontFiltered = -1;

bool sensorsReady = false;
MovementCommands usbCommands;
MovementCommands wifiCommands;


// ============================================================
// WiFi
// ============================================================

void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long started = millis();

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - started > 15000) {
            Serial.println();
            Serial.println("WiFi connection FAILED.");
            Serial.println("Robot will continue with USB Serial only.");
            return;
        }
    }

    Serial.println();
    Serial.println("WiFi connected successfully.");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi Serial port: ");
    Serial.println(WIFI_SERIAL_PORT);

    wifiSerialServer.begin();
    wifiSerialServer.setNoDelay(true);
    Serial.println("TCP serial server started.");
}

void handleWiFiClient() {
    if (WiFi.status() != WL_CONNECTED) {
        wifiCommands.reset();
        return;
    }

    if (wifiSerialClient && wifiSerialClient.connected()) {
        return;
    }

    if (wifiSerialClient) {
        wifiSerialClient.stop();
    }
    wifiCommands.reset();

    WiFiClient newClient = wifiSerialServer.accept();
    if (newClient) {
        wifiSerialClient = newClient;
        wifiSerialClient.setNoDelay(true);
        Serial.println();
        Serial.println("WiFi serial client connected.");
        wifiSerialClient.println("=================================");
        wifiSerialClient.println("ESP32 Micromouse connected");
        wifiSerialClient.print("ESP32 IP: ");
        wifiSerialClient.println(WiFi.localIP());
        wifiSerialClient.println("Send s for 7 continuous cells; d stops the run. Front brake: 40 mm.");
        wifiSerialClient.println("=================================");
    }
}


// ============================================================
// Debug output
// ============================================================

// String / char / integer / etc.
template<typename T>
void debugPrint(const T& value) {

    Serial.print(value);

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.print(value);
    }
}


template<typename T>
void debugPrintln(const T& value) {

    Serial.println(value);

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.println(value);
    }
}


void debugPrintln() {

    Serial.println();

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.println();
    }
}


// Float with decimal places
void debugPrint(float value, int digits) {

    Serial.print(value, digits);

    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.print(value, digits);
    }
}


// ============================================================
// Command input
// ============================================================

MovementCommand readCommand() {
    MovementCommand result = MovementCommand::None;
    // Bound each batch so neither input can monopolize the control loop.
    for (int i = 0; i < 64 && Serial.available(); ++i) {
        result = mergeCommands(result, usbCommands.feed(Serial.read(), millis()));
    }
    result = mergeCommands(result, usbCommands.poll(millis()));
    if (wifiSerialClient && wifiSerialClient.connected()) {
        for (int i = 0; i < 64 && wifiSerialClient.available(); ++i) {
            result = mergeCommands(result, wifiCommands.feed(wifiSerialClient.read(), millis()));
        }
        result = mergeCommands(result, wifiCommands.poll(millis()));
    }
    return result; // A stop wins over a start received in the same batch.
}

void discardPendingCommands() {
    // Never replay a start received during a run or its final reporting.
    int usbPending = Serial.available();
    while (usbPending-- > 0) Serial.read();
    int wifiPending = wifiSerialClient ? wifiSerialClient.available() : 0;
    while (wifiPending-- > 0) wifiSerialClient.read();
    usbCommands.reset();
    wifiCommands.reset();
}


// ============================================================
// Encoder interrupts
// ============================================================

void IRAM_ATTR countLeft() {
    ++leftTicks;
}

void IRAM_ATTR countRight() {
    ++rightTicks;
}


void readTicks(
    unsigned long &left,
    unsigned long &right
) {

    noInterrupts();

    left = leftTicks;
    right = rightTicks;

    interrupts();
}


// ============================================================
// Motors
// ============================================================

void motor(
    int a,
    int b,
    int speed,
    float factor,
    bool reversePolarity
) {

    int pwm =
        constrain(
            (int)(abs(speed) * factor),
            0,
            255
        );

    if (speed == 0) {

        analogWrite(a, 255);
        analogWrite(b, 255);

        return;
    }

    bool positive = speed > 0;

    if (reversePolarity) {
        positive = !positive;
    }

    analogWrite(
        a,
        positive ? pwm : 0
    );

    analogWrite(
        b,
        positive ? 0 : pwm
    );
}


void drive(int left, int right) {

    motor(
        LEFT_IN1,
        LEFT_IN2,
        left,
        LEFT_FACTOR,
        true
    );

    motor(
        RIGHT_IN1,
        RIGHT_IN2,
        right,
        RIGHT_FACTOR,
        false
    );
}


// ============================================================
// Sensors
// ============================================================

uint8_t probeTof(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission();
}

bool reportTofStep(const char *name, const char *step, uint8_t status) {
    debugPrint(name);
    debugPrint(" | ");
    debugPrint(step);
    debugPrint(" | I2C code=");
    debugPrint(status);
    debugPrintln(status == 0 ? " OK" : " FAILED");
    return status == 0;
}
bool initSensors() {
  Wire.begin();
  bool leftReady = false, rightReady = false, frontReady = false;
  // Match main.ino's sequence and settings; report diagnostics afterward.
  pinMode(LEFT_XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);
  pinMode(FRONT_XSHUT, OUTPUT);
  digitalWrite(LEFT_XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);
  digitalWrite(FRONT_XSHUT, LOW);
  delay(10);

  digitalWrite(LEFT_XSHUT, HIGH);
  delay(50);
  leftTof.init(); // VL6180X::init() returns void.
  uint8_t leftInitStatus = leftTof.last_status;
  leftTof.configureDefault();
  uint8_t leftConfigStatus = leftTof.last_status;
  leftTof.setAddress(0x30);
  uint8_t leftAddressStatus = leftTof.last_status;
  leftTof.setTimeout(200);
  leftTof.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, SIDE_CONVERGENCE_MS);
  bool leftTimingOk = leftTof.last_status == 0;

  digitalWrite(RIGHT_XSHUT, HIGH);
  delay(50);
  rightTof.init();
  uint8_t rightInitStatus = rightTof.last_status;
  rightTof.configureDefault();
  uint8_t rightConfigStatus = rightTof.last_status;
  rightTof.setAddress(0x31);
  uint8_t rightAddressStatus = rightTof.last_status;
  rightTof.setTimeout(200);
  rightTof.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, SIDE_CONVERGENCE_MS);
  bool rightTimingOk = rightTof.last_status == 0;

  digitalWrite(FRONT_XSHUT, HIGH);
  delay(50);
  bool frontInitOk = frontTof.init();
  bool frontModeOk = frontTof.setDistanceMode(VL53L1X::Short);
  bool frontTimingOk = frontTof.setMeasurementTimingBudget(FRONT_TIMING_BUDGET_US);
  frontTof.setAddress(0x32);
  uint8_t frontAddressStatus = frontTof.last_status;
  frontTof.setTimeout(200);
  frontTof.startContinuous(FRONT_PERIOD_MS);
  uint8_t frontStartStatus = frontTof.last_status;

  reportTofStep("LEFT", "init", leftInitStatus);
  reportTofStep("LEFT", "configureDefault", leftConfigStatus);
  reportTofStep("LEFT", "setAddress", leftAddressStatus);
  leftReady = reportTofStep("LEFT", "probe 0x30", probeTof(0x30));
  reportTofStep("RIGHT", "init", rightInitStatus);
  reportTofStep("RIGHT", "configureDefault", rightConfigStatus);
  reportTofStep("RIGHT", "setAddress", rightAddressStatus);
  rightReady = reportTofStep("RIGHT", "probe 0x31", probeTof(0x31));
  debugPrintln(frontInitOk ? "FRONT | init OK" : "FRONT | init FAILED");
  debugPrintln(frontModeOk ? "FRONT | distance mode OK" : "FRONT | distance mode FAILED");
  reportTofStep("FRONT", "setAddress", frontAddressStatus);
  reportTofStep("FRONT", "startContinuous", frontStartStatus);
  frontReady = reportTofStep("FRONT", "probe 0x32", probeTof(0x32)) && frontInitOk;
  debugPrintln("Address ACK allows diagnostic reads; check range validity below.");
  debugPrintln(leftTimingOk && rightTimingOk && frontTimingOk ?
      "ToF fast timing configured." : "ToF timing configuration FAILED.");
  return leftReady && rightReady && frontReady && frontModeOk &&
      leftTimingOk && rightTimingOk && frontTimingOk;
}

void serviceMotionSensors() {
    mpuYaw.service();
    if (motionActive && readCommand()==MovementCommand::Stop) {
        motionStopRequested=true;
        drive(0,0);
    }
}

void waitWithMotionService(unsigned long durationMs) {
    unsigned long began=millis();
    do { serviceMotionSensors(); delay(1); } while (millis()-began<durationMs);
}

int readSideForMotion(VL6180X &sensor) {
    // Both shots were launched together by observe().
    unsigned long began=millis();
    while (true) {
        serviceMotionSensors();
        if (motionStopRequested) return -1;
        uint8_t status=sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
        if (sensor.last_status!=0) { sideCommunicationFault=true; return -1; }
        if ((status&7)==4) break;
        if (millis()-began>=200) { sideCommunicationFault=true; return -1; }
        delay(1);
    }
    int mm=sensor.readRangeContinuousMillimeters(); // Already ready; no long wait.
    bool valid=!sensor.timeoutOccurred() && sensor.last_status==0;
    uint8_t rangeStatus=sensor.readRangeStatus();
    if (!valid || sensor.last_status!=0) { sideCommunicationFault=true; return -1; }
    return rangeStatus==0 ? mm : SIDE_NO_TARGET_MM;
}

bool observe(int &front,int &left,int &right,int &rightRaw) {
    sideCommunicationFault=false;
    tofFrontRaw=tofLeftRaw=tofFrontFiltered=-1;
    // Clear any unread result from a previously interrupted run, then start
    // both side shots while the front sensor is also measuring.
    leftTof.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR,0x01);
    if (leftTof.last_status!=0) sideCommunicationFault=true;
    rightTof.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR,0x01);
    if (rightTof.last_status!=0) sideCommunicationFault=true;
    leftTof.writeReg(VL6180X::SYSRANGE__START,0x01);
    if (leftTof.last_status!=0) sideCommunicationFault=true;
    rightTof.writeReg(VL6180X::SYSRANGE__START,0x01);
    if (rightTof.last_status!=0) sideCommunicationFault=true;
    unsigned long began=millis();
    while (true) {
        serviceMotionSensors();
        if (motionStopRequested) return false;
        bool ready=frontTof.dataReady();
        if (frontTof.last_status!=0) return false;
        if (ready) break;
        if (millis()-began>=200) return false;
        delay(1);
    }
    front=frontTof.read(false);
    bool frontValid=!frontTof.timeoutOccurred() && frontTof.last_status==0 &&
                    frontTof.ranging_data.range_status==VL53L1X::RangeValid;
    tofFrontRaw=front;
    tofFrontFiltered=frontFilter.update(front,frontValid);
    // Brake before side reads or logging when the front sample requires a stop.
    if (!frontValid || front <= FRONT_EMERGENCY_STOP_MM) {
        drive(0, 0);
        left=right=rightRaw=-1;
        return frontValid;
    }
    tofLeftRaw=readSideForMotion(leftTof);
    // A newly distant reading must release the wall immediately, even if the
    // median still contains two old close readings. Distances are chassis-based.
    left=leftFilter.update(tofLeftRaw,tofLeftRaw>=0 &&
        tofLeftRaw<WallModeDetector::RETAIN_MM);
    rightRaw=readSideForMotion(rightTof);
    right=sideClearanceMm(rightFilter.update(rightRaw,
        rightRaw>=0 && sideClearanceMm(rightRaw,RIGHT_TOF_INSET_MM)<WallModeDetector::RETAIN_MM), RIGHT_TOF_INSET_MM);
    serviceMotionSensors();
    return frontValid;
}

// ============================================================
// Movement
// ============================================================

void reportMovementConfig() {
    debugPrintln("Forward build: physical-motor-map-v9-yaw-sign | compiled " __DATE__ " " __TIME__);
    debugPrintln("Physical LEFT: motor 27/14, encoder 33 | RIGHT: motor 25/26, encoder 35");
    char config[480];
    snprintf(config,sizeof(config),
        "CONFIG | wall acquire/retain/confirm: %d/%d/%d | cruise: %d | factors L/R: %.3f/%.3f | MPU sign: %.0f | no-wall base/motor floor: %d/%d | encoder PID: %.2f/%.2f/%.2f | one-wall PID: %.2f/%.2f/%.2f | two-wall PID: %.2f/%.2f/%.2f",
        WallModeDetector::ACQUIRE_MM,WallModeDetector::RETAIN_MM,WallModeDetector::CONFIRM_SAMPLES,
        FORWARD_SPEED,LEFT_FACTOR,RIGHT_FACTOR,MPU_YAW_SIGN,
        NO_WALL_MIN_BASE_PWM,NO_WALL_MIN_MOTOR_PWM,
        NO_WALL_ENCODER_KP,NO_WALL_ENCODER_KI,NO_WALL_ENCODER_KD,
        SINGLE_WALL_KP,SINGLE_WALL_KI,SINGLE_WALL_KD,
        TWO_WALL_KP,TWO_WALL_KI,TWO_WALL_KD);
    debugPrintln(config);
    debugPrintln("MPU trim | one-wall Kp/Kd/max: 2.00/0.15/15 | no-wall Kp/Kd/max/weight: 2.00/1.50/15/80%");
}

bool runForwardDistance() {
    if (!sensorsReady) {
        debugPrintln("REFUSED: ToF initialization failed.");
        return false;
    }

    unsigned long startLeft, startRight;
    readTicks(startLeft, startRight);
    unsigned long started = millis();
    unsigned long lastLeftChange = started, lastRightChange = started;
    unsigned long previousLeft = 0, previousRight = 0, lastLog = started;
    const char *result = "TIMEOUT";
    bool distanceCompleted = false;
    unsigned long previousControlMs = started;
    approachPid.reset();
    twoWallPid.reset();
    singleWallPid.reset();
    noWallPid.reset();

    while (true) {
        handleWiFiClient();
        if (motionStopRequested || readCommand() == MovementCommand::Stop) {
            result = "ABORTED";
            break;
        }

        unsigned long rawLeft, rawRight;
        readTicks(rawLeft, rawRight);
        unsigned long left = rawLeft - startLeft, right = rawRight - startRight;
        unsigned long now = millis();
        if (left != previousLeft) { lastLeftChange = now; previousLeft = left; }
        if (right != previousRight) { lastRightChange = now; previousRight = right; }

        // Exactly the base sketch's OR stopping rule, with calibrated targets.
        if (cellEncoderLimitReached(left, right, LEFT_TARGET_TICKS - BRAKE_LEAD_TICKS,
                                    RIGHT_TARGET_TICKS - BRAKE_LEAD_TICKS)) {
            result = "RUN ENCODER LIMIT REACHED";
            distanceCompleted = true;
            break;
        }
        if (now - started >= MOVE_TIMEOUT_MS) break;
        if (now - lastLeftChange >= STALL_TIMEOUT_MS ||
            now - lastRightChange >= STALL_TIMEOUT_MS) {
            result = "ENCODER STALL";
            break;
        }

        int front = -1, sideLeft = -1, sideRight = -1, sideRightRaw = -1;
        bool frontValid = observe(front, sideLeft, sideRight, sideRightRaw);
        // Stop wins over a sensor fault received during a blocking read.
        handleWiFiClient();
        if (motionStopRequested || readCommand() == MovementCommand::Stop) {
            result = "ABORTED";
            break;
        }
        if (!frontValid) {
            result = "INVALID FRONT TOF READING";
            break;
        }
        if (front <= FRONT_EMERGENCY_STOP_MM) {
            result = "FRONT EMERGENCY BRAKE: 40 MM OR CLOSER";
            break;
        }
        if (sideCommunicationFault) { result="SIDE TOF COMMUNICATION FAILURE"; break; }
        WallMode mode=wallDetector.update(sideLeft,sideRight);
        if (mode==WallMode::None && (sideLeft<0 || sideRight<0)) {
            result="CASE 3 REQUIRES TWO VALID SIDE READINGS"; break;
        }
        const bool oneWallMode=mode==WallMode::LeftOnly || mode==WallMode::RightOnly;
        if (oneWallMode && MPU_YAW_SIGN==0) {
            result="SET MPU_YAW_SIGN BEFORE CASE 2"; break;
        }
        if (oneWallMode && !mpuYaw.healthy()) {
            result="MPU UNAVAILABLE OR STALE: CASE 2 STOPPED"; break;
        }

        // Sensor calls can take time: check distance again before applying PWM.
        readTicks(rawLeft, rawRight);
        left = rawLeft - startLeft;
        right = rawRight - startRight;
        if (cellEncoderLimitReached(left, right, LEFT_TARGET_TICKS - BRAKE_LEAD_TICKS,
                                    RIGHT_TARGET_TICKS - BRAKE_LEAD_TICKS)) {
            result = "RUN ENCODER LIMIT REACHED";
            distanceCompleted = true;
            break;
        }
        unsigned long leftRemaining = ticksBeforeBrake(left, LEFT_TARGET_TICKS, BRAKE_LEAD_TICKS);
        unsigned long rightRemaining = ticksBeforeBrake(right, RIGHT_TARGET_TICKS, BRAKE_LEAD_TICKS);
        unsigned long remaining = leftRemaining < rightRemaining ? leftRemaining : rightRemaining;
        unsigned long controlMs = millis();
        float dt = (controlMs - previousControlMs) / 1000.0f;
        previousControlMs = controlMs;
        int approachSpeed = approachPid.update((float)remaining, dt);
        ForwardMotorCommands commands;
        float wallError=0, wallPwm=0, mpuPwm=0;
        float encoderError=0, encoderPwm=0;
        const float yawRight=mpuYaw.yaw()*MPU_YAW_SIGN;
        const float rateRight=mpuYaw.rate()*MPU_YAW_SIGN;
        if (mode==WallMode::Two) {
            singleWallPid.reset();
            noWallPid.reset();
            commands=twoWallPid.update(sideLeft,sideRight,approachSpeed,
                TWO_WALL_PID_SETTINGS,dt,wallError,wallPwm);
        } else if (mode==WallMode::LeftOnly || mode==WallMode::RightOnly) {
            twoWallPid.reset();
            noWallPid.reset();
            commands=singleWallPid.update(mode,sideLeft,sideRight,yawRight,rateRight,
                approachSpeed,SINGLE_WALL_SETTINGS,dt,wallError,wallPwm,mpuPwm);
        } else {
            twoWallPid.reset();
            singleWallPid.reset();
            approachSpeed=noWallApproachSpeed(approachSpeed,NO_WALL_MIN_BASE_PWM,FORWARD_SPEED);
            commands=noWallPid.update(left,right,approachSpeed,
                NO_WALL_SETTINGS,dt,encoderError,encoderPwm);
            const bool noWallMpuReady=mpuYaw.healthy();
            mpuPwm=noWallMpuReady
                ? noWallYawCorrection(yawRight,rateRight,
                    NO_WALL_MPU_HEADING_KP,NO_WALL_MPU_RATE_KD,
                    NO_WALL_MPU_MAX_PWM)
                : 0.0f;
            commands=noWallCombinedCommands(approachSpeed,
                NO_WALL_MIN_MOTOR_PWM,encoderPwm,mpuPwm,
                noWallMpuReady ? NO_WALL_MPU_WEIGHT : 0.0f);
        }
        drive(commands.left, commands.right);

        if (millis() - lastLog >= 200) {
            // One complete line reduces the number of WiFi write calls.
            char telemetry[560];
            snprintf(telemetry, sizeof(telemetry),
                "ticks L/R: %lu/%lu | ToF mm F/Lfiltered/Rraw/Rfiltered-corrected: %d/%d/%d/%d | command L/R: %d/%d | remaining ticks: %lu | distance PID PWM: %d | case: %s | MPU yaw: %.2f deg | wall error: %.1f | wall PID PWM: %.2f | encoder error/PWM: %.1f/%.2f | MPU trim: %.2f | Ffiltered/Lraw: %d/%d | dt ms: %.1f | side walls detected: %d",
                left, right, front, sideLeft, sideRightRaw, sideRight,
                commands.left, commands.right, remaining, approachSpeed,
                mode==WallMode::Two?"two":(mode==WallMode::LeftOnly?"left+MPU":(mode==WallMode::RightOnly?"right+MPU":"none:encoder+MPU")),
                yawRight,wallError,wallPwm,encoderError,encoderPwm,mpuPwm,
                tofFrontFiltered,tofLeftRaw,dt*1000.0f,
                mode==WallMode::Two?2:(mode==WallMode::None?0:1));
            debugPrintln(telemetry);
            lastLog = millis();
        }
        waitWithMotionService(5); // MPU is also serviced during sensor waits.
    }

    drive(0, 0); // Always brake both wheels together.
    unsigned long brakeLeft, brakeRight;
    readTicks(brakeLeft, brakeRight);
    approachPid.reset();
    twoWallPid.reset();
    singleWallPid.reset();
    noWallPid.reset();
    waitWithMotionService(100); // Sample yaw and residual movement after braking.
    unsigned long endLeft, endRight;
    readTicks(endLeft, endRight);
    debugPrint(result);
    debugPrint(" | run limits L/R: "); debugPrint(LEFT_TARGET_TICKS);
    debugPrint('/'); debugPrint(RIGHT_TARGET_TICKS);
    debugPrint(" | final L/R: "); debugPrint(endLeft - startLeft);
    debugPrint('/'); debugPrint(endRight - startRight);
    debugPrint(" | ticks at brake L/R: "); debugPrint(brakeLeft - startLeft);
    debugPrint('/'); debugPrint(brakeRight - startRight);
    debugPrint(" | ticks after brake L/R: "); debugPrint(endLeft - brakeLeft);
    debugPrint('/'); debugPrint(endRight - brakeRight);
    debugPrint(" | final MPU yaw: "); debugPrint(mpuYaw.yaw() * MPU_YAW_SIGN, 2);
    debugPrint(" deg");
    debugPrint(" | elapsed ms: "); debugPrintln(millis() - started);
    debugPrintln("Both motors stop at the first encoder limit. Measure actual full-run travel against 1260 mm.");
    return distanceCompleted;
}

void runMove() {
    reportMovementConfig();
    frontFilter.reset(); leftFilter.reset(); rightFilter.reset();
    motionStopRequested=false;
    mpuYaw.reset();
    wallDetector.reset();
    debugPrintln("Moving 7 cells continuously (1260 mm); front brake at 40 mm.");
    if (!runForwardDistance()) {
        debugPrintln("Run stopped; send s when ready for a new run.");
        return;
    }
    debugPrintln("All 7 cells completed. Robot remains stopped.");
}


// ============================================================
// Setup
// ============================================================

void moveForwardSetup() {

    Serial.begin(115200);

    delay(500);


    // --------------------------------------------------------
    // Motors
    // --------------------------------------------------------

    pinMode(MOTOR_EN, OUTPUT);

    digitalWrite(
        MOTOR_EN,
        LOW
    );

    pinMode(
        LEFT_IN1,
        OUTPUT
    );

    pinMode(
        LEFT_IN2,
        OUTPUT
    );

    pinMode(
        RIGHT_IN1,
        OUTPUT
    );

    pinMode(
        RIGHT_IN2,
        OUTPUT
    );

    drive(0, 0);

    // Establish stopped motor outputs before enabling WiFi commands.
    initWiFi();

    // --------------------------------------------------------
    // Encoders
    // --------------------------------------------------------

    pinMode(
        LEFT_ENCODER,
        INPUT_PULLUP
    );

    pinMode(
        RIGHT_ENCODER,
        INPUT
    );


    attachInterrupt(
        digitalPinToInterrupt(
            LEFT_ENCODER
        ),
        countLeft,
        RISING
    );


    attachInterrupt(
        digitalPinToInterrupt(
            RIGHT_ENCODER
        ),
        countRight,
        RISING
    );


    // --------------------------------------------------------
    // Sensors
    // --------------------------------------------------------

    sensorsReady =
        initSensors();
    debugPrintln("MPU6050 calibration: keep the robot completely still.");
    bool mpuReady=mpuYaw.begin();
    debugPrintln(mpuReady?"MPU6050 ready (direct yaw, no software Kalman filter).":"MPU failed: only two-wall mode available.");
    if (MPU_YAW_SIGN==0) debugPrintln("Case 2 needs MPU_YAW_SIGN: +1 for right-positive yaw, -1 for right-negative yaw.");

    digitalWrite(
        MOTOR_EN,
        HIGH
    );


    debugPrintln(
        sensorsReady
            ? "ToF initialization OK."
            : "ToF initialization FAILED."
    );


    debugPrint(
        "Forward run on s command: "
    );

    debugPrint(
        FORWARD_CELLS
    );

    debugPrint(
        " cells, target mm: "
    );

    debugPrint(
        FORWARD_TARGET_MM
    );

    debugPrint(
        ", run target ticks L/R: "
    );

    debugPrint(
        LEFT_TARGET_TICKS
    );

    debugPrint('/');

    debugPrintln(
        RIGHT_TARGET_TICKS
    );


    debugPrintln(
        "main.ino wall steering + distance PID: cruise 70, approach floor 35; slow steering 30."
    );

    debugPrintln(
        "Front emergency brake at 40 mm or closer; invalid readings also stop the run."
    );

    debugPrintln(
        "Distance PID approaches over 300 ticks; both wheels brake 10 ticks before their nominal cell limits."
    );


    if (!sensorsReady) {

        debugPrintln(
            "Movement unavailable: ToF initialization failed. Reset to retry."
        );

        return;
    }


    debugPrintln(
        "Ready. Send s for 7 continuous cells; d cancels the run."
    );


    discardPendingCommands();
}


// ============================================================
// Loop
// ============================================================

void moveForwardLoop() {

    // One s command runs seven cells continuously.
    handleWiFiClient();
    mpuYaw.service();
    MovementCommand command = readCommand();
    if (command == MovementCommand::Stop) {
        drive(0, 0);

        discardPendingCommands();
        debugPrintln("Stopped. Send s for a fresh seven-cell run.");
    } else if (command == MovementCommand::Start) {
        debugPrintln("Start received.");
        motionActive=true;
        runMove();
        motionActive=false;
        discardPendingCommands();
        debugPrintln("Run ended. Staying stopped; send s for a new seven-cell run.");
    }

    delay(10);
}

void moveForwardStop() {
    motionStopRequested = true;
    drive(0, 0);
}
