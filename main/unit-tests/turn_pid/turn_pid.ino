// ============================================================
// turn_pid.ino - test the in-place turning controller (MPU yaw + PID).
// ============================================================
// !! PUT THE ROBOT ON THE FLOOR with ~50 cm of clear space around it. !!
//    Unlike the drive test, this one MUST be on the ground: a turn only works
//    if the body actually rotates, because the gyro measures the body's
//    rotation. With the wheels in the air the body stays put, the gyro reads
//    ~0, and the controller just runs to its timeout.
//
// WHAT THIS TESTS
//   The robot's turning primitive: spin in place until the MPU-6050 gyro says
//   we have rotated by the requested angle. This is the same controller as the
//   real firmware (main/rotation.cpp): a PID on the heading error.
//
//   The two wheels counter-rotate (one forward, one backward) so the robot
//   pivots about its own centre. The MPU yaw angle is the feedback; the PID
//   decides how much PWM to apply.
//
// WHY THE CONTROL RUNS "NON-BLOCKING"
//   The real firmware turns inside a while-loop, which freezes everything else.
//   Here the PID runs ONE step per loop() iteration instead, so the web server
//   keeps answering and you can watch yaw / error / PWM live while it turns.
//
// HOW TO CONNECT
//   1. Flash this sketch. Open Serial Monitor at 115200.
//   2. KEEP THE ROBOT COMPLETELY STILL for ~2 s while it calibrates the gyro.
//   3. Open http://turn-test.local  (or the printed IP) on the same WiFi.
//   4. Press a turn button and watch the numbers settle.
//   5. Tune LIVE - no re-flashing. Drag the bars for a quick feel, or type an
//      exact value in the number box next to it (e.g. 15.9 -> 16.0) and press
//      Enter. Either way the new value is applied on the next control step,
//      even mid-turn. "Reset to defaults" restores the best set found so far
//      (2026-10-20): Kp=15.2, Ki=5, Kd=0.6, min=115, max=170.
//
// PID IN ONE PARAGRAPH
//   error  = target - current heading      (how many degrees are left to go)
//   output = Kp*error + Ki*integral + Kd*derivative
//   Kp pushes harder the further away we are. Kd damps the approach so we do
//   not overshoot (it reacts to how fast the error is shrinking). Ki adds up a
//   leftover error and pushes it out, so the turn does not stop short. The
//   output is the PWM given to both wheels (opposite signs), clamped to
//   turnMinPwm..turnMaxPwm.
//
// Sign convention: positive angle = turn LEFT (counter-clockwise), the same as
// main/rotation.cpp. If a turn ever runs away instead of settling, the gyro Z
// axis is mounted the other way up: flip YAW_SIGN below.
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <math.h>
#include "../../secrets.h"   // WIFI_SSID / WIFI_PASSWORD (git-ignored)

// ---- verified motor pins (from main/MoveForward.cpp, tested in wifi_drive) ----
constexpr int LEFT_IN1 = 27, LEFT_IN2 = 14;
constexpr int RIGHT_IN1 = 25, RIGHT_IN2 = 26;
constexpr int MOTOR_EN = 23;
constexpr int STATUS_LED = 2;           // onboard blue LED: on = WiFi connected

// ---- MPU-6050 ----
constexpr int MPU_ADDR = 0x68;
constexpr float MPU_GYRO_SCALE = 65.5f; // 65.5 LSB per deg/s at +/-500 dps
constexpr float GYRO_DEADBAND_DPS = 0.5f;
constexpr float YAW_SIGN = -1.0f;       // matches main/rotation.cpp on this robot

// ---- PID gains, tuned live in the browser on 2026-10-20 ----
// These are the DEFAULTS: the best set found so far on the bench. The values
// actually used are the live variables below, which the web page changes while
// the robot is running.
constexpr float DEFAULT_KP = 15.2f;
constexpr float DEFAULT_KI = 5.0f;
constexpr float DEFAULT_KD = 0.6f;
constexpr int DEFAULT_MIN_PWM = 115;    // minimum to overcome gearbox stiction
constexpr int DEFAULT_MAX_PWM = 170;    // upper clamp
constexpr float ANGLE_TOLERANCE_DEG = 1.0f;
constexpr int STABLE_SAMPLES = 10;      // consecutive in-tolerance steps to finish

float turnKp = DEFAULT_KP;              // live-tunable from the browser
float turnKi = DEFAULT_KI;
float turnKd = DEFAULT_KD;
int turnMinPwm = DEFAULT_MIN_PWM;
int turnMaxPwm = DEFAULT_MAX_PWM;

constexpr const char *MDNS_NAME = "turn-test";

WebServer server(80);

// ---- gyro / heading state ----
float gyroBias = 0.0f;
bool mpuReady = false;

// ---- turn state (the PID lives between calls) ----
bool turning = false;
float headingDeg = 0.0f;         // accumulated yaw, + = left
float turnTargetHeading = 0.0f;
float turnIntegral = 0.0f;
float turnPrevError = 0.0f;
uint32_t turnLastMicros = 0;
uint32_t turnStartMs = 0;
uint32_t turnTimeoutMs = 0;
int turnStable = 0;
int lastPwm = 0;
float lastError = 0.0f;
float lastRate = 0.0f;
const char *turnResult = "-";

// ---- web page (served from flash) ----
const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turn PID test</title>
<style>
 :root{color-scheme:dark;font-family:system-ui,sans-serif}
 body{margin:0;background:#0b1220;color:#e5eefc;display:flex;flex-direction:column;align-items:center;gap:14px;padding:18px}
 h1{font-size:20px;margin:0}
 .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;width:320px}
 button{font:inherit;font-size:16px;padding:14px;border:1px solid #4b658b;border-radius:10px;background:#172944;color:#fff;cursor:pointer;touch-action:manipulation}
 button:active{background:#2b4a78}
 .stop{background:#9e2f38;grid-column:1/3}
 .zero{background:#3a4a63;grid-column:1/3;font-size:14px;padding:9px}
 .readout{font:13px ui-monospace,monospace;background:#111c30;border:1px solid #263956;border-radius:10px;padding:12px;width:296px;line-height:1.7}
 .readout b{color:#7fd1ff}
 .tune{display:flex;flex-direction:column;gap:9px;width:296px;background:#111c30;border:1px solid #263956;border-radius:10px;padding:12px}
 .hint{color:#8fa8ca;font-size:13px}
 .row{display:grid;grid-template-columns:28px 76px 1fr;align-items:center;gap:7px;font:13px ui-monospace,monospace}
 .row span{color:#cfe0f7}
 input[type=number]{width:100%;box-sizing:border-box;margin:0;padding:7px 4px;font:13px ui-monospace,monospace;text-align:right;background:#0b1220;color:#7fd1ff;border:1px solid #34507a;border-radius:6px}
 input[type=range]{width:100%;height:26px;margin:0;accent-color:#7fd1ff}
 .btn{font:inherit;font-size:13px;padding:8px;border-radius:8px;background:#3a4a63;color:#fff;border:1px solid #4b658b;cursor:pointer}
 #status{color:#8fa8ca}
</style>
</head>
<body>
 <h1>Turn PID test</h1>
 <div class="grid">
  <button onclick="go('l90')">&#8634; 90 left</button>
  <button onclick="go('r90')">90 right &#8635;</button>
  <button onclick="go('l180')">&#8634; 180 left</button>
  <button onclick="go('r180')">180 right &#8635;</button>
  <button onclick="go('l360')">&#8634; 360 left</button>
  <button onclick="go('r360')">360 right &#8635;</button>
  <button class="stop" onclick="stopTurn()">STOP</button>
  <button class="zero" onclick="zero()">Zero heading</button>
 </div>
 <div class="readout">
  Target: <b id="target">-</b> &nbsp; Yaw: <b id="yaw">-</b><br>
  Error: <b id="error">-</b> &nbsp; Gyro: <b id="rate">-</b> &deg;/s &nbsp; PWM: <b id="pwm">-</b>
 </div>
 <div class="tune">
  <div class="hint">Live PID tuning - drag a bar, or type an exact value in the box (press Enter)</div>
  <div class="row"><span>Kp</span><input id="kpn" type="number" min="0" max="100" step="0.1" value="15.2"><input id="kp" type="range" min="0" max="100" step="0.1" value="15.2"></div>
  <div class="row"><span>Ki</span><input id="kin" type="number" min="0" max="10" step="0.01" value="5"><input id="ki" type="range" min="0" max="10" step="0.01" value="5"></div>
  <div class="row"><span>Kd</span><input id="kdn" type="number" min="0" max="30" step="0.1" value="0.6"><input id="kd" type="range" min="0" max="30" step="0.1" value="0.6"></div>
  <div class="row"><span>min</span><input id="minn" type="number" min="0" max="255" step="1" value="115"><input id="min" type="range" min="0" max="255" step="1" value="115"></div>
  <div class="row"><span>max</span><input id="maxn" type="number" min="0" max="255" step="1" value="170"><input id="max" type="range" min="0" max="255" step="1" value="170"></div>
  <button class="btn" onclick="defaults()">Reset to defaults</button>
 </div>
 <div id="status">connecting...</div>
<script>
const el = id => document.getElementById(id);
const knobs = ['kp','ki','kd','min','max'];
const vals = {kp:15.2, ki:5, kd:0.6, min:115, max:170};
function go(d){ fetch('/turn?d='+d).catch(function(){}); }
function stopTurn(){ fetch('/stop').catch(function(){}); }
function zero(){ fetch('/zero').catch(function(){}); }
// One entry point for both controls of a knob: the bar and the typed box.
// Whichever changed is the source of truth; the other one is kept in step.
function tune(id, src){
  const v = parseFloat(src.value);
  if (!isFinite(v)) return;
  const box = el(id+'n');
  const bar = el(id);
  const c = Math.max(+box.min, Math.min(+box.max, v));   // clamp to the knob's range
  vals[id] = c;
  if (src !== bar) bar.value = c;
  if (src !== box) box.value = c;
  fetch('/tune?' + knobs.map(k => k+'='+vals[k]).join('&')).catch(function(){});
}
// Restore the firmware defaults; the next state poll re-syncs the controls.
function defaults(){ tuned = false; fetch('/defaults').catch(function(){}); }
knobs.forEach(id => {
  el(id).addEventListener('input', function(){ tune(id, this); });
  el(id+'n').addEventListener('input', function(){ tune(id, this); });
  el(id+'n').addEventListener('change', function(){ this.value = vals[id]; });  // tidy on Enter/blur
});
let tuned = false;
async function update(){
 try{
  const s = await fetch('/state',{cache:'no-store'}).then(r=>r.json());
  el('target').textContent = s.target.toFixed(1);
  el('yaw').textContent = s.yaw.toFixed(1);
  el('error').textContent = s.error.toFixed(1);
  el('rate').textContent = s.rate.toFixed(1);
  el('pwm').textContent = s.pwm;
  el('status').textContent = s.mpu ? (s.turning ? 'TURNING...' : 'idle ('+s.result+')') : 'MPU FAILED';
  if(!tuned){                       // adopt the ESP32's current values once on load
   tuned = true;
   for(const k of knobs){
    vals[k] = s[k];
    el(k).value = s[k];
    el(k+'n').value = s[k];
   }
  }
 }catch(e){ el('status').textContent = 'disconnected'; }
}
setInterval(update,150); update();
</script>
</body>
</html>
)HTML";

// ============================================================
// MPU-6050 helpers
// ============================================================
bool readMPU6050(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0 &&
      Wire.requestFrom((uint8_t)MPU_ADDR, count) == count) {
    for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
    return true;
  }
  return false;
}

bool writeMPU6050(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readGyroZ(float &rate) {
  uint8_t data[2];
  if (!readMPU6050(0x47, data, 2)) return false;   // 0x47 = GYRO_ZOUT_H
  int16_t raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  rate = raw / MPU_GYRO_SCALE;
  return true;
}

// Configure the gyro exactly like the real firmware and measure its bias.
bool initMPU() {
  uint8_t identity = 0;
  if (!readMPU6050(0x75, &identity, 1) || identity != 0x68) return false;
  if (!writeMPU6050(0x6B, 0x01)) return false;     // wake up
  delay(100);
  if (!writeMPU6050(0x1A, 0x06)) return false;     // DLPF 5 Hz
  if (!writeMPU6050(0x1B, 0x08)) return false;     // +/-500 deg/s
  delay(100);

  Serial.println("Calibrating gyro bias - keep the robot completely still...");
  float sum = 0.0f;
  for (int i = 0; i < 200; ++i) {
    float rate;
    if (!readGyroZ(rate)) return false;
    sum += rate;
    delay(10);
  }
  gyroBias = sum / 200.0f;
  return true;
}

// ============================================================
// Motors (verified polarity: the right motor is driven inverted)
// ============================================================
void setWheel(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(in1, 0);       analogWrite(in2, speed); }
  else if (speed < 0) { analogWrite(in1, -speed);  analogWrite(in2, 0); }
  else                { analogWrite(in1, 0);       analogWrite(in2, 0); }   // coast
}

// The right motor is mounted as a mirror image, so it must be driven with the
// OPPOSITE polarity to make both wheels push the same way. This is the exact
// inversion verified on hardware in wifi_drive. Without it the wheels turn the
// same direction and the robot drives forward/back instead of spinning.
void setLeft(int speed)  { setWheel(LEFT_IN1, LEFT_IN2,  speed); }
void setRight(int speed) { setWheel(RIGHT_IN1, RIGHT_IN2, -speed); }

void stopMotors() {
  setLeft(0);
  setRight(0);
}

// Positive pwm = turn LEFT: left wheel back, right wheel forward.
void driveTurn(int pwm) {
  setLeft(-pwm);
  setRight(pwm);
}

// ============================================================
// Turn controller (one PID step per call)
// ============================================================
void finishTurn(const char *result) {
  stopMotors();
  turning = false;
  lastPwm = 0;
  turnResult = result;
}

void startTurn(float degrees) {
  if (!mpuReady || turning) return;
  turnTargetHeading = headingDeg + degrees;
  turnIntegral = 0.0f;
  turnPrevError = 0.0f;
  turnStable = 0;
  turnStartMs = millis();
  turnLastMicros = micros();
  turnTimeoutMs = 1500 + (uint32_t)(fabsf(degrees) * 12.0f);   // longer for big turns
  turnResult = "running";
  turning = true;
}

void updateTurn() {
  if (!turning) return;

  const uint32_t now = micros();
  const float dt = (now - turnLastMicros) / 1000000.0f;
  if (dt < 0.005f) return;                 // run the control loop at ~200 Hz
  turnLastMicros = now;

  if (millis() - turnStartMs > turnTimeoutMs) {
    finishTurn("timeout");
    return;
  }

  float gyroZ;
  if (!readGyroZ(gyroZ)) {
    mpuReady = false;
    finishTurn("mpu-fail");
    return;
  }
  float rate = YAW_SIGN * (gyroZ - gyroBias);   // + = turning left
  if (fabsf(rate) < GYRO_DEADBAND_DPS) rate = 0.0f;
  headingDeg += rate * dt;

  const float error = turnTargetHeading - headingDeg;
  lastError = error;
  lastRate = rate;

  if (fabsf(error) <= ANGLE_TOLERANCE_DEG) {    // close enough: hold still
    stopMotors();
    lastPwm = 0;
    if (++turnStable >= STABLE_SAMPLES) finishTurn("ok");
    return;
  }
  turnStable = 0;

  const float derivative = (dt > 0) ? (error - turnPrevError) / dt : 0.0f;
  const float tentativeIntegral = turnIntegral + error * dt;
  const float tentativeOutput =
      turnKp * error + turnKi * tentativeIntegral + turnKd * derivative;
  if (tentativeOutput <= turnMaxPwm && tentativeOutput >= -turnMaxPwm) {
    turnIntegral = tentativeIntegral;     // anti-windup: only integrate when unsaturated
  }
  turnPrevError = error;

  float output = turnKp * error + turnKi * turnIntegral + turnKd * derivative;
  output = constrain(output, (float)-turnMaxPwm, (float)turnMaxPwm);
  if (output > 0 && output < turnMinPwm) output = turnMinPwm;
  if (output < 0 && output > -turnMinPwm) output = -turnMinPwm;

  lastPwm = (int)output;
  driveTurn((int)output);
}

// ============================================================
// Web handlers
// ============================================================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

void handleTurn() {
  const String d = server.arg("d");
  float degrees;
  if      (d == "l90")  degrees = 90.0f;
  else if (d == "r90")  degrees = -90.0f;
  else if (d == "l180") degrees = 180.0f;
  else if (d == "r180") degrees = -180.0f;
  else if (d == "l360") degrees = 360.0f;
  else if (d == "r360") degrees = -360.0f;
  else { server.send(400, "text/plain", "bad turn"); return; }
  startTurn(degrees);
  server.send(200, "text/plain", turning ? "started" : "busy/no-mpu");
}

void handleStop() {
  finishTurn("stopped");
  server.send(200, "text/plain", "stopped");
}

void handleZero() {
  if (!turning) headingDeg = 0.0f;
  server.send(200, "text/plain", "zeroed");
}

// Live PID tuning. Each value is optional, so the page can send one or all.
// Gains are clamped >= 0: a negative gain would invert the loop and make the
// robot run away instead of settling.
void handleTune() {
  if (server.hasArg("kp"))  turnKp = constrain(server.arg("kp").toFloat(), 0.0f, 1000.0f);
  if (server.hasArg("ki"))  turnKi = constrain(server.arg("ki").toFloat(), 0.0f, 1000.0f);
  if (server.hasArg("kd"))  turnKd = constrain(server.arg("kd").toFloat(), 0.0f, 1000.0f);
  if (server.hasArg("min")) turnMinPwm = constrain(server.arg("min").toInt(), 0, 255);
  if (server.hasArg("max")) turnMaxPwm = constrain(server.arg("max").toInt(), 0, 255);
  server.send(200, "text/plain", "ok");
}

void handleDefaults() {
  turnKp = DEFAULT_KP;
  turnKi = DEFAULT_KI;
  turnKd = DEFAULT_KD;
  turnMinPwm = DEFAULT_MIN_PWM;
  turnMaxPwm = DEFAULT_MAX_PWM;
  server.send(200, "text/plain", "defaults");
}

void handleState() {
  String json = "{";
  json += "\"yaw\":";    json += String(headingDeg, 1);
  json += ",\"target\":";json += String(turnTargetHeading, 1);
  json += ",\"error\":"; json += String(lastError, 1);
  json += ",\"rate\":";  json += String(lastRate, 1);
  json += ",\"pwm\":";   json += lastPwm;
  json += ",\"turning\":"; json += (turning ? "true" : "false");
  json += ",\"mpu\":";   json += (mpuReady ? "true" : "false");
  json += ",\"result\":\""; json += turnResult; json += "\"";
  json += ",\"kp\":";  json += String(turnKp, 2);
  json += ",\"ki\":";  json += String(turnKi, 3);
  json += ",\"kd\":";  json += String(turnKd, 2);
  json += ",\"min\":"; json += turnMinPwm;
  json += ",\"max\":"; json += turnMaxPwm;
  json += "}";
  server.send(200, "application/json", json);
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(700);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  stopMotors();

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Wire.begin(21, 22);
  mpuReady = initMPU();
  Serial.println(mpuReady ? "MPU initialization OK." : "MPU initialization FAILED.");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open: http://"); Serial.println(WiFi.localIP());
    if (MDNS.begin(MDNS_NAME)) { Serial.print("Or:   http://"); Serial.print(MDNS_NAME); Serial.println(".local"); }
  } else {
    Serial.println("WiFi FAILED - check secrets.h (must be a 2.4 GHz network).");
  }

  server.on("/", handleRoot);
  server.on("/turn", handleTurn);
  server.on("/stop", handleStop);
  server.on("/zero", handleZero);
  server.on("/state", handleState);
  server.on("/tune", handleTune);
  server.on("/defaults", handleDefaults);
  server.begin();

  digitalWrite(MOTOR_EN, HIGH);
  Serial.println("Ready. Put the robot on the floor, then press a turn button.");
}

void loop() {
  server.handleClient();
  updateTurn();
  digitalWrite(STATUS_LED, WiFi.status() == WL_CONNECTED ? HIGH : LOW);
}
