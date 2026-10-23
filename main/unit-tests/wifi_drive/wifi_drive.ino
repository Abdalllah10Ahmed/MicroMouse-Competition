// ============================================================
// wifi_drive.ino - drive the car from a browser (press AND hold).
// ============================================================
// !! LIFT THE WHEELS OFF THE GROUND, or put the robot on the floor
//    with clear space, because a WiFi command moves it for real. !!
//
// WHY A WEB PAGE INSTEAD OF THE TELNET CONSOLE?
//   The console test only tells us "a key was pressed". It cannot tell us when
//   you RELEASE the key. A web page can: it knows pointer-down and pointer-up.
//   So the ESP32 serves a small page itself and you open it in a browser.
//
// HOW "HOLD TO MOVE" WORKS
//   - Press and hold a button -> the page keeps sending /move?d=f about 10x/sec.
//   - Release -> the page immediately sends /stop.
//   - SAFETY (watchdog): if the ESP32 hears nothing for 400 ms (WiFi drop,
//     browser closed, phone sleeps) it stops the motors on its own.
//
// HOW TO CONNECT (do this once)
//   1. Flash this sketch and open Serial Monitor at 115200.
//   2. Wait for "WiFi connected" and read the printed address, e.g.
//        http://192.168.1.9
//      or  http://micromouse.local
//   3. Open that address in a browser on the SAME WiFi network.
//      The onboard BLUE LED turns on once the ESP32 has joined WiFi, and
//      turns off again if the connection is lost.
//   4. Hold a button (or the arrow / WASD keys) to drive. Release to stop.
//
// Pin mapping is the VERIFIED one from main/MoveForward.cpp:
//   physical LEFT  motor: IN1=27, IN2=14
//   physical RIGHT motor: IN1=25, IN2=26
//   enable = GPIO 23 (HIGH = enabled)
//   status LED = GPIO 2 (onboard blue LED): on = WiFi connected, off = not
// Brake = drive both input pins HIGH (verified in the motor tests).
// The right motor is driven with the OPPOSITE polarity from the left so that
// both wheels push the robot forward together (hardware-verified).
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "../../secrets.h"   // WIFI_SSID / WIFI_PASSWORD (git-ignored)

constexpr int LEFT_IN1 = 27, LEFT_IN2 = 14;
constexpr int RIGHT_IN1 = 25, RIGHT_IN2 = 26;
constexpr int MOTOR_EN = 23;

constexpr int STATUS_LED = 2;           // onboard blue LED: on = WiFi connected
constexpr int DRIVE_PWM = 150;          // default speed; the web slider overrides it
constexpr uint32_t WATCHDOG_MS = 400;   // stop if the browser goes quiet
constexpr const char *MDNS_NAME = "micromouse";

WebServer server(80);

char currentMove = 0;      // 0 = idle, else 'f' forward, 'b' back, 'l' left, 'r' right
uint32_t lastMoveMs = 0;   // last time we heard from the browser
uint8_t drivePwm = DRIVE_PWM;   // live speed, changed by the web slider

// The page the ESP32 serves. Stored in flash with PROGMEM to save RAM.
const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Micromouse drive</title>
<style>
 :root{color-scheme:dark;font-family:system-ui,sans-serif}
 body{margin:0;background:#0b1220;color:#e5eefc;display:flex;flex-direction:column;align-items:center;gap:14px;padding:18px}
 h1{font-size:20px;margin:0}
 .pad{display:grid;grid-template-columns:repeat(3,92px);grid-template-rows:repeat(3,92px);gap:10px}
 button{font-size:32px;border:1px solid #4b658b;border-radius:12px;background:#172944;color:#fff;cursor:pointer;touch-action:none;user-select:none}
 button:active{background:#2b4a78}
 .up{grid-area:1/2}.left{grid-area:2/1}.stop{grid-area:2/2;font-size:17px;background:#9e2f38}.right{grid-area:2/3}.down{grid-area:3/2}
 .speed{display:flex;flex-direction:column;align-items:center;gap:6px;width:280px}
 input[type=range]{width:100%}
 #status{color:#8fa8ca}
 small{color:#6f86a8}
</style>
</head>
<body>
 <h1>Micromouse drive</h1>
 <div class="pad">
  <button class="up"    data-dir="f">&#9650;</button>
  <button class="left"  data-dir="l">&#9664;</button>
  <button class="stop"  data-dir="s">STOP</button>
  <button class="right" data-dir="r">&#9654;</button>
  <button class="down"  data-dir="b">&#9660;</button>
 </div>
 <div class="speed">
  <label for="speed">Speed: <b id="speedValue">150</b> / 255</label>
  <input id="speed" type="range" min="0" max="255" value="150">
 </div>
 <div id="status">Hold a button to move.</div>
 <small>Keyboard: arrow keys or W A S D</small>
<script>
let held = null, timer = null, busy = false;
const status = document.getElementById('status');
const speed = document.getElementById('speed');
const speedValue = document.getElementById('speedValue');
let speedBusy = false;
function sendSpeed(){
 if (speedBusy) return;
 speedBusy = true;
 fetch('/speed?p=' + speed.value).catch(()=>{}).finally(()=>speedBusy = false);
}
speed.addEventListener('input', () => { speedValue.textContent = speed.value; sendSpeed(); });

function send(d){
 if (busy) return;                       // never queue requests up
 busy = true;
 fetch('/move?d=' + d).catch(()=>{}).finally(() => busy = false);
}

function start(d){
 if (held === d) return;                 // already doing this
 stop();                                 // cancel whatever was held before
 held = d; send(d);
 status.textContent = 'Moving: ' + d;
 timer = setInterval(() => send(d), 100);   // keep telling the robot we still hold
}

function stop(){
 if (timer) { clearInterval(timer); timer = null; }
 if (held){ fetch('/stop').catch(()=>{}); status.textContent = 'Stopped'; held = null; }
}

// On-screen buttons: pointerdown starts, releasing anywhere stops.
document.querySelectorAll('button[data-dir]').forEach(b => {
 const d = b.dataset.dir;
 b.addEventListener('pointerdown', e => {
  e.preventDefault();
  if (d === 's') stop(); else start(d);
 });
});

// Releasing the pointer anywhere on the page stops the robot.
document.addEventListener('pointerup', stop);
document.addEventListener('pointercancel', stop);
addEventListener('blur', stop);

// Keyboard control with the same hold-to-move behaviour.
const keys = {ArrowUp:'f',ArrowRight:'r',ArrowDown:'b',ArrowLeft:'l',w:'f',d:'r',s:'b',a:'l'};
addEventListener('keydown', e => { const k = keys[e.key] || keys[e.key.toLowerCase()]; if (k){ e.preventDefault(); start(k); } });
addEventListener('keyup',   e => { const k = keys[e.key] || keys[e.key.toLowerCase()]; if (k) stop(); });
</script>
</body>
</html>
)HTML";

// ---------- motors ----------------------------------------------------------
// dir: +1 forward, -1 backward, 0 brake (both pins HIGH = brake, as verified).
void setWheel(int in1, int in2, int dir) {
  if (dir > 0)      { analogWrite(in1, 0);        analogWrite(in2, drivePwm); }
  else if (dir < 0) { analogWrite(in1, drivePwm); analogWrite(in2, 0); }
  else              { analogWrite(in1, 255);       analogWrite(in2, 255); }
}

// The left and right motors are mounted as mirror images, so the right one must
// be driven with the opposite polarity to send both wheels forward together.
// Verified on hardware. Direction argument: +1 forward, -1 backward, 0 brake.
void setLeft(int dir)  { setWheel(LEFT_IN1, LEFT_IN2,  dir); }
void setRight(int dir) { setWheel(RIGHT_IN1, RIGHT_IN2, -dir); }

void applyMove(char d) {
  switch (d) {
    case 'f': setLeft(+1); setRight(+1); break;   // both wheels forward
    case 'b': setLeft(-1); setRight(-1); break;   // both wheels backward
    case 'l': setLeft(-1); setRight(+1); break;   // spin left, in place
    case 'r': setLeft(+1); setRight(-1); break;   // spin right, in place
    default:  setLeft(0);  setRight(0);  break;   // brake
  }
}

void stopMotors() {
  currentMove = 0;
  applyMove(0);
}

// ---------- web handlers ----------
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");   // always serve the current page
  server.send_P(200, "text/html", PAGE);
}

void handleMove() {
  String d = server.arg("d");
  char c = d.length() ? (char)d[0] : 0;
  if (c != 'f' && c != 'b' && c != 'l' && c != 'r') {
    server.send(400, "text/plain", "bad direction");
    return;
  }
  if (c != currentMove) {           // log only when the direction changes
    Serial.print("move: ");
    Serial.println(c);
  }
  currentMove = c;
  lastMoveMs = millis();
  applyMove(c);
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  stopMotors();
  server.send(200, "text/plain", "stopped");
}

void handleSpeed() {
  int p = server.arg("p").toInt();
  if (p < 0) p = 0;
  if (p > 255) p = 255;
  drivePwm = (uint8_t)p;
  if (currentMove != 0) applyMove(currentMove);   // take effect immediately
  server.send(200, "text/plain", String((int)drivePwm));
}

void setup() {
  Serial.begin(115200);
  delay(700);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  stopMotors();                     // braked until we are ready

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);    // off until WiFi is connected

  Serial.println("=== WiFi drive: hold a button to move ===");
  Serial.println("Lift the wheels or leave clear floor space.");

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
    Serial.print("Open: http://");
    Serial.println(WiFi.localIP());
    if (MDNS.begin(MDNS_NAME)) {
      Serial.print("Or:   http://");
      Serial.print(MDNS_NAME);
      Serial.println(".local");
    }
  } else {
    Serial.println("WiFi FAILED - check secrets.h (must be a 2.4 GHz network).");
  }

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/stop", handleStop);
  server.on("/speed", handleSpeed);
  server.begin();

  digitalWrite(MOTOR_EN, HIGH);     // motors live
  Serial.println("Ready. Open the address above in a browser.");
}

void loop() {
  server.handleClient();

  // Blue LED mirrors the WiFi link: on = connected (controllable), off = not.
  digitalWrite(STATUS_LED, WiFi.status() == WL_CONNECTED ? HIGH : LOW);

  // Watchdog: if the browser stopped talking to us, stop the motors.
  if (currentMove != 0 && millis() - lastMoveMs > WATCHDOG_MS) {
    Serial.println("watchdog: lost the browser -> stop");
    stopMotors();
  }
}
