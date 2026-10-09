# 02 — Motion Control (forward drive)

Source: `main/MoveForward.{h,cpp}`, `main/config.h`.

This is the largest and most-tuned module. It does five jobs: read sensors,
decide the wall regime, compute wheel commands, stop at the right distance, and
report telemetry.

---

## 1. Sensors and timing

### Front — VL53L1X (GPIO16 / addr 0x32)
- `setDistanceMode(Short)`, timing budget 20 000 µs, `startContinuous(25 ms)`.
- Read: `frontTof.read(false)`; `range_status`:
  - `0` valid; `6` valid (wrap-check not finished) → **usable**.
  - `2` signal fail, `4` out of bounds → **no target ⇒ treated as 200 mm open**
    (`FRONT_NO_TARGET_MM`).
  - anything else / timeout / I2C error → **fault** (stops motion).

### Sides — VL6180X (GPIO4→0x30, GPIO5→0x31)
- `configureDefault()`, `SYSRANGE__MAX_CONVERGENCE_TIME = 30 ms`, timeout 200 ms.
- Both single-shots are launched **simultaneously** with the front continuous
  measurement, so three ranges overlap in time.
- Range status `0` → valid; any other status → **no target ⇒ 200 mm**
  (`SIDE_NO_TARGET_MM`). I2C error / ready timeout → fault.

### Filters (`TofFilter`, `MoveForward.h`)
- **5-sample moving average** (`main`; the `demo` used a 3-sample median).
- Usable samples only. An invalid or no-target sample **resets** the history so
  an opening is recognised immediately.
- The right reading is corrected for the sensor's 10 mm physical inset
  (`sideClearanceMm`) **after** filtering.
- The front average is diagnostic; the front-wall **emergency brake uses the raw
  current sample**, not the average.

### `observe()` (the single acquisition function)
```
clear side interrupts; start both side shots; wait front dataReady (≤200 ms)
read front (raw); classify valid / no-target / fault
filter front
if (!valid || front <= FRONT_OPEN_MM):            # 100 mm
    brake
    if (!valid || (!stoppedScan && front <= 61))  # scan still needs sides at a dead end
        return
read left, filter (usable if 0..<100)
read right, filter + inset correction
return frontValid
```
During **motion** a front wall stops side reads; during a **stationary scan**
(`stoppedScan=true`) the sides are still read at a dead end so the robot can
choose a turn.

## 2. Wall-regime detector (`WallModeDetector`)

Each side is independently classified with hysteresis:
- **acquire** a wall after 3 consecutive filtered clearances `< 80 mm`
  (`ACQUIRE_MM`);
- **retain** while `< 100 mm` (`RETAIN_MM`);
- invalid/no-target ⇒ release.

Result: `Two`, `LeftOnly`, `RightOnly`, or `None`. Regime changes reset the
active controller.

## 3. The three steering controllers

All outputs are **pre-balance PWM**; the driver then applies `LEFT_FACTOR=0.99`,
`RIGHT_FACTOR=1.00`. Convention: **positive correction slows the physical right
wheel** (turns right).

### Case 1 — two walls (`TwoWallPidController`)
```
error = rightClearance - leftClearance      # +ve: left wall closer
if |error| <= 3.6 mm: no correction
correction = kp*error + ki*∫ + kd*d/dt, limited to the available speed range
cap = min(40, baseSpeed - WALL_SLOW_SPEED)
if correction > 0: right = base - correction
else:              left  = base + correction
```
Gains: `Kp=7.3, Ki=0, Kd=5.7`, cap 40 PWM, tolerance 3.6 mm. Centre the robot
between the two walls.

### Case 2 — one wall + MPU (`SingleWallController`)
```
wallError = (LeftOnly) ? leftTarget - left : right - rightTarget   # target 40 mm
if |wallError| > 3.6 mm: same PID shape as case 1 (wall PID)
else:                    MPU heading trim only
```
Wall gains `Kp=13, Ki=0, Kd=2.5`, cap 40 PWM. MPU trim:
`headingKp=2.0`, `yawRateKd=0.15`, cap 15 PWM. The wall PID **cannot be
overridden** by the MPU: the MPU only acts inside the distance deadband.

### Case 3 — no walls (`NoWallController`)
Two references blended:
- **Encoder progress**: `error = (leftCells - rightCells) * avgTicksPerCell`
  (positive ⇒ left has travelled farther ⇒ slow left). Gains
  `Kp=13, Ki=0, Kd=0`, cap 20 PWM.
- **MPU heading hold**: `Kp=22`, rate damping `Kd=8.2`, cap 45 PWM.
- Blend weight `NO_WALL_MPU_WEIGHT = 0.30` ⇒
  `steering = 0.30*mpu - 0.70*encoder`. If the MPU is unhealthy the weight goes
  to 0 (pure encoder).
- Floors keep wheels turning: `NO_WALL_MIN_BASE_PWM=130`,
  `NO_WALL_MIN_MOTOR_PWM=100`.

## 4. Distance control and stopping

- Encoder calibration: `LEFT_TICKS_PER_CELL = RIGHT_TICKS_PER_CELL = 628`
  (a rounded round-number; the hand-pushed data in `unit-tests` gave ≈622.7 /
  619.2). One cell = 180 mm.
- Target ticks for N cells = 628·N. `BRAKE_LEAD_TICKS = 10` (~2.9 mm early).
- **Both wheels brake when either wheel reaches the target** (`cellEncoderLimit
  Reached`). This avoids pivoting but makes total distance depend on the slower
  wheel.
- **Approach PID** (`CellApproachController`) reduces base PWM inside the last
  300 ticks (~87 mm): `Kp=0.24, Ki=0.01, Kd=0.01`, floor 100 PWM, cruise 150
  (`SPEED_RUN_CRUISE_PWM=190` when speed-running). Anti-windup at both limits.
- Stop conditions: front wall ≤ 100 mm → `FrontWallReached`; encoder target →
  `EncoderReached`; stop command → `Stopped`; 1.5 s no-tick on a wheel →
  `ENCODER STALL`; 105 s whole-move timeout; any sensor/MPU fault → `Failed`.

## 5. Front-wall reference (60 mm)

When a wall is near the endpoint, the robot does not trust the encoder alone:

1. `runForwardDistance` detects a candidate (front reading in roughly 61–190 mm
   with a clear decreasing trend) and takes a 5-sample front average.
2. `settleAtFrontWall()` pulses forward with `PWM=125±steering` (100–140) using
   `steering = 2·yawRight + 2·(extraLeft − extraRight)`, clamped ±20, until the
   front reading reaches the band `[59, 61] mm`
   (`FRONT_WALL_MIN_IN_BAND_MM` .. `FRONT_WALL_STOP_TRIGGER_MM`).
3. Safety bounds: extra travel ≤ 130 mm/wheel, total 3 s, wheel mismatch ≤ 8 mm,
   no 500 ms stall, and the range must keep decreasing (8 mm per 20 mm of
   travel). Any violation ⇒ `Failed`, not "arrived".

Rationale: the encoder is not a ground-truth distance; the wall is a physical
reference that corrects longitudinal stop drift. It does **not** correct lateral
position or give an absolute maze coordinate.

## 6. Motor driver

```
motor(a,b,speed,factor,reverse):
  if speed == 0: a=255, b=255      # active brake (both inputs high)
  else: pwm = clamp(|speed|*factor, 0, 255); positive/reverse selects a or b
drive(left,right): left on 27/14 (reverse), right on 25/26 (normal)
```
Encoders: rising-edge interrupts on GPIO33 (left) / GPIO35 (right),
single-channel ⇒ counts pulses only, no direction, no slip detection.

## 7. Commands and logging

- `MovementCommands`: one parser per transport (USB/TCP/web are merged). Accepts
  `s`/`start`; `d`/`stop` is immediate and overrides.
- Telemetry every 200 ms: `MOVE | <regime> | mm F/L/R | travel L/R | yaw | PWM L/R
  | correction wall/mpu/enc`. End summary: `MOVE END | <reason> | travel | yaw |
  time`. Config printed once per run by `reportMovementConfig()`.

## 8. Bench-verifiable facts (no maze needed)

- With wheels raised, a `start` command should spin both wheels forward; a `d`
  should stop immediately.
- Front obstacle within ~100 mm should brake and report `FRONT WALL NEAR`.
- `MOVE CONFIG` and `MOTOR BUILD` banners confirm which firmware is flashed.
- Encoder tick totals per move can be read from `MOVE END` and compared with a
  hand-pushed 180 mm measurement.
