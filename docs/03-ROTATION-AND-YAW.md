# 03 — Rotation, IMU, and the Yaw Sign Chain

Source: `main/rotation.{h,cpp}`, the `MpuYaw` class in `main/MoveForward.cpp`,
`main/main.ino` (`turnTo`, `alignArrivalHeading`).

The robot has **two independent MPU drivers** on the same sensor. Understanding
both, and the sign chain between them, is the single most confusing part of the
codebase.

---

## 1. Hardware configuration (identical in both drivers)

| Register | Value | Meaning |
|---|---|---|
| `0x75` (WHO_AM_I) | must read `0x68` | MPU-6050 identity check |
| `0x6B` (PWR_MGMT_1) | `0x01` | wake, use gyro X clock (clears sleep) |
| `0x1A` (CONFIG) | `0x06` | DLPF ≈ 5 Hz hardware gyro filter |
| `0x1B` (GYRO_CONFIG) | `0x08` | ±500 °/s full scale |
| `0x47–0x48` | int16 | gyro Z rate, scale **65.5 LSB/(°/s)** |

I2C address `0x68`, shared bus SDA21/SCL22. No DMP, no Kalman — direct register
reads plus numeric integration.

## 2. Calibration (bias)

Both drivers average **200 stationary samples** of gyro Z (10 ms apart, ~2 s) to
estimate `gyroZBias`. The robot **must be still** during both calibrations. If
sampling fails the driver reports a fault and refuses motion.

Measured rate after bias removal:
```
applied = -(gyroZ - gyroZBias)      # sign chosen so this is "positive = left"
if |applied| < 0.5 dps: applied = 0  # deadband kills stationary drift
```

## 3. Rotation controller (`rotation.cpp`) — in-place turns

Used by `turnTo()` and arrival alignment. Positive angle = physical **left**.

```
target = heading + relativeAngle
loop (≥5 ms cadence, service stop commands):
    read gyro; applied = -(gyroZ - bias); deadband
    heading += applied * dt                 # integrate yaw
    error = target - heading
    if |error| <= 1.5°: stop; stableTicks++; if stableTicks >= 10: done
    else: stableTicks = 0
    integral anti-windup (Ki = 0 anyway)
    output = Kp*error + Ki*∫ + Kd*derivative
    output = clamp(output, ±180); if 0<|output|<130: output = ±130   # stiction floor
    drive(output, -output)                  # equal and opposite
timeout: 2200 ms
```
Gains: `Kp=15.9, Ki=0, Kd=1.3`; tolerance 1.5°; 10 stable samples; floor 130;
cap 180 PWM.

Notes:
- The motor pin **labels** here are `LEFT=25/26, RIGHT=14/27`, i.e. the opposite
  of `MoveForward.cpp`. Because a turn is symmetric this does not matter, but it
  is a real maintenance/porting trap.
- `turnTo()` maps a heading change to one or two ±90° moves:
  +1 → 90° left; −1 → 90° right; 2 → two 90° left moves (a 180° U-turn is
  deliberately two tested 90° turns, each with its own timeout).
- After the turn, `robotHeading` is updated in the app only if the turn reported
  success.

## 4. Forward yaw estimator (`MpuYaw` in `MoveForward.cpp`)

Used for steering (case 2/3) and arrival alignment.

```
service() (called often, integrates only if ≥10 ms elapsed):
    read gyroZ (retry loop on I2C)
    rate = -(gyroZ - bias); deadband 0.5 dps
    if elapsed <= 100 ms: yaw += rate * dt      # else skip the unsafe interval
    lastGood = now
healthy(): ready && (now - lastGood) <= 250 ms
```
It deliberately does **not** integrate across a long blocking sensor interval
(>100 ms) to avoid a huge yaw jump.

## 5. The sign chain (read this twice)

There are three sign conventions in play, and they cancel in a non-obvious way:

1. Raw gyro Z is whatever the mounting produces.
2. Both drivers negate it: `applied = -(gyroZ - bias)`.
   - `rotation.h` documents the result as **positive = physical left**.
   - `MpuYaw` comments likewise: "positive is physical left".
3. `MPU_YAW_SIGN = -1.0f`. The forward code multiplies `MpuYaw.yaw()` by this to
   get `yawRight`, i.e. **positive = right** (after the multiplication).
   Rotation keeps the left-positive convention.

Consequences:
- Steering code works in "positive = right" (`yawRight`), and its corrections
  assume positive slows the physical right wheel.
- Arrival alignment obtains `yawRight` (right-positive). The rotation API is
  left-positive. Passing `yawRight` directly to `turnDegrees()` is correct
  because a **right drift needs a left turn**: `yawRight = +5° ⇒ turnDegrees(+5)`,
  which the rotation controller reads as a left turn. The comments in
  `main.ino` and `demo` document exactly this.
- If you ever change `MPU_YAW_SIGN`, re-check: steering direction, single-wall
  MPU trim, no-wall MPU trim, and arrival alignment — all four.

**Porting advice:** on a new robot, keep the raw negation (make left-positive the
internal convention) and set `MPU_YAW_SIGN` empirically with a bench test:
command `turnDegrees(+90)`, confirm the robot turned left, then apply a small
forward move and confirm `ALIGN` corrects the right direction.

## 6. Arrival alignment (`alignArrivalHeading`, main.ino)

```
if !demoHeadingError(yawRight): fail (MPU unavailable)
if |yawRight| <= 1.0°: done
else turnDegrees(yawRight)      # in-place correction, then settle
```
Tolerance `ARRIVAL_YAW_TOLERANCE_DEG = 1.0`. This fixes heading drift after each
move without re-commanding distance.

## 7. Two-driver duplication (known issue)

`beginRotation()` and `moveForwardSetup()` each fully initialise and bias the
MPU. Cost: ~4 s of stationary boot time and two writers to one register set.
It exists because the project reused two separately-validated modules rather than
merging them. Safe as-is, but see `06-PORTING-AND-LESSONS.md` (issue I2) for the
consolidation plan.

## 8. Bench checks (wheels raised)

- Send `r`/`l` (rotation test sketch) or observe `TURN`/`TURN END` in the app:
  positive target should rotate the chassis left.
- Leave the robot still after boot and watch `yaw` drift in telemetry; a healthy
  deadband keeps it within a few degrees over a minute.
- Tap the chassis: yaw rate should respond and then return near zero.
