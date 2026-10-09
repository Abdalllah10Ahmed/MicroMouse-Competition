# 04 — Tuning Guide

Every knob that changes behaviour, where it lives, what it does, and how to tune
it. **Rule: change one parameter at a time and record the result.** With a bench
only (no maze), most tuning is limited to sensor/geometry checks and encoder
calibration — mark maze-dependent items as unverifiable here.

---

## A. Openings and scan geometry — `main/config.h`

| Constant | Default | Effect / tuning |
|---|---|---|
| `SIDE_OPEN_MM` | 100 | Clearance above which a side reads OPEN. Raise if real walls are misread open; lower if corridors are wrongly blocked. Measure real corridor clearance first. |
| `FRONT_OPEN_MM` | 100 | Same for the front; also the "start braking" distance. |
| `OPEN_CONFIRM_SAMPLES` | 8 | Scans per cell; the last 5 are averaged. `static_assert(>=5)`. More = slower, steadier. |
| `SETTLE_MS` | 120 | Pause after a move before scanning. Too short ⇒ residual motion corrupts ranges. |
| `ARRIVAL_YAW_TOLERANCE_DEG` | 1.0 | Alignment deadband. Tighter ⇒ more corrections/oscillation. |
| `FRONT_WALL_TARGET_MM` | 60 | Desired front-wall gap for the reference stop. |
| `FRONT_WALL_TOLERANCE_PERCENT` | 2.0 | Band around target (±1.2 mm). **Doc comment still says 3% — stale.** |
| `FRONT_WALL_STOP_TRIGGER_MM` | 61 | Front reading at/below which forward travel brakes. |
| `FRONT_WALL_MIN_IN_BAND_MM` | 59 | Lower edge of the accepted band. |
| `FRONT_APPROACH_TIMEOUT_MS` | 3000 | Max time for the extra front-wall approach. |
| `FRONT_APPROACH_MAX_EXTRA_MM` | 130 | Max extra per-wheel travel during that approach. |

## B. Drivetrain geometry — `main/MoveForward.cpp`

| Constant | Default | Effect |
|---|---|---|
| `LEFT_TICKS_PER_CELL` | 628 | Encoder ticks per 180 mm, left. **Primary distance calibration.** |
| `RIGHT_TICKS_PER_CELL` | 628 | Same, right. |
| `CELL_LENGTH_MM` | 180 | Only used for telemetry in mm. |
| `LEFT_FACTOR` / `RIGHT_FACTOR` | 0.99 / 1.00 | Per-motor balance multiplier. Equalise wheel speeds. |
| `MAX_STRAIGHT_CELLS` | 16 | Upper bound for a speed-run segment. |

Calibration method (from `unit-tests/calibration/README.md`): push straight for
exactly 1260 mm, read ticks from the `encoder_ticks` sketch, average ≥5 trials
per wheel, divide by 7. The main firmware rounds to 628/628. If a completed,
unobstructed run shows a consistent distance error:
`new_ticks_per_cell = old * 1260 / mean_measured_mm`.

## C. Speed and floors — `main/MoveForward.cpp`

| Constant | Default | Effect |
|---|---|---|
| `FORWARD_SPEED` | 150 | Exploration cruise PWM. |
| `SPEED_RUN_CRUISE_PWM` | 190 | Speed-run cruise. |
| `WALL_SLOW_SPEED` | 85 | Floor when a wall correction slows a wheel; must stay above stall. |
| `MIN_APPROACH_PWM` | 100 | Floor near the endpoint. Lower ⇒ smoother stop but stall risk. |
| `NO_WALL_MIN_BASE_PWM` | 130 | Base floor in no-wall mode. |
| `NO_WALL_MIN_MOTOR_PWM` | 100 | Per-motor floor in no-wall mode. |
| `LEFT/RIGHT_WALL_THRESHOLD_MM` | 40 | Legacy threshold constants still referenced by single-wall settings. |

**Stall is the enemy.** The logs documented historical stalls when commands
dropped to 30–39 PWM. Prefer raising floors over raising gains when a wheel
stops moving.

## D. Controllers — `main/MoveForward.cpp`

| Controller | Gains | Notes |
|---|---|---|
| Two-wall | Kp 7.3 / Ki 0 / Kd 5.7, cap 40, tol 3.6 mm | Centres between two walls. |
| One-wall distance | Kp 13 / Ki 0 / Kd 2.5, cap 40, tol 3.6 mm | Slows one wheel. |
| One-wall MPU trim | Kp 2.0 / rate Kd 0.15, cap 15 | Only inside the distance deadband. |
| No-wall encoder | Kp 13 / Ki 0 / Kd 0, cap 20 | Normalised tick difference. |
| No-wall MPU | Kp 22 / rate Kd 8.2, cap 45, weight 0.30 | 30% MPU / 70% encoder. |
| Approach | Kp 0.24 / Ki 0.01 / Kd 0.01, zone 300 ticks, floor 100 | Slow the endpoint. |
| Brake lead | `BRAKE_LEAD_TICKS = 10` (~2.9 mm) | Increase to stop earlier. |

Method: log `MOVE` telemetry while pushing the robot by hand through a regime
(2-wall, 1-wall, none) and watch the correction signs. Then tune Kp until it
responds without oscillation, add Kd for damping, leave Ki near 0.

## E. Sensors — `main/MoveForward.cpp`

| Constant | Default | Effect |
|---|---|---|
| `FRONT_TIMING_BUDGET_US` | 20000 | VL53L1X measurement budget; lower = faster/noisier. |
| `FRONT_PERIOD_MS` | 25 | Continuous period. |
| `SIDE_CONVERGENCE_MS` | 30 | VL6180X max convergence. |
| `RIGHT_TOF_INSET_MM` | 10 | Right sensor recess; subtract from valid right range. |
| `SIDE_NO_TARGET_MM` / `FRONT_NO_TARGET_MM` | 200 | Open-space sentinel (> retain threshold). |
| `WallModeDetector ACQUIRE_MM / RETAIN_MM / CONFIRM_SAMPLES` | 80 / 100 / 3 | Wall hysteresis. |
| `MPU_I2C_ATTEMPTS` / `MPU_I2C_RETRY_MS` | 10 / 1000 | I2C retry policy. |

## F. Rotation — `main/rotation.cpp`

| Constant | Default | Effect |
|---|---|---|
| `turnKp` / `turnKi` / `turnKd` | 15.9 / 0 / 1.3 | Turn response. |
| `minOutput` | 130 | Stiction floor. |
| `maxTurnSpeed` | 180 | Cap. |
| `angleToleranceDeg` | 1.5 | Deadband. |
| `stableTicksNeeded` | 10 | Samples within deadband before success. |
| `turnTimeoutMs` | 2200 | Fail-safe. |
| `GYRO_DEADBAND_DPS` | 0.5 | Stationary drift rejection. |

⚠ The `demo/right-hand` README documents older values (95/150/1.0/2000); the
code above is authoritative.

## G. App: hand gesture and LED — `main/main.ino`

| Constant | Default | Effect |
|---|---|---|
| `HAND_MAX_NEAR_MM` | 100 | Cap on the "near" trigger distance. |
| `HAND_DROP_MM` | 25 | Baseline minus this = near threshold (10 if baseline < 70). |
| `HAND_RELEASE_MARGIN_MM` | 5 | Release threshold margin. |
| `HAND_HOLD_MS` | 3000 | Hold time to arm. |

Tuning: watch `HAND SENSOR`/`HAND` log lines. The detector needs 3 clean baseline
samples (front uncovered) first. If it false-triggers on a wall, lower the
trigger or shorten the hold. This is the least bench-testable feature because it
depends on the front sensor's near-field behaviour.

## H. Transports (not really "tuning")

- WiFi SSID/password and TCP port 23: `MoveForward.cpp` (move to a git-ignored
  `secrets.h` before publishing — see `06`).
- HTTP port 80, log ring 80 lines: `WebDashboard.cpp`.
