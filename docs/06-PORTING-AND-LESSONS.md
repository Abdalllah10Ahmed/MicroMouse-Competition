# 06 — Porting to a New Robot, Lessons & Known Issues

This is the document that should outlive the hardware. Read it before starting
a new micromouse so the same mistakes are not repeated.

---

## 1. Hardware abstraction seams (what must change on a new robot)

The firmware is **not** abstracted behind hardware interfaces; the values are
scattered across three files. These are the exact seams:

| Seam | File(s) | What changes |
|---|---|---|
| MCU / framework | both | ESP32 + Arduino core assumed |
| Motor pins, polarity, balance | `MoveForward.cpp` (`drive`/motors) **and** `rotation.cpp` (duplicated!) | GPIO numbers, `reversePolarity`, factors |
| Encoder pins | `MoveForward.cpp` | GPIO33/35, rising-edge only |
| Encoder ticks/cell | `MoveForward.cpp` | `LEFT/RIGHT_TICKS_PER_CELL` |
| ToF pins/addresses | `MoveForward.cpp` | XSHUT GPIO4/5/16, addr 0x30/0x31/0x32 |
| Right inset | `MoveForward.cpp` | `RIGHT_TOF_INSET_MM` |
| IMU + yaw sign | `MoveForward.cpp`, `rotation.cpp` | address 0x68, `MPU_YAW_SIGN` |
| Opening thresholds | `config.h` | `FRONT/SIDE_OPEN_MM` |
| Turn gains | `rotation.cpp` | turn PID |
| Maze size / goal | `MazeMap.h`/`.cpp` | `MAZE_SIZE`, `isGoal`, flood seed |
| Transports | `MoveForward.cpp`, `WebDashboard.cpp` | WiFi creds, ports |

**Highest-value refactor for reuse (do not do blind):** put all pin/sensor/geometry
constants in one `robot_config.h` and have both `MoveForward.cpp` and
`rotation.cpp` include it, eliminating the duplicate pin definitions. This is the
single change that most reduces porting errors.

## 2. Porting checklist (order matters)

1. **Motor mapping** — run `motor_mapping.ino`; confirm which GPIO pair is which
   physical wheel and which direction is forward; record factors.
2. **Encoders** — hand-push exactly 1260 mm, ≥5 runs, `encoder_ticks.ino`; set
   `*_TICKS_PER_CELL`; verify with a powered 7-cell run.
3. **ToF** — `tof_readings.ino`; confirm addresses, label↔position, measure
   mounting offset (esp. a recessed side sensor → inset constant).
4. **MPU** — `mpu_yaw.ino`; measure drift and confirm the sign of a commanded
   turn; set `MPU_YAW_SIGN` with a bench test.
5. **Forward acceptance** — 5×7 cells: 1260 ± 35 mm, drift ≤ 10 mm, heading ≤ 3°.
6. **Turn acceptance** — 5× each direction: ≤ 3°, centre ≤ 5 mm; then 360°.
7. **Openings** — set `FRONT/SIDE_OPEN_MM` from measured corridor clearance.
8. **Maze** — set `MAZE_SIZE`/goal; confirm the physical start orientation maps
   to `(0,0,North)`.
9. **Record everything** in `results.csv` and a per-robot `ROBOT.md`.

## 3. Known issues (root cause + fix)

### I1 — Stale documentation
- `main/README.md`: claims "no speed-run code" (there is one) and "three samples"
  (it's 8); omits the hand gesture.
- `moving_forward/TOF_TIMING.md`: says 3-sample median; `main` uses a 5-sample
  average.
- `unit-tests/README.md`: claims `MPU6500_WE` + Kalman; code is a direct MPU-6050
  driver, no Kalman.
- `config.h`: comment describes a 3% band; value is 2%.
- `demo` README turn gains (95/150/1.0/2000) differ from `rotation.cpp`
  (130/180/1.5/2200).
- **Fix:** correct to the code; treat code as truth. These docs are the reason
  this project needed a knowledge-recovery pass.

### I2 — Duplicate MPU initialisation
- Both `rotation.cpp` and `MoveForward.cpp` configure the MPU and run a 200-sample
  bias. ~4 s stationary boot, two writers to one sensor.
- **Fix (after toolchain exists, verify by build + bench):** let one driver own
  the device; expose `yaw()/rate()/healthy()` from a single object; have the
  other consume it. Preserve the 0.5 dps deadband and the >100 ms
  no-integration rule.

### I3 — Motor label mismatch
- `rotation.cpp` labels 25/26 LEFT; `MoveForward.cpp` + `MOTOR_MAPPING.md` say
  25/26 is physical RIGHT. Behaviour is correct because spins are symmetric, but
  any future asymmetric use of `rotation.cpp`'s drive will invert a wheel.
- **Fix:** adopt the verified physical mapping everywhere.

### I4 — Config drift between `main` and `demo`
- samples 8 vs 3, front tolerance 2% vs 3%, extra approach 130 vs 120 mm, settle
  120 vs 150 ms.
- **Fix:** treat `main` as canonical; delete or clearly archive `demo`, or add a
  banner saying it is history.

### I5 — Filter behaviour differs (average vs median)
- `main` 5-mean; `demo` 3-median. Docs describe median.
- **Fix:** pick one, document it, and align the tests.

### I6 — Hardcoded WiFi credentials (security)
- SSID/password are string literals in `MoveForward.cpp`; the SSID is also
  printed at boot.
- **Fix before any push:** move both into `main/secrets.h`, add `main/.gitignore`
  with `secrets.h`, commit `main/secrets.example.h`, and `#include "secrets.h"`.
  Never log the password. Consider not printing the SSID either.
  Not yet applied because it changes working code and must be build-verified
  first (toolchain pending).

### I7 — Test gaps
- No host test for the exploration loop or the speed-route planner; `results.csv`
  empty.
- **Fix:** add host tests (M3); populate `results.csv` on the bench.

### I8 — Single-channel encoders
- No direction, no slip detection. The 1.5 s stall check only catches a
  completely static wheel; creeping/slipping passes.
- **Mitigation now:** treat encoder travel as *relative*; use the front-wall
  reference and MPU heading as independent checks (already done in places).
- **New robots:** use quadrature encoders.

### I9 — No-target sentinel masking
- A healthy side sensor with no in-range target becomes 200 mm (open). Correct
  above the 100 mm retain threshold, but it means "open" and "sensor sees
  nothing" are indistinguishable in the log.
- **Fix:** log the sentinel distinctly (partially done via `SIDE_NO_TARGET_MM`).

### I10 — Front-wall candidate heuristic is hard to reason about
- `wallCandidate` in `runForwardDistance` combines filtered max/min and travel
  thresholds. A wrong classification can either skip a needed wall reference or
  trigger a failed approach.
- **Fix:** add host tests around the recorded examples before changing it.

### I11 — Hand-gesture reliability
- Uses the front ToF near-field; needs 3 clean baseline samples and a 3 s hold.
  A wall ahead can interfere.
- **Mitigation:** keep `s/start` as the primary reliable trigger; treat the
  gesture as convenience.

### I12 — Dead code / reuse leftovers
- `moveForwardLoop()`/`runMove()` (standalone forward sketch entry) are unused by
  `main`; `forwardWallCommands` is kept for reuse.
- **Do not delete while hardware exists** — they are the standalone bench entry.
  Mark clearly instead.

### I13 — Blocking logging during motion
- `debugPrintln` does Serial + TCP + `String` concatenation; called in the motion
  loop every 200 ms and inside sensor waits. At 115200 it can delay control.
- **Fix:** throttle telemetry (partially done: 200 ms), avoid `String` churn, or
  buffer.

## 4. Lessons for the next robot

1. **A wall reference beats an encoder** for longitudinal accuracy, but it costs
   complexity — implement it behind a clean interface from day one.
2. **One owner per hardware bus.** Two MPU drivers and duplicated pin tables
   cost boot time and created the naming trap.
3. **Calibrate and record before competing.** The single biggest loss was not
   having `results.csv` populated or credential-safe code.
4. **Docs rot fastest.** Generate config tables from constants or keep them next
   to the code.
5. **Single-channel encoders are a false economy** — quadrature is cheap and
   enables slip/direction detection.
6. **Separate "think" (maze/flood) from "do" (motion).** This codebase did that
   well: `MazeMap` is pure and host-testable; keep that boundary.
7. **Make the safety rules explicit and testable** (`traversed` cannot be
   re-closed; unknown edges are passable; both wheels brake together). These are
   the decisions most likely to be "simplified away" by a future edit.
