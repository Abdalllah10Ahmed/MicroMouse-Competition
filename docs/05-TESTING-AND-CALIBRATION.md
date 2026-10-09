# 05 — Testing & Calibration

Three layers exist: **host tests** (no hardware), **standalone calibration
sketches** (hardware, manual), and **in-app acceptance logs** (hardware). Today
only the host layer is automated, and it cannot run because `g++` is missing on
this machine.

---

## 1. Host tests (pure C++, no hardware)

| Location | What it covers | Toolchain |
|---|---|---|
| `main/tests/maze_map_test.cpp` | boundaries, conflicts, flood distances, already-traversed edges, `chooseNext`, goal | g++ `-std=c++17` via `tests/run_tests.ps1` |
| `main/unit-tests/moving_forward/tests/single_wall_test.cpp` | wall classification/hysteresis, both one-wall sides, heading/rate corrections, command bounds | g++ `-std=c++11` |
| `.../cell_approach_test.cpp` | approach-PID deceleration, limits, reset, anti-windup, brake-lead arithmetic | g++ |
| `.../forward_wall_control_test.cpp` | recorded sensor examples, thresholds, left-first priority, invalid-side handling, joint braking | g++ |
| `.../no_wall_test.cpp` | case-3 reference capture, manual calibration, both steering directions, integral, anti-windup | g++ |
| `.../movement_commands_test.cpp` | USB/WiFi command parser | g++ |
| `.../tof_filter_test.cpp`, `.../two_wall_pid_test.cpp` | filter + two-wall PID | g++ |
| `.../check_movement_sequence.ps1` | extracts real movement functions, fake time/sensors/motors, running `g++` | PowerShell + g++ |
| `.../check_motor_mapping.ps1` | actual pin writes, polarity, factors, braking | PowerShell + g++ |
| `main/unit-tests/turns/tests/run_tests.ps1` | rotation + command code with simulated Arduino/WiFi/I2C | PowerShell + g++ |
| `demo/right-hand/tests/run_tests.ps1`, `check_*` | right-hand navigation + forward arrivals | PowerShell + g++ |

Run from the relevant folder:
```
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_tests.ps1
```

### Toolchain setup (approved, in progress)
- **g++**: installed via `winget install --id BrechtSanders.WinLibs.POSIX.UCRT`
  (adds a MinGW `g++` to PATH). Verify with `g++ --version`.
- **arduino-cli**: for headless builds; still to be located/installed. Once
  present, a build is `arduino-cli compile --fqbn esp32:esp32:esp32 main`.
- If WinLibs is installed but PATH is stale, open a new shell or use the full
  path it reports.

## 2. Calibration sketches (`main/unit-tests/calibration/`)

Run in this order on a new robot **before** trusting the main firmware:

| Sketch | Purpose | Acceptance |
|---|---|---|
| `motor_full_power/` | raw direction + strength check, wheels raised | correct wheel forward for each side |
| `motor_mapping/` | map labelled outputs to physical wheels + encoder deltas | labels match reality; record mapping |
| `encoder_ticks/` | hand-push 1260 mm, average ≥5 runs | ticks/mm per wheel, low variance |
| `tof_readings/` | 3-ToF init + addresses + per-sensor distance | 0x30/0x31/0x32 present; correct label-to-position; record offsets |
| `mpu_yaw/` | raw bias-corrected rate + relative yaw | drift over 30 s; matches ±90/180/360 marks |

## 3. Module acceptance tests

`moving_forward/` (from the docs):
- 5 continuous 7-cell runs: 1260 ± 35 mm, lateral drift ≤ 10 mm, heading error
  ≤ 3°.
- `start` → 7 cells without intermediate stops; `d` cancels; obstacle stops.
- Front wall near 39/40/41 mm behaviour; both-wheel braking.

`turns/`:
- 5× left and 5× right: angle error ≤ 3°, centre displacement ≤ 5 mm.
- Commanded direction verified physically.

`turn_360/`:
- 5× each direction: one full revolution, final heading error ≤ 5°, centre
  displacement ≤ 5 mm.

## 4. Results log

`main/unit-tests/results.csv` is the intended record
(`behavior,direction,trial,target_mm,target_deg,left_ticks,right_ticks,
measured_mm,measured_deg,lateral_or_center_drift_mm,heading_error_deg,
stop_reason,pass,notes`). It is currently **empty besides the header** — i.e. no
physical trial data survives in the repo. On a bench, record at minimum:
motor mapping, encoder ticks/cell per wheel, ToF offsets, MPU drift/sign, and
turn-angle results.

## 5. Verifiable goals for the current (bench-only) environment

- **G-T1** `main` compiles for `esp32:esp32:esp32` with no warnings.
- **G-T2** all host tests pass (`maze_map_test`, moving_forward suite, turns
  suite).
- **G-T3** wheels-up: `start` rides both wheels forward, `d` stops instantly,
  front obstacle ≤ 100 mm brakes.
- **G-T4** encoder ticks reported by `MOVE END` reproduce a hand-measured
  180 mm within the documented tolerance.
- **G-T5** MPU: `turnDegrees(+90)` visibly turns the chassis left; yaw returns
  near zero when still.

Maze-dependent criteria (front-wall reference accuracy, exploration success,
speed run) are **not** verifiable in this environment and must be marked
unverified.

## 6. Test gaps (to close)

- No host test for `runFloodStep` (exploration state machine) or
  `planSpeedRoute` (speed-route BFS).
- No test harness for `main.ino`'s hand-gesture state machine.
- `results.csv` has no data.
- `demo` and `main` tests are not unified; `main` has only one host test.
