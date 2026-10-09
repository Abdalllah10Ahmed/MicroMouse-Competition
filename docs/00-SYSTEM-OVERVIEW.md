# 00 — System Overview

Purpose: a single mental model of the whole robot. Read this first; the other
docs zoom in. System of record: `main/`. Docs describe the code **as it exists**
(no edits made).

---

## 1. What the robot does

1. Boots, calibrates the gyro, initialises 3 time-of-flight (ToF) sensors, WiFi.
2. Starts exploration (hand gesture **or** `s`/`start`).
3. Explores the 16×16 maze cell-by-cell using **flood fill** on the walls it has
   discovered, until it reaches the 2×2 centre goal.
4. Saves the solved maze to flash (NVS) and arms a speed run.
5. Operator re-places the robot at the start; a second start gesture runs the
   **saved straight-line route** at higher speed.

There is also a scripted unit-test mode (`MAIN_UNIT_TEST_MODE`) for bench tests.

## 2. Hardware and wiring

| Item | Detail |
|---|---|
| MCU | ESP32 (Arduino-ESP32 core 3.x), WiFi STA |
| Front range | VL53L1X, I2C addr `0x32`, XSHUT GPIO16 |
| Left range | VL6180X, I2C addr `0x30`, XSHUT GPIO4 |
| Right range | VL6180X, I2C addr `0x31`, XSHUT GPIO5 |
| IMU | MPU-6050, I2C addr `0x68` (shared bus) |
| I2C bus | SDA GPIO21, SCL GPIO22 |
| Motor enable | GPIO23 (LOW = disabled at boot) |
| Physical LEFT motor | IN1 GPIO27, IN2 GPIO14, encoder GPIO33, balance factor 0.99 |
| Physical RIGHT motor | IN1 GPIO25, IN2 GPIO26, encoder GPIO35, balance factor 1.00 |
| Status LED | `LED_BUILTIN` (else GPIO2) |
| Commands | USB Serial 115200, TCP :23, HTTP :80 (dashboard) |

⚠ **Motor naming trap:** `rotation.cpp` labels GPIO25/26 as "LEFT"; the verified
mapping in `MOTOR_MAPPING.md` and `MoveForward.cpp` says GPIO25/26 is physical
**RIGHT**. Behaviours still come out correct (spins are symmetric), but never
trust the label — trust the pin pair.

## 3. Files and responsibilities

```
main/
  main.ino          application state machine, exploration, speed run, hand gesture
  MazeMap.{h,cpp}   wall model, flood fill, goal test, NVS save/load
  MoveForward.{h,cpp} sensors, filters, motor drive, wall PIDs, approach PID,
                      front-wall reference, command parser, telemetry, demo adapter
  rotation.{h,cpp}  dedicated MPU yaw + in-place turn PID
  WebDashboard.{h,cpp} HTTP server, JSON grid state, 80-line log ring
  DemoMotion.h      the adapter contract between app and motion/rotation
  config.h          opening thresholds and tolerances
  tests/            one runnable host test (maze map)
  unit-tests/       standalone calibration sketches + host controller tests
```

## 4. Boot sequence (why it takes seconds)

`setup()` in `main.ino`:

1. `beginRotation()` → `Wire.begin(21,22)` → `initMPU()` (verify WHO_AM_I,
   DLPF=6, ±500 dps) → **200-sample gyro bias (~2 s)**.
2. `moveForwardSetup()` → motor pins, encoder interrupts → `Wire.begin(21,22)`
   again → three-ToF init/addressing → `initWiFi()` (up to 15 s) → **a second
   200-sample gyro bias (~2 s)** via the separate `MpuYaw`.
3. `maze.load()` from NVS (else `resetMap()`), `dashboardBegin()`.

**Known waste:** the MPU is fully initialised and biased twice by two drivers.
See `06-PORTING-AND-LESSONS.md` (issue I2).

## 5. Runtime state machine

```
                 hand gesture / s / start
        Idle ────────────────────────────► Exploring (runFloodStep per cell)
          ▲                                     │ goal
          │ stop/reset                          ▼
          │                              GoalReached: save map, arm speed run
          │                                     │
          │  hand gesture / s / start (map loaded, robot at start)
          └────────────────────────────► SpeedRunning (runSpeedStep per segment)
                                              │ goal
                                              ▼
                                           Stopped (LED goal-blink)
```

Key app state (`main.ino`): `robotX, robotY, robotHeading, completedCells`,
`explorerRunning`, `speedRunning`, `savedMapReady`, `handState`, `ledMode`.

## 6. One exploration cell (the core loop)

`runFloodStep()`:

1. `settle()` — 120 ms pause, abortable.
2. `readOpenPaths()` — 8 scans; front/left/right = mean of the last 5; classify
   `OPEN` if `> 100 mm` (`FRONT_OPEN_MM`/`SIDE_OPEN_MM`).
3. `maze.observe(x,y,heading,frontWall,leftWall,rightWall)` + `floodFill()`.
4. If goal cell → `completeExploration()` (save, arm speed run).
5. `maze.chooseNext()` — neighbour with smallest flood distance (ties favour
   unvisited).
6. `turnTo(selected)` — 0/±90/180° via the rotation PID.
7. Re-scan to verify the chosen edge is still open; if blocked, stay and replan.
8. `demoMoveOneCell()` — wall-following drive to the next cell centre.
9. `alignArrivalHeading()` — MPU yaw correction if |yaw| > 1°.
10. Advance `robotX/robotY`, `markTraversed()`, `floodFill()`, LED blink, log.

Coordinates change **only** after a completed move + alignment; any fault leaves
the pose unchanged and stops the run (resume with `s`, or `reset` after
physically returning to start).

## 7. One speed-run segment

`runSpeedStep()`, route precomputed by `planSpeedRoute()`:

1. Group consecutive identical directions into a run of N cells.
2. `turnTo(direction)`, then `demoMoveStraightCells(N)` at cruise PWM 190.
3. If a front wall ends the run early, only accept it if the stored map already
   knows that wall; otherwise fault with uncertain pose.
4. Align heading (unless the endpoint is the goal), advance pose, next segment.

## 8. External interfaces

- **Command parser** (`MovementCommands`, one per transport): accepts `s`/`start`,
  `d`/`stop` (immediate, no Enter needed); stop beats start in a batch.
- **Hand gesture**: hold a hand/object within the computed near threshold of the
  front ToF for 3 s, then remove → start. See `04-TUNING-GUIDE.md`.
- **Web dashboard**: live 16×16 grid (walls/known/visited/distance), pose,
  status, log stream, Start/Stop/Reset buttons.
- **Telemetry**: `POSE`, `SENSORS`, `MAP`, `FLOOD`, `MOVE`, `MOVE END`,
  `TURN`, `TURN END`, `WALL APPROACH`, `SCAN ERROR`, `FAULT`, `SPEED`.

## 9. Where the risk lives

| Risk | Location |
|---|---|
| Boot assumes robot still for **two** gyro calibrations | `rotation.cpp` + `MoveForward.cpp` |
| Two MPU drivers sharing one register set | same |
| Hardcoded WiFi credentials | `MoveForward.cpp` |
| Single-channel encoders (no direction/slip) | `MoveForward.cpp` |
| Blocking ToF/network calls delay commands | `observe()`, `debugPrintln()` |
| Docs drift from code | `README.md`, `TOF_TIMING.md`, `unit-tests/README.md` |
