# PROJECT_MAP.md — MicroMouse ESP32 Firmware

> Purpose: persistent shared memory between sessions/agents. This file is the
> source of truth for *what the system is, how it flows, why choices were made,
> and what is unfinished*. Update it on every meaningful change.
> Status: M1 DONE (host suites + ESP32 build verified). Credential scrub and
> stale-doc corrections applied 2026-10-09. No firmware behaviour change has
> been made (only test-infra, secrets extraction, and docs).

Last updated: 2026-10-09

---

## [PROBLEM IN ONE SENTENCE]

An ESP32 micromouse that explores a 16x16 maze with flood-fill, stores the
solved maze in flash, then runs a saved straight-line speed route; the owner
wants to fully understand, document, tune, test, and de-risk this firmware
before the hardware is dismantled, so the knowledge transfers to future robots.

## [HARD CONSTRAINTS]

- Target: single ESP32 (Arduino framework). One cooperative `loop()`; blocking
  sensor reads are used. No FreeRTOS tasks beyond core WiFi/WebServer.
- Sensors: front VL53L1X (I2C 0x32) + left/right VL6180X (0x30/0x31) + MPU-6050
  (0x68), all sharing SDA=21 / SCL=22.
- Drive: 2 DC motors (IN pins below) + 2 **single-channel** encoders (no
  direction, no slip detection), motor enable pin 23.
- Build system: Arduino IDE sketch (`main/main.ino` + sibling `.cpp/.h`).
  Arduino build concatenates the `.ino`; `DemoMotion.h` is the adapter between
  the app and motion modules.
- No version control: this folder is **not** a git repo. Files are the only
  memory. (A repo exists at github.com/Abdalllah10Ahmed/MicroMouse-Competition.)
- Hardware is going to be dismantled; host tests + docs are the durable output.

## [TECH_STACK]

| Item | Version (verified locally) | Notes |
|---|---|---|
| arduino-esp32 core | **3.3.7 installed** (README claims 3.3.11 build) | pin to 3.x; `analogWrite`, `Wire`, `WiFi`, `WebServer`, `Preferences` used |
| Pololu VL53L1X | **1.3.1** (2022) | low release cadence; no CVE; acceptable |
| Pololu VL6180X | **1.4.0** | low release cadence; no CVE; acceptable |
| Host tests | **g++ 16.2.0 (MinGW-W64, UCRT) at `C:\mingw64`** | winget install lived under a path with spaces (`AppData\Local\WinGet\Packages\...`) which breaks MinGW `ld` internally; relocated to `C:\mingw64`; added to user PATH |
| arduino-cli | **1.5.1 at `%LOCALAPPDATA%\Programs\arduino-cli`** | downloaded from GitHub releases (not in winget); reuses the Arduino IDE's core dir `%LOCALAPPDATA%\Arduino15` (esp32 3.3.7 + tools already there) |
| Python | 3.14 present | unused by project |
| git | present | repo not initialized here |

Dependency risk: Arduino libraries are mature and low-risk. The real risk is
**tooling drift** (installed core 3.3.7 vs builds documented as 3.3.11) and the
**missing host compiler**, which blocks the regression tests.

## [SYSTEM_FLOW]

Startup (`main.ino`):
1. `beginRotation()` → `Wire.begin(21,22)` + MPU config + 200-sample gyro bias.
2. `moveForwardSetup()` → pins/motors, encoders, `Wire.begin(21,22)` again,
   three-ToF init/address assignment, WiFi connect (STA, 8.5 dBm), **second**
   MPU config + 200-sample bias.
3. `maze.reset()` or `maze.load()` from NVS. `dashboardBegin()` (HTTP :80).

Runtime `loop()`:
- `dashboardLoop()` collects web flags; `demoReadCommand()` merges USB serial,
  TCP :23, and web Start/Stop; hand-gesture detector can also Start.
- **Exploration step** (`runFloodStep`): settle → `readOpenPaths()` (front/left/
  right, `OPEN_CONFIRM_SAMPLES=8`, average of last 5) → `maze.observe()` +
  `floodFill()` → goal? save + arm speed run → `maze.chooseNext()` (flood
  distance, tie-break unknown cells) → `turnTo()` → re-scan verify edge →
  `demoMoveOneCell()` (wall-following + approach PID + front-wall 60 mm
  reference) → `alignArrivalHeading()` (MPU yaw) → advance pose →
  `markTraversed()` → `floodFill()`.
- **Speed step** (`runSpeedStep`): route built once by BFS over `traversed`
  edges from (0,0) to goal; per run of identical directions, `turnTo` then
  `demoMoveStraightCells(n)` at cruise PWM 190, goal detection stops.
- Hand gesture: cover/hold front ToF near a baseline for 3 s → arm → release
  → Start. `HAND_*` constants in `main.ino`.
- Web UI (`WebDashboard.cpp`): JSON grid (walls/known/visited/distance) + 80-line
  log ring, polled by browser every 400 ms.

## [ARCHITECTURE]

Feature-oriented, flat, minimal abstraction (matches `main/AGENTS.md`).

| File | Role |
|---|---|
| `main/main.ino` | App state machine: pose, exploration, speed run, hand gesture, LED, logging orchestration |
| `main/MazeMap.{h,cpp}` | Wall/known/traversed bitfields, flood fill, goal (2x2 center), NVS save/load |
| `main/MoveForward.{h,cpp}` | Motion: ToF drivers + filters, MPU yaw, encoders, motors, wall PID (2-wall / 1-wall / no-wall), approach PID, front-wall reference, command parser, telemetry |
| `main/rotation.{h,cpp}` | Independent MPU + in-place turn PID (relative angle) |
| `main/WebDashboard.{h,cpp}` | WebServer, JSON state, log ring, start/stop/reset flags |
| `main/DemoMotion.h` | Adapter contract the app uses to reach motion + rotation |
| `main/config.h` | Tunables: opening thresholds, tolerances, settle |
| `demo/right-hand/*` | Earlier right-hand-only snapshot (superseded by `main`) |
| `main/unit-tests/*` | Standalone calibration sketches + host controller tests = tuning knowledge base |
| `main/tests/maze_map_test.cpp` | Only host test runnable from `main/` today |

Data model: each cell has `walls`, `known`, `traversed` (bit per absolute
direction `N=0,E=1,S=2,W=3`), `visited`, `distance`. `traversed` is a
monotonic record of edges physically crossed and cannot be closed by a later
sensor reading.

## [FAILURE_MODES THE ARCHITECTURE MUST SURVIVE]

- ToF I2C fault / range timeout → `sideCommunicationFault` / `tofFailure`, scan
  retries 3x, invalid front never becomes an opening.
- MPU stale (`>250 ms`) / lost config → refuse movement; 1-wall mode stops;
  alignment fails safe.
- Encoder stall (1500 ms) / whole-move timeout (105 s) → stop, pose unchanged.
- Unexpected front wall during straight speed segment → verify against known
  edges or stop with pose uncertain.
- WiFi drop / client disconnect → USB-only operation must still work.
- Command fragmentation/partial input → per-transport `MovementCommands`; stop
  wins over start in the same batch.
- NVS save/load failure → speed run disabled, exploration still usable.
- Blocking reads delaying commands → mitigated by cooperative servicing and
  `discardPendingCommands()`.

## [DECISIONS_LOG] (running — inferred; confirm/annotate)

| # | Decision | Why (inferred) |
|---|---|---|
| D1 | Flood-fill exploration instead of wall-follower | optimal known path + enables saved speed run |
| D2 | `traversed` bitmask that sensors cannot clear | prevents a bad reading from invalidating a physically crossed edge |
| D3 | Outer boundary walls cannot be cleared | physical truth, guards map corruption |
| D4 | Brake both wheels at first encoder limit | avoids pivoting/heading error; trades exact distance |
| D5 | Front-wall 60 mm reference corrects longitudinal stop drift | encoder-only stopping drifts; wall is a physical reference |
| D6 | Right ToF 10 mm inset correction | sensor recessed; recovers true chassis clearance |
| D7 | Save map to NVS, speed run on next gesture | survives reset without re-exploring |
| D8 | Speed route = BFS over confirmed `traversed` edges only | safety: never drive an unverified edge |
| D9 | Two independent MPU drivers (rotation + forward) | reuse two already-tested modules; accepted redundancy |
| D10 | `OPEN_CONFIRM_SAMPLES` raised 3→8, averaged last 5 | improve opening reliability (demo used 3, conservative min) |
| D11 | Config values differ from `demo/right-hand` | main tuned later (see orphans) |
| D12 | Test infra fix: in-memory `Preferences` fake (`main/tests/fakes/`) | host maze test needs the ESP32 NVS API; swappable for a richer fake later (M3) |
| D13 | Toolchain at `C:\mingw64` (not the winget package dir) | MinGW `ld` breaks when the install path contains spaces; space-free path is the standard fix |

## [TUNING MAP] (where to change what)

- Openings / tolerances / settle: `main/config.h`.
- Motor pins/polarity/factors, encoder ticks/cell, cruise & floors, all PID
  gains, MPU sign, ToF timing, front-wall band: `main/MoveForward.cpp`.
- Turn PID (`Kp=15.9, Kd=1.3`, min 130, max 180, tol 1.5°, 10 stable samples,
  2200 ms) and its MPU pins: `main/rotation.cpp`.
- Hand-gesture thresholds: `main.ino` (`HAND_*`).
- WiFi SSID/password and TCP port: `main/MoveForward.cpp` (see security note).
- Calibration procedures + recorded invariants: `main/unit-tests/README.md`,
  `*/calibration/*/README.md`, `moving_forward/{README,MOTOR_MAPPING,TOF_TIMING}.md`.

## [KNOWLEDGE BASE] (M0/M2 — created 2026-10-09)

Deep-dive docs live in `docs/`. Index: `docs/README.md`.
- `00-SYSTEM-OVERVIEW.md` — wiring, boot, state machine, interfaces.
- `01-MAZE-ALGORITHM.md` — data model, `setWall`, flood BFS, `chooseNext`,
  speed-route planner, NVS.
- `02-MOTION-CONTROL.md` — ToF/filters, wall regimes, steering PIDs, approach PID,
  front-wall 60 mm reference, motors/encoders.
- `03-ROTATION-AND-YAW.md` — MPU config/drivers, yaw sign chain, turn PID.
- `04-TUNING-GUIDE.md` — every tunable.
- `05-TESTING-AND-CALIBRATION.md` — tests, sketches, acceptance, bench goals.
- `06-PORTING-AND-LESSONS.md` — hardware seams, porting checklist, issues I1–I13.

## [ORPHANS & PENDING]

> Root-cause analysis and proposed fixes for the items below are in
> `docs/06-PORTING-AND-LESSONS.md` (issues I1–I13). Items stay here until the
> code is actually changed and verified.

- [x] `main/README.md` stale — **corrected 2026-10-09**: now documents the speed
      run (straight segments at cruise PWM 190), the hand gesture, and 8-sample
      confirmation (average of last 5).
- [ ] Duplicate MPU driver: `rotation.cpp` and `MoveForward.cpp` both configure
      MPU at 0x68 and each run a 200-sample bias (~2 s x2) at boot.
- [ ] Motor pin naming inconsistency: `rotation.cpp` calls 25/26 "left"; 
      `MoveForward.cpp` + `MOTOR_MAPPING.md` call 25/26 physical right (27/14
      left). Functionally harmless for spin but confusing/portability hazard.
- [ ] `main/config.h` vs `demo/right-hand/config.h` diverge (samples 8 vs 3,
      front tolerance 2% vs 3%, extra-approach 130 vs 120 mm, settle 120 vs 150).
- [~] ToF filter mismatch — **documented 2026-10-09**: `main` uses a 5-sample
      sliding average; the standalone `moving_forward` sketch uses a 3-sample
      median. A scope note in `TOF_TIMING.md` flags the divergence; reconciling
      the two code paths remains open (M5).
- [x] `unit-tests/README.md` stale — **corrected 2026-10-09**: replaced
      MPU6500_WE/Kalman with the real direct MPU-6050 driver; fixed encoder
      targets (4314/4322, not 4359/4335), cruise PWM (140), case-3 MPU blend
      (Kp 1.7 + MPU trim), turn PID (95..150, 2 s), and ticks/cell
      (616.29/617.43).
- [~] `unit-tests/moving_forward/README.md` has wider numeric drift (base PWM
      70 vs 140, balance factors, SINGLE_WALL_KP 3.0 vs 6.0, wall thresholds
      120/140 vs 80/100). A verified drift banner was added at the top; a full
      rewrite is deferred (M5). Code is the source of truth.
- [ ] No host test for exploration state machine (`runFloodStep`) or speed-route
      BFS planner; only `tests/maze_map_test.cpp` exists under `main/`.
- [ ] Dead code in `main` context: `moveForwardLoop()`/`runMove()` (only used by
      the standalone forward sketch), `forwardWallCommands`, `MovementCommand`
      merge paths kept for reuse.
- [x] WiFi credentials hardcoded — **scrubbed 2026-10-09**: real values moved to
      git-ignored `main/secrets.h` (+ committed `main/secrets.example.h`, root
      `.gitignore`); `MoveForward.cpp` includes `"secrets.h"`; the three
      detection/calibration sketches now carry placeholders. Verified: esp32
      build still passes, all host suites still green.
- [x] Host compiler (g++) missing — **resolved M1**: g++ 16.2.0 at `C:\mingw64`;
      all host suites green (see MILESTONES). Note: keep the toolchain at a
      path without spaces (MinGW `ld` cannot handle them — not PowerShell's
      fault; rewriting the scripts to "fix" this would be wasted effort).
- [ ] No version control in this working copy.

## [OUT OF SCOPE — v1] (proposed; owner to confirm)

- New planning algorithms (A*, Dijkstra with diagonals, partial re-flood).
- Diagonal / slalom speed-run moves, PID speed profiling beyond straight runs.
- BLE, battery telemetry, PCB or mechanical redesign.
- Rewriting already-tested controllers (rotation/forward) while hardware is
  still needed as a known-good baseline.

## [OWNER DECISIONS] (2026-10-09 — answers to the planning gate)

- Objective #1: **knowledge capture + documentation** first; no firmware
  behaviour change until explicitly approved afterwards.
- Robot status: **bench-only** — assembled and flashable, but no maze available.
  Physical validation is limited to wheel-up / bench measurements; no
  competition-run claims may be made from this environment.
- System of record: **`main/` only**. `demo/right-hand/` is history/supporting
  evidence, not a deliverable.
- Toolchain: install **g++ (host tests) + arduino-cli (headless build)**.
- Publishing: a GitHub push is planned soon → **scrub the hardcoded WiFi
  credentials from source before any commit**; never log the password.
- 2026-10-09 follow-up approvals: (a) **apply the credential scrub now** (done);
  (b) **correct the stale docs under `main/`** (done — README, unit-tests/README,
  TOF_TIMING, plus a drift banner in moving_forward/README).

## [MILESTONES — verifiable goals]

| M | Goal | Verification |
|---|---|---|
| M0 | Knowledge baseline captured (this file + `docs/` explainers + algorithm math + pinout) | DONE — `docs/00`–`06` written; every source file mapped; owner review pending |
| M1 | Reproducible build + test baseline | DONE (2026-10-09) — g++ 16.2.0 (`C:\mingw64`) + arduino-cli 1.5.1 installed; `main` compiles for `esp32:esp32:esp32` (1025319 B flash / 78%, 52796 B RAM / 16%); all host suites PASS: maze-map (`main/tests`), 7 controller tests + 2 checkers (`moving_forward/tests`), turns (`turns/tests`), demo right-hand ([nav, scan, arrivals]) |
| M2 | Algorithm deep-dive docs: flood fill, wall FK, PID laws, yaw integration, ToF timing, hand gesture | DONE — `docs/01`,`02`,`03`; owner can predict behaviour from doc |
| M3 | Close test gaps: host tests for exploration state machine and speed-route BFS | new tests fail-before/pass-after; full suite green |
| M4 | Portability kit for future robots: hardware-seam inventory + calibration procedure + parameter manifest | checklist a new robot can follow end-to-end |
| M5 | (Approval-gated) de-dup MPU init, unify config, correct stale docs | build + host tests + before/after boot time/behaviour |
| M6 | Pre-disassembly capture: firmware version, calibration values, measured logs, map dump, wiring record | structured handover doc + archived logs |
