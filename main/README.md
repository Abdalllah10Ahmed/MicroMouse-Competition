# Flood-fill exploration firmware

Open main.ino in Arduino IDE and upload it to the ESP32. This firmware uses
the tested one-cell forward and MPU6050 rotation controllers from the
right-hand demo. It has two modes of operation:

- **Exploration** (`s`/Start): flood-fill exploration of the 16x16 maze. The
  solved map is saved to ESP32 flash (NVS) as soon as the goal is reached.
- **Speed run** (automatic when a saved map exists): replays the saved map as
  straight-line segments in runs of identical directions at cruise PWM 190,
  turning only between runs.

A hand gesture can also start a run: cover and hold the front ToF near a
baseline for 3 s to arm, then release to start (see `HAND_*` in main.ino).

Keep the robot still during startup calibration. When Wi-Fi connects, Serial
prints a URL such as:

    WEB | open http://10.234.131.104/

Open that address from a device on the same Wi-Fi network. The page shows:

- the robot's current cell and cardinal heading;
- visited cells and every known wall;
- the current flood-fill distance in each cell;
- the same diagnostic log written to USB Serial and TCP port 23;
- Start, Stop and Reset-map controls plus a command input.

The command input accepts s/start, d/stop, and r/reset. Start resumes the
stored pose and map. Reset stops an active run and returns the pose to (0,0)
facing north.

## Map convention

The maze is 16 by 16 cells. The starting cell is (0,0), north increases Y,
and east increases X. The four goal cells are (7,7), (7,8), (8,7) and (8,8).
If the physical starting direction is not north in this coordinate system,
rotate the maze convention or change robotHeading before running.

At each cell the firmware confirms each opening with 8 repeated samples
(`OPEN_CONFIRM_SAMPLES`, after a 120 ms settle) and classifies it from the
average of the last 5, stores the walls in absolute directions, recalculates
flood distances, turns toward the best reachable neighbor, verifies that edge
again, and moves one cell. Unknown
internal edges are considered traversable until measured. Outer boundary walls
cannot be cleared by a sensor reading.

Coordinates change only after a complete cell move and heading correction.
Movement, scan or turn faults leave the stored coordinates unchanged and stop
exploration. A fresh s resumes from that stored position; use Reset only when
the robot has physically returned to the start pose.

## Useful debug lines

- POSE: stored coordinates, heading, flood value and completed moves.
- SENSORS: confirmed front, left and right classifications and distances.
- MAP: absolute cell update and any contradiction with a previous reading.
- FLOOD: chosen direction, next coordinates, distance and visited state.
- TURN PLAN / MOVE PLAN: intended physical action before it begins.
- REPLAN: a post-turn scan found the selected edge blocked.
- FAULT: movement stopped and coordinates were not advanced.

Run the host map test with:

    powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_tests.ps1
