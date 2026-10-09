# Knowledge Base — Index

Deep-dive documentation recovered from the `main/` firmware. No firmware code
was modified to produce these docs. Code is the source of truth; if a doc and
the code disagree, the doc is wrong (and should be fixed).

| Doc | Read it for |
|---|---|
| [00 — System Overview](00-SYSTEM-OVERVIEW.md) | Whole-robot mental model, wiring, boot sequence, state machine, interfaces |
| [01 — Maze & Flood Fill](01-MAZE-ALGORITHM.md) | Wall model, `setWall` conflict policy, flood-fill BFS, `chooseNext`, speed-route planner, NVS |
| [02 — Motion Control](02-MOTION-CONTROL.md) | ToF timing/filters, wall regimes, all three steering PIDs, approach PID, front-wall 60 mm reference, motors/encoders |
| [03 — Rotation & Yaw](03-ROTATION-AND-YAW.md) | MPU config, the two drivers, the three-way sign chain, turn PID, alignment |
| [04 — Tuning Guide](04-TUNING-GUIDE.md) | Every tunable constant, its effect, and how to tune it |
| [05 — Testing & Calibration](05-TESTING-AND-CALIBRATION.md) | Host tests, calibration sketches, acceptance criteria, verifiable goals |
| [06 — Porting & Lessons](06-PORTING-AND-LESSONS.md) | Hardware seams, porting checklist, all known issues, lessons for the next robot |

Companion files:
- `../PROJECT_MAP.md` — living map, decisions log, orphans, milestones.
- `../main/unit-tests/` — original calibration sketches and controller tests.
