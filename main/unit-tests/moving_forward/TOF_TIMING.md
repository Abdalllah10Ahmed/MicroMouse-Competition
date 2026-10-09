# ToF sampling and filtering

> Scope note: this document describes the standalone `moving_forward`
> calibration sketch, whose `TofFilter` is a **three-sample median** (correct as
> written below). The current `main` firmware uses a **five-sample sliding
> average** instead (`TofFilter` in `main/MoveForward.h`, `samples_[5]`). The two
> filters have not been reconciled; treat the difference as a known divergence
> before reusing either (see `docs/06-PORTING-AND-LESSONS.md`).

Front VL53L1X: short mode, 20 ms measurement budget, 25 ms continuous period.
Left/right VL6180X: simultaneous single-shot starts, 30 ms maximum convergence
instead of the library default 49 ms. Shots overlap the front measurement wait.
The control loop yields for 5 ms after each sample; actual cadence depends on
measurement completion and communications. Read `dt ms` in telemetry to measure it.

Each sensor has a three-sample median filter. Valid side samples are filtered
before wall detection/PID; the 10 mm right inset is subtracted after filtering.
No-target (200 sentinel) and communication faults bypass smoothing and reset
history. Filters reset at each start. A real distance step normally takes two
samples to pass the median. Reduced convergence time can increase no-target
reports on weak targets; validate these settings on the robot.

Front filtering is diagnostic only: the emergency brake always uses the current
unfiltered front sample. Telemetry shows unfiltered front, filtered left,
unfiltered right (or sentinel), filtered/corrected right, filtered front and
unfiltered left (or sentinel). Front stop/fault skips side reads as before.

PID gains, motor factors and MPU sign are unchanged.

Wall detection now requires three consecutive filtered clearances below 80 mm
to acquire a wall, and releases it at 100 mm or on an invalid/no-target reading.
This excludes the logged open-space 87/134/190 mm right-clearance sequence.
Confirmation adds acquisition delay; verify real wall transitions on hardware.

No-wall approach uses a 70 PWM base floor and 60 PWM minimum corrected motor
command, replacing the logged stalled 35/30 output. These are starting values
requiring loaded hardware validation, including final stopping distance. The
encoder target and front emergency brake remain active. Encoder-only guidance
cannot independently measure or restore absolute heading after yaw drift.
