# Basic movement hardware tests

For sensor initialization failures or checking all three distances independently, run the [three-ToF diagnostic](calibration/tof_readings/README.md).

For hand-pushed encoder measurement over 144 cm and unfiltered MPU gyro/yaw checks, see [calibration](calibration/README.md).

These are standalone Arduino hardware tests, not automated unit tests. Run them in order before adding navigation. Each sketch is self-contained so it can be opened and uploaded directly; calibration changes are local to that sketch. The original main.ino is unchanged.

## Setup

Use the same ESP32 board configuration as the base sketch. Install the Pololu [VL6180X](https://github.com/pololu/vl6180x-arduino) and [VL53L1X](https://github.com/pololu/vl53l1x-arduino) libraries. Open one sketch, upload it, and open Serial Monitor at 115200 baud. The forward sketch waits for `start` over WiFi TCP (port 23) or USB Serial, and stops on `d`. Turn sketches retain their existing commands and `x` stop. Serial abort is checked between sensor reads, so it is not instantaneous during a blocking sensor read.

The pinout, motor polarity, brake behavior, balance factors, and encoder calibration come from main.ino. Initially check wheel directions with the wheels raised, then measure on the actual maze surface. Commands use the base sketch's left/right sign convention; confirm the physical direction before recording a pass. Single-channel encoders count pulses but cannot verify direction or detect wheel slip.

The MPU will share the ToF sensors' I2C bus (the board-default SDA/SCL used by Wire.begin()). The MPU is a direct MPU-6050 register driver at address 0x68 on SDA 21 / SCL 22: it verifies identity (WHO_AM_I), calibrates Z-gyro bias from 200 stationary samples, selects DLPF_6 and the +/-500 dps range, applies a 0.5 dps deadband, and integrates yaw without a software Kalman filter. The forward test uses MPU yaw in one-wall mode and blends it with encoders in no-wall mode; two-wall control remains unchanged. An MPU can help measure heading drift; it does not directly verify traveled distance.

## 1. Moving forward

Open `moving_forward/moving_forward.ino`. Send `start` to move seven 180 mm cells, continuously without intermediate stops. It remains stopped after the seventh cell. Send `d` during movement to cancel the remaining sequence. Each seven-cell run targets 4314 left ticks and 4322 right ticks, from the two hand-pushed seven-cell trials (4363/4360 and 4265/4283). If interrupted, the next start requests a new seven-cell sequence from the stopped position; reposition before retrying if necessary.

Mark a 1260 mm course and measure the final stop using the same chassis reference point. Encoder completion is not an independent distance measurement: record measured distance minus 1260 mm.

Forward movement now follows the working main.ino threshold steering when both walls exist: left below 40 mm commands 30/70, otherwise corrected right below 40 mm commands 70/30, otherwise 70/70. It brakes both wheels when either calibrated whole-run encoder limit is reached. The old wall-centering PID and corridor-sum rejection remain inactive. A separate encoder-distance PID slows the approach over the final 300 ticks, from cruise PWM 140 toward 35, to reduce overshoot. One-wall mode combines the remaining ToF with MPU yaw. No-wall case 3 blends encoder-progress PID (Kp=1.7, Ki=0.10, Kd=0.0, output limit 40 PWM) with MPU heading trim (Kp=2.0, rate damping 1.5, limit 15 PWM, 80% MPU weight), falling back to 100% encoder control only if the MPU is unavailable; progress is normalized by the manual 4314/4321.5 seven-cell averages. A healthy side measurement with no in-range target is treated as open space, while an I2C failure or timeout still stops the run. The front emergency brake stops the run at a valid reading of 40 mm or closer. See the [forward movement guide](moving_forward/README.md) for settings, transitions, fault handling, and tests.

Initial acceptance proposal: all five continuous seven-cell trials travel 1260 mm +/- 35 mm, lateral drift stays within 10 mm, and heading error stays within 3 degrees. These are bench targets to confirm, not competition requirements. Sensor initialization failure must refuse forward movement.

## 2. Turns

Open `turns/turns.ino`. Keep the robot still during MPU calibration, then mark the robot center and initial heading. Send `l` or `r` for a 90-degree turn; `d` or `x` aborts. Both wheels counter-rotate in place while a PID (Kp=15.9, Kd=1.3) commands the same PWM to both wheels, bounded between 95 and 150; MPU yaw is the feedback that controls the braking point (tolerance 1.0 degree, 10 stable samples, 2000 ms timeout). Encoders provide stall detection and diagnostics without changing motor commands. Repeat each command five times and measure angle and center displacement. See the [90-degree turn guide](turns/README.md) for settings and telemetry.

Initial acceptance proposal: angle error within 3 degrees, with center displacement within 5 mm. Verify the commanded direction as well as the angle. ToF readings do not protect the swept area, so provide clearance. The 180-degree turn remains deferred until both 90-degree directions are verified.

## 3. Full 360-degree turn

Open `turn_360/turn_360.ino`. Send `c` for clockwise or `a` for anticlockwise. Each commands one continuous rotation with 2480 ticks per wheel. Observe the entire rotation: matching the final heading alone cannot prove a full revolution. Repeat five times in each direction.

Initial acceptance proposal: one full revolution, final heading error within 5 degrees, and center displacement within 5 mm. A continuous 360-degree test also differs from four separate 90-degree turns; run four quarter turns afterward to check accumulated stop/start error.

## Results and calibration

Each run reports its stop reason, target, final left/right ticks, and elapsed time. The forward message RUN ENCODER LIMIT REACHED means at least one wheel reached its whole-run limit; physical measurements decide whether the behavior passed. The forward test brakes both wheels when either whole-run target is reached; the turn tests retain their own stopping behavior. A wheel without progress for 1500 ms ends the run. The total timeout is 105 seconds for continuous seven-cell forward movement and 2 seconds for a turn. Partial movement is a failed trial; reposition manually before retrying.

Record results in `results.csv` with target_mm = 1260 for continuous seven-cell trials. Change one parameter at a time. For consistent distance error across straight completed runs, an initial correction is new ticks_per_cell = old ticks_per_cell * 1260 / mean_measured_mm, separately for each wheel. Current values are approximately 616.29 left and 617.43 right ticks per cell (4314/7 and 4322/7); round only the final movement target. Rerun the test after updating the value; do not calibrate from obstructed, aborted, stalled, or curved runs. For turns, record physical angle, MPU yaw at brake/final, both encoder counts, and center displacement. Tune the yaw controller from repeated left/right results; encoder counts are diagnostics rather than the angle endpoint. Recheck both directions after any change. Copy confirmed values between sketches explicitly.

No physical run has been performed in this workspace. arduino-cli 1.5.1 is available and verifies that `main/` compiles for `esp32:esp32:esp32`; the standalone sketches still need the Arduino IDE plus the Pololu libraries listed under Setup.
