# BIOBUZZ Turret Robot

Neuron Pathing quickstart + FTC SDK 12.0 (2026-27). BIOBUZZ code is in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/`.

## Hardware config names
| Device | Name |
|---|---|
| Mecanum drive | `frontLeft`, `frontRight`, `backLeft`, `backRight` |
| goBILDA Pinpoint | `pinpoint` |
| Turret motor (312 rpm, 1:2 to turret) | `turret` |
| Flywheel motor (6000 rpm, 2:3 to 72 mm wheel) | `flywheel` |
| Intake motor | `intake` |
| Transfer motor | `transfer` |
| Gate servo | `gate` |

That's 8 motors, the maximum allowed (R503).

## Coordinates: the alliance frame
Origin is field center, **+x points away from your own alliance wall**, **+y points to your drivers' left**,
and heading 0 faces +x. The field is 180° rotationally symmetric, so red and blue use the **same
numbers**. There is no alliance switch anywhere. Auto must use the same frame.

* Our HIVE pivot: (-12.75, 0). The up-facing CELL is at y = ±15 (`CELL_Y_IN`), 59.5" high.
* **UP CELL** is `LEFT` or `RIGHT` as seen from our drive station. The operator sets it with dpad left/right.
* Reset/relocalize spot: robot squared in the far-left corner, back on our wall, facing +x.
* PARK: our LOADING ZONE, on our wall 24"-48" to the left of center.

## TeleOp controls (`BIOBUZZ TeleOp`)
**Gamepad 1 (driver):** left stick drive (field-centric), right stick turn, LB slow, RT intake, LT eject,
**RB FIRE (hold)** (shoots at ≥50% likely), **B hold = HOLD FIRE** (pauses auto-fire), **Y hold = auto-park** (sticks cancel),
options = reset heading, share = relocalize in corner.

**Gamepad 2 (operator):** **dpad L/R = which CELL is up**, RT = FIRE, dpad U/D = RPM trim, bumpers = aim trim,
Y = clear trims, A = pre-spin on/off, B = shoot-on-the-move on/off, X = manual turret (right stick),
left-stick-button = manual RPM (fallback if odometry is lost), share = re-zero turret (point it forward first),
**options = AUTO-FIRE on/off**.

Rumbles: ready-to-fire, 60 s left (all NECTAR unlocked), 20 s left (go park), 5 s left.

## Auto-fire (on by default)
The robot shoots **by itself** when all of these are true:
* It estimates at least an **85%** chance of scoring (`AUTO_FIRE_MIN_CONFIDENCE`). The estimate includes:
  * the turret and flywheel error right now
  * shot-to-shot scatter
  * odometry drift since the last relocalize (it grows as you drive)
  * velocity-measurement error, time-of-flight error and release timing, all of which grow with speed
  * acceleration
* It is moving slower than **60 in/s** (`SOTM_MAX_SPEED_IN_S`). The sim shows no shot reaches 85% above that.
  In every direction, shots stay ≥85% up to ~30 in/s. Between 30 and 60, only crossing shots qualify.
* It probably has pieces. There's no sensor, so it counts shots (4 max) and feed pulses that launch nothing.
  Intaking resets the count.
* The driver isn't holding B.

Auto-fire aims at the UP CELL the operator selected. **If that's wrong, it will shoot at the down CELL**, so keep it current.
Driving straight at the HIVE above ~20 in/s makes the shot impossible (the arc is too flat), and the robot won't fire.

## What's automated
* **Turret auto-aim** at the up CELL from anywhere. The turret picks the wrap direction that stays inside its soft limits,
  and a feedforward term counter-rotates it while the chassis spins.
* **Shoot from anywhere:** distance → flywheel RPM from the tuned shot table (interpolated), with
  battery-voltage-compensated flywheel control.
* **Shoot on the move:** aims at a *virtual target* = CELL − shooter velocity × time of flight (iterated).
  The RPM comes from the virtual distance too.
* **Aim zones:** field polygons that shift the aim point, RPM and turret trim. They can apply only when a given CELL is up.
* **Auto-fire gate:** holding FIRE opens the gate only while the flywheel is at speed, the turret is on target, and the shot is possible.
  The turret tolerance shrinks with distance. The gate closes during the RPM dip after each shot, so pieces go out
  as fast as the flywheel recovers. A minimum open time keeps it from pinching a piece.
* **Shot counter** from flywheel RPM dips. **Jam detection:** on intake/transfer over-current, the feeder auto-reverses and then resumes.
* **Pre-spin:** the flywheel idles at the right RPM for wherever you are.
* **Auto-park** to the LOADING ZONE with Neuron Pathing's point follower.
* **Auto → TeleOp hand-off** (`MatchState`): pose, up CELL, and turret zero.
* Bulk-cached hub reads. Battery voltage and motor currents are read at a slower rate.

## Tuning order
1. **Neuron Localizer Test** (from the quickstart). Set Pinpoint offsets, and put `PINPOINT_*` in the config.
2. **Tune: Gate + Intake**. Set the gate positions, motor directions, and jam current.
3. **Tune: Turret**. Hand-turn one revolution to confirm it reads 360°, then tune KS → KP → KD; then KV in TRACK mode.
4. **Tune: Flywheel**. Press Y (auto kV), then tune KS and KP. Set `FLY_SHOT_DIP_RPM` from the dip you see.
5. **Tune: Shot Table + SOTM**. Record 6-10 distances (A), save (X). Then on the SOTM page, tune `TOF_SCALE`.
6. **Tune: Aim Zones**. Fix the specific spots that still miss.

The tuners save to `/sdcard/FIRST/biobuzz/`: `config.properties` (every number in `BuzzConfig`),
`shot_table.csv`, and `zones.txt`. All are plain text files. Pull them with `adb pull /sdcard/FIRST/biobuzz` to back them up
or commit them, and push them to a spare Control Hub with `adb push`.

## Pre-tuned in simulation (`tools/sim`)
`sh tools/sim/run.sh` runs the robot's real control code (`Control`, `FireGate`, `VelocityWindow`,
`Ballistics`, `ShotSolver`) against physics models of the goBILDA motors, the turret, the flywheel,
the 1620 rpm / 48 mm transfer and the POLLEN ball. The full report is in `tools/sim/last_run.txt`.
The values the robot can't know yet (turret inertia/friction, flywheel inertia, ball mass/drag) were
swept over a range, and the defaults in `BuzzConfig` were chosen to work across that whole range.

| What | Sim result → default |
|---|---|
| Turret controller | Speed-command control. POS_GAIN 12, MAX_ACCEL 1500, KVEL 0.002, KS 0.07, KA 0.00008, KV 0.00126. 10° settles in ≤0.3 s; ~1.3° RMS while the chassis spins at 180°/s (5.9° without feedforward). |
| Flywheel | KV 0.000263, KS 0.03, KP 0.0015 (≥0.003 oscillates on the 28-count encoder). The ready check uses a 100 ms averaged rpm. |
| Ready tolerance | About ±15 rpm per inch of landing error, so the rpm tolerance is 3" × the shot table slope (~45 rpm). |
| Feeding | The transfer meters pieces: TRANSFER_FEED_POWER 0.35, FEED_PULSE_S 0.075 → every piece within ~2", 4 pieces in ~0.85 s. Streaming at full power puts pieces 2–4 up to ~14–46" short. |
| Ballistics | Drag is modeled, so TOF_SCALE = 1.0. Closest shot that drops into the CELL is ~50" from the turret. |
| Shoot on the move | Misses ≤ ~4" up to 60 in/s (13–40" without compensation). Driving straight at the HIVE faster than ~20 in/s makes the shot invalid, so strafe or back away while shooting. |
| Flywheel inertia | 4× the estimated inertia fires 4 pieces in ~0.5 s instead of 0.85 s. Worth adding a flywheel weight. |
| Drivetrain (500 rpm, 104 mm, 25 lb) | Wheel free speed 107 in/s; loaded ~96 in/s forward / ~79 strafe; ~5.9 m/s² (traction-limited); 0→60 in/s in 0.26 s. |
| Moving-shot drag | The robot's velocity changes the ball's drag, so the simple lead was several inches off at 40+ in/s. The solver now flies the real trajectory (Newton, ~10 integrations, <0.5 ms) and lands within 0.2". |
| Release latency | Pose and velocity are predicted `SHOT_LATENCY_S` (35 ms) ahead using measured acceleration. At full acceleration, skipping this would cost ~7". |
| Confidence model | Checked against 800-shot full-physics Monte Carlo per case. It reads at or below physics; 0 cases where it would fire at 85% but physics says <82%. |
| SOTM cap | `SOTM_MAX_SPEED_IN_S` = 60 in/s. |

**Field checklist (the parts simulation can't do):**
1. Measure, then set: `LAUNCH_HEIGHT_IN`, `TURRET_OFFSET_*`, `ROBOT_HALF_*`, Pinpoint offsets, and the turret soft limits.
   Weigh a POLLEN ball and set `BALL_MASS_KG`.
2. **Gate + Intake tuner:** set the gate positions. Press gamepad 2 Y repeatedly: each press must feed exactly
   ONE piece. If 0.35 power stalls on the 2 mm squeeze, use 0.50 power / 35 ms.
3. **Flywheel tuner:** press Y (auto-kV), then confirm KP 0.0015 recovers cleanly. Set `FLY_SHOT_DIP_RPM` to
   about 40% of the per-shot dip shown. If you add flywheel weight, the dip shrinks, so lower it.
4. **Turret tuner:** check that one hand-turned revolution reads 360°. Use TRACK mode to confirm KV, and use STEP to
   push MAX_ACCEL up until the turret overshoots, then back off.
5. **Shot table:** record 6–10 distances. The *implied exit efficiency* line tells you the shooter's real
   efficiency. Put the average in `EXIT_SPEED_EFFICIENCY` so distances you didn't record are seeded well.
   (At the 0.5 guess, the 3950 rpm cap reaches ~115", so efficiency sets your max range.)
6. **Calibrate the 85%:** in the Shot Table tuner's SOTM page, turn auto-fire on (gp2 A) and drive around.
   After each shot, press **gp1 dpad up = made / down = missed**. After 30+ shots, the analyzer's
   *CONFIDENCE* section compares predicted and actual hit rates and tells you which spreads to change.
7. Run a match, then `sh tools/pull_logs.sh` and `python3 tools/analyze_log.py`.

## Logging and loop time (target: under 20 ms)
Every OpMode writes one CSV row per loop, plus an events file, to `/sdcard/FIRST/biobuzz/logs/`.
Logging costs about 2 µs per loop on the robot thread. A background thread does all the formatting
and file I/O, and if it ever falls behind it drops rows rather than slowing the loop.
The 60 newest runs are kept.

* **CSV columns:**
  * Timing: `loop_ms`, `work_ms`, and the time per phase (`ms_pinpoint`, `ms_hubs`, `ms_slow`,
    `ms_drive`, `ms_logic`, `ms_write`, `ms_telem`).
  * Pose and velocity.
  * Shot solution: distance, virtual distance, time of flight, zone, and why it's invalid if it is.
  * Turret: position, target, error, velocity, feedforward and power.
  * Flywheel: rpm, target, error and power, plus battery voltage.
  * Firing: fire held, ready, what is blocking the shot, gate state.
  * Feeder mode and motor currents; shot and jam counts; trims and mode toggles.
* **Header:** the full config, the shot table and the zones in use, so every log explains itself.
* **Events file:** each shot (distance, turret error, speed, zone), recovery (dip depth and time), jam,
  CELL change, zone change, relocalize, turret re-zero, park, save, and a loop-time summary at the end.
* **Telemetry:** the `Loop` line shows the current loop time, its p95 and max, and how many loops went over 20 ms.

Loop-time measures already in place:
* Bulk-cached hub reads.
* Motor and servo writes are skipped when the value hasn't changed.
* Telemetry is built only every 100 ms (`TELEMETRY_PERIOD_MS`).
* Battery voltage is read every 250 ms; motor currents are read one motor at a time, every 40 ms, and only while running.
* No memory allocation on the per-loop path, so the garbage collector doesn't cause spikes.

**Reviewing a run:**
```
sh tools/pull_logs.sh                  # USB, or: sh tools/pull_logs.sh wifi
python3 tools/analyze_log.py           # newest log: loop health, turret, flywheel, firing, suggestions
python3 tools/analyze_log.py --all     # one line per log, to compare runs
python3 tools/analyze_log.py LOG.csv --plot
```
The analyzer flags:
* slow loop phases and time lost outside our code
* turret lag versus chassis spin (TURRET_KV), oscillation (KD/KP), and steady error (KS)
* flywheel steady-state bias (FLY_KV), noise (KP), and saturation
* what percentage of fire-held time each blocker caused
* shot recovery time and the maximum sustainable fire rate
* jams and battery sag

## Things to verify on a real field
* `LAUNCH_ANGLE_DEG = 65`. The 25° hood is assumed to be measured from vertical. If it really is 25° above
  horizontal, set 25. The robot then can't shoot from closer than ~8 ft.
* `Field.startingUpCell()` returns RIGHT, from the manual's "the CELL that points at a FLOWER starts down".
* `CELL_Y_IN`, `CELL_AIM_HEIGHT_IN`, `LAUNCH_HEIGHT_IN`, `TURRET_OFFSET_*`, `ROBOT_HALF_*`.
* Turret soft limits `TURRET_MIN_DEG`/`TURRET_MAX_DEG` (±180 by default). Set them to what your wiring allows.
