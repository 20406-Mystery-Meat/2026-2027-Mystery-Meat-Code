package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Every tunable number for the BIOBUZZ turret robot, in one place.
 *
 * All public static non-final fields here are saved to / loaded from
 * /sdcard/FIRST/biobuzz/config.properties by {@link ConfigIO}, so the tuning
 * OpModes can change them at the field and press "save" -- no rebuild needed.
 * The values written below are only the defaults used when no file exists yet.
 *
 * COORDINATES: everything is in the ALLIANCE FRAME (see field/Field.java):
 * origin at field center, +x points away from YOUR alliance wall (straight ahead
 * for your drivers), +y points to your drivers' left, heading 0 = facing +x,
 * counter-clockwise positive. Red and blue use identical numbers because the
 * BIOBUZZ field is 180-degree rotationally symmetric.
 */
public class BuzzConfig {
    // ---------------------------------------------------------------- hardware names
    public static final String TURRET_MOTOR = "turret";
    public static final String FLYWHEEL_MOTOR = "flywheel";
    public static final String INTAKE_MOTOR = "intake";
    public static final String TRANSFER_MOTOR = "transfer";
    public static final String GATE_SERVO = "gate";
    // Drive motors use NeuronDrive's defaults: frontLeft, frontRight, backLeft, backRight.
    // Pinpoint uses NeuronLocalizer's default: pinpoint.

    // ---------------------------------------------------------------- robot / drive
    /** Pinpoint pod offsets -- see the Neuron Pathing tuning guide. */
    public static double PINPOINT_X_OFFSET_MM = 0;
    public static double PINPOINT_Y_OFFSET_MM = 0;
    public static double TRACK_WIDTH_IN = 14;
    /** Half the robot's length/width, used to place the robot against walls for relocalizing. */
    public static double ROBOT_HALF_LENGTH_IN = 9;
    public static double ROBOT_HALF_WIDTH_IN = 9;
    public static double DRIVE_SLOW_SCALE = 0.4;
    public static double TURN_SCALE = 0.8;

    // ---------------------------------------------------------------- turret
    /** goBILDA 5203 312 rpm (19.2:1) encoder counts per motor output shaft rev. */
    public static double TURRET_TICKS_PER_MOTOR_REV = 537.7;
    /** Motor revolutions per one turret revolution (1:2 reduction -> 2). */
    public static double TURRET_GEAR_RATIO = 2.0;
    public static boolean TURRET_REVERSED = false;
    /** Soft limits in turret degrees (0 = shooting straight out the robot's front). Protects the wires. */
    public static double TURRET_MIN_DEG = -180;
    public static double TURRET_MAX_DEG = 180;
    /** Turret angle (deg) that fires straight out the robot's front, if the encoder zero isn't forward. */
    public static double TURRET_FORWARD_OFFSET_DEG = 0;
    /** Turret pivot location relative to the robot center: +x forward, +y left, inches. */
    public static double TURRET_OFFSET_X_IN = 0;
    public static double TURRET_OFFSET_Y_IN = 0;
    /**
     * Turret controller (see subsystems/Control.turretPower). Defaults are from tools/sim,
     * chosen to work for any turret inertia 0.008-0.05 kg m^2 and friction 0.15-0.45 N m.
     */
    /** Close-in approach speed per degree of error (1/s). Higher = snappier final settle. */
    public static double TURRET_POS_GAIN = 12;
    /** Deceleration the approach is planned for (deg/s^2). Raise if the turret is light. */
    public static double TURRET_MAX_ACCEL = 1500;
    public static double TURRET_MAX_VEL = 800;
    /** Power per deg/s^2 of planned deceleration ~ inertia / stall torque. Helps it brake on time. */
    public static double TURRET_KA = 0.00008;
    /** Power per deg/s of speed error. */
    public static double TURRET_KVEL = 0.002;
    public static double TURRET_KS = 0.07;
    /** 1 / (turret free speed in deg/s, after gear losses). 312rpm / 2 = 936 deg/s, ~85% efficient. */
    public static double TURRET_KV = 0.00126;
    public static double TURRET_DEADBAND_DEG = 0.4;
    public static double TURRET_MAX_POWER = 1.0;
    /** Aim tolerance is atan(cell half-width / distance), clamped to this range. */
    public static double TURRET_TOL_MIN_DEG = 1.0;
    public static double TURRET_TOL_MAX_DEG = 5.0;
    /** Manual turret speed (deg/s at full stick). */
    public static double TURRET_MANUAL_DEG_PER_S = 240;

    // ---------------------------------------------------------------- flywheel
    /** goBILDA 5203 6000 rpm (1:1) encoder counts per rev. */
    public static double FLYWHEEL_TICKS_PER_MOTOR_REV = 28;
    /** Main-wheel revs per motor rev. 2:3 for torque -> 0.6667 (4000 rpm max at the wheel). */
    public static double FLYWHEEL_GEAR_RATIO = 2.0 / 3.0;
    public static boolean FLYWHEEL_REVERSED = false;
    /** Feedforward: power at 12V per main-wheel rpm (sim: 1/3800; Flywheel tuner's auto-kV measures it). */
    public static double FLY_KV = 0.000263;
    public static double FLY_KS = 0.03;
    /** Sim: 0.0015-0.002 is the sweet spot; >= 0.003 oscillates on the coarse 28-count encoder. */
    public static double FLY_KP = 0.0015;
    /** Full power while more than this many rpm below target (0 = off). Speeds up shot recovery. */
    public static double FLY_BOOST_RPM = 0;
    /**
     * "At speed" tolerance. The solver uses RANGE_TOL_IN x (rpm per inch from the shot table),
     * clamped to [FLY_TOL_MIN_RPM, FLY_READY_TOL_RPM]: ~+-3" of landing error at any distance.
     */
    public static double FLY_READY_TOL_RPM = 60;
    public static double FLY_TOL_MIN_RPM = 30;
    public static double RANGE_TOL_IN = 3;
    /** Averaging window for the ready check (the 28-count encoder is coarse). */
    public static double FLY_FILTER_MS = 100;
    /** A drop this big below target while feeding counts as one shot (sim: dips are ~120-500 rpm). */
    public static double FLY_SHOT_DIP_RPM = 100;
    public static double FLY_MAX_WHEEL_RPM = 3950;
    public static double FLY_MANUAL_DEFAULT_RPM = 3000;

    // ---------------------------------------------------------------- intake / transfer / gate
    public static boolean INTAKE_REVERSED = false;
    public static boolean TRANSFER_REVERSED = false;
    public static double INTAKE_POWER = 1.0;
    /** Transfer: goBILDA 1620 rpm on 48 mm rollers, ~2 mm squeeze on POLLEN (only POLLEN is launched). */
    /** Transfer power while intaking with the gate closed -- stages pieces against the gate. */
    public static double TRANSFER_STAGE_POWER = 0.35;
    /**
     * Transfer power while feeding a shot. Sim: at full power queued pieces arrive ~35 ms apart and
     * can't be metered one at a time; 0.35 with a 75 ms pulse keeps every piece within ~2".
     * If 0.35 can't push through the 2 mm squeeze, use 0.50 with FEED_PULSE_S = 0.035.
     */
    public static double TRANSFER_FEED_POWER = 0.35;
    public static double EJECT_POWER = -0.8;
    public static double GATE_OPEN_POS = 0.60;
    public static double GATE_CLOSED_POS = 0.20;
    /** Time for the gate servo to physically open (the transfer waits this long before feeding). */
    public static double GATE_SERVO_S = 0.08;
    /**
     * One feed pulse: how long the transfer runs to push ONE piece into the flywheel.
     * ~ piece diameter / piece speed. 1620 rpm on 48 mm rollers at TRANSFER_FEED_POWER: see tools/sim.
     * Too short: pulses that don't launch anything (slow). Too long: two pieces per pulse (inaccurate).
     */
    public static double FEED_PULSE_S = 0.075;
    /** Fire only after every ready check has held for this long. */
    public static double FIRE_READY_DEBOUNCE_S = 0.05;
    /** Jam = current above this for JAM_TIME_S. Then reverse for UNJAM_TIME_S. */
    public static double JAM_CURRENT_AMPS = 6.5;
    public static double JAM_TIME_S = 0.30;
    public static double UNJAM_TIME_S = 0.25;
    public static boolean JAM_DETECT_ENABLED = true;

    // ---------------------------------------------------------------- ballistics
    /**
     * Launch ELEVATION above horizontal. A hood "25 deg" from vertical = 65 deg elevation.
     * (25 deg above horizontal can't reach the CELL from closer than ~8 ft.)
     */
    public static double LAUNCH_ANGLE_DEG = 65;
    /** Height the piece leaves the shooter, inches above the tiles. */
    public static double LAUNCH_HEIGHT_IN = 14;
    public static double MAIN_WHEEL_DIAMETER_MM = 72;
    /** Exit speed / main-wheel surface speed. Only used to seed the shot table. */
    public static double EXIT_SPEED_EFFICIENCY = 0.5;
    /** Final multiplier on the modeled time of flight (drag is already modeled). Tune in Shot Table Tuner. */
    public static double TOF_SCALE = 1.0;
    /** POLLEN: 2.8" perforated polyethylene ball. Mass is an estimate -- weigh one and update. */
    public static double BALL_DIAMETER_IN = 2.8;
    public static double BALL_MASS_KG = 0.030;
    /** Drag coefficient. Smooth ball ~0.47, perforated wiffle-style balls run higher. */
    public static double BALL_CD = 0.55;
    /** Pieces must be falling at least this steeply when they reach the CELL to drop in. */
    public static double MIN_DESCENT_DEG = 20;
    public static int SOTM_ITERATIONS = 3;
    /** Don't trust shots while spinning faster than this (deg/s). */
    public static double MAX_SHOT_TURN_RATE_DEG_S = 360;

    // ---------------------------------------------------------------- auto-fire & shot confidence
    /** Drivetrain: ~500 rpm at the wheel, 104 mm mecanum, 25 lb robot (used by tools/sim). */
    public static double DRIVE_WHEEL_RPM = 500;
    public static double DRIVE_WHEEL_DIAMETER_MM = 104;
    public static double ROBOT_MASS_LB = 25;
    /** Never AUTO-fire faster than this (in/s). From tools/sim: above it, typical shots fall below 85%. */
    public static double SOTM_MAX_SPEED_IN_S = 60;
    public static boolean AUTO_FIRE_DEFAULT = true;
    /** Auto-fire only when the estimated chance of scoring is at least this. */
    public static double AUTO_FIRE_MIN_CONFIDENCE = 0.85;
    /** Once a burst starts, keep going while the aim confidence stays above this (hysteresis). */
    public static double AUTO_FIRE_KEEP_CONFIDENCE = 0.80;
    /** Holding FIRE shoots at this lower confidence (the driver has decided). */
    public static double MANUAL_MIN_CONFIDENCE = 0.50;
    /** Max pieces the robot can hold (G407). After this many shots it assumes it's empty until it intakes. */
    public static int MAX_PIECES = 4;
    /** This many feed pulses in a row with no launch -> assume empty (first pulse may push a piece only partway). */
    public static int EMPTY_AFTER_DRY_PULSES = 3;
    /** Decision -> piece leaves the wheel (one loop + filters). Pose/velocity are predicted this far ahead. */
    public static double SHOT_LATENCY_S = 0.035;

    /** Up-facing CELL opening seen from above, minus the POLLEN radius (20" across x, 14" tilted 30 deg -> ~12" along y). */
    public static double HIT_HALF_X_IN = 8.6;
    public static double HIT_HALF_Y_IN = 4.6;
    /** Error model, 1 sigma. First guesses -- calibrate with made/missed marks (see BIOBUZZ.md). */
    public static double DISP_RANGE_IN = 1.5;      // shot-to-shot scatter along the shot
    public static double DISP_RANGE_FRAC = 0.015;  //   ... plus this fraction of distance
    public static double DISP_LAT_IN = 1.0;        // shot-to-shot scatter sideways
    public static double DISP_LAT_FRAC = 0.010;
    public static double POSE_SIGMA_IN = 0.75;     // odometry position error right after relocalizing
    public static double POSE_DRIFT_FRAC = 0.004;  //   ... growing with distance driven
    public static double HEADING_SIGMA_DEG = 0.3;
    public static double VEL_SIGMA_IN_S = 1.5;     // Pinpoint velocity noise
    public static double VEL_SIGMA_FRAC = 0.03;    //   ... plus this fraction of speed
    public static double TOF_SIGMA_FRAC = 0.04;    // time-of-flight model error
    public static double RELEASE_JITTER_S = 0.010; // when exactly the piece leaves
    public static double ACCEL_SIGMA_FRAC = 0.4;   // how wrong the acceleration prediction can be
    public static double TURRET_SIGMA_DEG = 0.3;
    public static double TURRET_RATE_SIGMA = 0.005; // extra deg of turret error per deg/s it's turning
    public static double FLY_SIGMA_RPM = 10;
    public static double TOWARD_SIGMA_PER_IN_S = 0.06; // extra range spread per in/s of closing speed (sim-fit)

    // ---------------------------------------------------------------- field (alliance frame)
    /** Our HIVE's pivot is 12.75" from field center toward our wall (HIVE centers are 25.5" apart). */
    public static double HIVE_X_IN = -12.75;
    /** Horizontal distance from the pivot to the center of the up-facing CELL opening. */
    public static double CELL_Y_IN = 15.0;
    /** Up-facing CELL opening spans 53.5" - 65.6" above the tiles. */
    public static double CELL_AIM_HEIGHT_IN = 59.5;
    /** LOADING ZONE is on our wall, 24"-48" to the drivers' left of center. */
    public static double PARK_X_IN = -61;
    public static double PARK_Y_IN = 36;
    public static double PARK_HEADING_DEG = 0;

    // ---------------------------------------------------------------- logging / loop time
    /** Per-loop CSV + events in /sdcard/FIRST/biobuzz/logs. Costs ~0.05 ms per loop. */
    public static boolean LOG_ENABLED = true;
    /** Telemetry is only built/sent this often; building it every loop costs several ms. */
    public static double TELEMETRY_PERIOD_MS = 100;
    /** One motor-current read (~1 ms) at most this often, alternating intake/transfer. */
    public static double CURRENT_READ_PERIOD_MS = 40;

    // ---------------------------------------------------------------- teleop
    public static double TELEOP_LENGTH_S = 120;
    public static boolean PRESPIN_DEFAULT = true;
    public static boolean SOTM_DEFAULT = true;
}
