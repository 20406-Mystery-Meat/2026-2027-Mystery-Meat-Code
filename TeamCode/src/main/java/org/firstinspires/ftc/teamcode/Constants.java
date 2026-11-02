package org.firstinspires.ftc.teamcode;

/**
 * ======================================================================================
 *  BIOBUZZ ROBOT CONSTANTS -- every number you might want to change lives here.
 * ======================================================================================
 *
 *  HOW TO CHANGE SOMETHING
 *    1. Edit the number below and redeploy. Your edit ALWAYS takes effect.
 *    2. Or tune it on the robot with a "Tune: ..." OpMode and press save. That stores an
 *       override on the robot (/sdcard/FIRST/biobuzz/config.properties) -- no redeploy needed.
 *       A saved override is only used while the code value is still what it was when you
 *       saved. Change the number here later and your new code value wins again.
 *    3. The TeleOp init screen lists every value the robot is overriding. The
 *       "Config: Saved Overrides" OpMode lists them too and can clear them all.
 *
 *  SECTIONS (search for the number, e.g. "SECTION 4")
 *    SECTION 1  Hardware names and motor directions
 *    SECTION 2  Robot measurements
 *    SECTION 3  Driver controls and TeleOp
 *    SECTION 4  Drive and path following (Neuron Pathing)
 *    SECTION 5  Turret
 *    SECTION 6  Flywheel
 *    SECTION 7  Intake, intake deploy, transfer and gate
 *    SECTION 8  Shooting and ballistics
 *    SECTION 9  Auto-fire and shot confidence
 *    SECTION 10 Field positions
 *    SECTION 11 Electrical and loop time
 *
 *  The mechanism classes (Turret, Flywheel, Feeder, NeuronDrive, NeuronLocalizer, ...) contain
 *  NO tunable numbers of their own -- they all read from here.
 *
 *  UNITS are in the names: _IN inches, _MM millimeters, _DEG degrees, _S seconds,
 *  _MS milliseconds, _RPM main-wheel rpm, _IN_S inches/second. Powers are -1..1.
 *
 *  COORDINATES are the ALLIANCE FRAME: origin at field center, +x points away from YOUR
 *  alliance wall (straight ahead for your drivers), +y is your drivers' left, heading 0 =
 *  facing +x, counter-clockwise positive. Red and blue use the same numbers (the BIOBUZZ
 *  field is 180-degree rotationally symmetric).
 *
 *  Values marked "sim" were picked by tools/sim (sh tools/sim/run.sh); see BIOBUZZ.md.
 *  Don't rename constants casually: values saved on the robot are matched by name.
 */
public class Constants {

    // ==================================================================================
    // SECTION 1  Hardware names and motor directions
    // Names must match the Driver Station robot configuration exactly.
    // If something spins the wrong way, flip its _REVERSED here.
    // ==================================================================================
    public static final String FRONT_LEFT_MOTOR = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR = "frontRight";
    public static final String BACK_LEFT_MOTOR = "backLeft";
    public static final String BACK_RIGHT_MOTOR = "backRight";
    public static final String PINPOINT = "pinpoint";
    public static final String TURRET_MOTOR = "turret";
    public static final String FLYWHEEL_MOTOR = "flywheel";
    public static final String INTAKE_MOTOR = "intake";
    public static final String TRANSFER_MOTOR = "transfer";
    public static final String GATE_SERVO = "gate";
    public static final String INTAKE_DEPLOY_SERVO = "intakeDeploy";

    public static boolean FRONT_LEFT_REVERSED = true;
    public static boolean BACK_LEFT_REVERSED = true;
    public static boolean FRONT_RIGHT_REVERSED = false;
    public static boolean BACK_RIGHT_REVERSED = false;
    public static boolean TURRET_REVERSED = false;     // turret runs away when enabled -> flip
    public static boolean FLYWHEEL_REVERSED = false;
    public static boolean INTAKE_REVERSED = false;
    public static boolean TRANSFER_REVERSED = false;
    public static boolean INTAKE_DEPLOY_REVERSED = false; // true if dropping the intake moves the servo toward 0
    /** Pinpoint pod directions: pushing the robot forward must INCREASE x, pushing left must increase y. */
    public static boolean PINPOINT_X_REVERSED = false;
    public static boolean PINPOINT_Y_REVERSED = true;

    // ==================================================================================
    // SECTION 2  Robot measurements -- measure these on the real robot
    // ==================================================================================
    /** Pinpoint pod offsets from the tracking center (see the Neuron Pathing tuning guide). */
    public static double PINPOINT_X_OFFSET_MM = 0;
    public static double PINPOINT_Y_OFFSET_MM = 0;
    /** Pinpoint pod resolution: goBILDA 4-bar pod 19.894, swingarm pod 13.263 ticks/mm. */
    public static double PINPOINT_TICKS_PER_MM = 19.89436789;
    public static double TRACK_WIDTH_IN = 14;
    /** Half the robot's length/width, used to place the robot against walls for relocalizing. */
    public static double ROBOT_HALF_LENGTH_IN = 9;
    public static double ROBOT_HALF_WIDTH_IN = 9;
    /** Turret pivot relative to the robot center: +x forward, +y left. */
    public static double TURRET_OFFSET_X_IN = 0;
    public static double TURRET_OFFSET_Y_IN = 0;
    /** Height the piece leaves the shooter, above the tiles. */
    public static double LAUNCH_HEIGHT_IN = 14;
    public static double MAIN_WHEEL_DIAMETER_MM = 72;
    /** Drivetrain: ~500 rpm at the wheel, 104 mm mecanum, 25 lb robot (used by tools/sim). */
    public static double DRIVE_WHEEL_RPM = 500;
    public static double DRIVE_WHEEL_DIAMETER_MM = 104;
    public static double ROBOT_MASS_LB = 25;

    // ==================================================================================
    // SECTION 3  Driver controls and TeleOp
    // ==================================================================================
    /** false = robot-centric (stick up = the way the robot faces). true = field-centric (stick up = away from our wall). */
    public static boolean FIELD_CENTRIC = false;
    public static double DRIVE_SLOW_SCALE = 0.4;       // left bumper slow mode
    public static double TURN_SCALE = 0.8;             // right stick turn sensitivity
    public static double TRIGGER_PRESSED = 0.3;        // intake / eject trigger counts as held past this
    public static double FIRE_TRIGGER_PRESSED = 0.5;   // operator fire trigger counts as held past this
    public static double RPM_TRIM_STEP = 25;           // operator dpad up/down
    public static double MANUAL_RPM_STEP = 100;        // dpad up/down in manual-RPM mode
    public static double AIM_TRIM_STEP_DEG = 0.5;      // operator bumpers
    public static double STICK_DEADZONE = 0.1;         // stick movement that cancels auto-park
    public static double PARK_MAX_SPEED = 1.0;         // auto-park drive power (hold Y)
    public static double TELEOP_LENGTH_S = 120;
    public static double RUMBLE_NECTAR_S = 60;         // 1 blip: all NECTAR unlocked
    public static double RUMBLE_PARK_S = 20;           // 3 blips: go park
    public static double RUMBLE_FINAL_S = 5;           // long rumble
    public static double READY_RUMBLE_MS = 80;         // short buzz when a shot becomes ready
    /** true = flywheel always at shot speed; false = it idles at FLY_IDLE_RPM until you shoot (operator A toggles). */
    public static boolean PRESPIN_DEFAULT = false;
    public static boolean SOTM_DEFAULT = true;
    public static boolean AUTO_FIRE_DEFAULT = true;
    public static double AUTO_HANDOFF_MAX_AGE_S = 180; // TeleOp trusts auto's end pose/turret zero if saved this recently

    // ==================================================================================
    // SECTION 4  Drive and path following (Neuron Pathing -- see its tuning guide)
    // ==================================================================================
    public static double STRAFE_MULT = 1.1;            // strafing is a bit weaker than driving; boost it
    public static double PATH_LOOKAHEAD_IN = 14;       // pure-pursuit lookahead: smaller cuts corners tighter
    public static double PATH_SPLINE_END_RADIUS_IN = 14; // hand off from spline to point-PID this close to the end
    public static double PATH_CENTRIPETAL_GAIN = 0.3;  // raise if it cuts inside curves
    public static double PATH_MIN_CURVE_SPEED = 0.7;   // spline speed on the tightest curves (1.0 on straights)
    public static double PATH_FULL_SPEED_RADIUS_IN = 200; // curves gentler than this radius run full speed
    public static double PATH_MAX_TURN = 0.9;          // max turn power while following a spline
    public static double PATH_CROSS_TRACK_GAIN = 0.01; // strafe power per inch off the spline...
    public static double PATH_CROSS_TRACK_DEADBAND_IN = 2; // ...once more than this far off
    public static double PATH_CROSS_TRACK_MAX = 0.2;   // ...capped at this
    public static double PATH_POINT_TOL_IN = 5;        // "arrived" for goToPoint / park
    public static double PATH_HEADING_TOL_DEG = 5;
    public static double PATH_CLOSE_POINT_TOL_IN = 2;  // final settle after arriving
    public static double PATH_CLOSE_HEADING_TOL_DEG = 2;
    public static double PATH_STRAFE_P = 0.12, PATH_STRAFE_I = 0, PATH_STRAFE_D = 0.013;
    public static double PATH_FORWARD_P = 0.1, PATH_FORWARD_I = 0, PATH_FORWARD_D = 0.015;
    public static double PATH_HEADING_P = 0.05, PATH_HEADING_I = 0, PATH_HEADING_D = 0.003;

    // ==================================================================================
    // SECTION 5  Turret (312 rpm motor, 40T -> 80T, motor encoder)
    // ==================================================================================
    // ---- LOCK: keep the turret still -----------------------------------------------
    /**
     * true = the turret never aims: it holds TURRET_LOCK_DEG and you aim by turning the ROBOT.
     * Auto-fire still works -- it fires when the robot itself is pointed at the CELL.
     * false = the turret auto-aims within the travel range below. ("Tune: Turret" ignores the lock.)
     */
    public static boolean TURRET_LOCKED = true;
    public static double TURRET_LOCK_DEG = 0;

    // ---- TRAVEL RANGE: clip the turret here ----------------------------------------
    // Turret degrees: 0 = shooting straight out the robot's front, + = counter-clockwise (to the
    // robot's left) seen from above. The turret never drives past these. If the HIVE is outside
    // the range it waits at the nearest limit and won't shoot (telemetry: "turret limit").
    //   full circle: -180 / 180      front half: -90 / 90      left side only: 0 / 200
    //   up to 1.5 turns each way also works (e.g. -270 / 270) if the wiring allows it.
    public static double TURRET_MIN_DEG = -10;
    public static double TURRET_MAX_DEG = 10;
    /**
     * Where the turret is when an OpMode starts (the encoder is relative, so this sets the zero).
     * 0 = pointing straight forward. For a repeatable zero, rest it against a hard stop at init and
     * put that stop's angle here (e.g. -185). Must be inside or at the limits above.
     */
    public static double TURRET_START_DEG = 0;
    /** Fine trim if "0" doesn't shoot exactly straight out the front. */
    public static double TURRET_FORWARD_OFFSET_DEG = 0;

    // ---- GEARING -------------------------------------------------------------------
    /** goBILDA 5203 312 rpm (19.2:1) encoder counts per motor output shaft rev. */
    public static double TURRET_TICKS_PER_MOTOR_REV = 537.7;
    public static double TURRET_MOTOR_GEAR_TEETH = 40;   // gear on the motor
    public static double TURRET_RING_GEAR_TEETH = 80;    // gear on the turret -> 2 motor turns per turret turn

    /** Motor revolutions per turret revolution (80 / 40 = 2). */
    public static double turretGearRatio() {
        return TURRET_RING_GEAR_TEETH / TURRET_MOTOR_GEAR_TEETH;
    }

    // ---- CONTROL -------------------------------------------------------------------
    // Controller (subsystems/Control.turretPower). Sim: works for inertia 0.008-0.05 kg m^2, friction 0.15-0.45 N m.
    public static double TURRET_POS_GAIN = 12;         // close-in approach speed per deg of error (1/s); higher = snappier
    public static double TURRET_MAX_ACCEL = 1500;      // planned deceleration (deg/s^2); raise if the turret is light
    public static double TURRET_MAX_VEL = 800;         // deg/s
    public static double TURRET_KA = 0.00008;          // power per deg/s^2 of planned deceleration
    public static double TURRET_KVEL = 0.002;          // power per deg/s of speed error
    public static double TURRET_KS = 0.07;             // power to overcome friction
    public static double TURRET_KV = 0.00126;          // 1 / turret free speed (312 rpm / 2 = 936 deg/s, ~85% efficient); redo if gearing changes
    public static double TURRET_DEADBAND_DEG = 0.4;
    public static double TURRET_TRACK_KS_MIN_DPS = 5;  // also add kS while tracking faster than this
    public static double TURRET_MAX_POWER = 1.0;
    /** Aim tolerance is atan(cell half-width / distance), clamped to this range. */
    public static double TURRET_TOL_MIN_DEG = 1.0;
    public static double TURRET_TOL_MAX_DEG = 5.0;
    public static double TURRET_MANUAL_DEG_PER_S = 240; // manual turret speed at full stick

    // ==================================================================================
    // SECTION 6  Flywheel (6000 rpm motor, 2:3 to a 72 mm wheel). RPM = MAIN-WHEEL rpm.
    // ==================================================================================
    /** goBILDA 5203 6000 rpm (1:1) encoder counts per rev. */
    public static double FLYWHEEL_TICKS_PER_MOTOR_REV = 28;
    /** Main-wheel revs per motor rev. 2:3 for torque -> 0.6667 (4000 rpm max at the wheel). */
    public static double FLYWHEEL_GEAR_RATIO = 2.0 / 3.0;
    public static double FLY_KV = 0.000263;            // power at 12 V per rpm (sim 1/3800; Flywheel tuner auto-kV measures it)
    public static double FLY_KS = 0.03;
    public static double FLY_KP = 0.0015;              // sim: 0.0015-0.002 best; >= 0.003 oscillates
    public static double FLY_BOOST_RPM = 0;            // full power while this far below target (0 = off)
    /**
     * "At speed" tolerance = RANGE_TOL_IN x (rpm per inch from the shot table),
     * clamped to [FLY_TOL_MIN_RPM, FLY_READY_TOL_RPM]: ~+-3" of landing error at any distance.
     */
    public static double FLY_READY_TOL_RPM = 60;
    public static double FLY_TOL_MIN_RPM = 30;
    public static double RANGE_TOL_IN = 3;
    public static double FLY_FILTER_MS = 100;          // averaging window for the ready check (coarse 28-count encoder)
    public static int FLY_FILTER_BUFFER = 48;          // samples kept for that window (enough down to ~2 ms loops)
    public static double FLY_SHOT_DIP_RPM = 100;       // a drop this big while feeding = one shot (sim dips ~120-500)
    public static double FLY_MAX_WHEEL_RPM = 3950;
    public static double FLY_MANUAL_DEFAULT_RPM = 1700;
    /**
     * Flywheel speed when you're NOT shooting. Negative = spins BACKWARD (keeps pieces off the wheel).
     * 0 = stop. It never fires while idling; holding shoot spins it up to the shot speed from here.
     */
    public static double FLY_IDLE_RPM = -1000;

    // ==================================================================================
    // SECTION 7  Intake, intake deploy, transfer and gate (1620 rpm on 48 mm rollers, ~2 mm squeeze; POLLEN only)
    // ==================================================================================
    // ---- INTAKE DEPLOY (goBILDA Speed servo, 40T on the servo -> 60T on the intake arm) ----
    public static double DEPLOY_STOWED_POS = 0.10;     // servo position (0..1) with the intake folded up -- set in "Tune: Gate + Intake"
    public static double DEPLOY_DOWN_DEG = 65;        // how far the intake ARM swings from folded up to down (measure it)
    public static double DEPLOY_SERVO_GEAR_TEETH = 40; // gear on the servo
    public static double DEPLOY_ARM_GEAR_TEETH = 60;   // gear on the intake arm -> arm moves 2/3 as far as the servo
    public static double DEPLOY_SERVO_RANGE_DEG = 300; // goBILDA dual-mode servo travel over DEPLOY_PWM_MIN..MAX
    public static int DEPLOY_PWM_MIN_US = 500;         // goBILDA servos use 500-2500 us for their full range
    public static int DEPLOY_PWM_MAX_US = 2500;
    public static double DEPLOY_SERVO_S_PER_60DEG = 0.11; // Speed servo ~0.09 s/60 deg at 6 V; hub servo ports are 5 V
    public static double DEPLOY_SETTLE_S = 0.05;       // extra time before the arm counts as down
    public static boolean DEPLOY_WITH_INTAKE = true;   // drop automatically while intaking or ejecting
    public static double DEPLOY_STOW_DELAY_S = 0.4;    // fold back up this long after you stop intaking
    public static boolean INTAKE_WAIT_FOR_DEPLOY = true; // don't spin the intake rollers until the arm is down

    // ---- INTAKE / TRANSFER / GATE ----
    public static double INTAKE_POWER = 1.0;
    public static boolean INTAKE_WHILE_FIRING = false ; // intake runs the whole time the shoot button is held
    public static double TRANSFER_STAGE_POWER = 0.35;  // while intaking: stage pieces against the closed gate
    /**
     * Transfer power while feeding a shot. Sim: at full power queued pieces arrive ~35 ms apart and
     * can't be metered one at a time; 0.35 with a 75 ms pulse keeps every piece within ~2".
     * If 0.35 can't push through the 2 mm squeeze, use 0.50 with FEED_PULSE_S = 0.035.
     */
    public static double TRANSFER_FEED_POWER = 0.35;
    /** One feed pulse: transfer run time to push ONE piece in. Too short = dry pulses, too long = two pieces. */
    public static double FEED_PULSE_S = 0.075;
    public static double EJECT_POWER = -0.8;
    public static double GATE_OPEN_POS = 0.60;
    public static double GATE_CLOSED_POS = 0.20;
    public static double GATE_SERVO_S = 0.08;          // time for the gate to physically open
    public static double FIRE_READY_DEBOUNCE_S = 0.05; // everything must be ready this long before feeding
    public static double JAM_CURRENT_AMPS = 6.5;       // jam = current above this for JAM_TIME_S...
    public static double JAM_TIME_S = 0.30;
    public static double UNJAM_TIME_S = 0.25;          // ...then reverse this long
    public static boolean JAM_DETECT_ENABLED = true;

    // ==================================================================================
    // SECTION 8  Shooting and ballistics
    // ==================================================================================
    /** Launch ELEVATION above horizontal. Hood "25 deg" from vertical = 65. (25 above horizontal can't score closer than ~8 ft.) */
    public static double LAUNCH_ANGLE_DEG = 65;
    public static double EXIT_SPEED_EFFICIENCY = 0.5;  // exit speed / wheel surface speed; seeds the shot table (Shot Table tuner shows the real one)
    public static double TOF_SCALE = 1.0;              // shoot-on-the-move lead multiplier; tune in Shot Table tuner SOTM page
    /** POLLEN: 2.8" perforated ball. Mass is an estimate -- weigh one. */
    public static double BALL_DIAMETER_IN = 2.8;
    public static double BALL_MASS_KG = 0.030;
    public static double BALL_CD = 0.55;               // drag coefficient (smooth ball ~0.47, perforated higher)
    public static double MIN_DESCENT_DEG = 20;         // pieces must be falling at least this steeply at the CELL
    public static int SOTM_ITERATIONS = 3;
    public static double MAX_SHOT_TURN_RATE_DEG_S = 360; // no shots while spinning faster than this

    // ---- MANUAL SHOOT mode (driver X): turret holds still, fixed flywheel speed, aim by turning the robot ----
    public static double MANUAL_SHOT_RPM = 3200;       // main-wheel rpm (~6 ft by the physics seed -- set it from your shot table)
    public static double MANUAL_SHOT_TOL_RPM = 60;     // FIRE feeds when the flywheel is within this of MANUAL_SHOT_RPM
    public static boolean MANUAL_SHOT_HOLD_CURRENT = true; // true: turret freezes where it is; false: goes to MANUAL_SHOT_TURRET_DEG
    public static double MANUAL_SHOT_TURRET_DEG = 0;   // 0 = straight out the front

    // ==================================================================================
    // SECTION 9  Auto-fire and shot confidence
    // ==================================================================================
    public static double SOTM_MAX_SPEED_IN_S = 60;       // never AUTO-fire faster than this (sim: nothing reaches 85% above)
    public static double AUTO_FIRE_MIN_CONFIDENCE = 0.85; // auto-fire when at least this likely to score
    public static double AUTO_FIRE_KEEP_CONFIDENCE = 0.80; // keep a started burst going above this
    public static double MANUAL_MIN_CONFIDENCE = 0.50;   // holding FIRE shoots at this lower bar
    public static int MAX_PIECES = 4;                    // G407; after this many shots assume empty until intaking
    public static int EMPTY_AFTER_DRY_PULSES = 3;        // feed pulses in a row that launch nothing -> assume empty
    public static double SHOT_LATENCY_S = 0.035;         // decision -> piece leaves; pose/velocity predicted this far ahead
    public static double ACCEL_FILTER_S = 0.04;          // smoothing on measured acceleration
    public static double AUTO_DRIFT_IN = 300;            // odometry drift assumed when TeleOp starts from auto's pose
    /** Up-facing CELL opening seen from above, minus the POLLEN radius (20" across x, ~12" along y). */
    public static double HIT_HALF_X_IN = 8.6;
    public static double HIT_HALF_Y_IN = 4.6;
    // Error model, 1 sigma. First guesses -- raise them if auto-fire is too eager, lower if too shy.
    public static double DISP_RANGE_IN = 1.5;          // shot-to-shot scatter along the shot
    public static double DISP_RANGE_FRAC = 0.015;      //   ... plus this fraction of distance
    public static double DISP_LAT_IN = 1.0;            // shot-to-shot scatter sideways
    public static double DISP_LAT_FRAC = 0.010;
    public static double POSE_SIGMA_IN = 0.75;         // odometry position error right after relocalizing
    public static double POSE_DRIFT_FRAC = 0.004;      //   ... growing with distance driven
    public static double HEADING_SIGMA_DEG = 0.3;
    public static double VEL_SIGMA_IN_S = 1.5;         // Pinpoint velocity noise
    public static double VEL_SIGMA_FRAC = 0.03;        //   ... plus this fraction of speed
    public static double TOF_SIGMA_FRAC = 0.04;        // time-of-flight model error
    public static double RELEASE_JITTER_S = 0.010;     // when exactly the piece leaves
    public static double ACCEL_SIGMA_FRAC = 0.4;       // how wrong the acceleration prediction can be
    public static double TURRET_SIGMA_DEG = 0.3;
    public static double TURRET_RATE_SIGMA = 0.005;    // extra deg of turret error per deg/s it's turning
    public static double FLY_SIGMA_RPM = 10;
    public static double TOWARD_SIGMA_PER_IN_S = 0.06; // extra range spread per in/s of closing speed (sim-fit)

    // ==================================================================================
    // SECTION 10 Field positions (alliance frame, see top)
    // ==================================================================================
    public static double HIVE_X_IN = -12.75;           // our HIVE pivot (HIVE centers are 25.5" apart)
    public static double CELL_Y_IN = 15.0;             // pivot -> up-facing CELL opening center, sideways
    public static double CELL_AIM_HEIGHT_IN = 59.5;    // CELL opening spans 53.5" - 65.6"
    public static double PARK_X_IN = -61;              // LOADING ZONE: our wall, 24"-48" left of center
    public static double PARK_Y_IN = 36;
    public static double PARK_HEADING_DEG = 0;

    // ==================================================================================
    // SECTION 11 Electrical and loop time (target 10-15 ms loops)
    // ==================================================================================
    public static double NOMINAL_VOLTS = 12.0;         // turret/flywheel gains are tuned as if the battery were this
    public static double MIN_COMP_VOLTS = 9.0;         // never compensate as if the battery were below this
    public static double VOLTAGE_READ_PERIOD_MS = 250; // battery voltage read (~1 ms) at most this often
    public static double MOTOR_WRITE_EPS = 0.005;      // skip turret/flywheel/intake writes smaller than this
    public static double TELEMETRY_PERIOD_MS = 100;    // build/send telemetry only this often
    public static double CURRENT_READ_PERIOD_MS = 40;  // one motor-current read (~1 ms) at most this often
    public static boolean PINPOINT_FAST_I2C = true;    // 400 kHz instead of 100 kHz (~3 ms/loop saved)
    public static double DRIVE_WRITE_EPS = 0.02;       // skip drive writes unless a wheel changes this much
    public static double FLY_WRITE_PERIOD_MS = 0;      // 0 = every loop (sim: throttling costs accuracy)
    public static double FLY_WRITE_FORCE_DELTA = 0.05; // ...always send a change this big at once
}
