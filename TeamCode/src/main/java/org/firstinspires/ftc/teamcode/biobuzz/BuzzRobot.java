package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.field.ZoneMap;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Ballistics;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotSolver;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotTable;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.IntakeDeploy;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

import java.util.List;

/**
 * All the robot's hardware, owned in one place. Every loop:
 * <pre>
 *   robot.read();          // bulk read + odometry + sensors
 *   ...decide...           // OpMode / ShotController sets targets
 *   robot.write();         // subsystems write motor powers
 *   if (robot.telemetryDue()) { ...telemetry... }
 * </pre>
 * Hubs use MANUAL bulk caching: one bulk read per hub per loop for all encoders/velocities.
 * Battery voltage and motor currents are separate (slow) reads, so they're rate-limited.
 */
public class BuzzRobot {
    public final NeuronLocalizer localizer;
    public final NeuronDrive drive;
    public final Turret turret;
    public final Flywheel flywheel;
    public final Feeder feeder;
    public final IntakeDeploy deploy;
    public final ShotTable shotTable;
    public final ZoneMap zones;
    public final ShotSolver solver;

    private final List<LynxModule> hubs;
    private final VoltageSensor battery;

    public Pose2D pose = new Pose2D(0, 0, 0);
    public double vx, vy, omega;
    /** Field acceleration (in/s^2), low-pass filtered from velocity. */
    public double ax, ay;
    /** Pose and velocity predicted SHOT_LATENCY_S ahead -- where the robot will be when the piece leaves. */
    public final Pose2D predPose = new Pose2D(0, 0, 0);
    public double predVx, predVy;
    /** Distance driven since the last relocalize (odometry drift grows with it). */
    public double traveledIn;
    private double lastVx, lastVy;
    public double volts = NOMINAL_VOLTS; // until the first battery read
    public double nowS, dtS;

    private long lastNs = System.nanoTime();
    private final long startNs = System.nanoTime();
    private double lastVoltsS = -1, lastCurrentS = -1, lastTelemS = -1;
    private final double[] lastDrive = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    public BuzzRobot(HardwareMap hw, Telemetry telemetry) {
        ConfigIO.load();
        shotTable = ShotTable.load();
        zones = ZoneMap.load();
        solver = new ShotSolver(shotTable, zones);
        Ballistics.warmUp(); // precompute the drag trajectory table now, not in the first loop

        hubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        localizer = new NeuronLocalizer(hw, telemetry); // name, offsets, pods, directions: Constants
        speedUpPinpoint(localizer.pinpoint);
        drive = new NeuronDrive(hw, localizer); // motor names/directions, track width, path gains: Constants
        turret = new Turret(hw);
        flywheel = new Flywheel(hw);
        feeder = new Feeder(hw);
        deploy = new IntakeDeploy(hw);
        battery = hw.voltageSensor.iterator().next();
    }

    /**
     * Set the starting pose and turret zero. Uses auto's hand-off if it's fresh and
     * useAuto is true; otherwise starts at {@code start} and zeroes the turret where it is
     * (so the turret MUST be pointing straight forward at init in that case).
     * @return true if auto's hand-off was used
     */
    public boolean initPose(Pose2D start, boolean useAuto) {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        boolean fromAuto = useAuto && MatchState.fresh();
        Pose2D p = fromAuto ? MatchState.pose : start;
        localizer.setPosition(p);
        localizer.calibrate(false);
        localizer.setPosition(p);
        if (fromAuto && MatchState.turretZeroValid) {
            turret.zeroTicks = MatchState.turretZeroTicks;
        } else {
            turret.zeroHere(TURRET_START_DEG);
        }
        pose = p;
        traveledIn = fromAuto ? AUTO_DRIFT_IN : 0; // after auto, assume some drift has built up
        return fromAuto;
    }

    /**
     * The Pinpoint read is the biggest single cost in the loop. By default the SDK driver reads
     * 40 bytes (including loop-time and raw-encoder registers we never use) over an I2C bus left at
     * the hub default of 100 kHz -- roughly 5 ms. Read only what we use (7 registers = 28 bytes;
     * needs Pinpoint firmware with flexible bulk reads, otherwise the driver ignores it) and run the
     * bus at 400 kHz (the Pinpoint supports it). NOTE: the speed applies to the whole I2C port, so
     * anything else on that port must also handle 400 kHz -- best to give the Pinpoint its own port.
     */
    private static void speedUpPinpoint(GoBildaPinpointDriver pp) {
        try {
            pp.setBulkReadScope(GoBildaPinpointDriver.Register.DEVICE_STATUS,
                    GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION,
                    GoBildaPinpointDriver.Register.H_ORIENTATION, GoBildaPinpointDriver.Register.X_VELOCITY,
                    GoBildaPinpointDriver.Register.Y_VELOCITY, GoBildaPinpointDriver.Register.H_VELOCITY);
        } catch (RuntimeException ignored) {
            // older firmware: the driver keeps its full read
        }
        I2cDeviceSynchSimple client = pp.getDeviceClient();
        if (PINPOINT_FAST_I2C && client instanceof LynxI2cDeviceSynch) {
            ((LynxI2cDeviceSynch) client).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }
    }

    /** True at most every TELEMETRY_PERIOD_MS: build + send telemetry only then (it's slow). */
    public boolean telemetryDue() {
        if (nowS - lastTelemS < TELEMETRY_PERIOD_MS / 1000.0) return false;
        lastTelemS = nowS;
        return true;
    }

    // ------------------------------------------------------------------ loop

    public void read() {
        long now = System.nanoTime();
        dtS = Math.max((now - lastNs) / 1e9, 1e-3);
        lastNs = now;
        nowS = (now - startNs) / 1e9;

        for (int i = 0; i < hubs.size(); i++) hubs.get(i).clearBulkCache();

        drive.update(); // reads the Pinpoint; also runs the path follower if one is active
        pose = drive.pose;
        vx = localizer.velX();
        vy = localizer.velY();
        omega = Math.toRadians(localizer.getAngularVelocityDegPerSec());
        // acceleration from velocity change, low-passed (~40 ms) so encoder noise doesn't dominate
        double k = Math.min(1, dtS / ACCEL_FILTER_S);
        ax += k * ((vx - lastVx) / dtS - ax);
        ay += k * ((vy - lastVy) / dtS - ay);
        lastVx = vx;
        lastVy = vy;
        double L = SHOT_LATENCY_S;
        predPose.x = pose.x + vx * L + 0.5 * ax * L * L;
        predPose.y = pose.y + vy * L + 0.5 * ay * L * L;
        predPose.h = pose.h + omega * L;
        predVx = vx + ax * L;
        predVy = vy + ay * L;
        traveledIn += Math.hypot(vx, vy) * dtS;

        turret.read();   // first encoder access triggers the hub bulk read
        flywheel.read(feeder.gateOpen);

        if (nowS - lastVoltsS > VOLTAGE_READ_PERIOD_MS / 1000.0) {
            volts = battery.getVoltage();
            lastVoltsS = nowS;
        }
        // Currents only matter while the intake/transfer are running. One motor per read.
        if (feeder.mode == Feeder.Mode.IDLE) {
            feeder.clearCurrents();
        } else if (nowS - lastCurrentS > CURRENT_READ_PERIOD_MS / 1000.0) {
            feeder.readOneCurrent();
            lastCurrentS = nowS;
        }
    }

    public void write() {
        turret.write(volts);
        flywheel.write(volts);
        // Intake arm drops automatically while intaking/ejecting; rollers wait until it's down.
        deploy.update(DEPLOY_WITH_INTAKE && (feeder.mode == Feeder.Mode.INTAKE || feeder.mode == Feeder.Mode.EJECT), nowS);
        feeder.write(nowS, deploy.isDown());
    }

    public void stopAll() {
        turret.stop();
        flywheel.stop();
        feeder.stop();
        drive.driveState = NeuronDrive.DriveState.DRIVE;
        drive.setDrivePower(0, 0, 0, 0);
        invalidateDriveCache();
    }

    /**
     * Mecanum drive from the sticks: left stick translates, right stick x turns.
     * Robot-centric by default (stick up = the way the robot faces). With FIELD_CENTRIC on,
     * stick up = away from our wall no matter which way the robot faces.
     */
    public void driveWithSticks(Gamepad g, boolean slow) {
        if (drive.driveState != NeuronDrive.DriveState.DRIVE) {
            // A path/park was running and wrote the motors itself: our cache is stale.
            drive.driveState = NeuronDrive.DriveState.DRIVE;
            invalidateDriveCache();
        }
        double fx = -g.left_stick_y;  // stick up
        double fy = -g.left_stick_x;  // stick left
        double turn = -g.right_stick_x * TURN_SCALE;
        double scale = slow ? DRIVE_SLOW_SCALE : 1.0;
        fx *= scale;
        fy *= scale;
        turn *= scale;

        double h = FIELD_CENTRIC ? pose.h : 0; // robot-centric: sticks are already in the robot's frame
        double c = Math.cos(h), s = Math.sin(h);
        double forward = fx * c + fy * s;
        double left = -fx * s + fy * c;
        left *= STRAFE_MULT; // strafing is a bit weaker than driving forward

        double lF = forward - left - turn;
        double lB = forward + left - turn;
        double rB = forward - left + turn;
        double rF = forward + left + turn;
        double max = Math.max(1, Math.max(Math.max(Math.abs(lF), Math.abs(lB)), Math.max(Math.abs(rB), Math.abs(rF))));
        setDriveCached(lF / max, rF / max, lB / max, rB / max);
    }

    /** Skips the 4 motor writes (~1 ms each) unless some wheel changed by DRIVE_WRITE_EPS, or we're stopping. */
    private void setDriveCached(double lF, double rF, double lB, double rB) {
        boolean zero = lF == 0 && rF == 0 && lB == 0 && rB == 0;
        boolean lastZero = lastDrive[0] == 0 && lastDrive[1] == 0 && lastDrive[2] == 0 && lastDrive[3] == 0;
        if (!Double.isNaN(lastDrive[0]) && (zero ? lastZero :
                Math.abs(lF - lastDrive[0]) < DRIVE_WRITE_EPS && Math.abs(rF - lastDrive[1]) < DRIVE_WRITE_EPS
                        && Math.abs(lB - lastDrive[2]) < DRIVE_WRITE_EPS && Math.abs(rB - lastDrive[3]) < DRIVE_WRITE_EPS)) {
            return;
        }
        drive.setDrivePower(lF, rF, lB, rB);
        lastDrive[0] = lF; lastDrive[1] = rF; lastDrive[2] = lB; lastDrive[3] = rB;
    }

    public void invalidateDriveCache() {
        lastDrive[0] = Double.NaN;
    }

    /** Stop the drive motors (through the cache). */
    public void stopDrive() {
        if (drive.driveState != NeuronDrive.DriveState.DRIVE) {
            drive.driveState = NeuronDrive.DriveState.DRIVE;
            invalidateDriveCache();
        }
        setDriveCached(0, 0, 0, 0);
    }

    public static boolean sticksActive(Gamepad g) {
        return Math.abs(g.left_stick_x) > STICK_DEADZONE || Math.abs(g.left_stick_y) > STICK_DEADZONE
                || Math.abs(g.right_stick_x) > STICK_DEADZONE;
    }

    /** Reset heading to 0 (robot facing straight away from our wall), keep x/y. */
    public void resetHeading() {
        localizer.setPosition(new Pose2D(pose.x, pose.y, 0));
    }

    public void relocalize(Pose2D p) {
        localizer.setPosition(p);
        traveledIn = 0;
    }
}
