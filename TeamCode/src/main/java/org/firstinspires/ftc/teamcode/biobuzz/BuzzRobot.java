package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.field.ZoneMap;
import org.firstinspires.ftc.teamcode.biobuzz.log.BuzzLogger;
import org.firstinspires.ftc.teamcode.biobuzz.log.LoopTimer;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Ballistics;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotSolver;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotTable;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
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
 *   robot.endLoop(shot);   // timing + one log row
 * </pre>
 * Call startLog("Name") after start and closeLog() when the OpMode ends (use try/finally).
 * Hubs use MANUAL bulk caching: one bulk read per hub per loop for all encoders/velocities.
 * Battery voltage and motor currents are separate (slow) reads, so they're rate-limited.
 */
public class BuzzRobot {
    public final NeuronLocalizer localizer;
    public final NeuronDrive drive;
    public final Turret turret;
    public final Flywheel flywheel;
    public final Feeder feeder;
    public final ShotTable shotTable;
    public final ZoneMap zones;
    public final ShotSolver solver;
    public final LoopTimer timer = new LoopTimer();
    public BuzzLogger log;

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
    public double volts = 12.5;
    public double nowS, dtS, loopMs;

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

        localizer = new NeuronLocalizer(hw, telemetry, PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM);
        drive = new NeuronDrive(hw, localizer, TRACK_WIDTH_IN);
        turret = new Turret(hw);
        flywheel = new Flywheel(hw);
        feeder = new Feeder(hw);
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
            turret.zeroHere();
        }
        pose = p;
        traveledIn = fromAuto ? 300 : 0; // after auto, assume some drift has built up
        return fromAuto;
    }

    // ------------------------------------------------------------------ logging

    public void startLog(String opModeName) {
        if (!LOG_ENABLED) return;
        log = new BuzzLogger(this, opModeName);
        event("START " + opModeName + " pose " + fmt(pose));
    }

    /** Record a discrete event (rare things only -- it allocates a String). */
    public void event(String msg) {
        if (log != null) log.event(msg);
    }

    /** Last call of every loop: finishes timing and writes one log row. shot may be null. */
    public void endLoop(ShotController shot) {
        timer.mark(LoopTimer.TELEM);
        timer.end();
        if (log != null) log.record(shot);
    }

    public void closeLog() {
        if (log != null) {
            log.close();
            log = null;
        }
    }

    /** True at most every TELEMETRY_PERIOD_MS: build + send telemetry only then (it's slow). */
    public boolean telemetryDue() {
        if (nowS - lastTelemS < TELEMETRY_PERIOD_MS / 1000.0) return false;
        lastTelemS = nowS;
        return true;
    }

    /** One telemetry line with loop health, for the bottom of every OpMode's display. */
    public String loopHealth() {
        return String.format(java.util.Locale.US, "%.1f ms (p95 %.1f, max %.1f, >%.0fms %d)%s",
                timer.loopMs, timer.percentile(0.95), timer.maxMs, LoopTimer.TARGET_MS, timer.over,
                log == null ? " LOG OFF" : log.error() != null ? " LOG ERR " + log.error()
                        : log.dropped() > 0 ? " dropped " + log.dropped() : "");
    }

    static String fmt(Pose2D p) {
        return String.format(java.util.Locale.US, "(%.1f, %.1f, %.1f deg)", p.x, p.y, Math.toDegrees(p.h));
    }

    // ------------------------------------------------------------------ loop

    public void read() {
        timer.start();
        long now = System.nanoTime();
        dtS = Math.max((now - lastNs) / 1e9, 1e-3);
        loopMs = dtS * 1000;
        lastNs = now;
        nowS = (now - startNs) / 1e9;

        for (int i = 0; i < hubs.size(); i++) hubs.get(i).clearBulkCache();

        drive.update(); // reads the Pinpoint; also runs the path follower if one is active
        pose = drive.pose;
        vx = localizer.velX();
        vy = localizer.velY();
        omega = Math.toRadians(localizer.getAngularVelocityDegPerSec());
        // acceleration from velocity change, low-passed (~40 ms) so encoder noise doesn't dominate
        double k = Math.min(1, dtS / 0.04);
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
        timer.mark(LoopTimer.PINPOINT);

        turret.read();   // first encoder access triggers the hub bulk read
        flywheel.read(feeder.gateOpen);
        timer.mark(LoopTimer.HUBS);

        if (nowS - lastVoltsS > 0.25) {
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
        timer.mark(LoopTimer.SLOW);
    }

    public void write() {
        timer.mark(LoopTimer.LOGIC);
        turret.write(volts);
        flywheel.write(volts);
        feeder.write(nowS);
        timer.mark(LoopTimer.WRITE);
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
     * Field-centric mecanum drive in the alliance frame (stick up = away from our wall).
     * Left stick translates, right stick x turns.
     */
    public void driveFieldCentric(Gamepad g, boolean slow) {
        timer.mark(LoopTimer.LOGIC);
        if (drive.driveState != NeuronDrive.DriveState.DRIVE) {
            // A path/park was running and wrote the motors itself: our cache is stale.
            drive.driveState = NeuronDrive.DriveState.DRIVE;
            invalidateDriveCache();
        }
        double fx = -g.left_stick_y;  // field +x
        double fy = -g.left_stick_x;  // field +y (drivers' left)
        double turn = -g.right_stick_x * TURN_SCALE;
        double scale = slow ? DRIVE_SLOW_SCALE : 1.0;
        fx *= scale;
        fy *= scale;
        turn *= scale;

        double c = Math.cos(pose.h), s = Math.sin(pose.h);
        double forward = fx * c + fy * s;
        double left = -fx * s + fy * c;
        left *= 1.1; // strafing is a bit weaker than driving forward

        double lF = forward - left - turn;
        double lB = forward + left - turn;
        double rB = forward - left + turn;
        double rF = forward + left + turn;
        double max = Math.max(1, Math.max(Math.max(Math.abs(lF), Math.abs(lB)), Math.max(Math.abs(rB), Math.abs(rF))));
        setDriveCached(lF / max, rF / max, lB / max, rB / max);
        timer.mark(LoopTimer.DRIVE);
    }

    /** Skips the 4 motor writes when nothing changed (e.g. sticks at rest). */
    private void setDriveCached(double lF, double rF, double lB, double rB) {
        boolean zero = lF == 0 && rF == 0 && lB == 0 && rB == 0;
        boolean lastZero = lastDrive[0] == 0 && lastDrive[1] == 0 && lastDrive[2] == 0 && lastDrive[3] == 0;
        if (!Double.isNaN(lastDrive[0]) && (zero ? lastZero :
                Math.abs(lF - lastDrive[0]) < 0.005 && Math.abs(rF - lastDrive[1]) < 0.005
                        && Math.abs(lB - lastDrive[2]) < 0.005 && Math.abs(rB - lastDrive[3]) < 0.005)) {
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
        timer.mark(LoopTimer.LOGIC);
        if (drive.driveState != NeuronDrive.DriveState.DRIVE) {
            drive.driveState = NeuronDrive.DriveState.DRIVE;
            invalidateDriveCache();
        }
        setDriveCached(0, 0, 0, 0);
        timer.mark(LoopTimer.DRIVE);
    }

    public static boolean sticksActive(Gamepad g) {
        return Math.abs(g.left_stick_x) > 0.1 || Math.abs(g.left_stick_y) > 0.1 || Math.abs(g.right_stick_x) > 0.1;
    }

    /** Reset heading to 0 (robot facing straight away from our wall), keep x/y. */
    public void resetHeading() {
        event("RESET_HEADING was " + fmt(pose));
        localizer.setPosition(new Pose2D(pose.x, pose.y, 0));
    }

    public void relocalize(Pose2D p) {
        event("RELOCALIZE " + fmt(pose) + " -> " + fmt(p) + String.format(java.util.Locale.US, " after %.0f in driven", traveledIn));
        localizer.setPosition(p);
        traveledIn = 0;
    }
}
