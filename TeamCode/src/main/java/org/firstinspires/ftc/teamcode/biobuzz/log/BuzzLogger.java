package org.firstinspires.ftc.teamcode.biobuzz.log;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.ShotController;
import org.firstinspires.ftc.teamcode.biobuzz.field.AimZone;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotSolver;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * The BIOBUZZ log schema: which columns exist and how each loop fills them. Also turns
 * changes between loops into events (shots, recoveries, jams, CELL changes, mode toggles),
 * so subsystems don't need to know logging exists.
 *
 * Adding a column: add it to COLS/DEC in the same position and fill it in record().
 * tools/analyze_log.py reads columns BY NAME, so order changes are safe.
 */
public class BuzzLogger {
    public static final String[] COLS = {
            "t", "loop_ms", "work_ms",
            "ms_pinpoint", "ms_hubs", "ms_slow", "ms_drive", "ms_logic", "ms_write", "ms_telem",
            "x", "y", "h_deg", "vx", "vy", "omega_dps", "drive_state",
            "cell", "zone", "dist", "dist_virt", "tof", "sol_valid", "sol_code",
            "tur_pos", "tur_tgt", "tur_err", "tur_vel", "tur_ff", "tur_pwr", "tur_tol", "tur_reach",
            "fly_rpm", "fly_rpm_f", "fly_tgt", "fly_err", "fly_tol", "fly_pwr", "volts",
            "fire", "ready", "block", "firing", "gate", "feed_mode", "in_amps", "tr_amps", "unjam",
            "shots", "jams", "rpm_trim", "aim_trim", "sotm", "man_tur", "man_rpm", "prespin",
            "conf", "aim_conf", "speed", "auto", "has_pieces", "ax", "ay", "traveled", "sig_rng", "sig_lat", "sig_move",
    };
    private static final int[] DEC = {
            3, 2, 2,
            2, 2, 2, 2, 2, 2, 2,
            2, 2, 2, 1, 1, 1, 0,
            0, 0, 2, 2, 3, 0, 0,
            2, 2, 2, 1, 1, 3, 2, 0,
            0, 0, 0, 0, 0, 3, 2,
            0, 0, 0, 0, 0, 0, 2, 2, 0,
            0, 0, 0, 2, 0, 0, 0, 0,
            3, 3, 1, 0, 0, 0, 0, 0, 2, 2, 2,
    };

    private final DataLog log;
    private final BuzzRobot robot;

    // previous-loop values for event detection
    private int lastShots, lastRecoveries, lastJams;
    private Object lastCell;
    private boolean lastSotm, lastManTur, lastManRpm, lastPrespin, lastReach = true;
    private int lastZone = -2;

    public BuzzLogger(BuzzRobot robot, String opModeName) {
        if (COLS.length != DEC.length) throw new IllegalStateException("BuzzLogger COLS/DEC length mismatch");
        this.robot = robot;
        log = new DataLog(opModeName, COLS, DEC, header(robot));
    }

    private static List<String> header(BuzzRobot robot) {
        List<String> h = new ArrayList<>();
        h.add("BIOBUZZ log v1  target_loop_ms=" + LoopTimer.TARGET_MS);
        for (Map.Entry<String, String> e : ConfigIO.snapshot().entrySet()) h.add("config " + e.getKey() + "=" + e.getValue());
        StringBuilder t = new StringBuilder("shot_table");
        for (Map.Entry<Double, Double> e : robot.shotTable.points.entrySet()) {
            t.append(String.format(Locale.US, " %.1f:%.0f", e.getKey(), e.getValue()));
        }
        h.add(t.toString());
        for (int i = 0; i < robot.zones.zones.size(); i++) h.add("zone " + i + " " + robot.zones.zones.get(i).toLine());
        h.add("enums block=NONE,FLYWHEEL,TURRET_LIMIT,TURRET_AIM,SHOT_INVALID,DEBOUNCE,RECOVERING,LOW_CONFIDENCE,TOO_FAST"
                + " feed_mode=IDLE,INTAKE,FEED,EJECT,FEED_HOLD sol_code=ok,too_close,outside_table,rpm_max,spinning"
                + " drive_state=DRIVE,PID,HOLD_POS,CLOSEPID,PARK,SPLINE cell=1:LEFT,-1:RIGHT"
                + " auto=OFF,HELD,EMPTY,TOO_FAST,LOW_CONFIDENCE,ARMED,MANUAL_MODE");
        return h;
    }

    public void event(String msg) {
        log.event(robot.nowS, msg);
    }

    public int dropped() {
        return log.dropped();
    }

    public String error() {
        return log.error();
    }

    public String fileName() {
        return log.baseName;
    }

    /** Call once at the very end of each loop. shot may be null (tuners without auto-aim). */
    public void record(ShotController shot) {
        LoopTimer tm = robot.timer;
        double[] r = log.obtain();
        if (r != null) {
            int i = 0;
            r[i++] = robot.nowS;
            r[i++] = tm.loopMs;
            r[i++] = tm.workMs;
            for (int p = 0; p < LoopTimer.NAMES.length; p++) r[i++] = tm.phaseMs[p];
            r[i++] = robot.pose.x;
            r[i++] = robot.pose.y;
            r[i++] = Math.toDegrees(robot.pose.h);
            r[i++] = robot.vx;
            r[i++] = robot.vy;
            r[i++] = Math.toDegrees(robot.omega);
            r[i++] = robot.drive.driveState.ordinal();

            ShotSolver.Solution s = shot != null ? shot.sol : null;
            r[i++] = shot != null ? shot.upCell.sign() : 0;
            r[i++] = s != null ? s.zoneIndex : -1;
            r[i++] = s != null ? s.realDistance : Double.NaN;
            r[i++] = s != null ? s.distance : Double.NaN;
            r[i++] = s != null ? s.tof : Double.NaN;
            r[i++] = s != null && s.valid ? 1 : 0;
            r[i++] = s != null ? s.invalidCode : -1;

            r[i++] = robot.turret.positionDeg;
            r[i++] = robot.turret.targetDeg;
            r[i++] = robot.turret.errorDeg();
            r[i++] = robot.turret.velocityDegS;
            r[i++] = robot.turret.targetRateDegS;
            r[i++] = robot.turret.power;
            r[i++] = s != null ? s.tolDeg : Double.NaN;
            r[i++] = robot.turret.reachable ? 1 : 0;

            r[i++] = robot.flywheel.rpm;
            r[i++] = robot.flywheel.rpmFiltered;
            r[i++] = robot.flywheel.targetRpm;
            r[i++] = robot.flywheel.error();
            r[i++] = shot != null ? shot.rpmTol : Double.NaN;
            r[i++] = robot.flywheel.power;
            r[i++] = robot.volts;

            r[i++] = shot != null && shot.fireHeld ? 1 : 0;
            r[i++] = shot != null && shot.ready ? 1 : 0;
            r[i++] = shot != null ? shot.block.ordinal() : -1;
            r[i++] = shot != null && shot.firing ? 1 : 0;
            r[i++] = robot.feeder.gateOpen ? 1 : 0;
            r[i++] = robot.feeder.mode.ordinal();
            r[i++] = robot.feeder.intakeAmps;
            r[i++] = robot.feeder.transferAmps;
            r[i++] = robot.feeder.unjamming ? 1 : 0;

            r[i++] = robot.flywheel.shots;
            r[i++] = robot.feeder.jams;
            r[i++] = shot != null ? shot.rpmTrim : 0;
            r[i++] = shot != null ? shot.aimTrimDeg : 0;
            r[i++] = shot != null && shot.shootOnMove ? 1 : 0;
            r[i++] = shot != null && shot.manualTurret ? 1 : 0;
            r[i++] = shot != null && shot.manualRpm ? 1 : 0;
            r[i++] = shot != null && shot.prespin ? 1 : 0;
            r[i++] = shot != null ? shot.confidence : Double.NaN;
            r[i++] = shot != null ? shot.aimConfidence : Double.NaN;
            r[i++] = Math.hypot(robot.vx, robot.vy);
            r[i++] = shot != null ? shot.auto.ordinal() : -1;
            r[i++] = shot != null && shot.hasPieces ? 1 : 0;
            r[i++] = robot.ax;
            r[i++] = robot.ay;
            r[i++] = robot.traveledIn;
            r[i++] = shot != null ? shot.confSpreads()[0] : Double.NaN;
            r[i++] = shot != null ? shot.confSpreads()[1] : Double.NaN;
            r[i] = shot != null ? shot.confSpreads()[2] : Double.NaN;
            log.push(r);
        }
        detectEvents(shot);
    }

    private void detectEvents(ShotController shot) {
        if (robot.flywheel.shots != lastShots) {
            lastShots = robot.flywheel.shots;
            ShotSolver.Solution s = shot != null ? shot.sol : null;
            event(String.format(Locale.US, "SHOT #%d conf=%.2f dist=%.1f virt=%.1f tgt=%.0f rpm preShotErrF=%.0f turretErr=%.2f tol=%.2f speed=%.0f in/s zone=%s",
                    lastShots, shot != null ? shot.lastShotConfidence : Double.NaN, s != null ? s.realDistance : Double.NaN, s != null ? s.distance : Double.NaN,
                    robot.flywheel.targetRpm, robot.flywheel.filteredError(), robot.turret.errorDeg(), s != null ? s.tolDeg : Double.NaN,
                    Math.hypot(robot.vx, robot.vy), s != null && s.zone != null ? s.zone.name : "-"));
        }
        if (robot.flywheel.recoveries != lastRecoveries) {
            lastRecoveries = robot.flywheel.recoveries;
            event(String.format(Locale.US, "RECOVERED dip=%.0f rpm in %.3f s",
                    robot.flywheel.lastDipDepth, robot.flywheel.lastRecoveryS));
        }
        if (robot.feeder.jams != lastJams) {
            lastJams = robot.feeder.jams;
            event(String.format(Locale.US, "JAM #%d intake=%.2fA transfer=%.2fA mode=%s",
                    lastJams, robot.feeder.intakeAmps, robot.feeder.transferAmps, robot.feeder.mode));
        }
        if (robot.turret.reachable != lastReach) {
            lastReach = robot.turret.reachable;
            event(lastReach ? "TURRET back in range" : String.format(Locale.US, "TURRET hit soft limit at %.1f deg", robot.turret.positionDeg));
        }
        if (shot == null) return;
        if (shot.upCell != lastCell) {
            if (lastCell != null) event("UP_CELL -> " + shot.upCell);
            lastCell = shot.upCell;
        }
        int zone = shot.sol != null ? shot.sol.zoneIndex : -1;
        if (zone != lastZone) {
            AimZone z = shot.sol != null ? shot.sol.zone : null;
            event("ZONE -> " + (z == null ? "none" : z.name));
            lastZone = zone;
        }
        if (shot.shootOnMove != lastSotm) { lastSotm = shot.shootOnMove; event("SOTM " + (lastSotm ? "on" : "off")); }
        if (shot.manualTurret != lastManTur) { lastManTur = shot.manualTurret; event("MANUAL_TURRET " + (lastManTur ? "on" : "off")); }
        if (shot.manualRpm != lastManRpm) { lastManRpm = shot.manualRpm; event("MANUAL_RPM " + (lastManRpm ? "on" : "off")); }
        if (shot.prespin != lastPrespin) { lastPrespin = shot.prespin; event("PRESPIN " + (lastPrespin ? "on" : "off")); }
    }

    public void close() {
        String summary = "SUMMARY " + robot.timer.summary()
                + String.format(Locale.US, " shots=%d jams=%d dropped_rows=%d",
                robot.flywheel.shots, robot.feeder.jams, log.dropped());
        log.event(robot.nowS, summary);
        log.close(null);
    }
}
