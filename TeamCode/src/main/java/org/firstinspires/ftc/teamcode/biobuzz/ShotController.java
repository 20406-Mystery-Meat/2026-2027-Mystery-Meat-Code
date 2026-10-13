package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotConfidence;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotSolver;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Feeder;

/**
 * The automation brain shared by TeleOp and the tuning OpModes:
 * auto-aim the turret, pick the flywheel speed, and fire only when every check passes.
 *
 * READY = flywheel at speed (tolerance from the shot table slope: ~+-3" of landing error)
 *         AND turret on target (tolerance shrinks with distance) AND the shot is possible.
 * While the driver holds FIRE, {@link FireGate} opens the gate for the burst and the transfer
 * pushes ONE piece per pulse whenever READY has held for FIRE_READY_DEBOUNCE_S, stops the
 * moment the flywheel dip shows it launched, and waits for the wheel to recover before the
 * next -- as fast as the flywheel allows, no manual timing.
 */
public class ShotController {
    private final BuzzRobot robot;

    public Field.Cell upCell = Field.startingUpCell();
    public boolean shootOnMove = SOTM_DEFAULT;
    public boolean prespin = PRESPIN_DEFAULT;
    public boolean manualTurret = false;
    public double manualTurretDeg = 0;
    /** In manual-RPM mode the flywheel ignores distance (for when odometry is lost). */
    public boolean manualRpm = false;
    public double manualRpmValue = FLY_MANUAL_DEFAULT_RPM;
    public double rpmTrim = 0, aimTrimDeg = 0;
    /** If not NaN, overrides the flywheel RPM (tuners). */
    public double rpmOverride = Double.NaN;

    /** Why we're not ready to fire. Logged as its ordinal, so only APPEND new values. */
    public enum Block { NONE, FLYWHEEL, TURRET_LIMIT, TURRET_AIM, SHOT_INVALID, DEBOUNCE, RECOVERING, LOW_CONFIDENCE, TOO_FAST }

    /** What auto-fire is doing (for telemetry/logs). Only APPEND new values. */
    public enum Auto { OFF, HELD, EMPTY, TOO_FAST, LOW_CONFIDENCE, ARMED, MANUAL_MODE }

    public ShotSolver.Solution sol;
    public boolean ready, firing, fireHeld;
    public Block block = Block.NONE;
    public double rpmTol;
    public final FireGate gate = new FireGate();

    // ---- auto-fire
    public boolean autoFire = AUTO_FIRE_DEFAULT;
    public Auto auto = Auto.OFF;
    /** Chance this shot scores right now (current turret + flywheel errors included). */
    public double confidence;
    /** Chance it would score once the flywheel is at speed (used to arm/keep a burst going). */
    public double aimConfidence;
    public double speed;
    /** Best guess whether pieces are loaded (no sensor: counts shots and dry feed pulses). */
    public boolean hasPieces = true;
    public int shotsSinceLoad = 0;
    /** Confidence when the feed pulse that produced the most recent shot started. */
    public double lastShotConfidence = Double.NaN;
    private double pulseConfidence = Double.NaN;
    private boolean armed;
    private final ShotConfidence confNow = new ShotConfidence(), confAim = new ShotConfidence();

    public ShotController(BuzzRobot robot) {
        this.robot = robot;
    }

    /** Old signature for the tuners: no hold-fire button. */
    public void update(boolean fireHeld, boolean intakeHeld, boolean ejectHeld, double manualStick) {
        update(fireHeld, false, intakeHeld, ejectHeld, manualStick);
    }

    /**
     * @param fireHeld    driver/operator is holding a fire button (shoots at MANUAL_MIN_CONFIDENCE)
     * @param holdFire    driver is holding "don't shoot" (blocks auto-fire)
     * @param intakeHeld  driver wants to intake
     * @param ejectHeld   driver wants to spit pieces out
     * @param manualStick operator turret stick (-1..1), used only in manual turret mode
     */
    public void update(boolean fireHeld, boolean holdFire, boolean intakeHeld, boolean ejectHeld, double manualStick) {
        // Solve for where the robot WILL be when the piece leaves (SHOT_LATENCY_S ahead).
        sol = robot.solver.solve(robot.predPose, robot.predVx, robot.predVy, robot.omega,
                upCell, shootOnMove, rpmTrim, aimTrimDeg);
        speed = Math.hypot(robot.vx, robot.vy);

        // ---- turret
        boolean aimValid;
        double tol;
        if (manualTurret) {
            manualTurretDeg += manualStick * TURRET_MANUAL_DEG_PER_S * robot.dtS;
            manualTurretDeg = Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, manualTurretDeg));
            robot.turret.setTarget(manualTurretDeg, 0);
            aimValid = true;
            tol = TURRET_TOL_MAX_DEG;
        } else {
            robot.turret.setTarget(sol.turretDeg, sol.turretRateDegS);
            manualTurretDeg = robot.turret.targetDeg; // so switching to manual doesn't jump
            aimValid = sol.valid;
            tol = sol.tolDeg;
        }

        // ---- magazine guess: intaking = probably loaded; MAX_PIECES shots or several dry pulses = empty
        if (intakeHeld || fireHeld) {
            if (intakeHeld) shotsSinceLoad = 0;
            hasPieces = true;
            gate.dryPulses = 0;
        }
        if (robot.flywheel.shotThisLoop) {
            shotsSinceLoad++;
            lastShotConfidence = pulseConfidence;
        }
        if (hasPieces && (shotsSinceLoad >= MAX_PIECES || gate.dryPulses >= EMPTY_AFTER_DRY_PULSES)) {
            hasPieces = false;
            robot.event("MAGAZINE looks empty (" + shotsSinceLoad + " shots, " + gate.dryPulses + " dry pulses)");
        }

        // ---- confidence (only meaningful with auto aim + auto rpm)
        boolean autoRpm = Double.isNaN(rpmOverride) && !manualRpm;
        boolean modelOk = autoRpm && !manualTurret && aimValid && robot.turret.reachable;
        if (modelOk) {
            double slope = robot.shotTable.slopeAt(sol.distance);
            double rate = sol.turretRateDegS;
            confidence = confNow.compute(sol.distance, sol.tof, sol.shotAngle, slope, robot.flywheel.filteredError(),
                    robot.turret.errorDeg(), rate, robot.predVx, robot.predVy, robot.ax, robot.ay, robot.traveledIn);
            aimConfidence = confAim.compute(sol.distance, sol.tof, sol.shotAngle, slope, 0,
                    robot.turret.errorDeg(), rate, robot.predVx, robot.predVy, robot.ax, robot.ay, robot.traveledIn);
        } else {
            confidence = aimConfidence = 0;
        }

        // ---- auto-fire: arm when we'd be >= 85% sure, under the speed cap, and probably loaded
        boolean tooFast = speed > SOTM_MAX_SPEED_IN_S
                || Math.abs(Math.toDegrees(robot.omega)) > MAX_SHOT_TURN_RATE_DEG_S;
        double armAt = armed ? AUTO_FIRE_KEEP_CONFIDENCE : AUTO_FIRE_MIN_CONFIDENCE;
        if (!autoFire) auto = Auto.OFF;
        else if (!modelOk && !(autoRpm && !manualTurret)) auto = Auto.MANUAL_MODE;
        else if (holdFire) auto = Auto.HELD;
        else if (!hasPieces) auto = Auto.EMPTY;
        else if (tooFast) auto = Auto.TOO_FAST;
        else if (aimConfidence < armAt) auto = Auto.LOW_CONFIDENCE;
        else auto = Auto.ARMED;
        boolean wasArmed = armed;
        armed = auto == Auto.ARMED;
        if (armed != wasArmed) {
            robot.event(String.format(java.util.Locale.US, "AUTO %s conf=%.2f aimConf=%.2f speed=%.0f dist=%.0f",
                    armed ? "ARMED" : "DISARMED (" + auto + ")", confidence, aimConfidence, speed, sol.distance));
        }

        // ---- flywheel
        double rpm;
        if (!Double.isNaN(rpmOverride)) rpm = rpmOverride;
        else if (manualRpm) rpm = manualRpmValue + rpmTrim;
        else rpm = sol.wheelRpm;
        boolean spin = fireHeld || prespin || (autoFire && hasPieces);
        robot.flywheel.setTargetRpm(spin ? rpm : 0);

        // ---- readiness
        rpmTol = autoRpm ? sol.rpmTol : FLY_READY_TOL_RPM;
        boolean flyOk = robot.flywheel.ready(rpmTol);
        boolean turretOk = robot.turret.onTarget(tol);
        boolean confOk;
        if (modelOk) {
            // Manual: the driver decided, lower bar. Auto: the full 85%.
            double need = fireHeld ? MANUAL_MIN_CONFIDENCE : AUTO_FIRE_MIN_CONFIDENCE;
            if (armed && !fireHeld) need = AUTO_FIRE_KEEP_CONFIDENCE;
            confOk = confidence >= need;
        } else {
            confOk = flyOk && turretOk && (aimValid || !autoRpm); // manual modes: plain tolerances
        }
        boolean allOk = confOk && robot.turret.reachable && (aimValid || !autoRpm)
                && !(tooFast && !fireHeld);

        // ---- gate: manual hold, or an armed auto burst
        boolean wasFiring = firing;
        boolean wantFire = fireHeld || armed;
        this.fireHeld = fireHeld;
        firing = gate.update(robot.nowS, wantFire, allOk, flyOk, robot.flywheel.shotThisLoop);
        ready = gate.ready;

        if (gate.recovering) block = Block.RECOVERING;
        else if (tooFast && !fireHeld) block = Block.TOO_FAST;
        else if (!robot.turret.reachable) block = Block.TURRET_LIMIT;
        else if (!aimValid && autoRpm) block = Block.SHOT_INVALID;
        else if (!flyOk && !confOk) block = Block.FLYWHEEL;
        else if (!turretOk && !confOk) block = Block.TURRET_AIM;
        else if (!confOk) block = Block.LOW_CONFIDENCE;
        else if (gate.debouncing(allOk)) block = Block.DEBOUNCE;
        else block = Block.NONE;

        if (firing && !wasFiring) pulseConfidence = confidence;
        if (firing != wasFiring) {
            robot.event(firing ? String.format(java.util.Locale.US, "FEED_PULSE start conf=%.2f %s", confidence, fireHeld ? "manual" : "auto")
                    : "FEED_PULSE end " + (wantFire ? "next blocked by " + block : "released"));
        }
        if (firing) robot.feeder.setMode(Feeder.Mode.FEED);
        else if (gate.gateOpen) robot.feeder.setMode(Feeder.Mode.FEED_HOLD);
        else if (ejectHeld) robot.feeder.setMode(Feeder.Mode.EJECT);
        else if (intakeHeld) robot.feeder.setMode(Feeder.Mode.INTAKE);
        else robot.feeder.setMode(Feeder.Mode.IDLE);
    }

    private final double[] spreads = new double[3];

    /** Spreads (1 sigma, in) from the current confidence estimate: range, lateral, motion. No allocation. */
    public double[] confSpreads() {
        spreads[0] = confNow.sigmaRange;
        spreads[1] = confNow.sigmaLat;
        spreads[2] = confNow.sigmaMove;
        return spreads;
    }

    /** Human-readable reason; only call when drawing telemetry (it allocates). */
    public String notReadyText() {
        switch (block) {
            case NONE: return "";
            case FLYWHEEL: return "flywheel " + Math.round(robot.flywheel.filteredError()) + " rpm off (tol " + Math.round(rpmTol) + ")";
            case RECOVERING: return "flywheel recovering from shot";
            case TURRET_LIMIT: return "turret limit";
            case TURRET_AIM: return String.format(java.util.Locale.US, "turret %.1f deg off", robot.turret.errorDeg());
            case SHOT_INVALID: return sol.invalidReason;
            case LOW_CONFIDENCE: return String.format(java.util.Locale.US, "only %.0f%% likely to score", confidence * 100);
            case TOO_FAST: return String.format(java.util.Locale.US, "moving %.0f in/s (auto max %.0f)", speed, SOTM_MAX_SPEED_IN_S);
            default: return "settling";
        }
    }
}
