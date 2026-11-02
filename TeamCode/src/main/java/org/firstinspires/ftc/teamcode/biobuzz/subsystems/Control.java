package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * The turret and flywheel control laws as plain functions with no hardware. The
 * subsystems call these on the robot, and tools/sim runs the SAME functions against
 * physics models of the motors, so gains tuned in simulation mean the same thing here.
 */
public final class Control {
    private Control() {}

    /**
     * Turret motor power -- a "velocity-command" controller, which stays well-behaved even
     * though we don't know the turret's inertia or friction yet:
     *
     *   1. Pick the speed we WANT to be turning at: fast when far away, slowing down so
     *      we could stop exactly at the target with TURRET_MAX_ACCEL (sqrt(2 a d)), and
     *      TURRET_POS_GAIN * error close in. Plus the shot solver's feedforward rate, so the
     *      turret counter-rotates while the chassis spins.
     *   2. Command that speed: kV * speed + kA * planned deceleration (feedforward)
     *      + kS (friction) + kVel * speed error.
     *
     * All scaled to NOMINAL_VOLTS so the gains behave the same on a fresh or a tired battery.
     */
    public static double turretPower(double errDeg, double velDegS, double rateFfDegS,
                                     double posDeg, double volts) {
        double a = Math.abs(errDeg);
        double approach = Math.min(Math.min(TURRET_POS_GAIN * a, Math.sqrt(2 * TURRET_MAX_ACCEL * a)), TURRET_MAX_VEL);
        double vDes = Math.signum(errDeg) * approach + rateFfDegS;
        // On the braking curve the plan is to decelerate at MAX_ACCEL: feed that forward too.
        boolean braking = approach < TURRET_MAX_VEL && Math.sqrt(2 * TURRET_MAX_ACCEL * a) < TURRET_POS_GAIN * a;
        double aDes = braking ? -Math.signum(errDeg) * TURRET_MAX_ACCEL : 0;

        double p = TURRET_KV * vDes + TURRET_KA * aDes + TURRET_KVEL * (vDes - velDegS);
        // Friction comp when we actually need to move: outside the deadband, or while tracking.
        if (a > TURRET_DEADBAND_DEG || Math.abs(rateFfDegS) > TURRET_TRACK_KS_MIN_DPS) p += Math.signum(vDes) * TURRET_KS;
        p *= NOMINAL_VOLTS / Math.max(volts, MIN_COMP_VOLTS);

        // Never drive further past a soft limit.
        if (posDeg >= TURRET_MAX_DEG && p > 0) p = 0;
        if (posDeg <= TURRET_MIN_DEG && p < 0) p = 0;
        return Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, p));
    }

    /**
     * Flywheel motor power: voltage-compensated feedforward (kV, kS) + proportional.
     * If the wheel is more than FLY_BOOST_RPM below target (just after a shot, or spinning
     * up), go full power -- it recovers much faster than P alone, and can't overshoot
     * because the boost ends FLY_BOOST_RPM before the target.
     */
    public static double flywheelPower(double targetRpm, double rpm, double volts) {
        if (targetRpm == 0) return 0;
        // Negative target (reverse idle): same controller, mirrored.
        if (targetRpm < 0) return -flywheelPower(-targetRpm, -rpm, volts);
        double err = targetRpm - rpm;
        if (FLY_BOOST_RPM > 0 && err > FLY_BOOST_RPM) return 1.0;
        double comp = NOMINAL_VOLTS / Math.max(volts, MIN_COMP_VOLTS);
        double ff = (FLY_KV * targetRpm) * comp + FLY_KS;
        return Math.max(0, Math.min(1, ff + FLY_KP * err * comp));
    }

    /**
     * Whether to re-send the flywheel power this loop: at most every FLY_WRITE_PERIOD_MS, but at once
     * for a stop or a change of FLY_WRITE_FORCE_DELTA or more (shot recovery, spin-up).
     */
    public static boolean flywheelShouldSend(double power, double lastSent, double msSinceSent) {
        return Double.isNaN(lastSent) || power == 0 || Math.abs(power - lastSent) >= FLY_WRITE_FORCE_DELTA
                || msSinceSent >= FLY_WRITE_PERIOD_MS;
    }
}
