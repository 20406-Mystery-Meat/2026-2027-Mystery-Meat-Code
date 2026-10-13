package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Motor-driven turret on a 312 rpm motor with a 1:2 reduction, using the motor's own
 * (relative) encoder. 0 deg = shooting straight out the robot's front, CCW positive.
 *
 * ZEROING: the encoder is relative, so the turret must be pointing straight forward
 * when it is zeroed -- at the start of auto (or teleop if auto didn't run). The zero is
 * kept in software ({@link #zeroTicks}) and handed from auto to teleop through MatchState,
 * so we never have to physically reset the encoder mid-event.
 *
 * Control ({@link Control#turretPower}): PD on angle + velocity feedforward (from the shot
 * solver, so the turret counter-rotates as the chassis spins) + static friction kS.
 */
public class Turret {
    public final DcMotorEx motor;
    private final HwCache.Power out;

    public int zeroTicks = 0;
    public double positionDeg, velocityDegS;
    public double targetDeg, targetRateDegS;
    public double power;
    /** False when the requested angle was outside the soft limits and got clamped. */
    public boolean reachable = true;

    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, TURRET_MOTOR);
        motor.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        out = new HwCache.Power(motor);
    }

    public static double ticksPerDeg() {
        return TURRET_TICKS_PER_MOTOR_REV * TURRET_GEAR_RATIO / 360.0;
    }

    /** Declare the current physical position as "pointing straight forward". */
    public void zeroHere() {
        zeroTicks = motor.getCurrentPosition();
        targetDeg = 0;
    }

    /** Read sensors. Call once per loop after the bulk cache is cleared. */
    public void read() {
        positionDeg = (motor.getCurrentPosition() - zeroTicks) / ticksPerDeg();
        velocityDegS = motor.getVelocity() / ticksPerDeg();
    }

    /**
     * Aim at a robot-relative angle. Picks whichever equivalent angle (a, a +/- 360) is inside
     * the soft limits and closest to where the turret is now, so it never winds up the wires.
     */
    public void setTarget(double robotRelativeDeg, double rateFeedforwardDegS) {
        double a = wrap180(robotRelativeDeg);
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) { // a-360, a, a+360 (no array: this runs every loop)
            double c = a + 360 * k;
            if (c < TURRET_MIN_DEG || c > TURRET_MAX_DEG) continue;
            if (Double.isNaN(best) || Math.abs(c - positionDeg) < Math.abs(best - positionDeg)) best = c;
        }
        if (Double.isNaN(best)) {
            // Dead zone between the limits: park at the nearer limit.
            double toMin = Math.abs(wrap180(a - TURRET_MIN_DEG));
            double toMax = Math.abs(wrap180(a - TURRET_MAX_DEG));
            best = toMin < toMax ? TURRET_MIN_DEG : TURRET_MAX_DEG;
            reachable = false;
            rateFeedforwardDegS = 0;
        } else {
            reachable = true;
        }
        targetDeg = best;
        targetRateDegS = rateFeedforwardDegS;
    }

    public double errorDeg() {
        return targetDeg - positionDeg;
    }

    public boolean onTarget(double tolDeg) {
        return reachable && Math.abs(errorDeg()) <= tolDeg;
    }

    /** Write motor power. Call once per loop after setTarget(). */
    public void write(double volts) {
        power = Control.turretPower(errorDeg(), velocityDegS, targetRateDegS, positionDeg, volts);
        out.set(power);
    }

    public void stop() {
        power = 0;
        out.set(0);
    }

    public static double wrap180(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
