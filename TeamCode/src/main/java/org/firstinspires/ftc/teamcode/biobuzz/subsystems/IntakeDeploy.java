package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Intake deploy: a goBILDA Speed servo swinging the intake arm down/up through a 40T -> 60T gear
 * (the arm moves 2/3 as far as the servo). All numbers are in Constants.java SECTION 7.
 *
 * Positions are set as ARM angles you can measure with a protractor: DEPLOY_STOWED_POS is the
 * servo position with the intake folded up, and DEPLOY_DOWN_DEG is how far the arm swings from
 * there to down. The gear ratio and servo travel turn that into a servo position.
 *
 * Automatic: drops while intaking or ejecting (DEPLOY_WITH_INTAKE) and folds back up
 * DEPLOY_STOW_DELAY_S after you stop. holdDown (driver toggle) keeps it down. Servos don't report
 * where they are, so "down" is estimated from the servo's speed (isDown()); the intake rollers
 * wait for it so they don't spin while the arm is still folded.
 */
public class IntakeDeploy {
    /** Logged as its ordinal, so only APPEND new values. */
    public enum State { STOWED, DEPLOYING, DOWN, STOWING }

    public final Servo servo;
    private final HwCache.Position out;

    public State state = State.STOWED;
    /** Driver toggle: keep the intake down even when not intaking. */
    public boolean holdDown;
    /** True if the requested arm angle needs a servo position outside 0..1 (check the constants). */
    public boolean clamped;
    /** Tuners only: if not NaN, the servo is held at this position instead. */
    public double positionOverride = Double.NaN;
    public double commandedPos;

    private boolean wantDown;
    private double changedAt = -1e9, lastRequestAt = -1e9;

    public IntakeDeploy(HardwareMap hw) {
        servo = hw.get(Servo.class, INTAKE_DEPLOY_SERVO);
        // goBILDA dual-mode servos travel their full DEPLOY_SERVO_RANGE_DEG over 500-2500 us;
        // the hub default (600-2400 us) would only use ~270 deg of it.
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(DEPLOY_PWM_MIN_US, DEPLOY_PWM_MAX_US));
        }
        out = new HwCache.Position(servo);
        commandedPos = clamp(servoPosForArm(0));
        out.set(commandedPos);
    }

    /** Servo position (0..1) for an arm angle measured from stowed. */
    public static double servoPosForArm(double armDeg) {
        double servoDeg = armDeg * DEPLOY_ARM_GEAR_TEETH / DEPLOY_SERVO_GEAR_TEETH;
        double sign = INTAKE_DEPLOY_REVERSED ? -1 : 1;
        return DEPLOY_STOWED_POS + sign * servoDeg / DEPLOY_SERVO_RANGE_DEG;
    }

    /** Estimated time for a full stowed <-> down swing, including settling. */
    public static double travelS() {
        double servoDeg = DEPLOY_DOWN_DEG * DEPLOY_ARM_GEAR_TEETH / DEPLOY_SERVO_GEAR_TEETH;
        return servoDeg / 60.0 * DEPLOY_SERVO_S_PER_60DEG + DEPLOY_SETTLE_S;
    }

    /**
     * Call once per loop.
     * @param request something wants the intake down right now (intaking / ejecting)
     */
    public void update(boolean request, double nowS) {
        if (request) lastRequestAt = nowS;
        boolean down = holdDown || request || (wantDown && nowS - lastRequestAt < DEPLOY_STOW_DELAY_S);
        if (down != wantDown) {
            // Reversing mid-swing: only the part already travelled has to be undone.
            double done = Math.min(1, (nowS - changedAt) / travelS());
            changedAt = nowS - (1 - done) * travelS();
            wantDown = down;
        }
        boolean arrived = nowS - changedAt >= travelS();
        state = wantDown ? (arrived ? State.DOWN : State.DEPLOYING) : (arrived ? State.STOWED : State.STOWING);

        double pos = Double.isNaN(positionOverride) ? servoPosForArm(wantDown ? DEPLOY_DOWN_DEG : 0) : positionOverride;
        clamped = pos < 0 || pos > 1;
        commandedPos = clamp(pos);
        out.set(commandedPos);
    }

    public boolean isDown() {
        return state == State.DOWN;
    }

    private static double clamp(double p) {
        return Math.max(0, Math.min(1, p));
    }
}
