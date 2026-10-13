package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Single-motor flywheel (6000 rpm motor, 2:3 to a 72 mm main wheel, 1" counter-roller
 * geared 3:1 off the main wheel). All RPM values here are MAIN-WHEEL rpm.
 *
 * Control ({@link Control#flywheelPower}): battery-voltage-compensated feedforward (kV, kS)
 * + proportional + full-power boost when far below target. Doing this ourselves instead of
 * RUN_USING_ENCODER keeps shots consistent as the battery sags.
 * Also counts shots by watching for the RPM dip each launched piece causes.
 */
public class Flywheel {
    public final DcMotorEx motor;
    private final HwCache.Power out;

    public double targetRpm, rpm, power;
    /** Main-wheel rpm averaged over FLY_FILTER_MS -- used for the "ready to fire" check. */
    public double rpmFiltered;
    private final VelocityWindow window = new VelocityWindow(48);
    public int shots;
    public boolean inDip;
    /** Seconds from the most recent dip back to "ready" -- shows how fast you can fire. */
    public double lastRecoveryS;
    /** How far below target the most recent shot pulled the wheel (rpm). */
    public double lastDipDepth;
    /** Count of completed dip -> recovered cycles (lets the logger spot new ones). */
    public int recoveries;
    /** Tuners only: if not NaN, this raw power is sent instead of the controller output. */
    public double rawPowerOverride = Double.NaN;
    private double dipMinRpm;
    private long dipStartNs;

    public Flywheel(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, FLYWHEEL_MOTOR);
        motor.setDirection(FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        out = new HwCache.Power(motor);
    }

    public static double ticksPerSecToWheelRpm(double tps) {
        return tps / FLYWHEEL_TICKS_PER_MOTOR_REV * 60.0 * FLYWHEEL_GEAR_RATIO;
    }

    public void setTargetRpm(double wheelRpm) {
        targetRpm = Math.max(0, Math.min(FLY_MAX_WHEEL_RPM, wheelRpm));
    }

    /**
     * @param feeding true while the gate is open (enables shot detection). Shot detection is
     *                done here in read() so the gate can close in the SAME loop.
     */
    public void read(boolean feeding) {
        rpm = ticksPerSecToWheelRpm(motor.getVelocity());
        window.add(System.nanoTime(), motor.getCurrentPosition()); // bulk-cached, free
        double tps = window.rate(FLY_FILTER_MS / 1000.0);
        rpmFiltered = Double.isNaN(tps) ? rpm : ticksPerSecToWheelRpm(tps);
        detectShot(feeding);
    }

    /** True for exactly one loop when a launch dip is first seen. */
    public boolean shotThisLoop;

    private void detectShot(boolean feeding) {
        shotThisLoop = false;
        if (feeding && !inDip && error() > FLY_SHOT_DIP_RPM) {
            inDip = true;
            shotThisLoop = true;
            shots++;
            dipStartNs = System.nanoTime();
            dipMinRpm = rpm;
        } else if (inDip) {
            dipMinRpm = Math.min(dipMinRpm, rpm);
            if (error() < FLY_READY_TOL_RPM) {
                inDip = false;
                lastRecoveryS = (System.nanoTime() - dipStartNs) / 1e9;
                lastDipDepth = targetRpm - dipMinRpm;
                recoveries++;
            } else if (targetRpm == 0) {
                inDip = false; // spun down mid-dip; don't report a bogus recovery
            }
        }
    }

    public double error() {
        return targetRpm - rpm;
    }

    public double filteredError() {
        return targetRpm - rpmFiltered;
    }

    /** At speed, judged on the filtered rpm, within tolRpm. */
    public boolean ready(double tolRpm) {
        return targetRpm > 0 && Math.abs(filteredError()) <= tolRpm;
    }

    public boolean ready() {
        return ready(FLY_READY_TOL_RPM);
    }

    public void write(double batteryVolts) {
        power = Control.flywheelPower(targetRpm, rpm, batteryVolts);
        if (!Double.isNaN(rawPowerOverride)) power = rawPowerOverride;
        out.set(power);
    }

    public void stop() {
        targetRpm = 0;
        power = 0;
        out.set(0);
    }
}
