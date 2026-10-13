package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.ShotController;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.Locale;

/**
 * Tune the turret position controller.
 *
 * Before tuning: with the robot disabled, turn the turret by hand exactly one full turn --
 * "turret deg" should change by 360. If it doesn't, fix TURRET_GEAR_RATIO /
 * TURRET_TICKS_PER_MOTOR_REV. If the turret runs away when enabled, flip TURRET_REVERSED.
 *
 * START WITH THE TURRET POINTING STRAIGHT FORWARD (that's the zero).
 *
 *   A ............ step test on/off (jumps between +/- step every 1.5 s)
 *   Y ............ cycle step size (10 / 45 / 90 / 170 deg)
 *   right stick x  manual target (when not stepping/tracking)
 *   dpad up ...... TRACK mode: aim at the up CELL while you drive with gamepad 2
 *                  (tests the chassis-spin feedforward -- spin in place and watch error)
 *   dpad down .... zero turret here
 *   options ...... save config
 *   bumpers / dpad l-r / X / B / LS-button: edit gains (see ParamEditor)
 *
 * Defaults come from tools/sim. Tuning order on the robot:
 *  1. KS: in MANUAL, raise it until tiny stick moves just start the turret moving.
 *  2. KV: in TRACK mode spin the robot in place; raise KV if the turret lags the spin,
 *     lower it if it leads. (The analyzer's "error vs chassis spin" line says which.)
 *  3. MAX_ACCEL: STEP 90 -- raise until it starts overshooting, then back off 25%.
 *  4. POS_GAIN: STEP 10 -- raise for a snappier finish, lower if it buzzes at the target.
 *  5. KVEL: raise if it's sluggish to follow, lower if it chatters.
 * Every step result is written to the log's events file (STEP lines) for later comparison.
 */
@TeleOp(name = "Tune: Turret", group = "BIOBUZZ Tuning")
public class TurretTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        BuzzRobot robot = new BuzzRobot(hardwareMap, telemetry);
        ShotController shot = new ShotController(robot);
        shot.autoFire = false; // tuners only shoot when you pull the trigger
        shot.prespin = false;
        ParamEditor params = new ParamEditor("TURRET_KS", "TURRET_KV", "TURRET_KVEL", "TURRET_POS_GAIN", "TURRET_MAX_ACCEL");
        double[] steps = {10, 45, 90, 170};
        int stepIdx = 1;
        boolean stepping = false, tracking = false;
        double manualTarget = 0;
        double stepTarget = 0, lastFlipS = 0, stepStartS = 0;
        double overshoot = 0, settleS = -1, inBandSince = -1;
        boolean reported = true;

        telemetry.addLine("Point the turret STRAIGHT FORWARD, then press start.");
        telemetry.update();
        waitForStart();
        robot.initPose(Field.resetPose(), false);
        robot.startLog("TurretTuner");

        try {
            while (opModeIsActive()) {
                robot.read();
                params.handle(gamepad1);
                if (gamepad1.aWasPressed()) { stepping = !stepping; tracking = false; lastFlipS = -10; }
                if (gamepad1.yWasPressed()) stepIdx = (stepIdx + 1) % steps.length;
                if (gamepad1.dpadUpWasPressed()) { tracking = !tracking; stepping = false; }
                if (gamepad1.dpadDownWasPressed()) { robot.turret.zeroHere(); manualTarget = 0; }
                if (gamepad1.optionsWasPressed()) { ConfigIO.save(); robot.event("SAVE config " + params.describe()); }

                if (stepping) {
                    if (robot.nowS - lastFlipS > 1.5) {
                        if (!reported) {
                            robot.event(String.format(Locale.US, "STEP %.0f deg overshoot=%.2f settle=%s %s",
                                    steps[stepIdx], overshoot, settleS < 0 ? "none" : String.format(Locale.US, "%.3fs", settleS),
                                    params.describe()));
                        }
                        stepTarget = (stepTarget > 0) ? -steps[stepIdx] / 2 : steps[stepIdx] / 2;
                        lastFlipS = robot.nowS;
                        stepStartS = robot.nowS;
                        overshoot = 0;
                        settleS = -1;
                        inBandSince = -1;
                        reported = false;
                    }
                    robot.turret.setTarget(stepTarget, 0);
                    double err = robot.turret.errorDeg();
                    double past = stepTarget > 0 ? -err : err; // how far past the target, in the step direction
                    overshoot = Math.max(overshoot, past);
                    if (Math.abs(err) < 1.0) {
                        if (inBandSince < 0) inBandSince = robot.nowS;
                        if (settleS < 0 && robot.nowS - inBandSince > 0.2) settleS = inBandSince - stepStartS;
                    } else {
                        inBandSince = -1;
                    }
                    robot.feeder.setMode(Feeder.Mode.IDLE);
                    robot.stopDrive();
                } else if (tracking) {
                    robot.driveFieldCentric(gamepad2, false);
                    shot.update(false, false, false, 0);
                } else {
                    manualTarget += gamepad1.right_stick_x * 180 * robot.dtS;
                    robot.turret.setTarget(manualTarget, 0);
                    robot.feeder.setMode(Feeder.Mode.IDLE);
                    robot.stopDrive();
                }
                robot.write();

                if (robot.telemetryDue()) {
                    telemetry.addData("Mode", stepping ? "STEP +/-" + steps[stepIdx] / 2 : tracking ? "TRACK (drive w/ gamepad2)" : "MANUAL");
                    telemetry.addData("turret deg", "%.2f", robot.turret.positionDeg);
                    telemetry.addData("target deg", "%.2f", robot.turret.targetDeg);
                    telemetry.addData("error deg", "%.2f", robot.turret.errorDeg());
                    telemetry.addData("vel deg/s", "%.0f (ff %.0f)", robot.turret.velocityDegS, robot.turret.targetRateDegS);
                    telemetry.addData("power", "%.3f", robot.turret.power);
                    if (stepping) telemetry.addData("overshoot / settle", "%.2f deg / %s", overshoot, settleS < 0 ? "-" : String.format(Locale.US, "%.2f s", settleS));
                    telemetry.addData("ticks (raw)", "%d  (%.3f ticks/deg)", robot.turret.motor.getCurrentPosition(), Turret.ticksPerDeg());
                    telemetry.addLine();
                    params.show(telemetry);
                    telemetry.addData("Config", ConfigIO.lastStatus);
                    telemetry.addData("Loop", robot.loopHealth());
                    telemetry.update();
                }
                robot.endLoop(tracking ? shot : null);
            }
        } finally {
            robot.stopAll();
            robot.closeLog();
        }
    }
}
