package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Feeder;

import java.util.Locale;

/**
 * Tune the flywheel velocity controller. All RPM is MAIN-WHEEL rpm.
 *
 *   A ............ flywheel on/off
 *   dpad up/down . target +/-100 rpm
 *   Y ............ AUTO kV: runs at 80% power for 2.5 s and computes FLY_KV for you
 *   right trigger  feed pieces (gate open, transfer on) -- watch the dip & recovery time
 *   options ...... save config
 *   bumpers / dpad l-r / X / B / LS-button: edit gains
 *
 * Order: Y (auto kV) -> KS so low targets reach speed -> KP until recovery after a shot is
 * fast without oscillating -> set FLY_SHOT_DIP_RPM a bit below the dip you see per shot.
 * If the wheel spins backwards, flip FLYWHEEL_REVERSED.
 */
@TeleOp(name = "Tune: Flywheel", group = "BIOBUZZ Tuning")
public class FlywheelTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        BuzzRobot robot = new BuzzRobot(hardwareMap, telemetry);
        ParamEditor params = new ParamEditor("FLY_KV", "FLY_KS", "FLY_KP", "FLY_READY_TOL_RPM", "FLY_SHOT_DIP_RPM");
        boolean on = false;
        double target = 3000;
        double autoKvUntil = -1, autoKvRpmSum = 0;
        int autoKvSamples = 0;
        double minRpmSeen = 0;
        String autoKvResult = "-";

        waitForStart();
        robot.initPose(Field.resetPose(), false);
        robot.startLog("FlywheelTuner");

        try {
            while (opModeIsActive()) {
                robot.read();
                params.handle(gamepad1);
                if (gamepad1.aWasPressed()) { on = !on; robot.event("FLYWHEEL " + (on ? "on " + target : "off")); }
                if (gamepad1.dpadUpWasPressed()) target = Math.min(FLY_MAX_WHEEL_RPM, target + 100);
                if (gamepad1.dpadDownWasPressed()) target = Math.max(0, target - 100);
                if (gamepad1.optionsWasPressed()) { ConfigIO.save(); robot.event("SAVE config " + params.describe()); }
                if (gamepad1.yWasPressed()) {
                    autoKvUntil = robot.nowS + 2.5;
                    autoKvRpmSum = 0;
                    autoKvSamples = 0;
                    robot.event("AUTO_KV start");
                }

                boolean feed = gamepad1.right_trigger > 0.3;
                robot.feeder.setMode(feed ? Feeder.Mode.FEED : Feeder.Mode.IDLE);
                robot.flywheel.setTargetRpm(on ? target : 0);
                robot.turret.setTarget(robot.turret.targetDeg, 0);
                robot.stopDrive();

                if (robot.nowS < autoKvUntil) {
                    // Fixed power; average the last second once it has spun up.
                    robot.flywheel.rawPowerOverride = 0.8;
                    if (autoKvUntil - robot.nowS < 1.0) {
                        autoKvRpmSum += robot.flywheel.rpm;
                        autoKvSamples++;
                    }
                } else {
                    robot.flywheel.rawPowerOverride = Double.NaN;
                    if (autoKvSamples > 0) {
                        double avg = autoKvRpmSum / autoKvSamples;
                        FLY_KV = (0.8 - FLY_KS) * (robot.volts / 12.0) / avg;
                        autoKvResult = String.format(Locale.US, "%.0f rpm at 80%% -> FLY_KV %.3g (options to save)", avg, FLY_KV);
                        robot.event(String.format(Locale.US, "AUTO_KV avg=%.0f rpm volts=%.2f -> FLY_KV=%.4g", avg, robot.volts, FLY_KV));
                        autoKvSamples = 0;
                    }
                }
                robot.write();
                if (feed) minRpmSeen = Math.min(minRpmSeen, robot.flywheel.rpm);
                else minRpmSeen = robot.flywheel.rpm;

                if (robot.telemetryDue()) {
                    telemetry.addData("Flywheel", on ? "ON" : "off");
                    telemetry.addData("target / rpm", "%.0f / %.0f", robot.flywheel.targetRpm, robot.flywheel.rpm);
                    telemetry.addData("error", "%.0f", robot.flywheel.error());
                    telemetry.addData("power / battery", "%.3f / %.2f V", robot.flywheel.power, robot.volts);
                    telemetry.addData("ready", robot.flywheel.ready());
                    telemetry.addData("shots counted", robot.flywheel.shots);
                    telemetry.addData("lowest rpm while feeding", "%.0f", minRpmSeen);
                    telemetry.addData("last dip / recovery", "%.0f rpm / %.2f s", robot.flywheel.lastDipDepth, robot.flywheel.lastRecoveryS);
                    telemetry.addData("auto kV", autoKvResult);
                    telemetry.addLine();
                    params.show(telemetry);
                    telemetry.addData("Config", ConfigIO.lastStatus);
                    telemetry.addData("Loop", robot.loopHealth());
                    telemetry.update();
                }
                robot.endLoop(null);
            }
        } finally {
            robot.stopAll();
            robot.closeLog();
        }
    }
}
