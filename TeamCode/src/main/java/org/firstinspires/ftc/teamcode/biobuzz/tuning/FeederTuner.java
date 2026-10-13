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
 * Set the gate servo positions and the jam-detection threshold.
 *
 *   dpad up/down .... move the gate +/-0.01 (live preview)
 *   A ............... store the preview as GATE_OPEN_POS
 *   B ............... store the preview as GATE_CLOSED_POS
 *   right trigger ... INTAKE mode     left trigger ... EJECT
 *   right bumper .... FEED mode (gate open, flywheel off -- pieces just roll out)
 *   dpad right/left . JAM_CURRENT_AMPS +/-0.5
 *   X ............... jam detection on/off
 *   options ......... save config
 *
 * FEED METERING (gamepad 2) -- the transfer feeds ONE piece per pulse while firing:
 *   Y ............... one feed pulse (gate opens, waits GATE_SERVO_S, runs the transfer FEED_PULSE_S)
 *   dpad up/down .... FEED_PULSE_S +/-5 ms        dpad left/right .. TRANSFER_FEED_POWER -/+0.05
 * Spin the flywheel up in the Flywheel tuner first if you want pieces launched, or just watch
 * them get pushed into the (stopped) wheel. Goal: EVERY Y press moves exactly one piece into the
 * wheel, never two. Sim defaults: 0.35 power / 75 ms. If 0.35 stalls on the 2 mm squeeze, try
 * 0.50 / 35 ms.
 *
 * Watch "peak amps" while intaking normally vs. while holding a piece stuck: put
 * JAM_CURRENT_AMPS between the two. The log's in_amps/tr_amps columns show the full trace.
 * Make sure the intake/transfer spin the right way (flip INTAKE_REVERSED /
 * TRANSFER_REVERSED in the config file if not).
 */
@TeleOp(name = "Tune: Gate + Intake", group = "BIOBUZZ Tuning")
public class FeederTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        BuzzRobot robot = new BuzzRobot(hardwareMap, telemetry);
        double preview = GATE_CLOSED_POS;
        double previewUntil = -1;
        double peakIn = 0, peakTr = 0;
        double pulseAt = -1; // time Y was pressed

        waitForStart();
        robot.initPose(Field.resetPose(), false);
        robot.startLog("FeederTuner");

        try {
            while (opModeIsActive()) {
                robot.read();
                if (gamepad1.dpadUpWasPressed()) { preview = Math.min(1, preview + 0.01); previewUntil = robot.nowS + 3; }
                if (gamepad1.dpadDownWasPressed()) { preview = Math.max(0, preview - 0.01); previewUntil = robot.nowS + 3; }
                if (gamepad1.aWasPressed()) { GATE_OPEN_POS = preview; robot.event("GATE_OPEN_POS=" + preview); }
                if (gamepad1.bWasPressed()) { GATE_CLOSED_POS = preview; robot.event("GATE_CLOSED_POS=" + preview); }
                if (gamepad1.dpadRightWasPressed()) JAM_CURRENT_AMPS += 0.5;
                if (gamepad1.dpadLeftWasPressed()) JAM_CURRENT_AMPS = Math.max(0.5, JAM_CURRENT_AMPS - 0.5);
                if (gamepad1.xWasPressed()) JAM_DETECT_ENABLED = !JAM_DETECT_ENABLED;
                if (gamepad2.dpadUpWasPressed()) FEED_PULSE_S += 0.005;
                if (gamepad2.dpadDownWasPressed()) FEED_PULSE_S = Math.max(0.01, FEED_PULSE_S - 0.005);
                if (gamepad2.dpadRightWasPressed()) TRANSFER_FEED_POWER = Math.min(1, TRANSFER_FEED_POWER + 0.05);
                if (gamepad2.dpadLeftWasPressed()) TRANSFER_FEED_POWER = Math.max(0.1, TRANSFER_FEED_POWER - 0.05);
                if (gamepad2.yWasPressed()) {
                    pulseAt = robot.nowS;
                    robot.event(String.format(Locale.US, "TEST_PULSE power=%.2f pulse=%.0fms", TRANSFER_FEED_POWER, FEED_PULSE_S * 1000));
                }
                if (gamepad1.optionsWasPressed()) {
                    ConfigIO.save();
                    robot.event(String.format(Locale.US, "SAVE config gate open=%.2f closed=%.2f jamA=%.1f feed=%.2f pulse=%.3f",
                            GATE_OPEN_POS, GATE_CLOSED_POS, JAM_CURRENT_AMPS, TRANSFER_FEED_POWER, FEED_PULSE_S));
                }

                Feeder.Mode m = Feeder.Mode.IDLE;
                double sincePulse = pulseAt < 0 ? 1e9 : robot.nowS - pulseAt;
                if (sincePulse < GATE_SERVO_S) m = Feeder.Mode.FEED_HOLD;            // gate opening
                else if (sincePulse < GATE_SERVO_S + FEED_PULSE_S) m = Feeder.Mode.FEED; // one pulse
                else if (sincePulse < GATE_SERVO_S + FEED_PULSE_S + 0.5) m = Feeder.Mode.FEED_HOLD;
                else if (gamepad1.right_bumper) m = Feeder.Mode.FEED;
                else if (gamepad1.right_trigger > 0.3) m = Feeder.Mode.INTAKE;
                else if (gamepad1.left_trigger > 0.3) m = Feeder.Mode.EJECT;
                robot.feeder.setMode(m);
                if (m == Feeder.Mode.IDLE) { peakIn = 0; peakTr = 0; }
                robot.feeder.gateOverride = robot.nowS < previewUntil ? preview : Double.NaN;
                robot.turret.setTarget(robot.turret.targetDeg, 0);
                robot.stopDrive();
                robot.write();

                peakIn = Math.max(peakIn, robot.feeder.intakeAmps);
                peakTr = Math.max(peakTr, robot.feeder.transferAmps);

                if (robot.telemetryDue()) {
                    telemetry.addData("Gate preview", "%.2f %s", preview, robot.nowS < previewUntil ? "(showing)" : "");
                    telemetry.addData("GATE_OPEN_POS / CLOSED", "%.2f / %.2f", GATE_OPEN_POS, GATE_CLOSED_POS);
                    telemetry.addData("Mode", m + (robot.feeder.unjamming ? " (UNJAMMING)" : ""));
                    telemetry.addData("Intake amps (peak)", "%.2f (%.2f)", robot.feeder.intakeAmps, peakIn);
                    telemetry.addData("Transfer amps (peak)", "%.2f (%.2f)", robot.feeder.transferAmps, peakTr);
                    telemetry.addData("JAM_CURRENT_AMPS", "%.1f  detect=%s", JAM_CURRENT_AMPS, JAM_DETECT_ENABLED);
                    telemetry.addData("jams", robot.feeder.jams);
                    telemetry.addData("Feed pulse (gp2 Y)", "%.2f power, %.0f ms", TRANSFER_FEED_POWER, FEED_PULSE_S * 1000);
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
