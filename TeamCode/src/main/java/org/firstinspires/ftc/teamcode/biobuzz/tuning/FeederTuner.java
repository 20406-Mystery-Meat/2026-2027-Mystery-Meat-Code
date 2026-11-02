package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Feeder;

import java.util.Locale;

/**
 * Intake + transfer: jam detection and the one-piece feed pulse.
 * (Set the gate and intake-deploy SERVO positions first, in "Tune: Deploy + Gate".)
 *
 * GAMEPAD 1
 *   right trigger ... INTAKE (the intake arm drops by itself; rollers start once it's down)
 *   left trigger .... EJECT
 *   right bumper .... FEED (gate open, transfer on, flywheel off -- pieces just roll out)
 *   dpad right/left . JAM_CURRENT_AMPS +/-0.5
 *   X ............... jam detection on/off
 *   options ......... save to the robot
 *
 * GAMEPAD 2 -- FEED METERING (while firing, the transfer feeds ONE piece per pulse)
 *   Y ............... one feed pulse (gate opens, waits GATE_SERVO_S, runs the transfer FEED_PULSE_S)
 *   dpad up/down .... FEED_PULSE_S +/-5 ms        dpad left/right .. TRANSFER_FEED_POWER -/+0.05
 * Goal: EVERY Y press moves exactly one piece into the wheel, never two. Sim defaults:
 * 0.35 power / 75 ms. If 0.35 stalls on the 2 mm squeeze, try 0.50 / 35 ms. Spin the flywheel
 * up in the Flywheel tuner first if you want pieces launched.
 *
 * Jam threshold: watch "peak amps" while intaking normally vs. while holding a piece stuck, and
 * put JAM_CURRENT_AMPS between the two (the log's in_amps/tr_amps columns show the full trace).
 * If the intake/transfer spin the wrong way, flip INTAKE_REVERSED / TRANSFER_REVERSED.
 */
@TeleOp(name = "Tune: Intake + Feed", group = "BIOBUZZ Tuning")
public class FeederTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        BuzzRobot robot = new BuzzRobot(hardwareMap, telemetry);
        double peakIn = 0, peakTr = 0;
        double pulseAt = -1; // time gamepad 2 Y was pressed

        waitForStart();
        robot.initPose(Field.resetPose(), false);

        try {
            while (opModeIsActive()) {
                robot.read();
                if (gamepad1.dpadRightWasPressed()) JAM_CURRENT_AMPS += 0.5;
                if (gamepad1.dpadLeftWasPressed()) JAM_CURRENT_AMPS = Math.max(0.5, JAM_CURRENT_AMPS - 0.5);
                if (gamepad1.xWasPressed()) JAM_DETECT_ENABLED = !JAM_DETECT_ENABLED;
                if (gamepad2.dpadUpWasPressed()) FEED_PULSE_S += 0.005;
                if (gamepad2.dpadDownWasPressed()) FEED_PULSE_S = Math.max(0.01, FEED_PULSE_S - 0.005);
                if (gamepad2.dpadRightWasPressed()) TRANSFER_FEED_POWER = Math.min(1, TRANSFER_FEED_POWER + 0.05);
                if (gamepad2.dpadLeftWasPressed()) TRANSFER_FEED_POWER = Math.max(0.1, TRANSFER_FEED_POWER - 0.05);
                if (gamepad2.yWasPressed()) {
                    pulseAt = robot.nowS;
                }
                if (gamepad1.optionsWasPressed()) {
                    ConfigIO.save();
                }

                Feeder.Mode m = Feeder.Mode.IDLE;
                double sincePulse = pulseAt < 0 ? 1e9 : robot.nowS - pulseAt;
                if (sincePulse < GATE_SERVO_S) m = Feeder.Mode.FEED_HOLD;            // gate opening
                else if (sincePulse < GATE_SERVO_S + FEED_PULSE_S) m = Feeder.Mode.FEED; // one pulse
                else if (sincePulse < GATE_SERVO_S + FEED_PULSE_S + 0.5) m = Feeder.Mode.FEED_HOLD;
                else if (gamepad1.right_bumper) m = Feeder.Mode.FEED;
                else if (gamepad1.right_trigger > TRIGGER_PRESSED) m = Feeder.Mode.INTAKE;
                else if (gamepad1.left_trigger > TRIGGER_PRESSED) m = Feeder.Mode.EJECT;
                robot.feeder.setMode(m);
                if (m == Feeder.Mode.IDLE) { peakIn = 0; peakTr = 0; }
                robot.turret.setTarget(robot.turret.targetDeg, 0);
                robot.stopDrive();
                robot.write();

                peakIn = Math.max(peakIn, robot.feeder.intakeAmps);
                peakTr = Math.max(peakTr, robot.feeder.transferAmps);

                if (robot.telemetryDue()) {
                    telemetry.addData("Mode", m + (robot.feeder.unjamming ? " (UNJAMMING)" : ""));
                    telemetry.addData("Intake arm", robot.deploy.state);
                    telemetry.addData("Intake amps (peak)", "%.2f (%.2f)", robot.feeder.intakeAmps, peakIn);
                    telemetry.addData("Transfer amps (peak)", "%.2f (%.2f)", robot.feeder.transferAmps, peakTr);
                    telemetry.addData("JAM_CURRENT_AMPS", "%.1f  detect=%s", JAM_CURRENT_AMPS, JAM_DETECT_ENABLED);
                    telemetry.addData("jams", robot.feeder.jams);
                    telemetry.addData("Feed pulse (gp2 Y)", "%.2f power, %.0f ms", TRANSFER_FEED_POWER, FEED_PULSE_S * 1000);
                    telemetry.addData("Config", ConfigIO.lastStatus);
                    telemetry.update();
                }
            }
        } finally {
            robot.stopAll();
        }
    }
}
