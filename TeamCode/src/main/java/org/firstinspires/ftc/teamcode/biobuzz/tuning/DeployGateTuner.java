package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.IntakeDeploy;

import java.util.Locale;

/**
 * Set the INTAKE DEPLOY and GATE servo positions by eye. Only these two servos are used -- no
 * motors -- so this works even before the rest of the robot is wired.
 *
 * GAMEPAD 1
 *   A ............ switch servo: INTAKE DEPLOY <-> GATE
 *   dpad up/down . move it +/-0.01            bumpers ... -/+0.05
 *   X ............ store this position as  DEPLOY: folded up (DEPLOY_STOWED_POS) | GATE: open
 *   B ............ store this position as  DEPLOY: down (works out DEPLOY_DOWN_DEG) | GATE: closed
 *   Y ............ test: swap between the two stored positions (deploy uses the real timing)
 *   options ...... save to the robot
 *
 * Intake deploy: fold the arm up, nudge until it sits right, X. Then nudge down to where it should
 * land, B -- the arm angle comes from the 40:60 gearing, and if the arm went the "negative" way
 * INTAKE_DEPLOY_REVERSED is flipped for you. Do X first.
 *
 * Saving keeps the values on the robot only; copy the "Constants.java" lines shown on screen into
 * the file to make them permanent.
 */
@TeleOp(name = "Tune: Deploy + Gate", group = "BIOBUZZ Tuning")
public class DeployGateTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        ConfigIO.load();
        Servo gate = hardwareMap.tryGet(Servo.class, GATE_SERVO);
        IntakeDeploy deploy = hardwareMap.tryGet(Servo.class, INTAKE_DEPLOY_SERVO) != null ? new IntakeDeploy(hardwareMap) : null;

        boolean onDeploy = deploy != null;            // which servo the controls move
        double deployPos = DEPLOY_STOWED_POS, gatePos = GATE_CLOSED_POS;
        boolean testing = false;                      // Y test mode: deploy runs its real logic
        boolean gateTestOpen = false;
        String note = "";

        if (gate != null) gate.setPosition(gatePos);
        telemetry.addLine("Tune: Deploy + Gate -- press start. Only the two servos move.");
        if (gate == null) telemetry.addLine("No servo named \"" + GATE_SERVO + "\" in the config.");
        if (deploy == null) telemetry.addLine("No servo named \"" + INTAKE_DEPLOY_SERVO + "\" in the config.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double now = getRuntime();
            if (gamepad1.aWasPressed()) {
                onDeploy = !onDeploy;
                if (onDeploy && deploy == null) onDeploy = false;
                if (!onDeploy && gate == null) onDeploy = deploy != null;
                testing = false;
                note = "";
            }

            // ---- nudge
            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step += 0.01;
            if (gamepad1.dpadDownWasPressed()) step -= 0.01;
            if (gamepad1.rightBumperWasPressed()) step += 0.05;
            if (gamepad1.leftBumperWasPressed()) step -= 0.05;
            if (step != 0) {
                testing = false;
                if (onDeploy) deployPos = clamp(deployPos + step);
                else gatePos = clamp(gatePos + step);
            }

            // ---- store
            if (gamepad1.xWasPressed()) {
                if (onDeploy) {
                    DEPLOY_STOWED_POS = deployPos;
                    note = "folded-up position stored";
                } else {
                    GATE_OPEN_POS = gatePos;
                    note = "gate OPEN stored";
                }
            }
            if (gamepad1.bWasPressed()) {
                if (onDeploy) {
                    // servo travel from folded-up -> arm degrees through the gears
                    double servoDeg = (deployPos - DEPLOY_STOWED_POS) * DEPLOY_SERVO_RANGE_DEG * (INTAKE_DEPLOY_REVERSED ? -1 : 1);
                    double armDeg = servoDeg * DEPLOY_SERVO_GEAR_TEETH / DEPLOY_ARM_GEAR_TEETH;
                    if (armDeg < 0) {
                        INTAKE_DEPLOY_REVERSED = !INTAKE_DEPLOY_REVERSED;
                        armDeg = -armDeg;
                        note = "down stored -- arm swings the other way, flipped INTAKE_DEPLOY_REVERSED";
                    } else {
                        note = "down stored";
                    }
                    DEPLOY_DOWN_DEG = Math.round(armDeg * 10) / 10.0;
                } else {
                    GATE_CLOSED_POS = gatePos;
                    note = "gate CLOSED stored";
                }
            }

            // ---- test the stored positions
            if (gamepad1.yWasPressed()) {
                if (onDeploy && deploy != null) {
                    testing = true;
                    deploy.holdDown = !deploy.holdDown;
                } else {
                    gateTestOpen = !gateTestOpen;
                    gatePos = gateTestOpen ? GATE_OPEN_POS : GATE_CLOSED_POS;
                }
            }
            if (gamepad1.optionsWasPressed()) {
                ConfigIO.save();
                note = ConfigIO.lastStatus;
            }

            // ---- drive the servos
            if (deploy != null) {
                deploy.positionOverride = testing ? Double.NaN : deployPos;
                deploy.update(false, now);
                if (testing) deployPos = deploy.commandedPos;
            }
            if (gate != null) gate.setPosition(gatePos);

            // ---- show
            telemetry.addData("Moving", onDeploy ? "INTAKE DEPLOY  (A = switch to gate)" : "GATE  (A = switch to intake deploy)");
            if (deploy != null) {
                telemetry.addData("Deploy servo", "%.2f%s", deployPos,
                        testing ? "  TEST: " + deploy.state + String.format(Locale.US, " (swing %.2f s)", IntakeDeploy.travelS()) : "");
                telemetry.addData("  folded up / down", "%.2f / %.2f  (arm swing %.0f deg)", DEPLOY_STOWED_POS,
                        IntakeDeploy.servoPosForArm(DEPLOY_DOWN_DEG), DEPLOY_DOWN_DEG);
                if (deploy.clamped) telemetry.addLine("  WARNING: down position is past the servo's range");
            }
            if (gate != null) {
                telemetry.addData("Gate servo", "%.2f", gatePos);
                telemetry.addData("  open / closed", "%.2f / %.2f", GATE_OPEN_POS, GATE_CLOSED_POS);
            }
            telemetry.addData("Last", note);
            telemetry.addLine();
            telemetry.addLine("Copy into Constants.java to keep for good:");
            telemetry.addLine(String.format(Locale.US, "  DEPLOY_STOWED_POS = %.2f;", DEPLOY_STOWED_POS));
            telemetry.addLine(String.format(Locale.US, "  DEPLOY_DOWN_DEG = %.1f;", DEPLOY_DOWN_DEG));
            telemetry.addLine("  INTAKE_DEPLOY_REVERSED = " + INTAKE_DEPLOY_REVERSED + ";");
            telemetry.addLine(String.format(Locale.US, "  GATE_OPEN_POS = %.2f;", GATE_OPEN_POS));
            telemetry.addLine(String.format(Locale.US, "  GATE_CLOSED_POS = %.2f;", GATE_CLOSED_POS));
            telemetry.addLine("dpad = nudge, X/B = store, Y = test, options = save to robot");
            telemetry.update();
            sleep(20);
        }
    }

    private static double clamp(double p) {
        return Math.max(0, Math.min(1, p));
    }
}
