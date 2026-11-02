package org.firstinspires.ftc.teamcode.biobuzz.opmodes;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.MatchState;
import org.firstinspires.ftc.teamcode.biobuzz.ShotController;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.Locale;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;


/**
 * BIOBUZZ competition TeleOp. Works for BOTH alliances with no alliance switch: all
 * positions are in the alliance frame (see Field.java).
 *
 * GAMEPAD 1 (driver)
 *   left stick ........ drive (robot-centric: up = the way the robot faces; FIELD_CENTRIC in Constants)
 *   right stick x ..... turn
 *   left bumper ....... slow mode (hold)
 *   right trigger ..... intake (hold) -- the intake arm drops automatically, folds up after
 *   dpad down ......... keep the intake arm DOWN on/off
 *   left trigger ...... eject (hold)
 *   right bumper ...... FIRE (hold): flywheel spins up to shot speed and the intake runs; pieces
 *                       feed one at a time once it's at speed. Released: flywheel idles at FLY_IDLE_RPM
 *   B (hold) .......... HOLD FIRE -- stop auto-fire (e.g. while collecting near opponents)
 *   X ................. MANUAL SHOOT on/off: turret freezes, flywheel spins to MANUAL_SHOT_RPM,
 *                       aim by turning the robot, RB fires (one piece at a time)
 *   Y (hold) .......... auto-drive to PARK in our LOADING ZONE (touch sticks to cancel)
 *   options ........... reset heading (robot facing straight away from our wall)
 *   share/back ........ relocalize: robot squared in the far-left corner (back on our wall)
 *
 * GAMEPAD 2 (operator)
 *   dpad left/right ... which CELL of our HIVE is UP (as seen from our drive station)
 *   right trigger ..... FIRE (hold)
 *   dpad up/down ...... RPM trim +/-25 (manual-RPM / manual-shoot mode: +/-100)
 *   left/right bumper . aim trim +/-0.5 deg
 *   Y ................. clear trims
 *   A ................. pre-spin on/off (on = flywheel stays at shot speed instead of idling)
 *   B ................. shoot-on-the-move on/off
 *   X ................. manual turret (right stick x) on/off
 *   left stick button . manual RPM on/off (fallback if odometry is lost)
 *   share/back ........ re-zero turret (point it straight forward first)
 *   options ........... AUTO-FIRE on/off
 *
 * AUTO-FIRE (default on): the robot shoots by itself whenever it is >= 85% likely to score
 * (AUTO_FIRE_MIN_CONFIDENCE), it is moving slower than SOTM_MAX_SPEED_IN_S, and it probably still
 * has pieces. It aims at the UP CELL the operator selected -- keep that correct, or it will
 * shoot at the wrong CELL.
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BuzzTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        BuzzRobot robot = new BuzzRobot(hardwareMap, telemetry);
        ShotController shot = new ShotController(robot);

        boolean useAuto = MatchState.fresh();
        if (MatchState.upCell != null && useAuto) shot.upCell = MatchState.upCell;

        while (opModeInInit()) {
            if (gamepad1.aWasPressed() && MatchState.fresh()) useAuto = true;
            if (gamepad1.bWasPressed()) useAuto = false;
            if (gamepad2.dpadLeftWasPressed() || gamepad1.dpadLeftWasPressed()) shot.upCell = Field.Cell.LEFT;
            if (gamepad2.dpadRightWasPressed() || gamepad1.dpadRightWasPressed()) shot.upCell = Field.Cell.RIGHT;

            telemetry.addData("Start", !MatchState.fresh() ? "far-left corner, " + Turret.startHint()
                    : useAuto ? "from auto's end pose (B = corner instead)" : "far-left corner (A = auto's pose)");
            telemetry.addData("Up cell", shot.upCell + "  (dpad left/right)");
            telemetry.addData("Config", ConfigIO.lastStatus);
            telemetry.update();
        }
        if (isStopRequested()) return;

        robot.initPose(Field.resetPose(), useAuto);
        try {
            loop(robot, shot);
        } finally {
            robot.stopAll();
        }
    }

    private void loop(BuzzRobot robot, ShotController shot) {
        boolean parking = false;
        boolean rumbled60 = false, rumbled20 = false, rumbled5 = false;
        boolean wasReady = false;
        robot.read();
        double startS = robot.nowS;

        while (opModeIsActive()) {
            robot.read();
            double left = TELEOP_LENGTH_S - (robot.nowS - startS);

            // ------------------------------------------------ operator selections
            if (gamepad2.dpadLeftWasPressed()) { shot.upCell = Field.Cell.LEFT; gamepad2.rumbleBlips(1); }
            if (gamepad2.dpadRightWasPressed()) { shot.upCell = Field.Cell.RIGHT; gamepad2.rumbleBlips(1); }
            double rpmStep = (shot.manualRpm || shot.manualShoot) ? MANUAL_RPM_STEP : RPM_TRIM_STEP;
            if (gamepad2.dpadUpWasPressed()) shot.rpmTrim += rpmStep;
            if (gamepad2.dpadDownWasPressed()) shot.rpmTrim -= rpmStep;
            if (gamepad2.leftBumperWasPressed()) shot.aimTrimDeg += AIM_TRIM_STEP_DEG;
            if (gamepad2.rightBumperWasPressed()) shot.aimTrimDeg -= AIM_TRIM_STEP_DEG;
            if (gamepad2.yWasPressed()) { shot.rpmTrim = 0; shot.aimTrimDeg = 0; }
            if (gamepad2.aWasPressed()) shot.prespin = !shot.prespin;
            if (gamepad2.bWasPressed()) shot.shootOnMove = !shot.shootOnMove;
            if (gamepad2.xWasPressed()) shot.manualTurret = !shot.manualTurret;
            if (gamepad2.leftStickButtonWasPressed()) {
                shot.manualRpm = !shot.manualRpm;
                if (shot.manualRpm && shot.sol != null) shot.manualRpmValue = shot.sol.wheelRpm;
                shot.rpmTrim = 0;
            }
            if (gamepad2.optionsWasPressed()) {
                shot.autoFire = !shot.autoFire;
                gamepad2.rumbleBlips(shot.autoFire ? 2 : 1);
            }
            if (gamepad2.shareWasPressed() || gamepad2.backWasPressed()) {
                robot.turret.zeroHere(0);
                shot.manualTurretDeg = 0;
                gamepad2.rumbleBlips(2);
            }

            // ------------------------------------------------ driver
            if (gamepad1.optionsWasPressed()) { robot.resetHeading(); gamepad1.rumbleBlips(1); }
            if (gamepad1.xWasPressed()) {
                shot.setManualShoot(!shot.manualShoot);
                gamepad1.rumbleBlips(shot.manualShoot ? 2 : 1);
            }
            if (gamepad1.dpadDownWasPressed()) {
                robot.deploy.holdDown = !robot.deploy.holdDown;
                gamepad1.rumbleBlips(robot.deploy.holdDown ? 2 : 1);
            }
            if (gamepad1.shareWasPressed() || gamepad1.backWasPressed()) {
                robot.relocalize(Field.resetPose());
                gamepad1.rumbleBlips(2);
            }

            boolean wantPark = gamepad1.y && !BuzzRobot.sticksActive(gamepad1);
            if (wantPark) {
                if (!parking) {
                    robot.drive.goToPoint(Field.parkPose(), false, PARK_MAX_SPEED);
                    parking = true;
                }
            } else {
                parking = false;
                robot.driveWithSticks(gamepad1, gamepad1.left_bumper);
            }

            boolean fire = gamepad1.right_bumper || gamepad2.right_trigger > FIRE_TRIGGER_PRESSED;
            boolean intake = gamepad1.right_trigger > TRIGGER_PRESSED;
            boolean eject = gamepad1.left_trigger > TRIGGER_PRESSED;
            shot.update(fire, gamepad1.b, intake, eject, gamepad2.right_stick_x);

            // ------------------------------------------------ feedback
            if (shot.ready && !wasReady && (fire || shot.auto == ShotController.Auto.ARMED)) gamepad1.rumble(0.3, 0.3, (int) READY_RUMBLE_MS);
            wasReady = shot.ready;
            if (!rumbled60 && left <= RUMBLE_NECTAR_S) { gamepad1.rumbleBlips(1); gamepad2.rumbleBlips(1); rumbled60 = true; }
            if (!rumbled20 && left <= RUMBLE_PARK_S) { gamepad1.rumbleBlips(3); gamepad2.rumbleBlips(3); rumbled20 = true; }
            if (!rumbled5 && left <= RUMBLE_FINAL_S) { gamepad1.rumble(1000); rumbled5 = true; }

            robot.write();
            MatchState.save(robot.pose, shot.upCell, robot.turret.zeroTicks);

            // ------------------------------------------------ telemetry (rate-limited)
            if (robot.telemetryDue()) {
                telemetry.addData("Time left", "%.0f s", left);
                telemetry.addData("Up cell", shot.upCell);
                telemetry.addData("Mode", shot.manualShoot ? "MANUAL SHOOT" : shot.autoFire ? "auto-fire" : "fire button only");
                telemetry.addData("Ready", shot.ready ? "YES" : "no");
                telemetry.addData("Flywheel", "%.0f / %.0f rpm", robot.flywheel.rpm, robot.flywheel.targetRpm);
                if (TURRET_LOCKED) telemetry.addData("Turret", "locked at %.0f deg -- %s", robot.turret.positionDeg,
                        shot.sol.valid ? String.format(Locale.US, "turn robot %.0f deg %s", Math.abs(shot.aimErrDeg), shot.aimErrDeg > 0 ? "left" : "right") : "no shot from here");
                else telemetry.addData("Turret", "%.0f deg", robot.turret.positionDeg);
                telemetry.addData("Intake arm", robot.deploy.state);
                telemetry.update();
            }
        }
    }
}
