package org.firstinspires.ftc.teamcode.biobuzz.opmodes;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.MatchState;
import org.firstinspires.ftc.teamcode.biobuzz.ShotController;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;

import java.util.Locale;

/**
 * BIOBUZZ competition TeleOp. Works for BOTH alliances with no alliance switch: all
 * positions are in the alliance frame (see Field.java).
 *
 * GAMEPAD 1 (driver)
 *   left stick ........ drive (field-centric, up = away from our wall)
 *   right stick x ..... turn
 *   left bumper ....... slow mode (hold)
 *   right trigger ..... intake (hold)
 *   left trigger ...... eject (hold)
 *   right bumper ...... FIRE (hold) -- shoot now if >= 50% likely to score
 *   B (hold) .......... HOLD FIRE -- stop auto-fire (e.g. while collecting near opponents)
 *   Y (hold) .......... auto-drive to PARK in our LOADING ZONE (touch sticks to cancel)
 *   options ........... reset heading (robot facing straight away from our wall)
 *   share/back ........ relocalize: robot squared in the far-left corner (back on our wall)
 *
 * GAMEPAD 2 (operator)
 *   dpad left/right ... which CELL of our HIVE is UP (as seen from our drive station)
 *   right trigger ..... FIRE (hold)
 *   dpad up/down ...... RPM trim +/-25 (manual-RPM mode: +/-100)
 *   left/right bumper . aim trim +/-0.5 deg
 *   Y ................. clear trims
 *   A ................. flywheel pre-spin on/off
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

            telemetry.addLine("== BIOBUZZ TeleOp ==");
            telemetry.addData("Config", ConfigIO.lastStatus);
            telemetry.addData("Shot table", robot.shotTable.lastStatus);
            telemetry.addData("Zones", robot.zones.lastStatus);
            if (MatchState.fresh()) {
                telemetry.addData("Start from", useAuto ? "AUTO end pose (B = corner instead)" : "corner reset pose (A = auto pose)");
                telemetry.addData("Auto pose", fmtPose(MatchState.pose.x, MatchState.pose.y, MatchState.pose.h));
            } else {
                telemetry.addLine("No auto hand-off: robot must start in the far-left corner,");
                telemetry.addLine("back on our wall, TURRET POINTING STRAIGHT FORWARD.");
            }
            telemetry.addData("UP CELL", shot.upCell + "  (dpad left/right to change)");
            telemetry.update();
        }
        if (isStopRequested()) return;

        boolean fromAuto = robot.initPose(Field.resetPose(), useAuto);
        robot.startLog("TeleOp");
        robot.event("INIT from " + (fromAuto ? "auto hand-off" : "corner reset pose") + ", up cell " + shot.upCell);
        try {
            loop(robot, shot);
        } finally {
            robot.stopAll();
            robot.closeLog();
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
            double rpmStep = shot.manualRpm ? 100 : 25;
            if (gamepad2.dpadUpWasPressed()) shot.rpmTrim += rpmStep;
            if (gamepad2.dpadDownWasPressed()) shot.rpmTrim -= rpmStep;
            if (gamepad2.leftBumperWasPressed()) shot.aimTrimDeg += 0.5;
            if (gamepad2.rightBumperWasPressed()) shot.aimTrimDeg -= 0.5;
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
                robot.event("AUTO_FIRE " + (shot.autoFire ? "enabled" : "disabled"));
            }
            if (gamepad2.shareWasPressed() || gamepad2.backWasPressed()) {
                robot.event(String.format(Locale.US, "TURRET_REZERO was reading %.1f deg", robot.turret.positionDeg));
                robot.turret.zeroHere();
                shot.manualTurretDeg = 0;
                gamepad2.rumbleBlips(2);
            }

            // ------------------------------------------------ driver
            if (gamepad1.optionsWasPressed()) { robot.resetHeading(); gamepad1.rumbleBlips(1); }
            if (gamepad1.shareWasPressed() || gamepad1.backWasPressed()) {
                robot.relocalize(Field.resetPose());
                gamepad1.rumbleBlips(2);
            }

            boolean wantPark = gamepad1.y && !BuzzRobot.sticksActive(gamepad1);
            if (wantPark) {
                if (!parking) {
                    robot.drive.goToPoint(Field.parkPose(), false, 1.0);
                    robot.event("PARK start");
                    parking = true;
                }
            } else {
                if (parking) robot.event("PARK end, at park pose=" + robot.drive.atPoint());
                parking = false;
                robot.driveFieldCentric(gamepad1, gamepad1.left_bumper);
            }

            boolean fire = gamepad1.right_bumper || gamepad2.right_trigger > 0.5;
            boolean intake = gamepad1.right_trigger > 0.3;
            boolean eject = gamepad1.left_trigger > 0.3;
            shot.update(fire, gamepad1.b, intake, eject, gamepad2.right_stick_x);

            // ------------------------------------------------ feedback
            if (shot.ready && !wasReady && (fire || shot.auto == ShotController.Auto.ARMED)) gamepad1.rumble(0.3, 0.3, 80);
            wasReady = shot.ready;
            if (!rumbled60 && left <= 60) { gamepad1.rumbleBlips(1); gamepad2.rumbleBlips(1); rumbled60 = true; robot.event("60s left"); }
            if (!rumbled20 && left <= 20) { gamepad1.rumbleBlips(3); gamepad2.rumbleBlips(3); rumbled20 = true; robot.event("20s left"); }
            if (!rumbled5 && left <= 5) { gamepad1.rumble(1000); rumbled5 = true; }

            robot.write();
            MatchState.save(robot.pose, shot.upCell, robot.turret.zeroTicks);

            // ------------------------------------------------ telemetry (rate-limited)
            if (robot.telemetryDue()) {
                telemetry.addData("Time left", "%.0f s%s", left, left <= 20 ? "  >>> PARK (hold Y) <<<" : "");
                telemetry.addData("UP CELL", shot.upCell);
                telemetry.addData("AUTO-FIRE", "%s  (%.0f%% likely, need %.0f%%)", shot.auto,
                        shot.confidence * 100, AUTO_FIRE_MIN_CONFIDENCE * 100);
                telemetry.addData("Speed", "%.0f / %.0f in/s max for auto", shot.speed, SOTM_MAX_SPEED_IN_S);
                telemetry.addData("READY", shot.ready ? "YES" : "no - " + shot.notReadyText());
                telemetry.addData("Pieces", shot.hasPieces ? "probably (" + shot.shotsSinceLoad + " shot since intake)" : "EMPTY? intake to reset");
                telemetry.addData("Shots / jams", "%d / %d", robot.flywheel.shots, robot.feeder.jams);
                telemetry.addData("Dist (virtual/real)", "%.0f / %.0f in", shot.sol.distance, shot.sol.realDistance);
                telemetry.addData("Zone", shot.sol.zone == null ? "-" : shot.sol.zone.name);
                telemetry.addData("Flywheel", "%.0f / %.0f rpm", robot.flywheel.rpm, robot.flywheel.targetRpm);
                telemetry.addData("Turret", "%.1f / %.1f deg (tol %.1f)", robot.turret.positionDeg, robot.turret.targetDeg, shot.sol.tolDeg);
                telemetry.addData("Trim", "rpm %+.0f  aim %+.1f deg", shot.rpmTrim, shot.aimTrimDeg);
                telemetry.addData("Modes", "%s%s%s%s",
                        shot.prespin ? "PRESPIN " : "",
                        shot.shootOnMove ? "SOTM " : "",
                        shot.manualTurret ? "MANUAL-TURRET " : "",
                        shot.manualRpm ? "MANUAL-RPM" : "");
                telemetry.addData("Pose", fmtPose(robot.pose.x, robot.pose.y, robot.pose.h));
                telemetry.addData("Feeder", "%s%s", robot.feeder.mode, robot.feeder.unjamming ? " UNJAM" : "");
                telemetry.addData("Battery", "%.1f V", robot.volts);
                telemetry.addData("Loop", robot.loopHealth());
                telemetry.update();
            }
            robot.endLoop(shot);
        }
    }

    static String fmtPose(double x, double y, double h) {
        return String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg", x, y, Math.toDegrees(h));
    }
}
