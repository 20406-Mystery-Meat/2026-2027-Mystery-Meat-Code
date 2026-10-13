package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;
import org.firstinspires.ftc.teamcode.biobuzz.ShotController;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Ballistics;

import java.util.Map;

/**
 * Build the distance -> RPM shot table, then tune shoot-on-the-move.
 * Tune the turret and flywheel first. Start in the far-left corner (back on our wall),
 * turret pointing forward. The turret auto-aims the whole time.
 *
 * GAMEPAD 1: drive (left stick / right stick x), left bumper slow, RT intake, LT eject.
 *   dpad up = last shot MADE, dpad down = last shot MISSED. These marks calibrate the
 *   "% likely to score" model: tools/analyze_log.py compares predicted vs actual.
 *
 * GAMEPAD 2 -- TABLE page (default):
 *   dpad left/right .. which CELL is up
 *   dpad up/down ..... rpm +/-25        bumpers: rpm -/+100
 *   right trigger .... FIRE (auto-gated on ready)
 *   A ................ record THIS distance -> current rpm (replaces a point within 3")
 *   B ................ delete the table point nearest this distance
 *   Y ................ toggle rpm source: MANUAL (what you're tuning) / TABLE (check it)
 *   X ................ save shot_table.csv
 *   back/share ....... switch to SOTM page
 * Park at 6-10 distances from close to far, find the rpm that drops pieces cleanly into
 * the CELL at each, press A, then X. Re-check with Y (TABLE) from in-between spots.
 *
 * GAMEPAD 2 -- SOTM page (shoot on the move):
 *   dpad up/down ..... TOF_SCALE +/-0.02   (pieces land BEHIND the cell -> increase it)
 *   A ................ auto-fire on/off (test the 85% auto-fire while driving around)
 *   right trigger .... FIRE while gamepad 1 drives past the HIVE at a steady speed
 *   options .......... save config (TOF_SCALE)
 *   back/share ....... back to TABLE page
 * Misses in the direction you're driving -> TOF_SCALE too big; trailing behind -> too small.
 */
@TeleOp(name = "Tune: Shot Table + SOTM", group = "BIOBUZZ Tuning")
public class ShotTableTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        BuzzRobot robot = new BuzzRobot(hardwareMap, telemetry);
        ShotController shot = new ShotController(robot);
        shot.autoFire = false; // tuners only shoot when you pull the trigger
        shot.prespin = true;
        double manualRpm = FLY_MANUAL_DEFAULT_RPM;
        boolean useTable = false;
        boolean sotmPage = false;

        telemetry.addLine("Far-left corner, back on our wall, turret forward. Then start.");
        telemetry.addData("Shot table", robot.shotTable.lastStatus);
        telemetry.update();
        waitForStart();
        robot.initPose(Field.resetPose(), false);
        robot.startLog("ShotTableTuner");

        try {
            while (opModeIsActive()) {
                robot.read();

                if (gamepad2.backWasPressed() || gamepad2.shareWasPressed()) sotmPage = !sotmPage;
                if (gamepad2.dpadLeftWasPressed() && !sotmPage) shot.upCell = Field.Cell.LEFT;
                if (gamepad2.dpadRightWasPressed() && !sotmPage) shot.upCell = Field.Cell.RIGHT;

                boolean markMade = gamepad1.dpadUpWasPressed(), markMissed = gamepad1.dpadDownWasPressed();
                if (markMade || markMissed) {
                    boolean made = markMade;
                    robot.event(String.format(java.util.Locale.US, "RESULT %s shot#%d conf=%.2f",
                            made ? "made" : "missed", robot.flywheel.shots, shot.lastShotConfidence));
                    gamepad1.rumbleBlips(made ? 1 : 2);
                }
                if (sotmPage) {
                    shot.shootOnMove = true;
                    if (gamepad2.aWasPressed()) shot.autoFire = !shot.autoFire;
                    shot.rpmOverride = Double.NaN;
                    if (gamepad2.dpadUpWasPressed()) TOF_SCALE += 0.02;
                    if (gamepad2.dpadDownWasPressed()) TOF_SCALE = Math.max(0.5, TOF_SCALE - 0.02);
                    if (gamepad2.optionsWasPressed()) { ConfigIO.save(); robot.event("SAVE TOF_SCALE=" + TOF_SCALE); }
                } else {
                    shot.shootOnMove = false;
                    if (gamepad2.dpadUpWasPressed()) manualRpm += 25;
                    if (gamepad2.dpadDownWasPressed()) manualRpm -= 25;
                    if (gamepad2.rightBumperWasPressed()) manualRpm += 100;
                    if (gamepad2.leftBumperWasPressed()) manualRpm -= 100;
                    manualRpm = Math.max(0, Math.min(FLY_MAX_WHEEL_RPM, manualRpm));
                    if (gamepad2.yWasPressed()) useTable = !useTable;
                    shot.rpmOverride = useTable ? Double.NaN : manualRpm;
                    if (gamepad2.aWasPressed() && shot.sol != null) {
                        robot.shotTable.record(shot.sol.realDistance, manualRpm);
                        robot.event(String.format(java.util.Locale.US, "TABLE_RECORD %.1f in -> %.0f rpm (pose %.1f, %.1f)",
                                shot.sol.realDistance, manualRpm, robot.pose.x, robot.pose.y));
                        gamepad2.rumbleBlips(1);
                    }
                    if (gamepad2.bWasPressed() && shot.sol != null) robot.shotTable.removeNearest(shot.sol.realDistance);
                    if (gamepad2.xWasPressed()) { robot.shotTable.save(); robot.event("TABLE_SAVE " + robot.shotTable.lastStatus); }
                }

                robot.driveFieldCentric(gamepad1, gamepad1.left_bumper);
                shot.update(gamepad2.right_trigger > 0.5, gamepad1.right_trigger > 0.3, gamepad1.left_trigger > 0.3, 0);
                robot.write();

                if (robot.telemetryDue()) {
                    telemetry.addData("PAGE", sotmPage ? "SOTM (back = table)" : "TABLE (back = SOTM)");
                    telemetry.addData("UP CELL", shot.upCell);
                    telemetry.addData("Distance real / virtual", "%.1f / %.1f in", shot.sol.realDistance, shot.sol.distance);
                    telemetry.addData("READY", shot.ready ? "YES" : "no - " + shot.notReadyText());
                    telemetry.addData("Likely to score", "%.0f%%  (mark last shot: gp1 dpad up=made, down=missed)", shot.confidence * 100);
                    telemetry.addData("Flywheel", "%.0f / %.0f rpm", robot.flywheel.rpm, robot.flywheel.targetRpm);
                    if (sotmPage) {
                        telemetry.addData("TOF_SCALE", "%.2f", TOF_SCALE);
                        telemetry.addData("Auto-fire (A)", "%s  %.0f%% likely  speed %.0f", shot.auto, shot.confidence * 100, shot.speed);
                        telemetry.addData("time of flight", "%.2f s", shot.sol.tof);
                        telemetry.addData("robot speed", "%.0f in/s", Math.hypot(robot.vx, robot.vy));
                        telemetry.addData("Config", ConfigIO.lastStatus);
                    } else {
                        telemetry.addData("RPM source", useTable ? "TABLE" : "MANUAL");
                        telemetry.addData("Manual rpm", "%.0f", manualRpm);
                        telemetry.addData("Table rpm here", "%.0f", robot.shotTable.rpmAt(shot.sol.realDistance));
                        // If THIS rpm scores from here, this is the shooter's real exit-speed efficiency.
                        // Put the average into EXIT_SPEED_EFFICIENCY so untuned distances seed well.
                        double vNeed = Ballistics.requiredExitSpeed(shot.sol.realDistance, Ballistics.heightToTarget());
                        double surface = manualRpm * Math.PI * (MAIN_WHEEL_DIAMETER_MM / 25.4) / 60.0;
                        telemetry.addData("Implied exit efficiency", "%.2f (config %.2f)", vNeed / surface, EXIT_SPEED_EFFICIENCY);
                        Map.Entry<Double, Double> near = robot.shotTable.nearest(shot.sol.realDistance);
                        if (near != null) telemetry.addData("Nearest point", "%.1f in -> %.0f rpm", near.getKey(), near.getValue());
                        telemetry.addData("Table", robot.shotTable.points.size() + " pts | " + robot.shotTable.lastStatus);
                        StringBuilder sb = new StringBuilder();
                        for (Map.Entry<Double, Double> e : robot.shotTable.points.entrySet()) {
                            sb.append(String.format("%.0f:%.0f ", e.getKey(), e.getValue()));
                        }
                        telemetry.addLine(sb.toString());
                    }
                    telemetry.addData("Pose", "x %.1f y %.1f h %.1f", robot.pose.x, robot.pose.y, Math.toDegrees(robot.pose.h));
                    telemetry.addData("Loop", robot.loopHealth());
                    telemetry.update();
                }
                robot.endLoop(shot);
            }
        } finally {
            robot.stopAll();
            robot.closeLog();
        }
    }
}
