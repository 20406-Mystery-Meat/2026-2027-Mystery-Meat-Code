package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.ShotController;
import org.firstinspires.ftc.teamcode.biobuzz.field.AimZone;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;

/**
 * Create aim zones and tune their corrections by actually shooting from them.
 * Tune the shot table first -- zones are for fixing specific spots, not for everything.
 * Start in the far-left corner (back on our wall), turret pointing forward.
 *
 * GAMEPAD 1: drive, left bumper slow, RT intake, LT eject, dpad left/right = which CELL is up.
 *
 * GAMEPAD 2 (edits the zone the robot is standing in):
 *   right trigger ....... FIRE (auto-gated)
 *   dpad up/down ........ aim point deeper / shorter   (aimDx +/-0.5")
 *   dpad left/right ..... aim point left / right        (aimDy +/-0.5", from our drivers' view)
 *   bumpers ............. rpm offset -/+25
 *   stick buttons L/R ... turret trim -/+0.25 deg
 *   X ................... cycle this zone's CELL filter (ANY -> LEFT -> RIGHT)
 *   A ................... mark corner 1 at the robot's position
 *   B ................... mark corner 2 -> creates a new rectangle zone (highest priority)
 *   back/share .......... delete the zone you're standing in
 *   options ............. save zones.txt
 */
@TeleOp(name = "Tune: Aim Zones", group = "BIOBUZZ Tuning")
public class ZoneTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        BuzzRobot robot = new BuzzRobot(hardwareMap, telemetry);
        ShotController shot = new ShotController(robot);
        shot.autoFire = false; // tuners only shoot when you pull the trigger
        shot.prespin = true;
        double[] corner1 = null;
        int newCount = 0;

        telemetry.addLine("Far-left corner, back on our wall, turret forward. Then start.");
        telemetry.addData("Zones", robot.zones.lastStatus);
        telemetry.update();
        waitForStart();
        robot.initPose(Field.resetPose(), false);
        robot.startLog("ZoneTuner");

        try {
            while (opModeIsActive()) {
                robot.read();
                if (gamepad1.dpadLeftWasPressed()) shot.upCell = Field.Cell.LEFT;
                if (gamepad1.dpadRightWasPressed()) shot.upCell = Field.Cell.RIGHT;

                AimZone z = robot.zones.find(robot.pose.x, robot.pose.y, shot.upCell);
                if (z != null) {
                    if (gamepad2.dpadUpWasPressed()) z.aimDx += 0.5;
                    if (gamepad2.dpadDownWasPressed()) z.aimDx -= 0.5;
                    if (gamepad2.dpadLeftWasPressed()) z.aimDy += 0.5;
                    if (gamepad2.dpadRightWasPressed()) z.aimDy -= 0.5;
                    if (gamepad2.rightBumperWasPressed()) z.rpmOffset += 25;
                    if (gamepad2.leftBumperWasPressed()) z.rpmOffset -= 25;
                    if (gamepad2.rightStickButtonWasPressed()) z.turretTrimDeg += 0.25;
                    if (gamepad2.leftStickButtonWasPressed()) z.turretTrimDeg -= 0.25;
                    if (gamepad2.xWasPressed()) {
                        z.cell = AimZone.CellFilter.values()[(z.cell.ordinal() + 1) % AimZone.CellFilter.values().length];
                    }
                    if (gamepad2.backWasPressed() || gamepad2.shareWasPressed()) robot.zones.zones.remove(z);
                }
                if (gamepad2.aWasPressed()) {
                    corner1 = new double[]{robot.pose.x, robot.pose.y};
                    gamepad2.rumbleBlips(1);
                }
                if (gamepad2.bWasPressed() && corner1 != null) {
                    newCount++;
                    robot.zones.zones.add(0, AimZone.rect("Z" + newCount, corner1[0], corner1[1], robot.pose.x, robot.pose.y));
                    robot.event("ZONE_NEW " + robot.zones.zones.get(0).toLine());
                    corner1 = null;
                    gamepad2.rumbleBlips(2);
                }
                if (gamepad2.optionsWasPressed()) {
                    robot.zones.save();
                    for (AimZone zz : robot.zones.zones) robot.event("ZONE_SAVE " + zz.toLine());
                }

                robot.driveFieldCentric(gamepad1, gamepad1.left_bumper);
                shot.update(gamepad2.right_trigger > 0.5, gamepad1.right_trigger > 0.3, gamepad1.left_trigger > 0.3, 0);
                robot.write();

                if (robot.telemetryDue()) {
                    telemetry.addData("UP CELL", shot.upCell);
                    telemetry.addData("ZONE", z == null ? "(none)" : z.name + "  cell=" + z.cell);
                    if (z != null) {
                        telemetry.addData("aimDx / aimDy", "%+.1f / %+.1f in", z.aimDx, z.aimDy);
                        telemetry.addData("rpm offset / trim", "%+.0f / %+.2f deg", z.rpmOffset, z.turretTrimDeg);
                    }
                    telemetry.addData("corner 1", corner1 == null ? "-" : String.format("%.1f, %.1f", corner1[0], corner1[1]));
                    telemetry.addData("READY", shot.ready ? "YES" : "no - " + shot.notReadyText());
                    telemetry.addData("Distance", "%.1f in", shot.sol.distance);
                    telemetry.addData("Flywheel", "%.0f / %.0f rpm", robot.flywheel.rpm, robot.flywheel.targetRpm);
                    telemetry.addData("Pose", "x %.1f y %.1f h %.1f", robot.pose.x, robot.pose.y, Math.toDegrees(robot.pose.h));
                    telemetry.addData("Zones", robot.zones.zones.size() + " | " + robot.zones.lastStatus);
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
