package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;

/**
 * Shows which Constants.java values this robot is overriding with values saved by the tuners,
 * and lets you wipe them so the robot runs exactly what's in Constants.java.
 *
 *   Y, then Y again within 3 s ... delete ALL saved overrides (shot table and zones are kept)
 *
 * Moves no motors.
 */
@TeleOp(name = "Config: Saved Overrides", group = "BIOBUZZ Tuning")
public class ConfigOverrides extends LinearOpMode {
    @Override
    public void runOpMode() {
        ConfigIO.load();
        double armedAt = -10;
        while (opModeInInit() || opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                if (getRuntime() - armedAt < 3) {
                    ConfigIO.clear();
                    armedAt = -10;
                    gamepad1.rumbleBlips(2);
                } else {
                    armedAt = getRuntime();
                }
            }
            telemetry.addLine("Values this robot uses INSTEAD of Constants.java:");
            telemetry.addData("Status", ConfigIO.lastStatus);
            for (String line : ConfigIO.describe()) telemetry.addLine(line);
            if (ConfigIO.overrides.isEmpty() && ConfigIO.ignored.isEmpty()) {
                telemetry.addLine("(none - the robot runs exactly what's in Constants.java)");
            }
            telemetry.addLine();
            telemetry.addLine("IGNORED = Constants.java was changed after that value was saved,");
            telemetry.addLine("so your code edit is being used.");
            telemetry.addLine();
            telemetry.addLine(getRuntime() - armedAt < 3 ? ">>> press Y AGAIN to delete all overrides <<<"
                    : "Y twice = delete all saved overrides");
            telemetry.addData("File", ConfigIO.CONFIG_FILE.getPath());
            telemetry.update();
            sleep(50);
        }
    }
}
