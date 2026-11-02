package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Localizer sanity check / tuning aid. Push the robot around by hand (motors
 * are never commanded here) and confirm x/y/heading track what you expect:
 * x increases driving away from your start, y increases strafing left,
 * heading increases turning counter-clockwise. If any of those are backwards,
 * fix PINPOINT_X/Y_REVERSED and the pod offsets in Constants.java, not the math.
 */
@TeleOp(name = "Neuron Localizer Test")
public class NeuronLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry); // offsets/pods/directions: Constants.java
        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);

        waitForStart();

        while (opModeIsActive()) {
            Pose2D pose = localizer.getPose();
            telemetry.addData("x (in)", pose.x);
            telemetry.addData("y (in)", pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.h));
            telemetry.update();
        }
    }
}
