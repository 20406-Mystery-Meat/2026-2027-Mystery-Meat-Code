package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;

/**
 * Manual TeleOp on top of Neuron Pathing. Drive with the left stick's y/x and
 * rotate with the right stick's x -- robot-centric, no field-oriented math, so
 * it's a good first sanity check that your motors/directions are wired
 * correctly before layering in autonomous paths.
 *
 * Motor names/directions and Pinpoint setup come from Constants.java.
 */
@TeleOp(name = "Neuron TeleOp (Basic)")
public class NeuronTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry); // Constants.java
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer);        // Constants.java

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.update();

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.update();
        }
    }
}
