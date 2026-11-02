package org.firstinspires.ftc.teamcode.neuronpathing;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Wraps the goBILDA Pinpoint odometry computer and exposes it as a Neuron Pathing
 * Pose2D source. The Pinpoint driver ships with the FTC SDK itself (Hardware
 * module), so no extra dependency is needed beyond the stock FtcRobotController.
 *
 * Configure xOffset/yOffset/podType/encoderDirections for your robot in the
 * constructor call in your OpMode or drive class -- these are NOT hardcoded
 * here, unlike a from-scratch Pinpoint wrapper, since every robot's pod
 * placement is different.
 */
public class NeuronLocalizer {
    private final Telemetry telemetry;
    public final GoBildaPinpointDriver pinpoint;

    public NeuronLocalizer(HardwareMap hwMap, Telemetry telemetry, String deviceName,
                            double xOffsetMm, double yOffsetMm,
                            GoBildaPinpointDriver.GoBildaOdometryPods podType,
                            GoBildaPinpointDriver.EncoderDirection xEncoderDirection,
                            GoBildaPinpointDriver.EncoderDirection yEncoderDirection) {
        this.telemetry = telemetry;
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, deviceName);

        pinpoint.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(podType);
        pinpoint.setEncoderDirections(xEncoderDirection, yEncoderDirection);
    }

    /**
     * Everything from Constants.java (SECTIONS 1-2): device name, pod offsets, pod resolution and
     * encoder directions. Use this one -- then the Localizer Test, autos and TeleOp all agree.
     */
    public NeuronLocalizer(HardwareMap hwMap, Telemetry telemetry) {
        this(hwMap, telemetry, PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM);
    }

    /** Constants for everything except the pod offsets. */
    public NeuronLocalizer(HardwareMap hwMap, Telemetry telemetry, double xOffsetMm, double yOffsetMm) {
        this(hwMap, telemetry, PINPOINT, xOffsetMm, yOffsetMm,
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(PINPOINT_TICKS_PER_MM, DistanceUnit.MM);
    }

    public Pose2D getPose() {
        pinpoint.update();
        return new Pose2D(
                pinpoint.getPosition().getX(DistanceUnit.INCH),
                pinpoint.getPosition().getY(DistanceUnit.INCH),
                pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
    }

    public double getAngularVelocityDegPerSec() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public double velX() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double velY() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    public void setPosition(Pose2D pos) {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D sdkPose =
                new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                        DistanceUnit.INCH, pos.x, pos.y, AngleUnit.RADIANS, pos.h);
        pinpoint.setPosition(sdkPose);
        pinpoint.update();
    }

    /**
     * Resets position/IMU (resetPos = true) or just recalibrates the IMU, then
     * blocks (briefly) until the Pinpoint reports READY. Call this once during
     * init, after setPosition() with your starting auto pose.
     */
    public void calibrate(boolean resetPos) {
        if (resetPos) {
            pinpoint.resetPosAndIMU();
        } else {
            pinpoint.recalibrateIMU();
        }

        for (int i = 0; i < 20; i++) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                return;
            }
        }
    }
}
