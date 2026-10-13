package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Skips hardware writes that wouldn't change anything. Every setPower()/setPosition()
 * is a ~1 ms round trip to the hub, so re-sending the same value every loop is the
 * easiest loop time to win back.
 */
public final class HwCache {
    private HwCache() {}

    public static final double POWER_EPS = 0.005;

    public static class Power {
        private final DcMotorSimple motor;
        private double last = Double.NaN;
        public int writes;

        public Power(DcMotorSimple motor) {
            this.motor = motor;
        }

        public void set(double p) {
            // Always send an exact 0 so "stop" is never swallowed by the epsilon.
            boolean changed = Double.isNaN(last) || Math.abs(p - last) >= POWER_EPS || (p == 0 && last != 0);
            if (!changed) return;
            motor.setPower(p);
            last = p;
            writes++;
        }

        public void invalidate() {
            last = Double.NaN;
        }
    }

    public static class Position {
        private final Servo servo;
        private double last = Double.NaN;

        public Position(Servo servo) {
            this.servo = servo;
        }

        public void set(double pos) {
            if (pos == last) return;
            servo.setPosition(pos);
            last = pos;
        }

        public void invalidate() {
            last = Double.NaN;
        }
    }
}
