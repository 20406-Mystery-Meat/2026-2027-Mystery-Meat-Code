package org.firstinspires.ftc.teamcode.biobuzz.shooting;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import org.firstinspires.ftc.teamcode.biobuzz.field.AimZone;
import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.biobuzz.field.ZoneMap;
import org.firstinspires.ftc.teamcode.neuronpathing.MathFunc;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Turns "where is the robot, how is it moving, which CELL is up" into a turret angle,
 * a turret feedforward rate, and a flywheel RPM.
 *
 * Shoot on the move: a piece launched from a moving robot keeps the robot's velocity.
 * So we aim at a VIRTUAL target = real target - (shooter velocity x time of flight).
 * TOF depends on distance to the virtual target, so we iterate a few times.
 * The flywheel RPM also comes from the distance to the virtual target, which
 * automatically shoots softer when driving toward the HIVE and harder driving away.
 */
public class ShotSolver {
    public static class Solution {
        public boolean valid;
        public String invalidReason = "";
        /** 0 valid, 1 too close, 2 outside table, 3 rpm > max, 4 spinning (for logs). */
        public int invalidCode;
        /** Turret angle relative to the robot front, degrees (before choosing a wrap within limits). */
        public double turretDeg;
        /** Turret angular rate needed to stay on target, deg/s (feedforward). */
        public double turretRateDegS;
        public double wheelRpm;
        /** Horizontal distance from turret to (virtual) target, inches. */
        public double distance;
        public double realDistance;
        public double tof;
        public double tolDeg;
        /** Alliance-frame direction the turret shoots (to the virtual target), rad. */
        public double shotAngle;
        /** Flywheel "at speed" tolerance for this distance (~RANGE_TOL_IN of landing error). */
        public double rpmTol;
        public double virtualX, virtualY;
        public AimZone zone;
        public int zoneIndex = -1;
        /** True if the moving-shot aim was refined with the full drag trajectory. */
        public boolean dragCorrected;
        /** Descent angle at the CELL for the refined moving shot (deg), NaN if not refined. */
        public double descentDeg;
    }

    public final ShotTable table;
    public final ZoneMap zones;
    private final Solution s = new Solution();

    public ShotSolver(ShotTable table, ZoneMap zones) {
        this.table = table;
        this.zones = zones;
    }

    /**
     * @param pose        robot pose, alliance frame (in, rad)
     * @param vx          robot field velocity x (in/s)
     * @param vy          robot field velocity y (in/s)
     * @param omega       robot angular velocity (rad/s, CCW +)
     * @param cell        which CELL of our HIVE is up
     * @param moving      apply shoot-on-the-move compensation
     * @param rpmTrim     operator RPM trim
     * @param aimTrimDeg  operator turret trim (deg, CCW +)
     */
    public Solution solve(Pose2D pose, double vx, double vy, double omega,
                          Field.Cell cell, boolean moving, double rpmTrim, double aimTrimDeg) {
        double cos = Math.cos(pose.h), sin = Math.sin(pose.h);
        // Turret pivot in the field, and its velocity (robot velocity + spin about the robot center).
        double rx = TURRET_OFFSET_X_IN * cos - TURRET_OFFSET_Y_IN * sin;
        double ry = TURRET_OFFSET_X_IN * sin + TURRET_OFFSET_Y_IN * cos;
        double tx = pose.x + rx, ty = pose.y + ry;
        double tvx = vx - omega * ry, tvy = vy + omega * rx;

        s.zone = zones.find(pose.x, pose.y, cell);
        s.zoneIndex = s.zone == null ? -1 : zones.zones.indexOf(s.zone);
        double goalX = Field.cellX(cell) + (s.zone != null ? s.zone.aimDx : 0);
        double goalY = Field.cellY(cell) + (s.zone != null ? s.zone.aimDy : 0);
        double dz = Ballistics.heightToTarget();

        s.realDistance = Math.hypot(goalX - tx, goalY - ty);
        double vX = goalX, vY = goalY;
        double tof = Ballistics.timeOfFlight(s.realDistance, dz);
        if (moving && !Double.isNaN(tof)) {
            for (int i = 0; i < SOTM_ITERATIONS; i++) {
                vX = goalX - tvx * tof;
                vY = goalY - tvy * tof;
                double t = Ballistics.timeOfFlight(Math.hypot(vX - tx, vY - ty), dz);
                if (Double.isNaN(t)) break;
                tof = t;
            }
        }
        s.virtualX = vX;
        s.virtualY = vY;
        s.tof = tof;

        double dx = vX - tx, dy = vY - ty;
        s.distance = Math.hypot(dx, dy);
        double fieldAngle = Math.atan2(dy, dx);
        s.dragCorrected = false;
        s.descentDeg = Double.NaN;
        // The virtual-target lead ignores that the robot's velocity also changes the piece's drag
        // (several inches off at 40+ in/s). Refine by flying the real trajectory and correcting.
        if (moving && Math.hypot(tvx, tvy) > 2 && !Double.isNaN(tof) && refine(tx, ty, tvx, tvy, goalX, goalY, dz, fieldAngle, s.distance)) {
            fieldAngle = rAngle;
            s.distance = rDist;
            s.tof = rTof;
            s.descentDeg = rDescent;
            s.virtualX = tx + rDist * Math.cos(rAngle);
            s.virtualY = ty + rDist * Math.sin(rAngle);
            s.dragCorrected = true;
            dx = s.virtualX - tx;
            dy = s.virtualY - ty;
        }
        s.shotAngle = fieldAngle;
        double trim = aimTrimDeg + (s.zone != null ? s.zone.turretTrimDeg : 0);
        s.turretDeg = Math.toDegrees(MathFunc.angleWrap(fieldAngle - pose.h)) - TURRET_FORWARD_OFFSET_DEG + trim;

        // d(fieldAngle)/dt for a fixed target seen from a moving turret, minus the robot's own spin.
        double r2 = Math.max(dx * dx + dy * dy, 1);
        double fieldRate = (dx * (-tvy) - dy * (-tvx)) / r2;
        s.turretRateDegS = Math.toDegrees(fieldRate - omega);

        s.wheelRpm = table.rpmAt(s.distance) + rpmTrim + (s.zone != null ? s.zone.rpmOffset : 0);
        double slope = Math.abs(table.slopeAt(s.distance));
        s.rpmTol = Double.isNaN(slope) ? FLY_READY_TOL_RPM
                : clamp(RANGE_TOL_IN * slope, FLY_TOL_MIN_RPM, FLY_READY_TOL_RPM);
        // Sideways room in the CELL depends on the shot direction (opening is ~20" x 12" from above).
        s.tolDeg = clamp(Math.toDegrees(Math.atan2(0.6 * ShotConfidence.lateralHalfWidth(fieldAngle), s.distance)),
                TURRET_TOL_MIN_DEG, TURRET_TOL_MAX_DEG);

        boolean steepEnough = s.dragCorrected ? s.descentDeg >= MIN_DESCENT_DEG : Ballistics.shotPossible(s.distance, dz);
        if (Double.isNaN(tof) || !steepEnough) {
            s.valid = false;
            s.invalidReason = "too close for hood angle";
            s.invalidCode = 1;
        } else if (!table.inRange(s.distance)) {
            s.valid = false;
            s.invalidReason = "outside shot table";
            s.invalidCode = 2;
        } else if (s.wheelRpm > FLY_MAX_WHEEL_RPM) {
            s.valid = false;
            s.invalidReason = "needs more RPM than max";
            s.invalidCode = 3;
        } else if (Math.abs(Math.toDegrees(omega)) > MAX_SHOT_TURN_RATE_DEG_S) {
            s.valid = false;
            s.invalidReason = "spinning too fast";
            s.invalidCode = 4;
        } else {
            s.valid = true;
            s.invalidReason = "";
            s.invalidCode = 0;
        }
        s.wheelRpm = clamp(s.wheelRpm, 0, FLY_MAX_WHEEL_RPM);
        return s;
    }

    // ---- drag-corrected moving shot (results in r* fields; no allocation)
    private final double[] fl = new double[4];
    private double rAngle, rDist, rTof, rDescent;

    /**
     * Find exit speed V and turret direction psi so that a piece launched from (tx, ty) with the
     * robot's velocity (scaled by TOF_SCALE, the field-tuned lead multiplier) comes down through
     * the aim height exactly on (gx, gy). Newton's method on the 2x2 system, starting from the
     * virtual-target answer; ~10 fast trajectory integrations. Returns false to keep the old answer.
     */
    private boolean refine(double tx, double ty, double tvx, double tvy, double gx, double gy, double dz,
                           double psi0, double d0) {
        double V = Ballistics.requiredExitSpeed(d0, dz), psi = psi0;
        if (Double.isNaN(V)) return false;
        double c = Math.cos(Math.toRadians(LAUNCH_ANGLE_DEG)), sn = Math.sin(Math.toRadians(LAUNCH_ANGLE_DEG));
        double lvx = tvx * TOF_SCALE, lvy = tvy * TOF_SCALE;
        for (int it = 0; it < 4; it++) {
            if (!Ballistics.flyTo(V * c * Math.cos(psi) + lvx, V * c * Math.sin(psi) + lvy, V * sn, dz, fl)) return false;
            double ex = tx + fl[0] - gx, ey = ty + fl[1] - gy;
            rTof = fl[2];
            rDescent = fl[3];
            if (ex * ex + ey * ey < 0.04) break; // within 0.2"
            double hV = V * 0.01, hP = 0.004;
            if (!Ballistics.flyTo((V + hV) * c * Math.cos(psi) + lvx, (V + hV) * c * Math.sin(psi) + lvy, (V + hV) * sn, dz, fl)) return false;
            double aV = (tx + fl[0] - gx - ex) / hV, bV = (ty + fl[1] - gy - ey) / hV;
            if (!Ballistics.flyTo(V * c * Math.cos(psi + hP) + lvx, V * c * Math.sin(psi + hP) + lvy, V * sn, dz, fl)) return false;
            double aP = (tx + fl[0] - gx - ex) / hP, bP = (ty + fl[1] - gy - ey) / hP;
            double det = aV * bP - aP * bV;
            if (Math.abs(det) < 1e-9) return false;
            V -= (ex * bP - aP * ey) / det;
            psi -= (aV * ey - ex * bV) / det;
            if (V <= 0 || Double.isNaN(V)) return false;
        }
        double dEq = Ballistics.distanceForExitSpeed(V);
        if (Double.isNaN(dEq)) return false;
        rAngle = psi;
        rDist = dEq;
        return true;
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
