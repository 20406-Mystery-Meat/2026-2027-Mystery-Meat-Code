package org.firstinspires.ftc.teamcode.biobuzz.shooting;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

/**
 * Projectile model for the fixed-hood shooter, WITH air drag (wiffle-style POLLEN balls
 * lose a lot of speed in flight, so drag-free math under-predicts time of flight).
 *
 * Used for:
 *  1. Time of flight vs distance -> shoot-on-the-move lead. With a fixed launch angle, the
 *     exit speed needed to hit a point depends only on geometry, so TOF depends only on
 *     distance. TOF_SCALE is a final multiplier tuned on the field (should end up near 1).
 *  2. Descent angle at the CELL: pieces must be coming DOWN into the up-facing CELL, so
 *     shots closer than MIN_DESCENT_DEG allows are reported as "too close".
 *  3. Seeding the shot table with first-guess RPMs before it's tuned.
 *
 * Integrating trajectories is too slow to do every loop, so a distance-indexed table
 * is precomputed once (and rebuilt automatically if a geometry/ball constant changes).
 * Per-loop calls are just a table interpolation.
 */
public final class Ballistics {
    private Ballistics() {}

    public static final double G = 9.80665;
    public static final double G_IN_PER_S2 = 386.09;
    private static final double IN = 0.0254;
    private static final double RHO_AIR = 1.2;

    public static final double TABLE_MIN_IN = 6, TABLE_STEP_IN = 2;
    public static final int TABLE_N = 118; // 6" .. 240"
    private static double[] speedTab, tofTab, descentTab;
    private static double builtKey = Double.NaN;

    // ------------------------------------------------------------------ public API (inches, seconds)

    /** Exit speed (in/s) to pass through a point d inches away and dz inches higher, NaN if impossible. */
    public static double requiredExitSpeed(double d, double dz) {
        return lookup(speedTabFor(dz), d);
    }

    /** Time of flight in seconds, including TOF_SCALE. NaN if the shot is impossible. */
    public static double timeOfFlight(double d, double dz) {
        return TOF_SCALE * lookup(tofTabFor(dz), d);
    }

    /** Angle below horizontal (deg) the piece is travelling when it reaches the CELL. */
    public static double descentDeg(double d, double dz) {
        return lookup(descentTabFor(dz), d);
    }

    /** True if a shot at d inches comes down steeply enough to drop into the up-facing CELL. */
    public static boolean shotPossible(double d, double dz) {
        double desc = descentDeg(d, dz);
        return !Double.isNaN(desc) && desc >= MIN_DESCENT_DEG;
    }

    /** First-guess main-wheel RPM for distance d (seeds the shot table). */
    public static double seedWheelRpm(double d, double dz) {
        double v = requiredExitSpeed(d, dz);
        if (Double.isNaN(v)) return Double.NaN;
        double wheelDiaIn = MAIN_WHEEL_DIAMETER_MM / 25.4;
        return (v / EXIT_SPEED_EFFICIENCY) * 60.0 / (Math.PI * wheelDiaIn);
    }

    public static double heightToTarget() {
        return CELL_AIM_HEIGHT_IN - LAUNCH_HEIGHT_IN;
    }

    // ------------------------------------------------------------------ trajectory math (SI units)

    /** Drag acceleration coefficient k (1/m): a_drag = -k |v| v. */
    public static double dragK() {
        double r = BALL_DIAMETER_IN * IN / 2;
        return 0.5 * RHO_AIR * BALL_CD * Math.PI * r * r / BALL_MASS_KG;
    }

    /**
     * Fly a piece launched at speed v (m/s) and the configured angle. Returns
     * {height (m) when it reaches horizontal distance dM, time (s), descent angle (deg)},
     * or null if it never gets that far.
     */
    public static double[] fly(double v, double dM) {
        double th = Math.toRadians(LAUNCH_ANGLE_DEG);
        double vx = v * Math.cos(th), vz = v * Math.sin(th);
        double x = 0, z = 0, t = 0, dt = 0.002, k = dragK();
        while (t < 4.0) {
            double sp = Math.hypot(vx, vz);
            double ax = -k * sp * vx, az = -G - k * sp * vz;
            double nvx = vx + ax * dt, nvz = vz + az * dt;
            double nx = x + 0.5 * (vx + nvx) * dt, nz = z + 0.5 * (vz + nvz) * dt;
            if (nx >= dM) {
                double f = (dM - x) / (nx - x);
                double zz = z + f * (nz - z), vvz = vz + f * (nvz - vz), vvx = vx + f * (nvx - vx);
                return new double[]{zz, t + f * dt, Math.toDegrees(Math.atan2(-vvz, vvx))};
            }
            if (nz < -2) return null; // hit the floor first
            vx = nvx; vz = nvz; x = nx; z = nz; t += dt;
        }
        return null;
    }

    /** Lowest-speed solution reaching (dM, dzM). Returns {v, t, descentDeg} or null. */
    public static double[] solve(double dM, double dzM) {
        double lo = 0.5, hi = 40;
        double[] hiRes = fly(hi, dM);
        if (hiRes == null || hiRes[0] < dzM) return null;
        double[] best = hiRes;
        for (int i = 0; i < 26; i++) { // ~0.6 mm/s resolution
            double mid = 0.5 * (lo + hi);
            double[] r = fly(mid, dM);
            if (r != null && r[0] >= dzM) { hi = mid; best = r; } else lo = mid;
        }
        return new double[]{hi, best[1], best[2]};
    }

    // ------------------------------------------------------------------ table

    private static double key(double dz) {
        return LAUNCH_ANGLE_DEG * 1e9 + dz * 1e5 + BALL_DIAMETER_IN * 1e3 + BALL_CD * 1e2 + BALL_MASS_KG * 1e4;
    }

    private static synchronized void build(double dz) {
        double k = key(dz);
        if (k == builtKey) return;
        double[] sp = new double[TABLE_N], tf = new double[TABLE_N], de = new double[TABLE_N];
        for (int i = 0; i < TABLE_N; i++) {
            double d = TABLE_MIN_IN + i * TABLE_STEP_IN;
            double[] s = solve(d * IN, dz * IN);
            sp[i] = s == null ? Double.NaN : s[0] / IN;
            tf[i] = s == null ? Double.NaN : s[1];
            de[i] = s == null ? Double.NaN : s[2];
        }
        speedTab = sp; tofTab = tf; descentTab = de;
        builtKey = k;
    }

    private static double[] speedTabFor(double dz) { build(dz); return speedTab; }
    private static double[] tofTabFor(double dz) { build(dz); return tofTab; }
    private static double[] descentTabFor(double dz) { build(dz); return descentTab; }

    private static double lookup(double[] tab, double d) {
        double f = (d - TABLE_MIN_IN) / TABLE_STEP_IN;
        if (f < 0 || f > TABLE_N - 1) return Double.NaN;
        int i = (int) Math.floor(f);
        if (i >= TABLE_N - 1) return tab[TABLE_N - 1];
        double a = tab[i], b = tab[i + 1];
        if (Double.isNaN(a) || Double.isNaN(b)) return Double.NaN;
        return a + (f - i) * (b - a);
    }

    /**
     * Stationary distance whose required exit speed is vInPerS (inverse of requiredExitSpeed on the
     * normal, far side of the curve). Lets a moving shot's exit speed be turned back into a shot-table
     * distance, since the table is distance -> rpm. NaN if out of range.
     */
    public static double distanceForExitSpeed(double vInPerS) {
        double[] sp = speedTabFor(heightToTarget());
        // find the slowest-shot distance, then search the increasing branch beyond it
        int iMin = 0;
        for (int i = 1; i < TABLE_N; i++) if (!Double.isNaN(sp[i]) && (Double.isNaN(sp[iMin]) || sp[i] < sp[iMin])) iMin = i;
        if (Double.isNaN(sp[iMin]) || vInPerS < sp[iMin]) return Double.NaN;
        for (int i = iMin; i < TABLE_N - 1; i++) {
            double a = sp[i], b = sp[i + 1];
            if (Double.isNaN(a) || Double.isNaN(b)) return Double.NaN;
            if (vInPerS >= a && vInPerS <= b) {
                return TABLE_MIN_IN + TABLE_STEP_IN * (i + (vInPerS - a) / (b - a));
            }
        }
        return Double.NaN;
    }

    /**
     * Fast 3D flight with drag (midpoint method, 10 ms steps -- good to well under an inch over a
     * 1.5 s flight). Initial velocity in in/s, field frame; z up. Fills out = {dx, dy (in), time (s),
     * descent angle (deg)} where the piece comes DOWN through height dzIn. Returns false if it never does.
     * No allocation: runs several times per loop while shooting on the move.
     */
    public static boolean flyTo(double vxIn, double vyIn, double vzIn, double dzIn, double[] out) {
        final double h = 0.01;
        double k = dragK();
        double vx = vxIn * IN, vy = vyIn * IN, vz = vzIn * IN, x = 0, y = 0, z = 0, dz = dzIn * IN;
        for (double t = 0; t < 3.0; t += h) {
            double sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
            double mvx = vx - 0.5 * h * k * sp * vx, mvy = vy - 0.5 * h * k * sp * vy, mvz = vz - 0.5 * h * (G + k * sp * vz);
            double msp = Math.sqrt(mvx * mvx + mvy * mvy + mvz * mvz);
            double nvx = vx - h * k * msp * mvx, nvy = vy - h * k * msp * mvy, nvz = vz - h * (G + k * msp * mvz);
            double nx = x + h * mvx, ny = y + h * mvy, nz = z + h * mvz;
            if (nvz < 0 && z >= dz && nz < dz) {
                double f = (z - dz) / (z - nz);
                out[0] = (x + f * (nx - x)) / IN;
                out[1] = (y + f * (ny - y)) / IN;
                out[2] = t + f * h;
                double hv = Math.hypot(vx + f * (nvx - vx), vy + f * (nvy - vy));
                out[3] = Math.toDegrees(Math.atan2(-(vz + f * (nvz - vz)), hv));
                return true;
            }
            if (nz < -0.5) return false;
            vx = nvx; vy = nvy; vz = nvz; x = nx; y = ny; z = nz;
        }
        return false;
    }

    /** Build the table now (call during init so the first loop isn't slow). */
    public static void warmUp() {
        build(heightToTarget());
    }
}
