package org.firstinspires.ftc.teamcode.biobuzz.shooting;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import java.util.Random;

/**
 * "How likely is this shot to go in?" -- estimated every loop, used to decide auto-fire.
 *
 * Every error source is modeled as a mean offset or a spread (1 sigma, inches at the CELL),
 * in the direction it actually pushes the piece:
 *
 *   along the shot (range):   flywheel rpm error right now (mean) + rpm noise, shot-to-shot scatter
 *   across the shot (lateral): turret error right now (mean) + turret noise, heading error,
 *                              extra turret error while it's turning fast, shot-to-shot scatter
 *   every direction:           odometry position error (grows with distance driven since the last
 *                              relocalize), velocity-measurement noise x time of flight
 *   along the robot's motion:  velocity error that grows with speed, time-of-flight model error,
 *                              release-timing jitter -- this is why fast shots get less certain
 *   along its acceleration:    leftover error from predicting velocity through the release delay
 *   along the shot, extra:     when driving toward the CELL (flatter arc = more range-sensitive)
 *
 * The landing spot must fall inside the up-facing CELL opening seen from above
 * (HIT_HALF_X_IN x HIT_HALF_Y_IN in the alliance frame, already shrunk by the ball radius).
 *
 * The probability is the fraction of N fixed pseudo-random error samples that land inside.
 * The samples are generated once, so the answer is smooth and repeatable (no flicker around
 * the 85% threshold), and N=512 costs only ~20 microseconds per loop.
 *
 * The spreads are first guesses. They get CALIBRATED from real shots: mark made/missed in the
 * Shot Table tuner and tools/analyze_log.py compares predicted vs actual hit rates.
 */
public class ShotConfidence {
    private static final int N = 512;
    private static final double[] Z1 = new double[N], Z2 = new double[N], Z3 = new double[N], Z4 = new double[N];

    static {
        Random r = new Random(20261012L);
        for (int i = 0; i < N; i++) {
            Z1[i] = r.nextGaussian();
            Z2[i] = r.nextGaussian();
            Z3[i] = r.nextGaussian();
            Z4[i] = r.nextGaussian();
        }
    }

    // ---- last result (for telemetry / logs)
    public double p;
    public double meanRange, meanLat, sigmaRange, sigmaLat, sigmaMove, sigmaAccel;

    /**
     * @param d              horizontal distance turret -> (virtual) aim point, in
     * @param tof            time of flight, s
     * @param shotAngle      direction the turret is shooting, alliance-frame radians
     * @param rpmPerIn       shot-table slope (rpm per inch of distance)
     * @param rpmErr         flywheel target - actual (filtered), rpm. Use 0 for "if the wheel were at speed".
     * @param turretErrDeg   turret target - actual, deg
     * @param turretRateDegS how fast the turret is being asked to turn, deg/s
     * @param vx,vy          robot field velocity, in/s
     * @param ax,ay          robot field acceleration, in/s^2
     * @param traveledIn     distance driven since the last relocalize, in
     */
    public double compute(double d, double tof, double shotAngle, double rpmPerIn, double rpmErr,
                          double turretErrDeg, double turretRateDegS,
                          double vx, double vy, double ax, double ay, double traveledIn) {
        if (Double.isNaN(d) || Double.isNaN(tof)) {
            p = 0;
            return 0;
        }
        double slope = Math.max(Math.abs(rpmPerIn), 3);
        double speed = Math.hypot(vx, vy), accel = Math.hypot(ax, ay);

        // mean offsets right now
        meanRange = -rpmErr / slope;                          // slow wheel -> short
        meanLat = -d * Math.tan(Math.toRadians(turretErrDeg)); // turret behind target -> lands right

        // spreads
        double pos = POSE_SIGMA_IN + POSE_DRIFT_FRAC * traveledIn;
        double iso2 = pos * pos + sq(VEL_SIGMA_IN_S * tof);
        double turretSig = TURRET_SIGMA_DEG + TURRET_RATE_SIGMA * Math.abs(turretRateDegS);
        // Driving TOWARD the CELL flattens the trajectory, so range errors grow (tools/sim: the plain model
        // was 5-7 points optimistic there).
        double c0 = Math.cos(shotAngle), s0 = Math.sin(shotAngle);
        double closing = Math.max(0, vx * c0 + vy * s0);
        sigmaRange = Math.sqrt(sq(DISP_RANGE_IN) + sq(DISP_RANGE_FRAC * d) + sq(FLY_SIGMA_RPM / slope) + iso2
                + sq(TOWARD_SIGMA_PER_IN_S * closing));
        sigmaLat = Math.sqrt(sq(DISP_LAT_IN) + sq(DISP_LAT_FRAC * d)
                + sq(d * Math.tan(Math.toRadians(turretSig)))
                + sq(d * Math.tan(Math.toRadians(HEADING_SIGMA_DEG))) + iso2);
        sigmaMove = Math.sqrt(sq(VEL_SIGMA_FRAC * speed * tof) + sq(TOF_SIGMA_FRAC * tof * speed)
                + sq(RELEASE_JITTER_S * speed));
        sigmaAccel = ACCEL_SIGMA_FRAC * accel * SHOT_LATENCY_S * tof;

        double c = Math.cos(shotAngle), s = Math.sin(shotAngle);
        double cv = speed > 1e-6 ? vx / speed : 1, sv = speed > 1e-6 ? vy / speed : 0;
        double ca = accel > 1e-6 ? ax / accel : 1, sa = accel > 1e-6 ? ay / accel : 0;

        int hits = 0;
        for (int i = 0; i < N; i++) {
            double er = meanRange + sigmaRange * Z1[i];
            double el = meanLat + sigmaLat * Z2[i];
            double em = sigmaMove * Z3[i];
            double ea = sigmaAccel * Z4[i];
            double dx = er * c - el * s + em * cv + ea * ca;
            double dy = er * s + el * c + em * sv + ea * sa;
            if (Math.abs(dx) <= HIT_HALF_X_IN && Math.abs(dy) <= HIT_HALF_Y_IN) hits++;
        }
        p = hits / (double) N;
        return p;
    }

    /**
     * Half-width of the CELL opening measured sideways to a shot in direction shotAngle
     * (center to edge of the HIT_HALF_X_IN x HIT_HALF_Y_IN rectangle along the lateral axis).
     */
    public static double lateralHalfWidth(double shotAngle) {
        double ux = Math.abs(-Math.sin(shotAngle)), uy = Math.abs(Math.cos(shotAngle));
        double tx = ux < 1e-6 ? Double.MAX_VALUE : HIT_HALF_X_IN / ux;
        double ty = uy < 1e-6 ? Double.MAX_VALUE : HIT_HALF_Y_IN / uy;
        return Math.min(tx, ty);
    }

    private static double sq(double v) {
        return v * v;
    }
}
