package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

/**
 * Speed from encoder position over a longer time window than the hub uses.
 *
 * The 6000 rpm motor's encoder has only 28 counts/rev, so the hub's own velocity reading
 * jumps in steps of ~29 main-wheel rpm. That's as big as the tolerance an accurate shot
 * needs (~25-45 rpm, see tools/sim). Averaging position change over ~100 ms halves the
 * step size and noise, at the cost of ~50 ms lag -- fine for deciding "ready to fire",
 * while the controller keeps using the faster raw reading.
 *
 * Fixed-size ring buffer: no allocation per loop.
 */
public class VelocityWindow {
    private final long[] tNs;
    private final double[] pos;
    private int head = 0, count = 0;

    public VelocityWindow(int capacity) {
        tNs = new long[capacity];
        pos = new double[capacity];
    }

    public void add(long nowNs, double position) {
        tNs[head] = nowNs;
        pos[head] = position;
        head = (head + 1) % tNs.length;
        if (count < tNs.length) count++;
    }

    /** Units of position per second over (at least) windowS, or NaN until there's enough history. */
    public double rate(double windowS) {
        if (count < 2) return Double.NaN;
        int newest = (head - 1 + tNs.length) % tNs.length;
        long windowNs = (long) (windowS * 1e9);
        int idx = newest;
        for (int k = 1; k < count; k++) {
            int i = (newest - k + tNs.length) % tNs.length;
            idx = i;
            if (tNs[newest] - tNs[i] >= windowNs) break;
        }
        long dt = tNs[newest] - tNs[idx];
        return dt <= 0 ? Double.NaN : (pos[newest] - pos[idx]) / (dt / 1e9);
    }

    public void clear() {
        count = 0;
        head = 0;
    }
}
