package org.firstinspires.ftc.teamcode.biobuzz.log;

import java.util.Locale;

/**
 * Splits each loop into phases so we can see exactly where the milliseconds go,
 * and keeps a running histogram for loop-time percentiles (no allocation per loop).
 *
 * Usage per loop: start(); ... mark(PINPOINT); ... mark(HUBS); ... end();
 * mark(p) adds the time since the previous mark to phase p.
 */
public class LoopTimer {
    public static final int PINPOINT = 0, HUBS = 1, SLOW = 2, DRIVE = 3, LOGIC = 4, WRITE = 5, TELEM = 6;
    public static final String[] NAMES = {"pinpoint", "hubs", "slow", "drive", "logic", "write", "telem"};
    public static final double TARGET_MS = 20.0;

    public final double[] phaseMs = new double[NAMES.length];
    /** Time between consecutive loop starts (the real loop period). */
    public double loopMs;
    /** Time spent inside start()..end() (excludes the SDK's own between-loop overhead). */
    public double workMs;

    private long loopStartNs = 0, lastMarkNs, prevLoopStartNs = 0;

    // 0.5 ms buckets up to 100 ms; the last bucket catches everything slower.
    private final int[] hist = new int[201];
    public long loops, over;
    public double maxMs;

    public void start() {
        long now = System.nanoTime();
        loopMs = prevLoopStartNs == 0 ? 0 : (now - prevLoopStartNs) / 1e6;
        prevLoopStartNs = now;
        loopStartNs = now;
        lastMarkNs = now;
        for (int i = 0; i < phaseMs.length; i++) phaseMs[i] = 0;
        if (loopMs > 0) {
            loops++;
            if (loopMs > TARGET_MS) over++;
            if (loopMs > maxMs) maxMs = loopMs;
            hist[(int) Math.min(hist.length - 1, loopMs * 2)]++;
        }
    }

    public void mark(int phase) {
        long now = System.nanoTime();
        phaseMs[phase] += (now - lastMarkNs) / 1e6;
        lastMarkNs = now;
    }

    public void end() {
        workMs = (System.nanoTime() - loopStartNs) / 1e6;
    }

    public double percentile(double p) {
        if (loops == 0) return 0;
        long target = (long) Math.ceil(p * loops), seen = 0;
        for (int i = 0; i < hist.length; i++) {
            seen += hist[i];
            if (seen >= target) return (i + 1) * 0.5;
        }
        return maxMs;
    }

    public String summary() {
        return String.format(Locale.US, "loops=%d p50=%.1fms p95=%.1fms p99=%.1fms max=%.1fms over%.0fms=%d (%.1f%%)",
                loops, percentile(0.5), percentile(0.95), percentile(0.99), maxMs, TARGET_MS, over,
                loops == 0 ? 0 : 100.0 * over / loops);
    }
}
