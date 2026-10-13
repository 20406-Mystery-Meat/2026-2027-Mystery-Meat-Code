package org.firstinspires.ftc.teamcode.biobuzz.log;

import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

/**
 * Low-overhead CSV logger built for a sub-20 ms control loop.
 *
 * The loop thread only copies numbers into a pre-allocated double[] row and hands it to a
 * queue (no string formatting, no file I/O, no allocation). A background writer thread
 * does all the formatting and disk writes. If the writer ever falls behind, rows are
 * DROPPED (and counted) instead of ever blocking the loop.
 *
 * Files (in /sdcard/FIRST/biobuzz/logs/):
 *   yyyyMMdd_HHmmss_OpMode.csv      one row per loop; header lines starting with '#' hold a
 *                                   snapshot of the config, shot table and zones
 *   yyyyMMdd_HHmmss_OpMode_events.txt  timestamped discrete events (shots, jams, mode changes...)
 */
public class DataLog {
    public static final File LOG_DIR = ConfigIO.file("logs");
    /** Oldest logs beyond this many runs are deleted at startup so the hub never fills up. */
    public static final int KEEP_RUNS = 60;
    private static final int POOL_ROWS = 2048;

    public final String baseName;
    public final File csvFile, eventFile;
    private final String[] cols;
    private final int[] decimals;
    private final ArrayBlockingQueue<double[]> free = new ArrayBlockingQueue<>(POOL_ROWS);
    private final ArrayBlockingQueue<double[]> full = new ArrayBlockingQueue<>(POOL_ROWS);
    private final ConcurrentLinkedQueue<String> events = new ConcurrentLinkedQueue<>();
    private final List<String> header;
    private volatile boolean running = true;
    private volatile int dropped = 0;
    private volatile long rowsWritten = 0;
    private volatile String error = null;
    private final Thread thread;
    private String footer = null;

    public DataLog(String opModeName, String[] cols, int[] decimals, List<String> headerComments) {
        this.cols = cols;
        this.decimals = decimals;
        this.header = headerComments;
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        baseName = stamp + "_" + opModeName.replaceAll("[^A-Za-z0-9_-]", "");
        csvFile = new File(LOG_DIR, baseName + ".csv");
        eventFile = new File(LOG_DIR, baseName + "_events.txt");
        for (int i = 0; i < POOL_ROWS; i++) free.offer(new double[cols.length]);
        thread = new Thread(this::writerLoop, "BuzzDataLog");
        thread.setPriority(Thread.NORM_PRIORITY - 1);
        thread.setDaemon(true);
        thread.start();
    }

    public int columnCount() {
        return cols.length;
    }

    /** Get an empty row to fill, or null if the writer is behind (row is dropped). */
    public double[] obtain() {
        double[] r = free.poll();
        if (r == null) dropped++;
        return r;
    }

    public void push(double[] row) {
        if (!full.offer(row)) {
            dropped++;
            free.offer(row);
        }
    }

    /** Rare discrete events. Allocates a String, so don't call it every loop. */
    public void event(double tS, String msg) {
        events.offer(String.format(Locale.US, "%9.3f  %s", tS, msg));
    }

    public int dropped() {
        return dropped;
    }

    public String error() {
        return error;
    }

    public long rowsWritten() {
        return rowsWritten;
    }

    /** Flushes everything and stops the writer. Blocks up to ~1.5 s. */
    public void close(String summary) {
        footer = summary;
        running = false;
        try {
            thread.join(1500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // ------------------------------------------------------------------ writer thread

    private void writerLoop() {
        //noinspection ResultOfMethodCallIgnored
        LOG_DIR.mkdirs();
        pruneOldLogs();
        try (Writer csv = new BufferedWriter(new FileWriter(csvFile), 1 << 16);
             Writer ev = new BufferedWriter(new FileWriter(eventFile), 1 << 12)) {
            for (String h : header) csv.write("# " + h + "\n");
            csv.write(String.join(",", cols));
            csv.write('\n');

            StringBuilder sb = new StringBuilder(1024);
            long lastFlush = System.nanoTime();
            while (running || !full.isEmpty() || !events.isEmpty()) {
                double[] r = full.poll(50, TimeUnit.MILLISECONDS);
                if (r != null) {
                    sb.setLength(0);
                    for (int i = 0; i < r.length; i++) {
                        if (i > 0) sb.append(',');
                        appendFixed(sb, r[i], decimals[i]);
                    }
                    sb.append('\n');
                    csv.append(sb);
                    rowsWritten++;
                    free.offer(r);
                }
                String e;
                while ((e = events.poll()) != null) {
                    ev.write(e);
                    ev.write('\n');
                }
                if (System.nanoTime() - lastFlush > 1_000_000_000L) {
                    csv.flush();
                    ev.flush();
                    lastFlush = System.nanoTime();
                }
            }
            if (footer != null) {
                ev.write(footer);
                ev.write('\n');
            }
            ev.write("rows=" + rowsWritten + " dropped=" + dropped + "\n");
        } catch (IOException | InterruptedException ex) {
            error = ex.getMessage();
        }
    }

    private void pruneOldLogs() {
        File[] files = LOG_DIR.listFiles((d, n) -> n.endsWith(".csv"));
        if (files == null || files.length <= KEEP_RUNS) return;
        Arrays.sort(files, (a, b) -> a.getName().compareTo(b.getName()));
        for (int i = 0; i < files.length - KEEP_RUNS; i++) {
            String base = files[i].getName().replace(".csv", "");
            //noinspection ResultOfMethodCallIgnored
            files[i].delete();
            //noinspection ResultOfMethodCallIgnored
            new File(LOG_DIR, base + "_events.txt").delete();
        }
    }

    /** Fast fixed-point formatting without String.format. */
    static void appendFixed(StringBuilder sb, double v, int dec) {
        if (Double.isNaN(v)) { sb.append("nan"); return; }
        if (Double.isInfinite(v)) { sb.append(v > 0 ? "inf" : "-inf"); return; }
        if (dec <= 0) { sb.append(Math.round(v)); return; }
        long scale = 1;
        for (int i = 0; i < dec; i++) scale *= 10;
        long n = Math.round(Math.abs(v) * scale);
        if (v < 0 && n != 0) sb.append('-');
        sb.append(n / scale).append('.');
        long frac = n % scale;
        for (long s = scale / 10; s > frac && s > 1; s /= 10) sb.append('0');
        sb.append(frac);
    }
}
