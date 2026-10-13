package org.firstinspires.ftc.teamcode.biobuzz.shooting;

import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/**
 * Distance (inches, turret to CELL aim point, horizontal) -> main-wheel RPM, with linear
 * interpolation. Tuned with the "Tune: Shot Table" OpMode and stored in
 * /sdcard/FIRST/biobuzz/shot_table.csv. Until that file exists, it's seeded from
 * {@link Ballistics} so the robot can at least roughly shoot out of the box.
 */
public class ShotTable {
    public static final File FILE = ConfigIO.file("shot_table.csv");

    public final TreeMap<Double, Double> points = new TreeMap<>();
    public String lastStatus = "";

    public static ShotTable seeded() {
        ShotTable t = new ShotTable();
        double dz = Ballistics.heightToTarget();
        for (double d = 24; d <= 168; d += 12) {
            double rpm = Ballistics.seedWheelRpm(d, dz);
            if (!Double.isNaN(rpm)) t.points.put(d, Math.rint(rpm));
        }
        return t;
    }

    public static ShotTable load() {
        if (!FILE.exists()) {
            ShotTable t = seeded();
            t.lastStatus = "no shot_table.csv, using physics seed (" + t.points.size() + " pts) - TUNE ME";
            return t;
        }
        ShotTable t = new ShotTable();
        try (BufferedReader r = new BufferedReader(new FileReader(FILE))) {
            String line;
            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("distance")) continue;
                String[] p = line.split(",");
                try {
                    t.points.put(Double.parseDouble(p[0].trim()), Double.parseDouble(p[1].trim()));
                } catch (RuntimeException ignored) {
                    // skip bad line
                }
            }
        } catch (IOException e) {
            ShotTable s = seeded();
            s.lastStatus = "shot_table.csv read failed, using physics seed";
            return s;
        }
        if (t.points.size() < 2) {
            ShotTable s = seeded();
            s.points.putAll(t.points);
            s.lastStatus = "shot_table.csv has <2 points, merged with physics seed";
            return s;
        }
        t.lastStatus = "loaded " + t.points.size() + " points";
        return t;
    }

    public boolean save() {
        //noinspection ResultOfMethodCallIgnored
        ConfigIO.DIR.mkdirs();
        try (Writer w = new FileWriter(FILE)) {
            w.write("distance_in,wheel_rpm\n");
            for (Map.Entry<Double, Double> e : points.entrySet()) {
                w.write(String.format(Locale.US, "%.1f,%.0f\n", e.getKey(), e.getValue()));
            }
            lastStatus = "saved " + points.size() + " points";
            return true;
        } catch (IOException e) {
            lastStatus = "save failed: " + e.getMessage();
            return false;
        }
    }

    /** Interpolated RPM. Beyond the ends it extrapolates from the last two points. NaN if empty. */
    public double rpmAt(double d) {
        if (points.isEmpty()) return Double.NaN;
        if (points.size() == 1) return points.firstEntry().getValue();
        Map.Entry<Double, Double> lo = points.floorEntry(d);
        Map.Entry<Double, Double> hi = points.ceilingEntry(d);
        if (lo == null) {
            lo = points.firstEntry();
            hi = points.higherEntry(lo.getKey());
        } else if (hi == null) {
            hi = points.lastEntry();
            lo = points.lowerEntry(hi.getKey());
        }
        if (lo.getKey().equals(hi.getKey())) return lo.getValue();
        double t = (d - lo.getKey()) / (hi.getKey() - lo.getKey());
        return lo.getValue() + t * (hi.getValue() - lo.getValue());
    }

    /** rpm per inch of distance at d (slope of the interpolated table). */
    public double slopeAt(double d) {
        if (points.size() < 2) return Double.NaN;
        return (rpmAt(d + 2) - rpmAt(d - 2)) / 4.0;
    }

    public boolean inRange(double d) {
        return !points.isEmpty() && d >= points.firstKey() - 6 && d <= points.lastKey() + 6;
    }

    /** Record a tuned point, replacing any point within 3 inches. */
    public void record(double d, double rpm) {
        Map.Entry<Double, Double> near = nearest(d);
        if (near != null && Math.abs(near.getKey() - d) < 3) points.remove(near.getKey());
        points.put(Math.rint(d * 10) / 10, Math.rint(rpm));
    }

    public void removeNearest(double d) {
        Map.Entry<Double, Double> near = nearest(d);
        if (near != null) points.remove(near.getKey());
    }

    public Map.Entry<Double, Double> nearest(double d) {
        Map.Entry<Double, Double> lo = points.floorEntry(d), hi = points.ceilingEntry(d);
        if (lo == null) return hi;
        if (hi == null) return lo;
        return (d - lo.getKey() <= hi.getKey() - d) ? lo : hi;
    }
}
