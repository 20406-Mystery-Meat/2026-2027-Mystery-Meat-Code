package org.firstinspires.ftc.teamcode.biobuzz.field;

import org.firstinspires.ftc.teamcode.biobuzz.ConfigIO;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

/**
 * The list of {@link AimZone}s, stored in /sdcard/FIRST/biobuzz/zones.txt.
 * The first zone (in file order) that contains the robot and matches the up CELL wins,
 * so put small special-case zones ABOVE the big default bands.
 */
public class ZoneMap {
    public static final File FILE = ConfigIO.file("zones.txt");

    public final List<AimZone> zones = new ArrayList<>();
    public String lastStatus = "";

    /** Starting layout: 3 depth bands x 2 sides, all with zero correction. */
    public static ZoneMap defaults() {
        ZoneMap m = new ZoneMap();
        m.zones.add(AimZone.rect("BACK-L", -72, 0, -36, 72));
        m.zones.add(AimZone.rect("BACK-R", -72, -72, -36, 0));
        m.zones.add(AimZone.rect("MID-L", -36, 0, 0, 72));
        m.zones.add(AimZone.rect("MID-R", -36, -72, 0, 0));
        m.zones.add(AimZone.rect("FAR-L", 0, 0, 72, 72));
        m.zones.add(AimZone.rect("FAR-R", 0, -72, 72, 0));
        return m;
    }

    public static ZoneMap load() {
        if (!FILE.exists()) {
            ZoneMap m = defaults();
            m.lastStatus = "no zones.txt, using " + m.zones.size() + " default zones";
            return m;
        }
        ZoneMap m = new ZoneMap();
        int bad = 0;
        try (BufferedReader r = new BufferedReader(new FileReader(FILE))) {
            String line;
            while ((line = r.readLine()) != null) {
                String t = line.trim();
                if (t.isEmpty() || t.startsWith("#")) continue;
                AimZone z = AimZone.fromLine(t);
                if (z != null) m.zones.add(z);
                else bad++;
            }
        } catch (IOException e) {
            ZoneMap d = defaults();
            d.lastStatus = "zones.txt read failed, using defaults";
            return d;
        }
        m.lastStatus = "loaded " + m.zones.size() + " zones" + (bad > 0 ? " (" + bad + " bad lines skipped)" : "");
        return m;
    }

    public boolean save() {
        //noinspection ResultOfMethodCallIgnored
        ConfigIO.DIR.mkdirs();
        try (Writer w = new FileWriter(FILE)) {
            w.write("# name | cell(ANY/LEFT/RIGHT) | x,y; x,y; ... | aimDx | aimDy | rpmOffset | turretTrimDeg\n");
            w.write("# Alliance frame: +x away from our wall, +y drivers' left. First match wins.\n");
            for (AimZone z : zones) w.write(z.toLine() + "\n");
            lastStatus = "saved " + zones.size() + " zones";
            return true;
        } catch (IOException e) {
            lastStatus = "save failed: " + e.getMessage();
            return false;
        }
    }

    /** The active zone at (x, y) for this up CELL, or null if none. */
    public AimZone find(double x, double y, Field.Cell upCell) {
        for (int i = 0; i < zones.size(); i++) { // indexed: no Iterator allocation per loop
            AimZone z = zones.get(i);
            if (z.appliesTo(upCell) && z.contains(x, y)) return z;
        }
        return null;
    }
}
