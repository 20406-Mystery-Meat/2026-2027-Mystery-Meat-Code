package org.firstinspires.ftc.teamcode.biobuzz.field;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A region of the field (polygon, alliance frame) that adjusts how the robot shoots
 * while the robot center is inside it. Use zones to correct spots where the
 * distance-only shot table isn't quite right (bounce-outs off the CELL lip, shooting
 * past the HIVE frame, etc.).
 *
 * File line format (one zone per line, '#' comments allowed):
 *   name | cell | x1,y1; x2,y2; x3,y3 ... | aimDx | aimDy | rpmOffset | turretTrimDeg
 * cell is ANY, LEFT or RIGHT -- the zone only applies while that CELL is the up CELL.
 */
public class AimZone {
    public enum CellFilter { ANY, LEFT, RIGHT }

    public String name;
    public CellFilter cell = CellFilter.ANY;
    public final List<double[]> polygon = new ArrayList<>();
    /** Moves the aim point (inches, alliance frame). +dx = deeper (away from us), +dy = drivers' left. */
    public double aimDx, aimDy;
    public double rpmOffset;
    public double turretTrimDeg;

    public AimZone(String name) {
        this.name = name;
    }

    public static AimZone rect(String name, double x1, double y1, double x2, double y2) {
        AimZone z = new AimZone(name);
        double minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
        double minY = Math.min(y1, y2), maxY = Math.max(y1, y2);
        z.polygon.add(new double[]{minX, minY});
        z.polygon.add(new double[]{maxX, minY});
        z.polygon.add(new double[]{maxX, maxY});
        z.polygon.add(new double[]{minX, maxY});
        return z;
    }

    public boolean appliesTo(Field.Cell upCell) {
        return cell == CellFilter.ANY
                || (cell == CellFilter.LEFT && upCell == Field.Cell.LEFT)
                || (cell == CellFilter.RIGHT && upCell == Field.Cell.RIGHT);
    }

    /** Ray-casting point-in-polygon. */
    public boolean contains(double x, double y) {
        boolean inside = false;
        int n = polygon.size();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon.get(i)[0], yi = polygon.get(i)[1];
            double xj = polygon.get(j)[0], yj = polygon.get(j)[1];
            if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
                inside = !inside;
            }
        }
        return inside;
    }

    public String toLine() {
        StringBuilder pts = new StringBuilder();
        for (int i = 0; i < polygon.size(); i++) {
            if (i > 0) pts.append("; ");
            pts.append(String.format(Locale.US, "%.1f,%.1f", polygon.get(i)[0], polygon.get(i)[1]));
        }
        return String.format(Locale.US, "%s | %s | %s | %.2f | %.2f | %.0f | %.2f",
                name, cell, pts, aimDx, aimDy, rpmOffset, turretTrimDeg);
    }

    /** Returns null if the line is blank/comment/malformed. */
    public static AimZone fromLine(String line) {
        line = line.trim();
        if (line.isEmpty() || line.startsWith("#")) return null;
        String[] parts = line.split("\\|");
        if (parts.length < 7) return null;
        try {
            AimZone z = new AimZone(parts[0].trim());
            z.cell = CellFilter.valueOf(parts[1].trim().toUpperCase(Locale.US));
            for (String p : parts[2].split(";")) {
                String[] xy = p.trim().split(",");
                z.polygon.add(new double[]{Double.parseDouble(xy[0].trim()), Double.parseDouble(xy[1].trim())});
            }
            if (z.polygon.size() < 3) return null;
            z.aimDx = Double.parseDouble(parts[3].trim());
            z.aimDy = Double.parseDouble(parts[4].trim());
            z.rpmOffset = Double.parseDouble(parts[5].trim());
            z.turretTrimDeg = Double.parseDouble(parts[6].trim());
            return z;
        } catch (RuntimeException e) {
            return null;
        }
    }
}
