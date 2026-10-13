package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.field.Field;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Hand-off from AUTO to TELEOP. Static fields survive between OpModes as long as the
 * Robot Controller app keeps running, so auto writes these at the end (every loop is
 * easiest) and teleop picks them up in init.
 */
public class MatchState {
    public static Pose2D pose = null;
    public static Field.Cell upCell = null;
    /** Turret encoder count that means "pointing forward", valid if turretZeroValid. */
    public static int turretZeroTicks = 0;
    public static boolean turretZeroValid = false;
    public static long savedAtMs = 0;

    /** Called every loop, so it reuses one Pose2D instead of allocating. */
    public static void save(Pose2D p, Field.Cell cell, int zeroTicks) {
        if (pose == null) pose = new Pose2D(p.x, p.y, p.h);
        else { pose.x = p.x; pose.y = p.y; pose.h = p.h; }
        upCell = cell;
        turretZeroTicks = zeroTicks;
        turretZeroValid = true;
        savedAtMs = System.currentTimeMillis();
    }

    /** Auto data is only trusted if it was saved recently (default: within 3 minutes). */
    public static boolean fresh() {
        return pose != null && System.currentTimeMillis() - savedAtMs < 3 * 60 * 1000;
    }

    public static void clear() {
        pose = null;
        upCell = null;
        turretZeroValid = false;
    }
}
