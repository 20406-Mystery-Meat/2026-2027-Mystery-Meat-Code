package org.firstinspires.ftc.teamcode.biobuzz.field;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * BIOBUZZ field geometry (Competition Manual section 9) in the ALLIANCE FRAME.
 *
 * ALLIANCE FRAME: origin at field center, +x points away from your own alliance
 * wall (forward for your drivers), +y is your drivers' left, heading 0 faces +x,
 * CCW positive, inches. Because the field is 180-degree rotationally symmetric,
 * red and blue share every number in this file -- there is no alliance mirroring
 * anywhere in this code. Just start the localizer in this frame.
 *
 * Field facts used (manual TU04):
 *  - 144" x 144" field; HIVE Structure at the center.
 *  - Red HIVE sits on the red side, blue HIVE on the blue side, 25.5" center-to-center.
 *  - Each HIVE has two CELLS, one toward the audience and one toward the far wall.
 *    Exactly one is facing up at any time; it TIPS to the other side when enough
 *    POLLEN/NECTAR are LAUNCHED into the up-facing CELL. Only that CELL scores a TIP.
 *  - CELL opening: 20" wide, its bottom/top are 53.5" / 65.6" above the tiles.
 *  - LOADING ZONE (PARK): 23" x 11" on your own wall, between the tile seams 24" and 48"
 *    to your drivers' left of center.
 *  - Max 4 SCORING ELEMENTS controlled at once (G407).
 */
public final class Field {
    private Field() {}

    public static final double HALF_FIELD_IN = 72;

    /** Which CELL of OUR HIVE is facing up, as seen by our drivers standing at our wall. */
    public enum Cell {
        /** Up CELL on the drivers' left (+y). Red: far-wall side. Blue: audience side. */
        LEFT,
        /** Up CELL on the drivers' right (-y). Red: audience side. Blue: far-wall side. */
        RIGHT;

        public double sign() {
            return this == LEFT ? 1 : -1;
        }

        public Cell other() {
            return this == LEFT ? RIGHT : LEFT;
        }
    }

    /**
     * At match start the CELL that "points at a FLOWER" is tilted down (manual 10.3.1). From the
     * field drawings that leaves the drivers'-RIGHT CELL facing up for both alliances.
     * VERIFY THIS on a real field and flip it here if it's wrong.
     */
    public static Cell startingUpCell() {
        return Cell.RIGHT;
    }

    /** Aim point (x, y) of the up-facing CELL opening. Height is CELL_AIM_HEIGHT_IN. */
    public static double cellX(Cell cell) {
        return HIVE_X_IN;
    }

    public static double cellY(Cell cell) {
        return cell.sign() * CELL_Y_IN;
    }

    /** Robot pose that counts as PARKED (at least partly in our LOADING ZONE). */
    public static Pose2D parkPose() {
        return new Pose2D(PARK_X_IN, PARK_Y_IN, Math.toRadians(PARK_HEADING_DEG));
    }

    /**
     * Relocalize spot: robot back flat against OUR alliance wall and its left side flat against
     * the left side wall (the far-left corner as seen from our drivers), facing +x.
     */
    public static Pose2D resetPose() {
        return new Pose2D(-HALF_FIELD_IN + ROBOT_HALF_LENGTH_IN,
                HALF_FIELD_IN - ROBOT_HALF_WIDTH_IN, 0);
    }

    /** True if the robot center is on our side of the HIVE structure (closer to our wall). */
    public static boolean onOurHalf(Pose2D pose) {
        return pose.x < 0;
    }
}
