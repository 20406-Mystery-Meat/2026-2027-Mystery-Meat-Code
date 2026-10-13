package org.firstinspires.ftc.teamcode.biobuzz.tuning;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig;

import java.lang.reflect.Field;

/**
 * Edits a handful of BuzzConfig doubles from a gamepad:
 *   bumpers ........... select parameter
 *   dpad right/left ... x1.05 / /1.05   (fine)
 *   X / B ............. x1.5  / /1.5    (coarse)
 *   left stick button . set to 0 (a 0 value grows from a small seed when increased)
 * Save with ConfigIO.save() from the OpMode.
 */
public class ParamEditor {
    private final String[] names;
    private final Field[] fields;
    public int selected = 0;

    public ParamEditor(String... names) {
        this.names = names;
        fields = new Field[names.length];
        for (int i = 0; i < names.length; i++) {
            try {
                fields[i] = BuzzConfig.class.getField(names[i]);
            } catch (NoSuchFieldException e) {
                throw new IllegalArgumentException("No BuzzConfig field " + names[i]);
            }
        }
    }

    public double get(int i) {
        try {
            return fields[i].getDouble(null);
        } catch (IllegalAccessException e) {
            return Double.NaN;
        }
    }

    public void set(int i, double v) {
        try {
            fields[i].setDouble(null, v);
        } catch (IllegalAccessException ignored) {
        }
    }

    private void scale(double k) {
        double v = get(selected);
        if (v == 0) v = k > 1 ? 1e-4 : 0;
        else v *= k;
        set(selected, v);
    }

    public void handle(Gamepad g) {
        if (g.rightBumperWasPressed()) selected = (selected + 1) % names.length;
        if (g.leftBumperWasPressed()) selected = (selected + names.length - 1) % names.length;
        if (g.dpadRightWasPressed()) scale(1.05);
        if (g.dpadLeftWasPressed()) scale(1 / 1.05);
        if (g.xWasPressed()) scale(1.5);
        if (g.bWasPressed()) scale(1 / 1.5);
        if (g.leftStickButtonWasPressed()) set(selected, 0);
    }

    /** "KP=0.025 KD=0.0012 ..." for log events. */
    public String describe() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < names.length; i++) {
            if (i > 0) sb.append(' ');
            sb.append(names[i]).append('=').append(String.format(java.util.Locale.US, "%.5g", get(i)));
        }
        return sb.toString();
    }

    public void show(Telemetry t) {
        for (int i = 0; i < names.length; i++) {
            t.addData((i == selected ? ">> " : "   ") + names[i], "%.6g", get(i));
        }
    }
}
