package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import java.util.SortedMap;
import java.util.TreeMap;

/**
 * Robot-side overrides for {@link Constants}, stored in /sdcard/FIRST/biobuzz/config.properties.
 *
 * Rules, so editing Constants.java always does what you expect:
 *  - Every OpMode starts from the values written in Constants.java (the "code values").
 *  - The file only holds values a tuner SAVED that differ from the code value, each with
 *    the code value it was saved against ("KEY.code").
 *  - A saved value is used only while Constants.java still has that same code value. If you
 *    edit the number in Constants.java and redeploy, your edit wins and the stale saved
 *    value is ignored (and listed as IGNORED on the init screen).
 * Unknown keys are ignored and missing keys keep their code values, so adding or removing
 * a constant never breaks loading.
 */
public class ConfigIO {
    public static final File DIR = new File(AppUtil.FIRST_FOLDER, "biobuzz");
    public static final File CONFIG_FILE = new File(DIR, "config.properties");

    /** Code values of every tunable, captured once -- before anything (a file, a tuner) changes them. */
    private static final SortedMap<String, String> CODE = snapshot();

    /** Short status for telemetry, e.g. "2 saved overrides". */
    public static String lastStatus = "not loaded";
    /** Values the robot is using from the saved file instead of Constants.java. */
    public static final List<String> overrides = new ArrayList<>();
    /** Saved values skipped because Constants.java changed since they were saved. */
    public static final List<String> ignored = new ArrayList<>();

    public static File file(String name) {
        return new File(DIR, name);
    }

    /** Reset every tunable to its Constants.java value, then apply the still-valid saved overrides. */
    public static void load() {
        overrides.clear();
        ignored.clear();
        for (Field f : tunables()) set(f, CODE.get(f.getName()));
        if (!CONFIG_FILE.exists()) {
            lastStatus = "no saved overrides - using Constants.java";
            return;
        }
        Properties props = new Properties();
        try (Reader r = new FileReader(CONFIG_FILE)) {
            props.load(r);
        } catch (IOException e) {
            lastStatus = "couldn't read saved overrides (" + e.getMessage() + ") - using Constants.java";
            return;
        }
        for (Field f : tunables()) {
            String name = f.getName();
            String saved = props.getProperty(name);
            if (saved == null) continue;
            saved = saved.trim();
            String code = CODE.get(name);
            String savedAgainst = props.getProperty(name + ".code");
            if (savedAgainst != null && !same(f, savedAgainst.trim(), code)) {
                ignored.add(name + ": Constants.java now " + code + " (saved " + saved + " was for " + savedAgainst.trim() + ")");
                continue;
            }
            if (same(f, saved, code)) continue;
            if (set(f, saved)) overrides.add(name + " = " + saved + "  (Constants.java: " + code + ")");
        }
        lastStatus = overrides.size() + " saved override" + (overrides.size() == 1 ? "" : "s")
                + (ignored.isEmpty() ? "" : ", " + ignored.size() + " IGNORED (Constants.java changed)");
    }

    /** Save every value that now differs from Constants.java (values back at their code value are dropped). */
    public static boolean save() {
        Properties props = new Properties();
        int n = 0;
        for (Field f : tunables()) {
            String now = get(f), code = CODE.get(f.getName());
            if (same(f, now, code)) continue;
            props.setProperty(f.getName(), now);
            props.setProperty(f.getName() + ".code", code);
            n++;
        }
        //noinspection ResultOfMethodCallIgnored
        DIR.mkdirs();
        try (Writer w = new FileWriter(CONFIG_FILE)) {
            props.store(w, "BIOBUZZ robot overrides of Constants.java (KEY.code = the Constants.java value it was"
                    + " saved against). To change a value for good, edit Constants.java instead.");
        } catch (IOException e) {
            lastStatus = "save failed: " + e.getMessage();
            return false;
        }
        // Refresh the override list without disturbing the values in memory (they ARE the saved ones).
        load();
        lastStatus = "saved " + n + " override" + (n == 1 ? "" : "s");
        return true;
    }

    /** Delete all saved overrides and go back to Constants.java. */
    public static boolean clear() {
        boolean ok = !CONFIG_FILE.exists() || CONFIG_FILE.delete();
        load();
        lastStatus = ok ? "cleared - using Constants.java" : "couldn't delete " + CONFIG_FILE;
        return ok;
    }

    /** Every tunable's current value, sorted by name (written into each log's header). */
    public static SortedMap<String, String> snapshot() {
        TreeMap<String, String> m = new TreeMap<>();
        for (Field f : tunables()) m.put(f.getName(), get(f));
        return m;
    }

    /** For telemetry: every override and ignored value, one per line. */
    public static List<String> describe() {
        List<String> lines = new ArrayList<>();
        for (String o : overrides) lines.add("override  " + o);
        for (String i : ignored) lines.add("IGNORED   " + i);
        return lines;
    }

    // ------------------------------------------------------------------ helpers

    private static String get(Field f) {
        try {
            return String.valueOf(f.get(null));
        } catch (IllegalAccessException e) {
            return "?";
        }
    }

    private static boolean set(Field f, String v) {
        if (v == null) return false;
        try {
            if (f.getType() == double.class) f.setDouble(null, Double.parseDouble(v));
            else if (f.getType() == int.class) f.setInt(null, Integer.parseInt(v));
            else if (f.getType() == boolean.class) f.setBoolean(null, Boolean.parseBoolean(v));
            return true;
        } catch (Exception e) {
            return false; // bad value in the file: keep the code value
        }
    }

    /** Numeric comparison, so "0.6" and "0.60" (or "1" and "1.0") count as the same value. */
    private static boolean same(Field f, String a, String b) {
        if (a == null || b == null) return a == null && b == null;
        try {
            if (f.getType() == boolean.class) return Boolean.parseBoolean(a) == Boolean.parseBoolean(b);
            return Double.parseDouble(a) == Double.parseDouble(b);
        } catch (NumberFormatException e) {
            return a.equals(b);
        }
    }

    private static List<Field> tunables() {
        List<Field> out = new ArrayList<>();
        for (Field f : Constants.class.getFields()) {
            int m = f.getModifiers();
            if (!Modifier.isStatic(m) || Modifier.isFinal(m)) continue;
            Class<?> t = f.getType();
            if (t == double.class || t == int.class || t == boolean.class) out.add(f);
        }
        return out;
    }
}
