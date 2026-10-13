package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Properties;

/**
 * Saves/loads every public static non-final double/int/boolean in {@link BuzzConfig}
 * to /sdcard/FIRST/biobuzz/config.properties. Unknown keys in the file are ignored and
 * missing keys keep their code defaults, so adding a new constant never breaks loading.
 */
public class ConfigIO {
    public static final File DIR = new File(AppUtil.FIRST_FOLDER, "biobuzz");
    public static final File CONFIG_FILE = new File(DIR, "config.properties");

    /** Short status for telemetry, e.g. "loaded 63 values". */
    public static String lastStatus = "not loaded";

    public static File file(String name) {
        return new File(DIR, name);
    }

    public static void load() {
        if (!CONFIG_FILE.exists()) {
            lastStatus = "no config file, using code defaults";
            return;
        }
        Properties props = new Properties();
        try (Reader r = new FileReader(CONFIG_FILE)) {
            props.load(r);
        } catch (IOException e) {
            lastStatus = "load failed: " + e.getMessage();
            return;
        }
        int count = 0;
        for (Field f : tunables()) {
            String v = props.getProperty(f.getName());
            if (v == null) continue;
            try {
                if (f.getType() == double.class) f.setDouble(null, Double.parseDouble(v.trim()));
                else if (f.getType() == int.class) f.setInt(null, Integer.parseInt(v.trim()));
                else if (f.getType() == boolean.class) f.setBoolean(null, Boolean.parseBoolean(v.trim()));
                count++;
            } catch (Exception ignored) {
                // bad value in the file: keep the default
            }
        }
        lastStatus = "loaded " + count + " values";
    }

    public static boolean save() {
        Properties props = new Properties();
        try {
            for (Field f : tunables()) {
                props.setProperty(f.getName(), String.valueOf(f.get(null)));
            }
        } catch (IllegalAccessException e) {
            lastStatus = "save failed: " + e.getMessage();
            return false;
        }
        //noinspection ResultOfMethodCallIgnored
        DIR.mkdirs();
        try (Writer w = new FileWriter(CONFIG_FILE)) {
            props.store(w, "BIOBUZZ turret robot config");
            lastStatus = "saved " + props.size() + " values";
            return true;
        } catch (IOException e) {
            lastStatus = "save failed: " + e.getMessage();
            return false;
        }
    }

    /** Every tunable's current value, sorted by name (written into each log's header). */
    public static java.util.SortedMap<String, String> snapshot() {
        java.util.TreeMap<String, String> m = new java.util.TreeMap<>();
        for (Field f : tunables()) {
            try {
                m.put(f.getName(), String.valueOf(f.get(null)));
            } catch (IllegalAccessException ignored) {
            }
        }
        return m;
    }

    private static java.util.List<Field> tunables() {
        java.util.List<Field> out = new java.util.ArrayList<>();
        for (Field f : BuzzConfig.class.getFields()) {
            int m = f.getModifiers();
            if (!Modifier.isStatic(m) || Modifier.isFinal(m)) continue;
            Class<?> t = f.getType();
            if (t == double.class || t == int.class || t == boolean.class) out.add(f);
        }
        return out;
    }
}
