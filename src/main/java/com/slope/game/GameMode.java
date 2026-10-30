package com.slope.game;

import java.time.LocalDate;
import java.time.format.DateTimeFormatter;

/** The ways a run can be played. Each one decides how it is seeded and how it ends. */
public enum GameMode {
    /** The default: one endless hill, a fresh layout every time, ends when you crash. */
    ENDLESS("Endless", "Run until you crash"),

    /** Sixty seconds on the clock. Cover as much ground as you can before it runs out. */
    TIME_TRIAL("Time trial", "Sixty seconds, go as far as you can"),

    /** One layout per day, the same for every run that day, so attempts are comparable. */
    DAILY("Daily run", "One fixed hill per day");

    /** How long a time trial lasts, in seconds. */
    public static final float TIME_TRIAL_SECONDS = 60f;

    private final String label;
    private final String blurb;

    GameMode(String label, String blurb) {
        this.label = label;
        this.blurb = blurb;
    }

    public String getLabel() {
        return label;
    }

    public String getBlurb() {
        return blurb;
    }

    public boolean isTimed() {
        return this == TIME_TRIAL;
    }

    /** Today's date as a stable string, used to seed and to date the daily board. */
    public static String todayStamp() {
        return LocalDate.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd"));
    }

    /**
     * The layout seed for a new run. Endless and time trial get a fresh hill each time;
     * the daily is pinned to the date so everyone playing today runs the same one.
     */
    public long seedFor() {
        if (this == DAILY) {
            return todayStamp().hashCode() * 0x9E3779B97F4A7C15L;
        }

        return System.nanoTime();
    }
}
