package com.slope.game;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Properties;

/**
 * Player-owned configuration that outlives the process: window preferences, volumes,
 * key bindings and the debug toggle. Stored as a flat properties file under the user's
 * home directory, so a reinstall of the game does not wipe it.
 *
 * <p>Reads are tolerant. A missing file, an unreadable one, or a junk value all fall
 * back to the built-in default rather than stopping the game from starting.
 */
public final class Settings {
    private static final String DIR_NAME = ".slope-game";
    private static final String FILE_NAME = "settings.properties";

    // Window
    private boolean fullscreen = false;
    private boolean vsync = true;
    private int windowWidth = 1280;
    private int windowHeight = 720;

    // Audio, all in 0..1
    private float masterVolume = 0.8f;
    private float musicVolume = 0.5f;
    private float sfxVolume = 0.9f;

    // Presentation
    private boolean debugOverlay = false;
    private boolean screenShake = true;
    private boolean dynamicFov = true;

    private final Keybinds keybinds = new Keybinds();
    private final Properties props = new Properties();

    public static Path directory() {
        return Paths.get(System.getProperty("user.home", "."), DIR_NAME);
    }

    private static Path file() {
        return directory().resolve(FILE_NAME);
    }

    /** Loads settings from disk, keeping defaults for anything absent or malformed. */
    public void load() {
        Path path = file();

        if (!Files.isReadable(path)) {
            return;
        }

        try (InputStream in = Files.newInputStream(path)) {
            props.clear();
            props.load(in);
        } catch (IOException e) {
            System.err.println("Could not read settings, using defaults: " + e.getMessage());
            return;
        }

        fullscreen = bool("window.fullscreen", fullscreen);
        vsync = bool("window.vsync", vsync);
        windowWidth = clampInt(integer("window.width", windowWidth), 640, 7680);
        windowHeight = clampInt(integer("window.height", windowHeight), 360, 4320);

        masterVolume = clamp01(decimal("audio.master", masterVolume));
        musicVolume = clamp01(decimal("audio.music", musicVolume));
        sfxVolume = clamp01(decimal("audio.sfx", sfxVolume));

        debugOverlay = bool("hud.debug", debugOverlay);
        screenShake = bool("camera.shake", screenShake);
        dynamicFov = bool("camera.dynamicFov", dynamicFov);

        keybinds.load(props);
    }

    /** Writes the current settings out. Failures are reported but never fatal. */
    public void save() {
        props.setProperty("window.fullscreen", Boolean.toString(fullscreen));
        props.setProperty("window.vsync", Boolean.toString(vsync));
        props.setProperty("window.width", Integer.toString(windowWidth));
        props.setProperty("window.height", Integer.toString(windowHeight));

        props.setProperty("audio.master", Float.toString(masterVolume));
        props.setProperty("audio.music", Float.toString(musicVolume));
        props.setProperty("audio.sfx", Float.toString(sfxVolume));

        props.setProperty("hud.debug", Boolean.toString(debugOverlay));
        props.setProperty("camera.shake", Boolean.toString(screenShake));
        props.setProperty("camera.dynamicFov", Boolean.toString(dynamicFov));

        keybinds.save(props);

        try {
            Files.createDirectories(directory());

            try (OutputStream out = Files.newOutputStream(file())) {
                props.store(out, "Slope Game settings");
            }
        } catch (IOException e) {
            System.err.println("Could not save settings: " + e.getMessage());
        }
    }

    private boolean bool(String key, boolean fallback) {
        String raw = props.getProperty(key);
        if (raw == null) {
            return fallback;
        }
        raw = raw.trim();
        return raw.equalsIgnoreCase("true") || raw.equals("1")
                ? true
                : (raw.equalsIgnoreCase("false") || raw.equals("0") ? false : fallback);
    }

    private int integer(String key, int fallback) {
        try {
            return Integer.parseInt(props.getProperty(key, "").trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private float decimal(String key, float fallback) {
        try {
            return Float.parseFloat(props.getProperty(key, "").trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }

    private static int clampInt(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    public Keybinds getKeybinds() {
        return keybinds;
    }

    public boolean isFullscreen() {
        return fullscreen;
    }

    public void setFullscreen(boolean value) {
        fullscreen = value;
    }

    public boolean isVsync() {
        return vsync;
    }

    public void setVsync(boolean value) {
        vsync = value;
    }

    public int getWindowWidth() {
        return windowWidth;
    }

    public int getWindowHeight() {
        return windowHeight;
    }

    public void setWindowSize(int width, int height) {
        windowWidth = clampInt(width, 640, 7680);
        windowHeight = clampInt(height, 360, 4320);
    }

    public float getMasterVolume() {
        return masterVolume;
    }

    public void setMasterVolume(float value) {
        masterVolume = clamp01(value);
    }

    public float getMusicVolume() {
        return musicVolume;
    }

    public void setMusicVolume(float value) {
        musicVolume = clamp01(value);
    }

    public float getSfxVolume() {
        return sfxVolume;
    }

    public void setSfxVolume(float value) {
        sfxVolume = clamp01(value);
    }

    public boolean isDebugOverlay() {
        return debugOverlay;
    }

    public void setDebugOverlay(boolean value) {
        debugOverlay = value;
    }

    public boolean isScreenShake() {
        return screenShake;
    }

    public void setScreenShake(boolean value) {
        screenShake = value;
    }

    public boolean isDynamicFov() {
        return dynamicFov;
    }

    public void setDynamicFov(boolean value) {
        dynamicFov = value;
    }
}
