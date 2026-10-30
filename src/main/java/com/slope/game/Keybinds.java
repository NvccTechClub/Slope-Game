package com.slope.game;

import java.util.EnumMap;
import java.util.Map;
import java.util.Properties;

import org.lwjgl.glfw.GLFW;

/**
 * The map from what the player wants to do to the key that does it. Every binding is
 * rebindable and persisted, so nothing in the game reads a hardcoded key.
 *
 * <p>Each action carries two bindings: a primary and an alternate. That way WASD and the
 * arrow keys can both steer without either being privileged, and rebinding one leaves
 * the other alone.
 */
public final class Keybinds {
    public enum Action {
        STEER_LEFT("Steer left", GLFW.GLFW_KEY_A, GLFW.GLFW_KEY_LEFT),
        STEER_RIGHT("Steer right", GLFW.GLFW_KEY_D, GLFW.GLFW_KEY_RIGHT),
        JUMP("Jump", GLFW.GLFW_KEY_SPACE, GLFW.GLFW_KEY_W),
        CONFIRM("Confirm", GLFW.GLFW_KEY_ENTER, GLFW.GLFW_KEY_SPACE),
        BACK("Back", GLFW.GLFW_KEY_ESCAPE, GLFW.GLFW_KEY_BACKSPACE),
        PAUSE("Pause", GLFW.GLFW_KEY_ESCAPE, GLFW.GLFW_KEY_P),
        RESTART("Restart", GLFW.GLFW_KEY_R, GLFW.GLFW_KEY_UNKNOWN),
        MENU_UP("Menu up", GLFW.GLFW_KEY_UP, GLFW.GLFW_KEY_W),
        MENU_DOWN("Menu down", GLFW.GLFW_KEY_DOWN, GLFW.GLFW_KEY_S),
        MENU_LEFT("Menu left", GLFW.GLFW_KEY_LEFT, GLFW.GLFW_KEY_A),
        MENU_RIGHT("Menu right", GLFW.GLFW_KEY_RIGHT, GLFW.GLFW_KEY_D),
        FREE_CAM("Free camera", GLFW.GLFW_KEY_F1, GLFW.GLFW_KEY_UNKNOWN),
        DEBUG_OVERLAY("Debug overlay", GLFW.GLFW_KEY_F3, GLFW.GLFW_KEY_UNKNOWN),
        FULLSCREEN("Fullscreen", GLFW.GLFW_KEY_F11, GLFW.GLFW_KEY_UNKNOWN);

        private final String label;
        private final int defaultPrimary;
        private final int defaultAlternate;

        Action(String label, int defaultPrimary, int defaultAlternate) {
            this.label = label;
            this.defaultPrimary = defaultPrimary;
            this.defaultAlternate = defaultAlternate;
        }

        public String getLabel() {
            return label;
        }

        /** Actions a player is offered in the rebinding screen. */
        public boolean isRebindable() {
            return this != CONFIRM && this != BACK
                    && this != MENU_UP && this != MENU_DOWN
                    && this != MENU_LEFT && this != MENU_RIGHT;
        }
    }

    private final Map<Action, Integer> primary = new EnumMap<>(Action.class);
    private final Map<Action, Integer> alternate = new EnumMap<>(Action.class);

    public Keybinds() {
        resetToDefaults();
    }

    public void resetToDefaults() {
        for (Action a : Action.values()) {
            primary.put(a, a.defaultPrimary);
            alternate.put(a, a.defaultAlternate);
        }
    }

    public int getPrimary(Action action) {
        return primary.getOrDefault(action, GLFW.GLFW_KEY_UNKNOWN);
    }

    public int getAlternate(Action action) {
        return alternate.getOrDefault(action, GLFW.GLFW_KEY_UNKNOWN);
    }

    /**
     * Points an action at a new key. The key is taken off any other action that was
     * using it first, so a rebind can never leave two actions fighting over one press.
     */
    public void rebind(Action action, int key) {
        if (key == GLFW.GLFW_KEY_UNKNOWN) {
            return;
        }

        for (Action other : Action.values()) {
            if (other == action || !other.isRebindable()) {
                continue;
            }

            if (primary.getOrDefault(other, GLFW.GLFW_KEY_UNKNOWN) == key) {
                primary.put(other, GLFW.GLFW_KEY_UNKNOWN);
            }

            if (alternate.getOrDefault(other, GLFW.GLFW_KEY_UNKNOWN) == key) {
                alternate.put(other, GLFW.GLFW_KEY_UNKNOWN);
            }
        }

        primary.put(action, key);
    }

    public void load(Properties props) {
        for (Action a : Action.values()) {
            primary.put(a, readKey(props, "key." + a.name() + ".primary", a.defaultPrimary));
            alternate.put(a, readKey(props, "key." + a.name() + ".alternate", a.defaultAlternate));
        }
    }

    public void save(Properties props) {
        for (Action a : Action.values()) {
            props.setProperty("key." + a.name() + ".primary", Integer.toString(getPrimary(a)));
            props.setProperty("key." + a.name() + ".alternate", Integer.toString(getAlternate(a)));
        }
    }

    private static int readKey(Properties props, String key, int fallback) {
        try {
            int value = Integer.parseInt(props.getProperty(key, "").trim());

            // Reject anything outside the range GLFW actually reports, so a corrupt file
            // cannot bind an action to a key that can never be pressed.
            if (value == GLFW.GLFW_KEY_UNKNOWN
                    || (value >= GLFW.GLFW_KEY_SPACE && value <= GLFW.GLFW_KEY_LAST)) {
                return value;
            }

            return fallback;
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    /** A short printable name for a key, for the bindings screen. */
    public static String keyName(int key) {
        if (key == GLFW.GLFW_KEY_UNKNOWN) {
            return "--";
        }

        switch (key) {
            case GLFW.GLFW_KEY_SPACE: return "SPACE";
            case GLFW.GLFW_KEY_ENTER: return "ENTER";
            case GLFW.GLFW_KEY_ESCAPE: return "ESC";
            case GLFW.GLFW_KEY_BACKSPACE: return "BKSP";
            case GLFW.GLFW_KEY_TAB: return "TAB";
            case GLFW.GLFW_KEY_LEFT: return "LEFT";
            case GLFW.GLFW_KEY_RIGHT: return "RIGHT";
            case GLFW.GLFW_KEY_UP: return "UP";
            case GLFW.GLFW_KEY_DOWN: return "DOWN";
            case GLFW.GLFW_KEY_LEFT_SHIFT: return "LSHIFT";
            case GLFW.GLFW_KEY_RIGHT_SHIFT: return "RSHIFT";
            case GLFW.GLFW_KEY_LEFT_CONTROL: return "LCTRL";
            case GLFW.GLFW_KEY_RIGHT_CONTROL: return "RCTRL";
            case GLFW.GLFW_KEY_LEFT_ALT: return "LALT";
            case GLFW.GLFW_KEY_RIGHT_ALT: return "RALT";
            default: break;
        }

        if (key >= GLFW.GLFW_KEY_F1 && key <= GLFW.GLFW_KEY_F25) {
            return "F" + (key - GLFW.GLFW_KEY_F1 + 1);
        }

        String name = GLFW.glfwGetKeyName(key, 0);

        if (name != null && !name.isEmpty()) {
            return name.toUpperCase(java.util.Locale.ROOT);
        }

        return "K" + key;
    }
}
