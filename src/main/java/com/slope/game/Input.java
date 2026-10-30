package com.slope.game;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWGamepadState;

/**
 * Everything the player can press, in one place. Keys arrive through a GLFW callback
 * rather than being polled, so a tap that starts and ends inside one frame is still
 * seen; a polled loop drops those.
 *
 * <p>Actions are looked up through {@link Keybinds}, so no caller mentions a key code.
 * Steering is reported as a float: a gamepad stick passes its own value through, and the
 * keyboard is ramped towards full deflection instead of snapping, which makes small
 * corrections possible on a digital key.
 */
public final class Input {
    /** How fast a held key reaches full steering lock, and how fast it returns. */
    private static final float KEY_STEER_ATTACK = 6.0f;
    private static final float KEY_STEER_RELEASE = 9.0f;

    /** Stick movement below this is treated as the stick resting off-centre. */
    private static final float STICK_DEADZONE = 0.22f;

    private final boolean[] down = new boolean[GLFW.GLFW_KEY_LAST + 1];
    private final boolean[] pressedThisTick = new boolean[GLFW.GLFW_KEY_LAST + 1];
    private final boolean[] edgeConsumed = new boolean[GLFW.GLFW_KEY_LAST + 1];

    private final Keybinds binds;

    private float steerAxis = 0f;
    private int gamepad = -1;
    private boolean gamepadPresent = false;

    /** Set while a rebinding screen wants the next keypress rather than an action. */
    private int capturedKey = GLFW.GLFW_KEY_UNKNOWN;
    private boolean capturing = false;

    // Gamepad buttons, latched the same way keys are so edges work identically.
    private final boolean[] padDown = new boolean[GLFW.GLFW_GAMEPAD_BUTTON_LAST + 1];
    private final boolean[] padPressed = new boolean[GLFW.GLFW_GAMEPAD_BUTTON_LAST + 1];

    public Input(Keybinds binds) {
        this.binds = binds;
    }

    /** Hooks the key callback onto the window. Call once, after the window exists. */
    public void attach(Window window) {
        GLFW.glfwSetKeyCallback(window.getWindow(), (handle, key, scancode, action, mods) -> {
            if (key < 0 || key > GLFW.GLFW_KEY_LAST) {
                return;
            }

            if (action == GLFW.GLFW_PRESS) {
                down[key] = true;
                pressedThisTick[key] = true;

                if (capturing) {
                    capturedKey = key;
                }
            } else if (action == GLFW.GLFW_RELEASE) {
                down[key] = false;
            }
            // Repeats are ignored: a held key is already down, and an auto-repeat is not
            // a new press as far as the game is concerned.
        });

        findGamepad();
    }

    /**
     * Closes the frame. Press edges live for exactly one tick, so this clears them after
     * the game has had its look. Call at the end of the update, never in the middle.
     */
    public void endFrame(float dt) {
        for (int i = 0; i < pressedThisTick.length; i++) {
            pressedThisTick[i] = false;
            edgeConsumed[i] = false;
        }

        for (int i = 0; i < padPressed.length; i++) {
            padPressed[i] = false;
        }

        updateSteerAxis(dt);
    }

    /** Polls the pad and advances the smoothed steering value. */
    private void updateSteerAxis(float dt) {
        float stick = readGamepadSteer();

        if (Math.abs(stick) > STICK_DEADZONE) {
            // Rescale so the axis still reaches 1.0 once past the deadzone.
            float sign = Math.signum(stick);
            steerAxis = sign * ((Math.abs(stick) - STICK_DEADZONE) / (1f - STICK_DEADZONE));
            steerAxis = clamp(steerAxis, -1f, 1f);
            return;
        }

        float target = 0f;

        if (isDown(Keybinds.Action.STEER_LEFT)) {
            target -= 1f;
        }
        if (isDown(Keybinds.Action.STEER_RIGHT)) {
            target += 1f;
        }

        // Ramp towards the target. Releasing recentres faster than pressing deflects,
        // so the ball settles promptly but still steers smoothly.
        float rate = (target == 0f) ? KEY_STEER_RELEASE : KEY_STEER_ATTACK;
        steerAxis += (target - steerAxis) * Math.min(1f, rate * dt);

        if (Math.abs(steerAxis) < 1e-3f) {
            steerAxis = 0f;
        }
    }

    /** Smoothed steering, -1 (full left) to +1 (full right). */
    public float getSteer() {
        return clamp(steerAxis, -1f, 1f);
    }

    public boolean isDown(Keybinds.Action action) {
        return keyDown(binds.getPrimary(action)) || keyDown(binds.getAlternate(action))
                || padDownFor(action, false);
    }

    /**
     * True on the tick an action's key went down. The edge is consumed, so two different
     * actions sharing a key cannot both fire from one press in the same frame.
     */
    public boolean justPressed(Keybinds.Action action) {
        if (consumeEdge(binds.getPrimary(action)) || consumeEdge(binds.getAlternate(action))) {
            return true;
        }

        return padDownFor(action, true);
    }

    /** Checks an action without consuming its edge, for ordering-sensitive callers. */
    public boolean peekPressed(Keybinds.Action action) {
        return keyPressed(binds.getPrimary(action)) || keyPressed(binds.getAlternate(action))
                || padDownFor(action, true);
    }

    private boolean consumeEdge(int key) {
        if (key == GLFW.GLFW_KEY_UNKNOWN || key > GLFW.GLFW_KEY_LAST || key < 0) {
            return false;
        }

        if (pressedThisTick[key] && !edgeConsumed[key]) {
            edgeConsumed[key] = true;
            return true;
        }

        return false;
    }

    private boolean keyDown(int key) {
        return key != GLFW.GLFW_KEY_UNKNOWN && key >= 0 && key <= GLFW.GLFW_KEY_LAST && down[key];
    }

    private boolean keyPressed(int key) {
        return key != GLFW.GLFW_KEY_UNKNOWN && key >= 0 && key <= GLFW.GLFW_KEY_LAST
                && pressedThisTick[key];
    }

    // ---- Gamepad ----

    private void findGamepad() {
        gamepad = -1;
        gamepadPresent = false;

        for (int jid = GLFW.GLFW_JOYSTICK_1; jid <= GLFW.GLFW_JOYSTICK_LAST; jid++) {
            if (GLFW.glfwJoystickPresent(jid) && GLFW.glfwJoystickIsGamepad(jid)) {
                gamepad = jid;
                gamepadPresent = true;
                System.out.println("Gamepad: " + GLFW.glfwGetGamepadName(jid));
                return;
            }
        }
    }

    /** Re-scans for a pad. Cheap enough to call on a menu tick, not every frame. */
    public void rescanGamepad() {
        if (!gamepadPresent || !GLFW.glfwJoystickPresent(gamepad)) {
            findGamepad();
        }
    }

    /**
     * Reads the pad once per frame and latches its buttons. Returns the steering value
     * from the left stick, or 0 when there is no pad.
     */
    private float readGamepadSteer() {
        if (!gamepadPresent) {
            return 0f;
        }

        if (!GLFW.glfwJoystickPresent(gamepad)) {
            gamepadPresent = false;
            return 0f;
        }

        try (GLFWGamepadState state = GLFWGamepadState.calloc()) {
            if (!GLFW.glfwGetGamepadState(gamepad, state)) {
                return 0f;
            }

            ByteBuffer buttons = state.buttons();

            for (int b = 0; b < padDown.length && b < buttons.limit(); b++) {
                boolean isDown = buttons.get(b) == GLFW.GLFW_PRESS;

                if (isDown && !padDown[b]) {
                    padPressed[b] = true;
                }

                padDown[b] = isDown;
            }

            FloatBuffer axes = state.axes();
            float x = axes.limit() > GLFW.GLFW_GAMEPAD_AXIS_LEFT_X
                    ? axes.get(GLFW.GLFW_GAMEPAD_AXIS_LEFT_X)
                    : 0f;

            // D-pad counts as full lock, so a pad without a usable stick still steers.
            if (padDown[GLFW.GLFW_GAMEPAD_BUTTON_DPAD_LEFT]) {
                x = -1f;
            } else if (padDown[GLFW.GLFW_GAMEPAD_BUTTON_DPAD_RIGHT]) {
                x = 1f;
            }

            return x;
        }
    }

    /** Maps an action onto pad buttons. Edge or held is chosen by the caller. */
    private boolean padDownFor(Keybinds.Action action, boolean edge) {
        if (!gamepadPresent) {
            return false;
        }

        boolean[] source = edge ? padPressed : padDown;

        switch (action) {
            case JUMP:
            case CONFIRM:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_A];
            case BACK:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_B];
            case PAUSE:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_START];
            case RESTART:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_Y];
            case MENU_UP:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_DPAD_UP];
            case MENU_DOWN:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_DPAD_DOWN];
            case MENU_LEFT:
            case STEER_LEFT:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_DPAD_LEFT];
            case MENU_RIGHT:
            case STEER_RIGHT:
                return source[GLFW.GLFW_GAMEPAD_BUTTON_DPAD_RIGHT];
            default:
                return false;
        }
    }

    public boolean hasGamepad() {
        return gamepadPresent;
    }

    // ---- Key capture, for the rebinding screen ----

    public void beginCapture() {
        capturing = true;
        capturedKey = GLFW.GLFW_KEY_UNKNOWN;
    }

    public void endCapture() {
        capturing = false;
        capturedKey = GLFW.GLFW_KEY_UNKNOWN;
    }

    public boolean isCapturing() {
        return capturing;
    }

    /** The key pressed since capture began, or GLFW_KEY_UNKNOWN if none yet. */
    public int getCapturedKey() {
        return capturedKey;
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
