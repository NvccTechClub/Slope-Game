package com.slope.game;

import org.joml.Math;
import org.joml.Vector3f;
import org.lwjgl.glfw.GLFW;

import com.slope.game.utils.BufferModel;

/**
 * The game itself: what state the player is in, what the hill is doing, and how the two
 * are presented. Everything that is not geometry or a draw call lives here.
 */
public class Game extends Core {
    /** Where a run begins, on the road surface. The road descends from here. */
    private static final Vector3f START_POS = new Vector3f(200f, 0f, 0f);

    // Chase camera placement, measured along the road rather than in world space, so
    // the view rides over rises and drops instead of burying itself in the hill.
    private static final float CAM_BACK = 32.0f;      // behind the ball, along the road
    private static final float CAM_UP = 9.0f;         // above the road surface
    private static final float CAM_LOOK_AHEAD = 34.0f;// aim down the road, not at the ball

    /**
     * Extra look-ahead at full speed.
     *
     * <p>Deliberately small. Pushing the aim point a long way down the road does buy
     * reaction time, but it flattens the view towards distant geometry, and distant
     * things barely move on screen: it was aiming the camera at the slowest part of the
     * picture exactly as the ball sped up, which is why going faster stopped looking
     * faster. The warning is bought from field of view instead, which widens the view
     * without tilting it off the near ground.
     */
    private static final float CAM_LOOK_AHEAD_SPEED = 12.0f;
    private static final float CAM_LOOK_UP = 4.0f;    // lift the aim point off the road
    private static final float CAM_SMOOTH = 14.0f;    // higher snaps tighter to the ball

    private static final float FLY_SPEED = 0.3f;

    /** The run starts at speed instead of crawling up from a standstill. */
    private static final float START_SPEED = 40.0f;

    // Death sequence.
    private static final float DEATH_FREEZE_DELAY = 2.2f;  // stop simulating the tumble
    private static final float DEATH_SLOWMO_IN = 0.35f;    // how fast time drops away
    private static final float DEATH_TIME_SCALE = 0.25f;
    private static final float DEATH_CAM_BACK = 26f;       // the view pulls out and up
    private static final float DEATH_CAM_UP = 22f;

    /** Slow-motion pickups run the world at this rate. */
    private static final float SLOW_TIME_SCALE = 0.45f;

    // Speed-driven camera.
    // A wide lens at speed. This is where the sense of motion comes from: it pulls
    // more of the near ground and more of the edges of the screen into view, and both
    // of those are the parts that sweep past fastest.
    private static final float FOV_SPEED_GAIN = Math.toRadians(34f);
    private static final float FOV_SMOOTH = 4.0f;

    private static final float SHAKE_DECAY = 3.4f;
    private static final float SHAKE_SCALE = 1.4f;

    public enum State { TITLE, MODE_SELECT, SETTINGS, BINDINGS, SCORES, READY, PLAYING, PAUSED, DEAD }

    private static final int ALL_PLATFORM_MODELS = 1;

    private ClockBufferGL clockBuffer;

    // TODO: Probably have an ArrayList for the red obstacles in every platform. (Feeshy Task Only)
    private final BufferModel[] bufferPool = new BufferModel[ALL_PLATFORM_MODELS];

    private final Settings settings;
    private final HighScores scores = new HighScores();
    private final Input input;
    private final AudioManager audio = new AudioManager();
    private final Theme theme = new Theme();
    private final Hud hud = new Hud();
    private final Hud.Data hudData = new Hud.Data();

    private State state = State.TITLE;
    private GameMode mode = GameMode.ENDLESS;

    private Menu titleMenu;
    private Menu modeMenu;
    private Menu settingsMenu;
    private Menu bindingsMenu;
    private Menu scoresMenu;
    private Menu pauseMenu;
    private Menu activeMenu;

    /** The action waiting for a key, while the bindings screen is capturing. */
    private Keybinds.Action rebinding = null;

    private float runClock = 0f;
    private float deadTimer = 0f;
    private float hudTime = 0f;
    private float timeScale = 1f;
    private boolean timeExpired = false;
    private boolean newRecord = false;
    private int lastScore = 0;

    // Presentation state.
    private float shake = 0f;
    private float flash = 0f;
    private float fovCurrent = CameraMatrices.BASE_FOV;
    private final Vector3f shakeOffset = new Vector3f();

    // Frame-rate measurement for the debug readout.
    private long fpsWindowStart = System.nanoTime();
    private int fpsFrames = 0;
    private float fps = 0f;

    private boolean freeCam = false;
    protected float sensitivity = 0.015f;
    protected boolean mouseActive = false;
    protected boolean initialMouseCentering = true;

    public Game() {
        super();

        settings = Main.getSettings();
        input = new Input(settings.getKeybinds());
    }

    public void move(float dx, float dy, float dz) {
        camMatrices.position.add(dx, dy, dz);
    }

    @Override
    public void init() {
        // TODO: Add stuff above our pre-init to it can get loaded to the renderer. (Feeshy Task Only)

        bufferPool[0] = loader.loadGLTFBuffer("src/main/resources/models/ramp.glb");
        clockBuffer = new ClockBufferGL(bufferPool[0].getSize(), 1);
        clockBuffer.put(bufferPool[0]);

        super.init();

        input.attach(Engine.getMain().getPrimaryWindow());

        scores.load(GameMode.todayStamp());
        audio.init(settings);

        // Apply the stored window preferences now that there is a window to apply to.
        Window window = Engine.getMain().getPrimaryWindow();
        window.setVsync(settings.isVsync());
        window.setFullscreen(settings.isFullscreen());

        buildMenus();

        // A hill exists from the moment the game opens, so the title screen has something
        // behind it rather than a black void.
        newRun(GameMode.ENDLESS, false);
        state = State.TITLE;
        openMenu(titleMenu);

        audio.startMusic();
    }

    // ---- Menus ----

    private void buildMenus() {
        titleMenu = new Menu("Slope", "an endless hill")
                .button("Play", () -> {
                    state = State.MODE_SELECT;
                    openMenu(modeMenu);
                })
                .button("High scores", () -> {
                    state = State.SCORES;
                    openMenu(scoresMenu);
                })
                .button("Settings", () -> {
                    state = State.SETTINGS;
                    openMenu(settingsMenu);
                })
                .button("Quit", () -> Engine.getMain().stop());

        modeMenu = new Menu("Choose a run", null);

        for (GameMode m : GameMode.values()) {
            modeMenu.picker(m.getLabel(),
                    () -> "best " + scores.getBest(m),
                    () -> startRun(m));
        }

        modeMenu.button("Back", this::backToTitle);

        settingsMenu = new Menu("Settings", null)
                .option("Master volume",
                        () -> percent(settings.getMasterVolume()),
                        delta -> {
                            settings.setMasterVolume(settings.getMasterVolume() + (delta * 0.05f));
                            audio.applyVolumes(settings);
                        })
                .option("Music volume",
                        () -> percent(settings.getMusicVolume()),
                        delta -> {
                            settings.setMusicVolume(settings.getMusicVolume() + (delta * 0.05f));
                            audio.applyVolumes(settings);
                        })
                .option("Sound volume",
                        () -> percent(settings.getSfxVolume()),
                        delta -> {
                            settings.setSfxVolume(settings.getSfxVolume() + (delta * 0.05f));
                            audio.applyVolumes(settings);
                        })
                .option("Fullscreen",
                        () -> onOff(settings.isFullscreen()),
                        delta -> {
                            settings.setFullscreen(!settings.isFullscreen());
                            Engine.getMain().getPrimaryWindow()
                                    .setFullscreen(settings.isFullscreen());
                        })
                .option("Vertical sync",
                        () -> onOff(settings.isVsync()),
                        delta -> {
                            settings.setVsync(!settings.isVsync());
                            Engine.getMain().getPrimaryWindow().setVsync(settings.isVsync());
                        })
                .option("Screen shake",
                        () -> onOff(settings.isScreenShake()),
                        delta -> settings.setScreenShake(!settings.isScreenShake()))
                .option("Speed field of view",
                        () -> onOff(settings.isDynamicFov()),
                        delta -> settings.setDynamicFov(!settings.isDynamicFov()))
                .option("Debug overlay",
                        () -> onOff(settings.isDebugOverlay()),
                        delta -> settings.setDebugOverlay(!settings.isDebugOverlay()))
                .button("Controls", () -> {
                    state = State.BINDINGS;
                    openMenu(bindingsMenu);
                })
                .button("Back", () -> {
                    settings.save();
                    backToPrevious();
                });

        bindingsMenu = new Menu("Controls", "pick a row, then press a key");

        for (Keybinds.Action action : Keybinds.Action.values()) {
            if (!action.isRebindable()) {
                continue;
            }

            bindingsMenu.picker(action.getLabel(),
                    () -> Keybinds.keyName(settings.getKeybinds().getPrimary(action)),
                    () -> beginRebind(action));
        }

        bindingsMenu
                .button("Reset to defaults", () -> {
                    settings.getKeybinds().resetToDefaults();
                    audio.play(AudioManager.Sfx.UI_SELECT);
                })
                .button("Back", () -> {
                    settings.save();
                    state = State.SETTINGS;
                    openMenu(settingsMenu);
                });

        scoresMenu = new Menu("High scores", null);

        for (GameMode m : GameMode.values()) {
            scoresMenu.label(m.getLabel(), () -> boardText(m));
        }

        scoresMenu.button("Back", this::backToTitle);

        pauseMenu = new Menu("Paused", null)
                .button("Resume", this::resume)
                .button("Restart", this::retry)
                .button("Settings", () -> {
                    state = State.SETTINGS;
                    openMenu(settingsMenu);
                })
                .button("Quit to title", this::backToTitle);
    }

    private static String percent(float v) {
        return ((int) ((v * 100f) + 0.5f)) + "%";
    }

    private static String onOff(boolean v) {
        return v ? "on" : "off";
    }

    private String boardText(GameMode m) {
        java.util.List<Integer> board = scores.getBoard(m);

        if (board.isEmpty()) {
            return "no runs yet";
        }

        StringBuilder sb = new StringBuilder();

        for (int i = 0; i < board.size() && i < 3; i++) {
            if (sb.length() > 0) {
                sb.append("  ");
            }
            sb.append(board.get(i));
        }

        return sb.toString();
    }

    private void openMenu(Menu menu) {
        activeMenu = menu;

        if (menu != null) {
            menu.resetCursor();
        }
    }

    private void beginRebind(Keybinds.Action action) {
        rebinding = action;
        input.beginCapture();
        audio.play(AudioManager.Sfx.UI_SELECT);
    }

    /** Settings can be opened from the title or from a pause, so it remembers which. */
    private void backToPrevious() {
        if (Physics.active().getTrack() != null && (state == State.SETTINGS || state == State.BINDINGS)
                && pausedUnderneath) {
            state = State.PAUSED;
            openMenu(pauseMenu);
            return;
        }

        backToTitle();
    }

    private boolean pausedUnderneath = false;

    private void backToTitle() {
        pausedUnderneath = false;
        state = State.TITLE;
        openMenu(titleMenu);
        settings.save();
        scores.save();
    }

    // ---- Run lifecycle ----

    /**
     * Builds a fresh hill and puts the ball on it.
     *
     * @param newMode the mode to run
     * @param play true to hand control to the player, false to leave it as a backdrop
     */
    private void newRun(GameMode newMode, boolean play) {
        mode = newMode;

        long seed = newMode.seedFor();
        newTrack(START_POS, seed);

        Physics.active().reset(START_POS, new Vector3f(0f, 0f, 0f));

        runClock = 0f;
        deadTimer = 0f;
        timeScale = 1f;
        timeExpired = false;
        newRecord = false;
        lastScore = 0;
        shake = 0f;
        flash = 0f;

        theme.update(0f);
        snapCameraToBall();

        if (play) {
            state = State.READY;
            activeMenu = null;
            pausedUnderneath = false;
        }
    }

    /**
     * Restarts the current mode and drops straight into it.
     *
     * <p>Retrying used to cost two presses: one to leave the game-over screen and another
     * to launch out of the ready state. On runs that last about fifteen seconds that
     * prompt is a tax paid constantly, and it is in the way of the one thing a player
     * wants after dying, which is to be moving again.
     */
    private void retry() {
        audio.play(AudioManager.Sfx.UI_SELECT);
        newRun(mode, true);
        Physics.active().launch(START_SPEED);
        state = State.PLAYING;
    }

    private void startRun(GameMode newMode) {
        audio.play(AudioManager.Sfx.UI_SELECT);
        newRun(newMode, true);
    }

    private void pause() {
        if (state != State.PLAYING && state != State.READY) {
            return;
        }

        state = State.PAUSED;
        pausedUnderneath = true;
        openMenu(pauseMenu);
        audio.play(AudioManager.Sfx.UI_SELECT);
    }

    private void resume() {
        state = Physics.active().isAlive() ? State.PLAYING : State.DEAD;
        activeMenu = null;
        pausedUnderneath = false;
    }

    private void endRun(boolean outOfTime) {
        if (state == State.DEAD) {
            return;
        }

        state = State.DEAD;
        activeMenu = null;
        deadTimer = 0f;
        timeExpired = outOfTime;
        lastScore = Physics.active().getScore();

        newRecord = scores.submit(mode, lastScore);
        scores.save();

        flash = outOfTime ? 0.35f : 0.8f;
        shake = outOfTime ? 0.2f : 1f;

        audio.play(AudioManager.Sfx.IMPACT, outOfTime ? 0.5f : 1f, 1f);
    }

    // ---- Frame ----

    @Override
    public void update() {
        final float realDt = Engine.getMain().getFrameArea();

        measureFps();
        hudTime += realDt;

        input.rescanGamepad();
        handleGlobalKeys();

        if (input.isCapturing()) {
            handleRebindCapture();
        } else {
            switch (state) {
                case TITLE, MODE_SELECT, SETTINGS, BINDINGS, SCORES -> updateMenu();
                case READY -> updateReady();
                case PLAYING -> updatePlaying(realDt);
                case PAUSED -> updateMenu();
                case DEAD -> updateDead(realDt);
            }
        }

        updatePresentation(realDt);
        drawHud();

        input.endFrame(realDt);
    }

    private void measureFps() {
        fpsFrames++;
        long now = System.nanoTime();
        long elapsed = now - fpsWindowStart;

        if (elapsed >= 500_000_000L) {
            fps = (float) (fpsFrames / (elapsed / 1_000_000_000.0));
            fpsFrames = 0;
            fpsWindowStart = now;
        }
    }

    private void handleGlobalKeys() {
        if (input.justPressed(Keybinds.Action.FULLSCREEN)) {
            Window window = Engine.getMain().getPrimaryWindow();
            window.toggleFullscreen();
            settings.setFullscreen(window.isFullscreen());
            settings.save();
        }

        if (input.justPressed(Keybinds.Action.DEBUG_OVERLAY)) {
            settings.setDebugOverlay(!settings.isDebugOverlay());
            settings.save();
        }

        if (input.justPressed(Keybinds.Action.FREE_CAM)) {
            freeCam = !freeCam;

            if (freeCam) {
                // Carry the current view into the flycam so toggling does not snap it.
                Vector3f look = new Vector3f(camMatrices.lookAt).normalize();
                camMatrices.horizontalAngle = Math.atan2(look.x, look.z);
                camMatrices.verticalAngle = Math.asin(Math.clamp(look.y, -1f, 1f));
            }
        }
    }

    /** While the bindings screen waits for a key, that key is all the game listens to. */
    private void handleRebindCapture() {
        int key = input.getCapturedKey();

        if (key == GLFW.GLFW_KEY_UNKNOWN) {
            return;
        }

        if (key != GLFW.GLFW_KEY_ESCAPE && rebinding != null) {
            settings.getKeybinds().rebind(rebinding, key);
            settings.save();
            audio.play(AudioManager.Sfx.UI_SELECT);
        }

        rebinding = null;
        input.endCapture();
    }

    private void updateMenu() {
        if (activeMenu == null) {
            return;
        }

        if (input.justPressed(Keybinds.Action.MENU_DOWN) && activeMenu.move(1)) {
            audio.play(AudioManager.Sfx.UI_MOVE);
        }

        if (input.justPressed(Keybinds.Action.MENU_UP) && activeMenu.move(-1)) {
            audio.play(AudioManager.Sfx.UI_MOVE);
        }

        if (input.justPressed(Keybinds.Action.MENU_RIGHT) && activeMenu.adjust(1)) {
            audio.play(AudioManager.Sfx.UI_MOVE);
        }

        if (input.justPressed(Keybinds.Action.MENU_LEFT) && activeMenu.adjust(-1)) {
            audio.play(AudioManager.Sfx.UI_MOVE);
        }

        if (input.justPressed(Keybinds.Action.CONFIRM)) {
            activeMenu.select();
        } else if (input.justPressed(Keybinds.Action.BACK)) {
            handleMenuBack();
        }
    }

    private void handleMenuBack() {
        switch (state) {
            case TITLE -> Engine.getMain().stop();
            case MODE_SELECT, SCORES -> backToTitle();
            case SETTINGS -> {
                settings.save();
                backToPrevious();
            }
            case BINDINGS -> {
                settings.save();
                state = State.SETTINGS;
                openMenu(settingsMenu);
            }
            case PAUSED -> resume();
            default -> { }
        }
    }

    private void updateReady() {
        if (input.justPressed(Keybinds.Action.PAUSE)) {
            pause();
            return;
        }

        // The ball rests on the hill until the player commits.
        if (input.justPressed(Keybinds.Action.CONFIRM) || input.justPressed(Keybinds.Action.JUMP)) {
            Physics.active().launch(START_SPEED);
            state = State.PLAYING;
        }
    }

    private void updatePlaying(float realDt) {
        if (input.justPressed(Keybinds.Action.PAUSE)) {
            pause();
            return;
        }

        if (input.justPressed(Keybinds.Action.RESTART)) {
            retry();
            return;
        }

        // Slow-motion scales the world, not the input, so the player keeps full control.
        timeScale = Physics.active().getSlowTimer() > 0f ? SLOW_TIME_SCALE : 1f;
        final float dt = realDt * timeScale;

        runClock += dt;

        Physics.active().setSteerInput(input.getSteer());

        if (input.justPressed(Keybinds.Action.JUMP)) {
            Physics.active().requestJump();
        }

        Physics.active().update(dt);
        drainPhysicsEvents();

        if (!Physics.active().isAlive()) {
            endRun(false);
            return;
        }

        if (mode.isTimed() && runClock >= GameMode.TIME_TRIAL_SECONDS) {
            endRun(true);
        }
    }

    private void updateDead(float realDt) {
        Physics.active().setSteerInput(0f);

        // Time drops away rather than stopping dead, so the crash reads as a crash.
        timeScale = Math.max(DEATH_TIME_SCALE,
                timeScale - (realDt / DEATH_SLOWMO_IN) * (1f - DEATH_TIME_SCALE));

        if (deadTimer < DEATH_FREEZE_DELAY) {
            deadTimer += realDt;
            Physics.active().update(realDt * timeScale);
            drainPhysicsEvents();
        }

        audio.setMusicDuck(0.35f + (0.65f * Math.min(1f, deadTimer / DEATH_FREEZE_DELAY)));

        if (input.justPressed(Keybinds.Action.CONFIRM)) {
            audio.setMusicDuck(1f);
            retry();
        } else if (input.justPressed(Keybinds.Action.BACK)) {
            audio.setMusicDuck(1f);
            backToTitle();
        }
    }

    /** Turns simulation events into sound and screen effects. */
    private void drainPhysicsEvents() {
        Track.Pickup picked = Physics.active().consumePickup();

        if (picked != null) {
            switch (picked.type) {
                case COIN -> audio.play(AudioManager.Sfx.COIN, 0.8f, 1f);
                case BOOST -> {
                    audio.play(AudioManager.Sfx.BOOST, 0.9f, 1f);
                    shake = Math.max(shake, 0.35f);
                }
                case SLOW -> audio.play(AudioManager.Sfx.BOOST, 0.7f, 0.6f);
            }
        }

        if (Physics.active().consumeJumpEvent()) {
            audio.play(AudioManager.Sfx.JUMP, 0.7f, 1f);
        }

        if (Physics.active().consumeLandEvent()) {
            float impact = Physics.active().getLandImpact();
            float strength = Math.min(1f, impact / 45f);

            if (strength > 0.12f) {
                audio.play(AudioManager.Sfx.LAND, 0.35f + (0.6f * strength),
                        0.9f + (0.3f * (1f - strength)));
                shake = Math.max(shake, strength * 0.8f);
                flash = Math.max(flash, strength * 0.12f);
            }
        }

        if (Physics.active().consumeHitEvent()) {
            // A survivable scrape: felt, but not fatal.
            audio.play(AudioManager.Sfx.IMPACT, 0.45f, 1.5f);
            shake = Math.max(shake, 0.6f);
            flash = Math.max(flash, 0.25f);
        }

        Physics.active().consumeDeathEvent();
    }

    // ---- Camera, effects and HUD ----

    private void updatePresentation(float realDt) {
        theme.update(Physics.active().getDistance());
        renderer.setTheme(theme);

        boolean running = state == State.PLAYING || state == State.DEAD || state == State.READY;

        checkMouseActive();

        if (freeCam) {
            processInput();
        } else if (state == State.DEAD) {
            updateDeathCamera(realDt);
        } else {
            updateChaseCamera(realDt);
        }

        // Shake and flash decay on real time, so a slowed world still settles promptly.
        shake = Math.max(0f, shake - (SHAKE_DECAY * realDt));
        flash = Math.max(0f, flash - (2.2f * realDt));

        applyShake();
        applyFov(realDt, running);

        camMatrices.updateViewMat();

        float speedFraction = running ? Physics.active().getSpeedFraction() : 0f;
        renderer.setScreenEffects(running ? speedFraction : 0f, flash);

        audio.updateMotion(speedFraction,
                !Physics.active().isOnGround(),
                state == State.PLAYING);

        // Lay out the road for this frame, now the camera knows where it is looking.
        updateTrackView(theme, hudTime);
    }

    /** Widens the view with speed, which is what makes fast feel fast. */
    private void applyFov(float realDt, boolean running) {
        float target = CameraMatrices.BASE_FOV;

        if (settings.isDynamicFov() && running) {
            float f = Physics.active().getSpeedFraction();

            // Boost pushes a little past what raw speed would give, so it reads as extra.
            if (Physics.active().isBoosting()) {
                f = Math.min(1.2f, f + 0.2f);
            }

            target = CameraMatrices.BASE_FOV + (FOV_SPEED_GAIN * f * f);
        }

        fovCurrent += (target - fovCurrent) * Math.min(1f, FOV_SMOOTH * realDt);
        camMatrices.setFov(fovCurrent);
    }

    /** Offsets the camera by a decaying jitter. Disabled from the settings screen. */
    private void applyShake() {
        if (shake <= 0f || !settings.isScreenShake()) {
            return;
        }

        // Two incommensurate frequencies per axis, so the motion does not look periodic.
        float t = hudTime * 37f;
        float amount = shake * shake * SHAKE_SCALE;

        shakeOffset.set(
                (float) (Math.sin(t * 1.0f) + Math.sin(t * 2.37f)) * 0.5f,
                (float) (Math.sin(t * 1.31f) + Math.sin(t * 2.91f)) * 0.5f,
                (float) (Math.sin(t * 0.83f) + Math.sin(t * 1.77f)) * 0.5f);

        camMatrices.position.fma(amount, shakeOffset);
    }

    private void drawHud() {
        Track track = Physics.active().getTrack();

        hudData.state = state;
        hudData.mode = mode;
        hudData.score = (state == State.DEAD) ? lastScore : Physics.active().getScore();
        hudData.best = scores.getBest(mode);
        hudData.coins = Physics.active().getCoins();
        hudData.newRecord = newRecord;

        hudData.speed = Physics.active().getSpeed();
        hudData.speedFraction = Physics.active().getSpeedFraction();
        hudData.distance = Physics.active().getDistance();
        hudData.difficulty = track == null ? 0f : track.difficultyAt(Physics.active().getDistance());

        hudData.boostTimer = Physics.active().getBoostTimer();
        hudData.slowTimer = Physics.active().getSlowTimer();
        hudData.airborne = !Physics.active().isOnGround();
        hudData.inTunnel = Physics.active().isInTunnel();

        hudData.timeLeft = mode.isTimed()
                ? Math.max(0f, GameMode.TIME_TRIAL_SECONDS - runClock)
                : -1f;
        hudData.runClock = runClock;

        hudData.zoneName = theme.getZoneName();
        hudData.zoneNumber = theme.getZoneNumber();
        hudData.deathCause = Physics.active().getDeathCause();

        hudData.debug = settings.isDebugOverlay();
        hudData.freeCam = freeCam;
        hudData.gamepad = input.hasGamepad();
        hudData.fps = fps;

        hudData.travelled = Physics.active().getTravelled();
        hudData.lateral = Physics.active().getLateralOffset();
        hudData.tileIndex = track == null ? 0 : track.tileIndexAt(Physics.active().getTravelled());
        hudData.roadDrawn = getRoadDrawn();
        hudData.propsDrawn = getPropsDrawn();
        hudData.audioOn = audio.isEnabled();

        hud.draw(hudCanvas, hudData, activeMenu, theme, input.isCapturing());

        renderer.setHudState(phaseCode(), hudTime, settings.isDebugOverlay());
    }

    // ---- Cameras ----

    private void processInput() {
        Window window = Engine.getMain().getPrimaryWindow();

        Vector3f forward = new Vector3f(camMatrices.lookAt).normalize();
        Vector3f right = new Vector3f(forward).cross(CameraMatrices.LOOK_UP).normalize();
        Vector3f up = new Vector3f(right).cross(forward).normalize();

        // Accumulate first, then move once, so holding two keys does not compound the step.
        Vector3f delta = new Vector3f();

        if (window.isKeyPressed(GLFW.GLFW_KEY_W)) delta.add(forward);
        if (window.isKeyPressed(GLFW.GLFW_KEY_S)) delta.sub(forward);
        if (window.isKeyPressed(GLFW.GLFW_KEY_A)) delta.sub(right);
        if (window.isKeyPressed(GLFW.GLFW_KEY_D)) delta.add(right);
        if (window.isKeyPressed(GLFW.GLFW_KEY_E)) delta.add(up);
        if (window.isKeyPressed(GLFW.GLFW_KEY_Q)) delta.sub(up);

        if (delta.lengthSquared() > 1e-8f) {
            camMatrices.position.fma(FLY_SPEED, delta.normalize());
        }
    }

    private void handleMouseInput() {
        Window window = Engine.getMain().getPrimaryWindow();

        if (mouseActive) {
            float[] mousePos = window.getMousePosition();
            float deltaX = mousePos[0] - (window.getWidth() / 2.0f);
            float deltaY = mousePos[1] - (window.getHeight() / 2.0f);

            camMatrices.horizontalAngle -= deltaX * sensitivity;
            camMatrices.verticalAngle -= deltaY * sensitivity;

            camMatrices.verticalAngle = Math.clamp(camMatrices.verticalAngle,
                    Math.toRadians(-89.0f), Math.toRadians(89.0f));

            camMatrices.lookAt.set(
                    (float) (Math.cos(camMatrices.verticalAngle)
                            * Math.sin(camMatrices.horizontalAngle)),
                    (float) Math.sin(camMatrices.verticalAngle),
                    (float) (Math.cos(camMatrices.verticalAngle)
                            * Math.cos(camMatrices.horizontalAngle)));
        }

        window.setMousePosition(window.getWidth() / 2.0f, window.getHeight() / 2.0f);
    }

    public void checkMouseActive() {
        Window window = Engine.getMain().getPrimaryWindow();

        // The debug flycam owns the mouse while it is active; the game needs no mouse.
        if (freeCam) {
            if (!mouseActive) {
                mouseActive = true;
                initialMouseCentering = true;
                GLFW.glfwSetInputMode(window.getWindow(), GLFW.GLFW_CURSOR,
                        GLFW.GLFW_CURSOR_DISABLED);
            }
        } else if (mouseActive) {
            mouseActive = false;
            GLFW.glfwSetInputMode(window.getWindow(), GLFW.GLFW_CURSOR, GLFW.GLFW_CURSOR_NORMAL);
        }

        if (mouseActive && initialMouseCentering) {
            window.setMousePosition(window.getWidth() / 2.0f, window.getHeight() / 2.0f);
            initialMouseCentering = false;
        }

        if (mouseActive) {
            handleMouseInput();
        }
    }

    /** Parks the camera over the road behind the ball, looking down the run. */
    private void updateChaseCamera(float dt) {
        float t = java.lang.Math.min(1f, CAM_SMOOTH * dt);
        camMatrices.position.lerp(cameraSpot(CAM_BACK, CAM_UP), t);
        aimCameraDownTrack();
    }

    /**
     * Pulls the view out and up after a crash and keeps it on the ball, so the tumble is
     * something you watch rather than something that happens off screen.
     */
    private void updateDeathCamera(float dt) {
        float progress = Math.min(1f, deadTimer / DEATH_FREEZE_DELAY);
        float back = CAM_BACK + (DEATH_CAM_BACK * progress);
        float up = CAM_UP + (DEATH_CAM_UP * progress);

        float t = java.lang.Math.min(1f, (CAM_SMOOTH * 0.5f) * dt);
        camMatrices.position.lerp(cameraSpot(back, up), t);

        // Aim at the ball itself now, not down the road it will never reach.
        Vector3f dir = new Vector3f(Physics.active().getPosition()).sub(camMatrices.position);

        if (dir.lengthSquared() > 1e-8f) {
            camMatrices.lookAt.set(dir.normalize());
        }
    }

    private void snapCameraToBall() {
        camMatrices.position.set(cameraSpot(CAM_BACK, CAM_UP));
        aimCameraDownTrack();
        camMatrices.updateViewMat();
    }

    /**
     * Where the camera wants to be: over the road a fixed distance behind the ball.
     * Following the road rather than the ball keeps a constant height above the surface,
     * so a steep pitch does not drop the view into the hillside.
     */
    private Vector3f cameraSpot(float back, float up) {
        Vector3f spot = new Vector3f();

        // Sit in the ball's own lane, not on the centreline, or the ball slides out of
        // frame as soon as the player steers away from the middle of the road.
        Physics.active().surfacePoint(Physics.active().getTravelled() - back, Physics.active().getLateralOffset(), spot);

        // Ride with the ball once it is above the road, so a jump stays in shot instead
        // of flying over the top of the camera.
        spot.y = java.lang.Math.max(spot.y, Physics.active().getPosition().y) + up;
        return spot;
    }

    /**
     * How far the ball is above the road beneath it, or zero while it is rolling. The
     * camera uses this to follow a jump without losing the road.
     */
    private float ballHeightOverRoad() {
        Vector3f under = new Vector3f();
        Physics.active().surfacePoint(Physics.active().getTravelled(), Physics.active().getLateralOffset(), under);
        return java.lang.Math.max(0f, Physics.active().getPosition().y - under.y);
    }

    private void aimCameraDownTrack() {
        Vector3f lookTarget = new Vector3f();

        // Look down the lane the ball is actually in, aimed at the road ahead. Taking the
        // ball's own height here would pull the aim up on every descent, because the road
        // ahead is always below the ball, and that pushed the road out of frame entirely.
        Physics.active().surfacePoint(Physics.active().getTravelled() + CAM_LOOK_AHEAD,
                Physics.active().getLateralOffset(), lookTarget);
        lookTarget.y += CAM_LOOK_UP;

        // Only a real jump lifts the aim, and only part way, so the road stays in shot.
        lookTarget.y += ballHeightOverRoad() * 0.5f;

        Vector3f dir = lookTarget.sub(camMatrices.position);

        if (dir.lengthSquared() > 1e-8f) {
            camMatrices.lookAt.set(dir.normalize());
        }
    }

    @Override
    public void destroy() {
        settings.save();
        scores.save();
        audio.destroy();
        super.destroy();
    }

    /**
     * A coarse phase for the screen pass, which only needs to know whether to dim, wash
     * red, or leave the scene alone. Deliberately not the enum's ordinal: the shader
     * should not break because a state was inserted in the middle of the enum.
     */
    private int phaseCode() {
        return switch (state) {
            case READY -> 1;
            case PLAYING -> 2;
            case PAUSED -> 3;
            case DEAD -> 4;
            default -> 0;   // a menu is up; the overlay draws its own backdrop
        };
    }

    public State getState() {
        return state;
    }

    public GameMode getMode() {
        return mode;
    }

    public int getBestScore() {
        return scores.getBest(mode);
    }
}
