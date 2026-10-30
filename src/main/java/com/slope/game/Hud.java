package com.slope.game;

import java.util.List;

import org.joml.Vector3f;

/**
 * Everything the player reads, drawn onto the {@link HudCanvas}. Kept apart from the game
 * loop so layout decisions live in one file, and apart from the shader so the text can be
 * arbitrary rather than a hand-coded expression per glyph.
 */
public final class Hud {
    private static final int W = HudCanvas.WIDTH;
    private static final int H = HudCanvas.HEIGHT;

    private static final float[] PANEL_FILL = { 0.02f, 0.03f, 0.07f, 0.86f };
    private static final float[] PANEL_EDGE = { 0.55f, 0.75f, 0.95f, 0.9f };
    private static final float[] METER_FRAME = { 0.8f, 0.86f, 0.95f, 0.75f };

    /** What the HUD needs to know. Filled in by the game each tick. */
    public static final class Data {
        public Game.State state = Game.State.TITLE;
        public GameMode mode = GameMode.ENDLESS;

        public int score;
        public int best;
        public int coins;
        public boolean newRecord;

        public float speed;
        public float speedFraction;
        public float distance;
        public float difficulty;

        public float boostTimer;
        public float slowTimer;
        public boolean airborne;
        public boolean inTunnel;

        public float timeLeft = -1f;     // negative when the mode is not timed
        public float runClock;

        public String zoneName = "";
        public int zoneNumber = 1;

        public Physics.DeathCause deathCause = Physics.DeathCause.NONE;

        public boolean debug;
        public boolean freeCam;
        public boolean gamepad;
        public float fps;

        // Debug-only detail.
        public float travelled;
        public float lateral;
        public int tileIndex;
        public int roadDrawn;
        public int propsDrawn;
        public boolean audioOn;
    }

    private final Vector3f accent = new Vector3f(1f, 1f, 1f);

    /** Repaints the whole overlay for this frame. */
    public void draw(HudCanvas c, Data d, Menu menu, Theme theme, boolean awaitingKey) {
        c.clear();
        accent.set(theme.getAccent());

        switch (d.state) {
            case TITLE, SETTINGS, BINDINGS, SCORES, MODE_SELECT -> {
                drawBackdrop(c, 0.72f);
                drawMenu(c, menu, awaitingKey);
            }
            case READY -> {
                drawRunHud(c, d);
                drawReadyPrompt(c, d);
            }
            case PLAYING -> drawRunHud(c, d);
            case PAUSED -> {
                drawRunHud(c, d);
                drawBackdrop(c, 0.6f);
                drawMenu(c, menu, awaitingKey);
            }
            case DEAD -> {
                drawRunHud(c, d);
                drawGameOver(c, d, menu);
            }
        }

        if (d.debug) {
            drawDebug(c, d);
        }
    }

    /** Dims the scene so menu text stays readable over whatever is behind it. */
    private void drawBackdrop(HudCanvas c, float strength) {
        c.fillRect(0, 0, W, H, 0.01f, 0.02f, 0.05f, strength);
    }

    // ---- In-run HUD ----

    private void drawRunHud(HudCanvas c, Data d) {
        // Score, big and centred. This is the number the player actually cares about.
        c.textCentered(W / 2, 14, Integer.toString(d.score), 4,
                1f, 1f, 1f, 1f);

        String bestLine = "BEST " + d.best;
        c.textCentered(W / 2, 48, bestLine, 1, 0.72f, 0.82f, 1f, 0.9f);

        // Speed, bottom right, as a meter plus a number. A bar reads at a glance while
        // the number is there when you want to compare runs.
        int meterW = 104;
        int meterX = W - meterW - 14;
        int meterY = H - 26;

        float[] speedFill = { accent.x, accent.y, accent.z, 0.95f };
        c.meter(meterX, meterY, meterW, 9, d.speedFraction, speedFill, METER_FRAME);
        // The raw figure, with no unit on it. Multiplying world units by 2.2369 dressed
        // 225 up as "503 MPH", which promised a sensation nothing in the scene was ever
        // going to deliver and made the mismatch worse than no number at all.
        c.textRight(W - 14, meterY - 12, "SPEED " + ((int) d.speed), 1,
                1f, 1f, 1f, 0.95f);

        // Coins, bottom left.
        if (d.coins > 0) {
            c.shadowText(14, H - 26, "COINS " + d.coins, 1, accent.x, accent.y, accent.z, 1f);
        }

        // Zone, top left, so the palette change has a name attached to it.
        c.shadowText(14, 14, "ZONE " + d.zoneNumber + "  " + d.zoneName.toUpperCase(), 1,
                0.85f, 0.9f, 1f, 0.85f);
        c.shadowText(14, 26, d.mode.getLabel().toUpperCase(), 1, 0.6f, 0.68f, 0.85f, 0.8f);

        // Difficulty, as a thin bar under the zone, since it is the thing that is quietly
        // making the run harder.
        float[] diffFill = { 1f, 0.45f, 0.35f, 0.8f };
        c.meter(14, 40, 70, 5, d.difficulty, diffFill, new float[] { 0.5f, 0.5f, 0.6f, 0.5f });

        drawTimer(c, d);
        drawStatusFlags(c, d);
    }

    /** The time trial clock. Turns red and flashes as it runs out. */
    private void drawTimer(HudCanvas c, Data d) {
        if (d.timeLeft < 0f) {
            return;
        }

        int secs = (int) Math.ceil(d.timeLeft);
        boolean urgent = d.timeLeft <= 10f;

        // Flash once a second when it is nearly up, so it cannot be missed.
        boolean blink = urgent && (((int) (d.timeLeft * 2f)) % 2 == 0);
        float r = urgent ? 1f : 1f;
        float g = urgent ? (blink ? 0.3f : 0.55f) : 1f;
        float b = urgent ? (blink ? 0.25f : 0.45f) : 1f;

        String text = secs + "S";
        c.textCentered(W / 2, 62, text, urgent ? 3 : 2, r, g, b, 1f);
    }

    /** Boost, slow-mo, airborne and tunnel badges, stacked under the score. */
    private void drawStatusFlags(HudCanvas c, Data d) {
        int y = 84;

        if (d.boostTimer > 0f) {
            drawBadge(c, y, "BOOST", d.boostTimer / 2.5f, 1f, 0.72f, 0.2f);
            y += 16;
        }

        if (d.slowTimer > 0f) {
            drawBadge(c, y, "SLOW MOTION", d.slowTimer / 3f, 0.45f, 0.8f, 1f);
            y += 16;
        }

        if (d.inTunnel) {
            c.textCentered(W / 2, y, "TUNNEL", 1, 0.8f, 0.7f, 1f, 0.9f);
            y += 14;
        }

        if (d.airborne) {
            c.textCentered(W / 2, y, "AIRBORNE", 1, 1f, 1f, 1f, 0.75f);
        }
    }

    private void drawBadge(HudCanvas c, int y, String text, float remaining,
                           float r, float g, float b) {
        int w = HudCanvas.textWidth(text, 1) + 10;
        int x = (W / 2) - (w / 2);

        c.fillRect(x, y - 3, w, 13, r * 0.25f, g * 0.25f, b * 0.25f, 0.8f);

        // The badge drains left to right, so the time left is visible without a number.
        int fill = (int) (w * Math.max(0f, Math.min(1f, remaining)));
        c.fillRect(x, y - 3, fill, 13, r, g, b, 0.35f);
        c.strokeRect(x, y - 3, w, 13, r, g, b, 0.9f);
        c.textCentered(W / 2, y, text, 1, 1f, 1f, 1f, 1f);
    }

    private void drawReadyPrompt(HudCanvas c, Data d) {
        int boxW = 260;
        int boxH = 56;
        int x = (W / 2) - (boxW / 2);
        int y = (H / 2) - 10;

        c.panel(x, y, boxW, boxH, PANEL_FILL, PANEL_EDGE);
        c.textCentered(W / 2, y + 10, d.mode.getLabel().toUpperCase(), 2,
                accent.x, accent.y, accent.z, 1f);
        c.textCentered(W / 2, y + 32, "PRESS SPACE TO DROP IN", 1, 1f, 1f, 1f, 0.95f);
    }

    // ---- Game over ----

    private void drawGameOver(HudCanvas c, Data d, Menu menu) {
        drawBackdrop(c, 0.55f);

        int boxW = 300;
        int boxH = 168;
        int x = (W / 2) - (boxW / 2);
        int y = (H / 2) - (boxH / 2);

        c.panel(x, y, boxW, boxH, PANEL_FILL, PANEL_EDGE);

        String headline = d.deathCause == Physics.DeathCause.FELL ? "YOU FELL" : "CRASHED";

        if (d.mode.isTimed() && d.timeLeft <= 0f) {
            headline = "TIME UP";
        }

        c.textCentered(W / 2, y + 12, headline, 3, 1f, 0.42f, 0.38f, 1f);

        if (d.newRecord) {
            c.textCentered(W / 2, y + 40, "NEW RECORD", 1,
                    accent.x, accent.y, accent.z, 1f);
        }

        c.textCentered(W / 2, y + 56, Integer.toString(d.score), 4, 1f, 1f, 1f, 1f);
        c.textCentered(W / 2, y + 88, "BEST " + d.best, 1, 0.75f, 0.85f, 1f, 0.9f);

        int statsY = y + 104;
        c.shadowText(x + 20, statsY, "DISTANCE", 1, 0.6f, 0.68f, 0.85f, 0.9f);
        c.textRight(x + boxW - 20, statsY, ((int) d.distance) + " M", 1, 1f, 1f, 1f, 0.95f);

        c.shadowText(x + 20, statsY + 12, "COINS", 1, 0.6f, 0.68f, 0.85f, 0.9f);
        c.textRight(x + boxW - 20, statsY + 12, Integer.toString(d.coins), 1,
                1f, 1f, 1f, 0.95f);

        c.shadowText(x + 20, statsY + 24, "ZONE REACHED", 1, 0.6f, 0.68f, 0.85f, 0.9f);
        c.textRight(x + boxW - 20, statsY + 24, d.zoneNumber + " " + d.zoneName.toUpperCase(),
                1, 1f, 1f, 1f, 0.95f);

        c.textCentered(W / 2, y + boxH - 14, "SPACE RETRY     ESC MENU", 1,
                0.8f, 0.86f, 1f, 0.9f);
    }

    // ---- Menus ----

    private void drawMenu(HudCanvas c, Menu menu, boolean awaitingKey) {
        if (menu == null) {
            return;
        }

        List<Menu.Item> items = menu.getItems();

        final int rowH = 16;
        final int boxW = 340;
        final int headerH = menu.getSubtitle() == null ? 44 : 58;
        final int boxH = headerH + (items.size() * rowH) + 24;

        int x = (W / 2) - (boxW / 2);
        int y = Math.max(8, (H / 2) - (boxH / 2));

        c.panel(x, y, boxW, boxH, PANEL_FILL, PANEL_EDGE);
        c.textCentered(W / 2, y + 12, menu.getTitle().toUpperCase(), 3,
                accent.x, accent.y, accent.z, 1f);

        if (menu.getSubtitle() != null) {
            c.textCentered(W / 2, y + 38, menu.getSubtitle().toUpperCase(), 1,
                    0.7f, 0.78f, 0.95f, 0.9f);
        }

        int rowY = y + headerH;

        for (int i = 0; i < items.size(); i++) {
            Menu.Item item = items.get(i);
            boolean selected = i == menu.getCursor();

            if (selected) {
                c.fillRect(x + 8, rowY - 3, boxW - 16, rowH - 2,
                        accent.x, accent.y, accent.z, 0.22f);
                c.shadowText(x + 14, rowY, ">", 1, accent.x, accent.y, accent.z, 1f);
            }

            float bright = item.isEnabled() ? (selected ? 1f : 0.82f) : 0.45f;
            c.shadowText(x + 26, rowY, item.getLabel().toUpperCase(), 1,
                    bright, bright, bright, 1f);

            String value = item.getValue();

            if (value != null) {
                boolean capturing = selected && awaitingKey;
                String shown = capturing ? "PRESS A KEY" : value.toUpperCase();

                float vr = capturing ? 1f : (selected ? accent.x : 0.75f);
                float vg = capturing ? 0.8f : (selected ? accent.y : 0.82f);
                float vb = capturing ? 0.3f : (selected ? accent.z : 0.95f);

                c.textRight(x + boxW - 14, rowY, shown, 1, vr, vg, vb, 1f);
            }

            rowY += rowH;
        }

        String hint = awaitingKey
                ? "PRESS A KEY TO BIND, ESC TO CANCEL"
                : "ARROWS MOVE, LEFT/RIGHT CHANGE, ENTER PICK";
        c.textCentered(W / 2, y + boxH - 14, hint, 1, 0.6f, 0.68f, 0.85f, 0.9f);
    }

    // ---- Debug ----

    /**
     * The developer readout. This used to be drawn unconditionally in the fragment
     * shader; it is now behind a toggle, because a crosshair on the ball and a set of
     * axes in the corner are not things a player should be looking at.
     */
    private void drawDebug(HudCanvas c, Data d) {
        final int lineH = 10;
        int x = 10;
        int y = 72;

        String[] lines = {
            "FPS        " + (int) d.fps,
            "SPEED      " + fmt(d.speed) + " M/S",
            "DIST S     " + fmt(d.travelled),
            "LATERAL    " + fmt(d.lateral),
            "TILE       " + d.tileIndex,
            "DIFFICULTY " + fmt(d.difficulty),
            "GROUNDED   " + (d.airborne ? "NO" : "YES"),
            "TUNNEL     " + (d.inTunnel ? "YES" : "NO"),
            "DRAWN      " + d.roadDrawn + " ROAD / " + d.propsDrawn + " PROPS",
            "AUDIO      " + (d.audioOn ? "ON" : "OFF"),
            "GAMEPAD    " + (d.gamepad ? "YES" : "NO"),
            "FREECAM    " + (d.freeCam ? "ON" : "OFF"),
        };

        int boxH = (lines.length * lineH) + 10;
        c.fillRect(x - 4, y - 5, 164, boxH, 0f, 0f, 0f, 0.55f);
        c.strokeRect(x - 4, y - 5, 164, boxH, 0.4f, 1f, 0.6f, 0.5f);

        for (String line : lines) {
            c.text(x, y, line, 1, 0.55f, 1f, 0.7f, 1f);
            y += lineH;
        }
    }

    private static String fmt(float v) {
        return String.format(java.util.Locale.ROOT, "%.1f", v);
    }
}
