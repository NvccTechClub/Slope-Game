package com.slope.game;

import org.joml.Vector3f;

/**
 * The look of the hill, which changes as the run goes on. Long runs need to read as
 * progress, and a shifting palette does that without touching the geometry: the player
 * can see how deep they are from the colour of the world.
 *
 * <p>Zones are a fixed distance apart and blend into one another over the last stretch
 * of each, so a change arrives as a wash rather than a cut.
 */
public final class Theme {
    /** How much ground one palette covers. */
    private static final float ZONE_LENGTH = 1100f;

    /** Share of a zone spent fading into the next. */
    private static final float BLEND_SHARE = 0.22f;

    /**
     * Hazards, in every zone. Bright enough to glow, saturated enough never to read as
     * part of the green world it sits in.
     */
    private static final Vector3f BLOCK_RED = new Vector3f(1.0f, 0.13f, 0.13f);

    /**
     * One palette, built from a single colour. The road, the ramps, the scenery and the
     * sky are all that colour, separated by brightness alone rather than by hue.
     *
     * <p>Blocks are the single exception: they are red in every zone, because a hazard
     * that takes its colour from the scenery is one you have to re-learn to spot each
     * time the scenery changes.
     */
    public static final class Palette {
        public final String name;
        public final Vector3f base;
        public final Vector3f skyTop;
        public final Vector3f skyBottom;
        public final Vector3f road;
        public final Vector3f block;
        public final Vector3f wall;
        public final Vector3f kicker;
        public final Vector3f fog;
        public final Vector3f accent;

        Palette(String name, int colour) {
            this.name = name;
            this.base = rgb(colour);

            // Neon on black. The road runs at full strength rather than half, because
            // the look is lit lines against nothing: holding the road back left it
            // reading as painted-on rather than glowing. Depth comes from the towers
            // being dimmer, not from the road being dark.
            this.road = scale(base, 1.00f);
            this.kicker = scale(base, 1.00f);
            this.wall = scale(base, 0.62f);

            // Blocks are red in every zone, never a shade of the zone colour. A hazard
            // that changes colour with the scenery is a hazard you have to re-learn to
            // see; this one reads the instant it comes over the horizon, whatever the
            // world happens to be doing. It is the one thing here that is not the zone.
            this.block = BLOCK_RED;

            // Pickups lift towards white so they separate from a full-strength road.
            this.accent = lighten(base, 0.50f);

            // The background is black, not a dark tint of the zone. The far end of the
            // road fades into it rather than into a coloured haze.
            this.fog = scale(base, 0.045f);
            this.skyTop = scale(base, 0.0f);
            this.skyBottom = scale(base, 0.025f);
        }

        private static Vector3f rgb(int hex) {
            return new Vector3f(
                    ((hex >> 16) & 0xFF) / 255f,
                    ((hex >> 8) & 0xFF) / 255f,
                    (hex & 0xFF) / 255f);
        }

        /** Lifts towards white, for the few things that must out-read a lit road. */
        private static Vector3f lighten(Vector3f c, float t) {
            return new Vector3f(
                    c.x + ((1f - c.x) * t),
                    c.y + ((1f - c.y) * t),
                    c.z + ((1f - c.z) * t));
        }

        /** Scales brightness while keeping the hue, clamping rather than desaturating. */
        private static Vector3f scale(Vector3f c, float k) {
            return new Vector3f(
                    Math.min(1f, c.x * k),
                    Math.min(1f, c.y * k),
                    Math.min(1f, c.z * k));
        }
    }

    /**
     * The palettes, in the order a run meets them. None of them sit near red, because
     * red is reserved for blocks: a zone that drifts towards it would camouflage the one
     * thing the player has to see.
     *
     * <p>The palettes, in the order a run meets them. They darken and get colder with
     * depth, so the hill feels like it is going somewhere.
     */
    private static final Palette[] ZONES = {
        new Palette("Dawn",   0x2BFF2B),
        new Palette("Cyan",   0x1FF5F5),
        new Palette("Violet", 0xB44BFF),
        new Palette("Amber",  0xFFD21F),
        new Palette("Abyss",  0x24FFC4),
        new Palette("Neon",   0xFF2BC8),
    };

    // The blended result, rebuilt in place each tick so no allocation happens per frame.
    private final Palette current;
    private final Vector3f base = new Vector3f();
    private final Vector3f skyTop = new Vector3f();
    private final Vector3f skyBottom = new Vector3f();
    private final Vector3f road = new Vector3f();
    private final Vector3f block = new Vector3f();
    private final Vector3f wall = new Vector3f();
    private final Vector3f kicker = new Vector3f();
    private final Vector3f fog = new Vector3f();
    private final Vector3f accent = new Vector3f();

    private int zoneIndex = 0;
    private String zoneName = ZONES[0].name;

    public Theme() {
        current = ZONES[0];
        update(0f);
    }

    /** Rebuilds the live palette for a distance down the hill. */
    public void update(float distance) {
        if (distance < 0f) {
            distance = 0f;
        }

        float zonePos = distance / ZONE_LENGTH;
        int index = (int) zonePos;
        float within = zonePos - index;

        Palette from = ZONES[index % ZONES.length];
        Palette to = ZONES[(index + 1) % ZONES.length];

        // Hold the palette steady, then blend across the tail of the zone.
        float t = within <= (1f - BLEND_SHARE)
                ? 0f
                : (within - (1f - BLEND_SHARE)) / BLEND_SHARE;

        // Smoothstep, so the change eases in and out instead of starting abruptly.
        t = t * t * (3f - (2f * t));

        mix(from.base, to.base, t, base);
        mix(from.skyTop, to.skyTop, t, skyTop);
        mix(from.skyBottom, to.skyBottom, t, skyBottom);
        mix(from.road, to.road, t, road);
        mix(from.block, to.block, t, block);
        mix(from.wall, to.wall, t, wall);
        mix(from.kicker, to.kicker, t, kicker);
        mix(from.fog, to.fog, t, fog);
        mix(from.accent, to.accent, t, accent);

        zoneIndex = index;
        zoneName = (t > 0.5f ? to : from).name;
    }

    private static void mix(Vector3f a, Vector3f b, float t, Vector3f dest) {
        dest.set(
                a.x + ((b.x - a.x) * t),
                a.y + ((b.y - a.y) * t),
                a.z + ((b.z - a.z) * t));
    }

    /** The one colour this zone is built from. */
    public Vector3f getBase() {
        return base;
    }

    public Vector3f getSkyTop() {
        return skyTop;
    }

    public Vector3f getSkyBottom() {
        return skyBottom;
    }

    public Vector3f getRoad() {
        return road;
    }

    public Vector3f getBlock() {
        return block;
    }

    public Vector3f getWall() {
        return wall;
    }

    public Vector3f getKicker() {
        return kicker;
    }

    public Vector3f getFog() {
        return fog;
    }

    public Vector3f getAccent() {
        return accent;
    }

    /** Which zone the player is in, counting from one, for the HUD. */
    public int getZoneNumber() {
        return zoneIndex + 1;
    }

    public String getZoneName() {
        return zoneName;
    }
}
