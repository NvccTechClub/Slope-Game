package com.slope.game;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * The hill, generated as the player runs down it. The road is an endless run of tiles of
 * equal horizontal length, each with its own constant slope, so the surface is piecewise
 * linear with the joints at tile boundaries.
 *
 * <p>Track space has three coordinates. {@code s} runs downhill from the start and is a
 * horizontal distance, so the surface height is a true function {@code h(s)}.
 * {@code lateral} runs across the road, positive to the right of the screen. Height is
 * measured from the start, so {@code h(0) == 0}.
 *
 * <p>Only a window of tiles around the ball exists at any moment. {@link #streamTo}
 * extends the window forward and drops what is behind, so a run is bounded by the
 * player's nerve rather than by an array length. Generation only ever moves forward,
 * which is what lets one sequential RNG stay reproducible for a given seed.
 *
 * <p>What the road does to you gets harsher with distance: see {@link #difficultyAt}.
 */
public final class Track {
    /** What a tile is. The kind decides whether there is anything to land on. */
    public enum Kind {
        /** The opening stretch: full width, shallow, nothing on it. */
        LEAD_IN,
        /** Ordinary road. */
        ROAD,
        /** Tilted upwards; the ball keeps its speed, leaves the lip and flies. */
        KICKER,
        /** No floor at all. Only ever placed where a kicker has just launched you. */
        CHASM,
    }

    /** A block resting on the road. */
    public static final class Obstacle {
        public final float distance;
        public final float lateral;
        public final float halfLength;  // along the track
        public final float halfWidth;   // across the track
        public final float height;

        Obstacle(float distance, float lateral, float halfLength, float halfWidth, float height) {
            this.distance = distance;
            this.lateral = lateral;
            this.halfLength = halfLength;
            this.halfWidth = halfWidth;
            this.height = height;
        }
    }

    /** Something worth steering towards, rather than away from. */
    public static final class Pickup {
        public enum Type { COIN, BOOST, SLOW }

        public final Type type;
        public final float distance;
        public final float lateral;

        /** Set once collected, so it neither scores nor draws again. */
        boolean taken;

        Pickup(Type type, float distance, float lateral) {
            this.type = type;
            this.distance = distance;
            this.lateral = lateral;
        }

        public boolean isTaken() {
            return taken;
        }
    }

    /** One stretch of road. */
    public static final class Tile {
        public final int index;
        public final Kind kind;
        public final float slope;          // dh/ds across this tile
        public final float heightAtStart;  // height at the uphill edge
        public final float halfWidth;
        public final boolean tunnel;

        /** A lateral span with no floor, or NaN bounds when the tile is whole. */
        public final float holeFrom;
        public final float holeTo;

        public final List<Obstacle> obstacles = new ArrayList<>(2);
        public final List<Pickup> pickups = new ArrayList<>(2);

        Tile(int index, Kind kind, float slope, float heightAtStart, float halfWidth,
             boolean tunnel, float holeFrom, float holeTo) {
            this.index = index;
            this.kind = kind;
            this.slope = slope;
            this.heightAtStart = heightAtStart;
            this.halfWidth = halfWidth;
            this.tunnel = tunnel;
            this.holeFrom = holeFrom;
            this.holeTo = holeTo;
        }

        public boolean hasHole() {
            return !Float.isNaN(holeFrom) && holeTo > holeFrom;
        }
    }

    // ---- Shape of the road ----

    /**
     * Road widths, as a fraction of a full tile, narrowest last.
     *
     * <p>Only one entry, so the road is the same width the whole way down. Narrowing was
     * the first thing difficulty reached for and it made long runs worse rather than
     * harder: the road closed in until there was no room left to steer. Depth is already
     * paid for by speed, blocks, holes, tunnels and chasms. Adding entries here turns the
     * squeeze back on; the generator needs no other change.
     */
    private static final float[] WIDTH_STEPS = { 1.0f };

    /** Downhill pitches in degrees, shallowest last. Negative means the road drops. */
    private static final float[] DESCENTS = { -44f, -40f, -36f, -32f, -26f, -20f };

    private static final float LEAD_IN_SLOPE = -30f;
    private static final float RAMP_SLOPE = 13f;      // the kicker you launch off
    /**
     * Earliest a ramp may follow another. Each one costs three tiles of rising road: two
     * easing onto it and the kicker itself. At four, with the chance below, ramps landed
     * about every six tiles, which left half the hill tilted upwards and read as the road
     * curving up at the end of every stretch.
     */
    private static final int RAMP_GAP_TILES = 10;
    private static final int RAMP_CLEAR_TILES = 3;    // ramp plus its landing, kept open

    /**
     * Tiles spent lifting the road towards a kicker. Without them the road met the
     * ramp at a corner of forty degrees and more, and the ball lost a quarter of its
     * speed slamming into the foot of it. The lip at the far end stays sharp, because
     * that corner is what throws the ball into the air.
     */
    private static final int RAMP_APPROACH_TILES = 2;

    private static final int LEAD_IN_TILES = 4;
    private static final float BLOCK_HALF_LENGTH = 3.0f;
    private static final float BLOCK_HEIGHT = 7.0f;
    private static final float MIN_BLOCK_HALF_WIDTH = 2.0f;

    /**
     * Blocks are discrete obstacles, not walls with a slot in them. A tile is split into
     * lanes and at least one lane is always left open, so a block can always be gone
     * round. The earlier design walled off the whole road except a fixed-width gap, which
     * only worked while the road was narrow: at full width it asked for more lateral
     * speed than the ball has, and the dodge became impossible rather than hard.
     */
    private static final int MAX_BLOCKS_PER_TILE = 3;

    /** Gates along one tile. More than one turns a tile from a decision into a sequence. */
    private static final int MAX_OBSTACLE_ROWS = 3;

    /** Clear tiles after a pattern finishes, so obstacles arrive as discrete things. */
    private static final int REST_AFTER_PATTERN = 1;

    /** Most obstacles in one set. The original tops out at four. */
    private static final int MAX_SET_SIZE = 4;

    // Coins come in runs along a lane, not as lone cubes.
    private static final float COIN_TRAIL_CHANCE = 0.62f;
    private static final int COIN_TRAIL_MIN = 3;
    private static final int COIN_TRAIL_MAX = 6;

    /** A lane has to be worth aiming at, or splitting the road into lanes is pointless. */
    private static final float MIN_LANE_WIDTH = 17.0f;

    /** No single block is wider than this, however wide the road is. */
    private static final float MAX_BLOCK_HALF_WIDTH = 13.0f;

    /** Holes need a way round, so they never eat more than this share of the road. */
    private static final float MAX_HOLE_SHARE = 0.55f;

    /**
     * The narrowest hole worth cutting. A gap no wider than the ball's diameter is
     * bridged rather than fallen into, so anything at or below 6 would be scenery; this
     * leaves enough margin that every hole generated is a hole the ball can drop through.
     */
    private static final float MIN_HOLE_WIDTH = 11.0f;

    // Tunnels are a change of pace, so they want to stay occasional: long runs of them
    // at a short spacing turned a third of the road into tunnel, which stopped reading
    // as a section and started reading as the normal state of the hill.
    private static final int TUNNEL_MIN_TILES = 3;
    private static final int TUNNEL_MAX_TILES = 6;
    private static final int TUNNEL_GAP_TILES = 16;

    // ---- Difficulty ----

    /**
     * Distance over which the hill works up to its full nastiness. Tiles are long, so a
     * run that ends at a few hundred metres is only a handful of tiles; the ramp and the
     * thresholds below are set so a player who is doing well actually meets every feature
     * rather than dying in the easy stretch before any of them appear.
     */
    private static final float DIFFICULTY_RAMP = 2800f;

    /** Nothing dangerous at all before this, so a run always opens cleanly. */
    private static final float HAZARD_START = 150f;
    private static final float HOLE_START = 380f;
    private static final float TUNNEL_START = 560f;
    private static final float CHASM_START = 760f;

    // ---- Streaming window ----

    /** Tiles kept behind the ball, so the camera never looks at an empty hillside. */
    private static final int TILES_BEHIND = 4;

    private final float tileSpan;
    private final float fullHalfWidth;
    private final long seed;
    private final Random rng;

    private final List<Tile> tiles = new ArrayList<>();
    private int firstIndex = 0;

    // Generator state, carried from tile to tile.
    private int widthStep = 0;
    private int widthHold = 0;
    private int descent = 2;
    private int sinceRamp = RAMP_GAP_TILES;
    private int clearTilesLeft = 0;
    private int tunnelTilesLeft = 0;
    private int sinceTunnel = TUNNEL_GAP_TILES;
    private boolean lastWasKicker = false;

    // Which lane was left open on the last tile that had blocks. The open lane used to be
    // chosen fresh each tile, so the safe line teleported across the road from one tile to
    // the next and the player had to cross lanes blind between gates. Carrying it forward
    // turns a sequence of unrelated puzzles into one line to follow.
    private int lastThroughLane = -1;

    // Where the open line actually is, across the road. Held as a distance rather
    // than a lane number because lane numbers are relative to a tile's width, and
    // the road widens and narrows: holding lane 1 through a narrowing is a sideways
    // jump, which is how gates ended up asking for moves the ball could not make.
    private float lastOpenLateral = 0f;

    // The pattern being laid down, and how far through it we are.
    private Archetype pattern = null;
    private int patternStep = 0;
    private int patternDir = 1;
    private int restTilesLeft = 0;

    // Section bookkeeping: which set of which section the next obstacle belongs to.
    private int sectionIndex = 0;
    private int tierCursor = 0;
    private int obstaclesLeftInSet = 1;
    private boolean breatherDue = false;
    private boolean breatherBoostDue = false;

    private boolean kickerDue = false;
    private int kickerApproach = 0;
    private float approachFrom = 0f;
    private float nextHeight = 0f;
    private int nextIndex = 0;

    /** Running totals, for the HUD and for scoring. */
    private int coinsPlaced = 0;

    public Track(float tileSpan, float fullHalfWidth, long seed) {
        this.tileSpan = tileSpan;
        this.fullHalfWidth = fullHalfWidth;
        this.seed = seed;
        this.rng = new Random(seed);

        // Tile 0 sits behind the start and tile 1 straddles it. Both are lead-in, and
        // the pair is generated by hand so h(0) can be pinned to exactly zero.
        float leadSlope = tanDeg(LEAD_IN_SLOPE);

        // h(0) = heightAtStart(1) + slope*(0 - tileStart(1)) = 0
        float heightAtStart1 = -leadSlope * tileSpan * 0.5f;
        float heightAtStart0 = heightAtStart1 - (leadSlope * tileSpan);

        tiles.add(new Tile(0, Kind.LEAD_IN, leadSlope, heightAtStart0, fullHalfWidth,
                false, Float.NaN, Float.NaN));
        tiles.add(new Tile(1, Kind.LEAD_IN, leadSlope, heightAtStart1, fullHalfWidth,
                false, Float.NaN, Float.NaN));

        firstIndex = 0;
        nextIndex = 2;
        nextHeight = heightAtStart1 + (leadSlope * tileSpan);
    }

    private static float tanDeg(float degrees) {
        return (float) Math.tan(Math.toRadians(degrees));
    }

    /**
     * How hard the hill is trying at a given distance, from 0 at the top to 1 once the
     * player is a long way down. Everything that can hurt scales off this, which is what
     * turns a flat random layout into a run that builds.
     */
    public float difficultyAt(float s) {
        float d = s / DIFFICULTY_RAMP;
        return d < 0f ? 0f : (d > 1f ? 1f : d);
    }

    /**
     * Makes sure the road exists from a little behind {@code s} to {@code lookAhead}
     * past it, and forgets tiles that are well behind. Called every tick.
     */
    public void streamTo(float s, float lookAhead) {
        int needed = tileIndexAt(s + lookAhead) + 1;

        while (nextIndex <= needed) {
            generateNext();
        }

        int keepFrom = tileIndexAt(s) - TILES_BEHIND;

        while (firstIndex < keepFrom && tiles.size() > 1) {
            tiles.remove(0);
            firstIndex++;
        }
    }

    /** Appends one tile, taking its shape from the generator state and the difficulty. */
    private void generateNext() {
        final int i = nextIndex;
        final float centre = tileSpan * (i - 1);
        final float diff = difficultyAt(centre);
        final boolean hazardsAllowed = centre > HAZARD_START;

        Kind kind = Kind.ROAD;
        float slope;
        float halfWidth;
        boolean tunnel = false;
        float holeFrom = Float.NaN;
        float holeTo = Float.NaN;

        if (i < LEAD_IN_TILES) {
            kind = Kind.LEAD_IN;
            slope = tanDeg(LEAD_IN_SLOPE);
            halfWidth = fullHalfWidth;
            addTile(kind, slope, halfWidth, false, holeFrom, holeTo);
            return;
        }

        // A chasm only ever follows a kicker, so there is always the speed to clear it.
        boolean wantChasm = lastWasKicker
                && centre > CHASM_START
                && rng.nextFloat() < (0.18f + (0.35f * diff));

        if (wantChasm) {
            lastWasKicker = false;
            clearTilesLeft = Math.max(clearTilesLeft, 1);

            // The void keeps the previous slope so the landing lines up with the launch.
            addTile(Kind.CHASM, tanDeg(DESCENTS[descent]), fullHalfWidth,
                    false, Float.NaN, Float.NaN);
            return;
        }

        // Tunnel runs: the road cannot be fallen off inside one, but it is tight and the
        // blocks keep coming, so it trades one hazard for another.
        if (tunnelTilesLeft > 0) {
            tunnelTilesLeft--;
            tunnel = true;
        } else if (breatherDue && hazardsAllowed && centre > TUNNEL_START
                && clearTilesLeft <= 0 && sinceTunnel >= TUNNEL_GAP_TILES) {
            // The speedtunnel: the original ends every section with one, so it doubles as
            // a rest and as the punctuation that makes the escalation legible.
            breatherDue = false;
            tunnelTilesLeft = TUNNEL_MIN_TILES
                    + rng.nextInt(TUNNEL_MAX_TILES - TUNNEL_MIN_TILES + 1) - 1;
            sinceTunnel = 0;
            tunnel = true;
        }

        sinceTunnel++;

        boolean inClearZone = clearTilesLeft > 0;

        if (inClearZone) {
            clearTilesLeft--;
        }

        // Lifting the road onto a kicker, one step at a time.
        if (kickerApproach > 0) {
            float t = (RAMP_APPROACH_TILES - kickerApproach + 1f) / (RAMP_APPROACH_TILES + 1f);
            kickerApproach--;

            lastWasKicker = false;
            addTile(Kind.ROAD, tanDeg(approachFrom + ((RAMP_SLOPE - approachFrom) * t)),
                    fullHalfWidth, false, Float.NaN, Float.NaN);
            return;
        }

        // The ramp itself, once the approach has brought the road up to meet it.
        if (kickerDue) {
            kickerDue = false;
            lastWasKicker = true;

            addTile(Kind.KICKER, tanDeg(RAMP_SLOPE), fullHalfWidth,
                    false, Float.NaN, Float.NaN);
            return;
        }

        // Kickers. Not inside a tunnel, and never back-to-back.
        if (!tunnel && !inClearZone && hazardsAllowed
                && sinceRamp >= RAMP_GAP_TILES
                && rng.nextFloat() < (0.20f + (0.15f * diff))) {
            sinceRamp = 0;

            // Schedule it, then spend the next few tiles easing the road up to it.
            kickerDue = true;
            kickerApproach = RAMP_APPROACH_TILES;
            approachFrom = DESCENTS[descent];

            // Keep the approach, the ramp and the ground it throws you onto open; there
            // is no steering worth the name while airborne.
            clearTilesLeft = RAMP_APPROACH_TILES + RAMP_CLEAR_TILES;
            widthStep = 0;
            widthHold = 0;

            // This tile is the first step of the approach.
            float t = 1f / (RAMP_APPROACH_TILES + 1f);
            kickerApproach--;
            lastWasKicker = false;

            addTile(Kind.ROAD, tanDeg(approachFrom + ((RAMP_SLOPE - approachFrom) * t)),
                    fullHalfWidth, false, Float.NaN, Float.NaN);
            return;
        }

        sinceRamp++;
        lastWasKicker = false;

        // Descent. Steeper pitches unlock as the run goes on, so speed climbs with it.
        int steepestAllowed = 0;
        int shallowestAllowed = DESCENTS.length - 1;
        int hardestByDiff = (int) Math.floor((1f - diff) * (DESCENTS.length - 1));
        steepestAllowed = Math.max(0, Math.min(shallowestAllowed, hardestByDiff));

        int drift = rng.nextInt(3) - 1;
        descent = clampInt(descent + drift, steepestAllowed, shallowestAllowed);
        slope = tanDeg(DESCENTS[descent]);

        // Width. A tunnel is always full width, since its walls do the squeezing.
        if (tunnel || inClearZone) {
            halfWidth = fullHalfWidth;
            widthStep = 0;
            widthHold = 0;
        } else {
            // How narrow the road is allowed to get rises with difficulty.
            int narrowestStep = 1 + (int) Math.floor(diff * (WIDTH_STEPS.length - 1));
            narrowestStep = Math.min(WIDTH_STEPS.length - 1, narrowestStep);

            if (widthHold <= 0) {
                int wDrift = rng.nextInt(3) - 1;
                widthStep = clampInt(widthStep + wDrift, 0, narrowestStep);
                widthHold = 2 + rng.nextInt(3);
            }

            widthHold--;
            widthStep = Math.min(widthStep, narrowestStep);
            halfWidth = fullHalfWidth * WIDTH_STEPS[widthStep];
        }

        // Holes are switched off: the road has no missing floor to steer round. The
        // tile still carries the fields and the physics and the renderer still honour
        // them, so cutting them again is a matter of filling holeFrom and holeTo here.
        // Chasms are a separate thing and still generate; they only ever follow a kicker,
        // so they are jumped rather than dodged.
        addTile(kind, slope, halfWidth, tunnel, holeFrom, holeTo);
    }

    /** Finishes a tile: appends it, advances the height, then dresses it with props. */
    private void addTile(Kind kind, float slope, float halfWidth, boolean tunnel,
                         float holeFrom, float holeTo) {
        Tile tile = new Tile(nextIndex, kind, slope, nextHeight, halfWidth,
                tunnel, holeFrom, holeTo);

        tiles.add(tile);
        nextHeight += slope * tileSpan;
        nextIndex++;

        populate(tile);
    }

    /** Puts the blocks and the pickups on a finished tile. */
    private void populate(Tile tile) {
        final float centre = tileSpan * (tile.index - 1);
        final float diff = difficultyAt(centre);

        // Anything that is not plain road interrupts a pattern: a kicker, a chasm or a
        // tunnel breaks the read, and resuming a half-finished weave on the far side of
        // one would be unreadable.
        if (tile.kind != Kind.ROAD || tile.tunnel || centre <= HAZARD_START) {
            abortPattern();

            if (tile.tunnel && breatherBoostDue) {
                // The speedtunnel's pad: a rest, and a shove into the next section.
                breatherBoostDue = false;
                tile.pickups.add(new Pickup(Pickup.Type.BOOST, centre, 0f));
            } else {
                maybeAddPickup(tile, centre, diff, true);
            }

            return;
        }

        // A lane change costs more road than one tile is long, so a gate that moves the
        // line is always followed by a clear tile. Without that the pattern would be
        // asking for a correction the ball has no room to make.
        if (restTilesLeft > 0) {
            restTilesLeft--;
            layCoinTrail(tile, centre, holdLine(tile));
            maybeAddPickup(tile, centre, diff, false);
            return;
        }

        if (pattern == null && !beginPattern(tile)) {
            maybeAddPickup(tile, centre, diff, false);
            return;
        }

        applyPatternStep(tile, centre, diff);
    }

    // ---- Obstacle patterns ----

    /**
     * How a gate blocks the road around the lane that stays open. Keeping the shapes
     * named is the point: the original game is built from a vocabulary of recognisable
     * obstacles, and a player who can name what is coming can learn to beat it. Uniform
     * random blocks are varied on paper and identical to play.
     */
    private enum GateStyle {
        /** Only the outer lanes. The middle stays open: a corridor to hold. */
        EDGES,
        /** The middle lanes. You have to commit to one side or the other. */
        CENTRE,
        /** Everything but the open lane. The tightest gate there is. */
        ALL_BUT_OPEN,
        /** A random subset, so the shape is not always readable in advance. */
        SCATTER,
    }

    /** When in a run an obstacle is allowed to appear. */
    public enum Tier { EARLY, MIDDLE, LATE }

    /** One gate of a pattern: how it blocks, and how far the open lane moves to reach it. */
    private record Step(GateStyle style, int delta) { }

    /**
     * The obstacle vocabulary, after the shapes the original game is assembled from.
     * Each is a sequence of gates spread over consecutive tiles rather than crammed into
     * one, because a single lane change costs more road than one tile is long.
     */
    public enum Archetype {
        /** A corridor with both edges walled. Hold the middle. */
        STRAIGHT(Tier.EARLY, new Step(GateStyle.EDGES, 0), new Step(GateStyle.EDGES, 0)),

        /** Blocks drift to one side, pushing the line across with them. */
        SLANT(Tier.EARLY, new Step(GateStyle.SCATTER, 0), new Step(GateStyle.SCATTER, 1),
                new Step(GateStyle.SCATTER, 1)),

        /** Scattered blocks with no shape to read. The plain case. */
        RNG_BLOCKS(Tier.EARLY, new Step(GateStyle.SCATTER, 0), new Step(GateStyle.SCATTER, 0)),

        /**
         * The iconic one: a pair on the edges, then a block through the middle, then the
         * edges again. Middle, out to a side, back to the middle.
         */
        TREBLOCK(Tier.MIDDLE, new Step(GateStyle.EDGES, 0), new Step(GateStyle.CENTRE, 1),
                new Step(GateStyle.EDGES, -1)),

        /** Long, narrow and winding: the line walks steadily across the road. */
        SNAKE(Tier.MIDDLE, new Step(GateStyle.ALL_BUT_OPEN, 0),
                new Step(GateStyle.ALL_BUT_OPEN, 1), new Step(GateStyle.ALL_BUT_OPEN, 1),
                new Step(GateStyle.ALL_BUT_OPEN, 1)),

        /** Out and straight back. */
        WEAVE(Tier.MIDDLE, new Step(GateStyle.ALL_BUT_OPEN, 0),
                new Step(GateStyle.ALL_BUT_OPEN, 1), new Step(GateStyle.ALL_BUT_OPEN, -1)),

        /** A tight line held for a long time, with no let-up. */
        NARROWS(Tier.LATE, new Step(GateStyle.ALL_BUT_OPEN, 0),
                new Step(GateStyle.ALL_BUT_OPEN, 0), new Step(GateStyle.ALL_BUT_OPEN, 0),
                new Step(GateStyle.ALL_BUT_OPEN, 0)),

        /** Side to side, over and over. */
        ZIGZAG(Tier.LATE, new Step(GateStyle.ALL_BUT_OPEN, 0),
                new Step(GateStyle.ALL_BUT_OPEN, 1), new Step(GateStyle.ALL_BUT_OPEN, -1),
                new Step(GateStyle.ALL_BUT_OPEN, 1), new Step(GateStyle.ALL_BUT_OPEN, -1)),

        /** Every shape in one run: corridor, commit to a side, then thread the needle. */
        GAUNTLET(Tier.LATE, new Step(GateStyle.EDGES, 0), new Step(GateStyle.CENTRE, 1),
                new Step(GateStyle.ALL_BUT_OPEN, -1), new Step(GateStyle.ALL_BUT_OPEN, 0));

        private final Tier tier;
        private final Step[] steps;

        Archetype(Tier tier, Step... steps) {
            this.tier = tier;
            this.steps = steps;
        }

        public Tier getTier() {
            return tier;
        }

        public int length() {
            return steps.length;
        }
    }

    private static final Archetype[] ALL_ARCHETYPES = Archetype.values();

    /** Picks the lane a style leaves open, as near as possible to the line so far. */
    private int openLaneFor(GateStyle style, int lanes, int from, int delta) {
        switch (style) {
            case EDGES: {
                // The middle is open; take the middle lane closest to the current line.
                int best = -1;

                for (int lane = 1; lane < lanes - 1; lane++) {
                    if (best < 0 || Math.abs(lane - from) < Math.abs(best - from)) {
                        best = lane;
                    }
                }

                return best < 0 ? clampInt(from, 0, lanes - 1) : best;
            }
            case CENTRE: {
                // Only the outermost lanes survive; take whichever is nearer.
                int high = lanes - 1;
                return (Math.abs(from) <= Math.abs(high - from)) ? 0 : high;
            }
            default:
                return clampInt(from + (delta * patternDir), 0, lanes - 1);
        }
    }

    /** Starts a pattern suited to where the run has got to. Returns false if it cannot. */
    private boolean beginPattern(Tile tile) {
        if (laneCount(tile) < 2) {
            return false;
        }

        Tier tier = nextTier();
        Archetype chosen = null;
        int seen = 0;

        // Reservoir pick, so no list has to be built per tile.
        for (Archetype a : ALL_ARCHETYPES) {
            if (a.getTier() != tier) {
                continue;
            }

            seen++;

            if (rng.nextInt(seen) == 0) {
                chosen = a;
            }
        }

        if (chosen == null) {
            return false;
        }

        pattern = chosen;
        patternStep = 0;
        patternDir = rng.nextBoolean() ? 1 : -1;
        return true;
    }

    private void abortPattern() {
        pattern = null;
        patternStep = 0;
    }

    /** Lays down one gate of the running pattern. */
    private void applyPatternStep(Tile tile, float centre, float diff) {
        final int lanes = laneCount(tile);

        if (lanes < 2) {
            abortPattern();
            return;
        }

        final float laneWidth = (tile.halfWidth * 2f) / lanes;
        final float blockHalf = Math.min(MAX_BLOCK_HALF_WIDTH, (laneWidth * 0.5f) * 0.72f);

        if (blockHalf < MIN_BLOCK_HALF_WIDTH) {
            abortPattern();
            return;
        }

        final Step step = pattern.steps[patternStep];
        final int from = laneNearest(tile, lastOpenLateral, lanes, laneWidth);

        int open = openLaneFor(step.style(), lanes, from, step.delta());

        // Never ask for more than one lane of movement per gate. A lane change costs
        // more road than a tile is long, and a gate that wants two is not a hard gate,
        // it is an impossible one. The open lane is always left clear whatever the
        // style says, so clamping only reshapes the gate, it never seals it.
        open = clampInt(open, from - 1, from + 1);
        open = avoidHole(tile, open, lanes, laneWidth);
        open = clampInt(open, from - 1, from + 1);

        final boolean moved = open != from;

        for (int lane = 0; lane < lanes; lane++) {
            if (lane == open || !blocksLane(step.style(), lane, lanes, diff)) {
                continue;
            }

            float lateral = laneCentre(tile, lane, laneWidth);

            // A hole already removes this ground, so a block there would float.
            if (tile.hasHole() && lateral > tile.holeFrom && lateral < tile.holeTo) {
                continue;
            }

            tile.obstacles.add(new Obstacle(centre, lateral,
                    BLOCK_HALF_LENGTH, blockHalf, BLOCK_HEIGHT));
        }

        lastThroughLane = open;
        lastOpenLateral = laneCentre(tile, open, laneWidth);
        layCoinTrail(tile, centre, lastOpenLateral);

        patternStep++;

        if (patternStep >= pattern.length()) {
            abortPattern();
            restTilesLeft = REST_AFTER_PATTERN;
        } else if (moved) {
            // The next gate wants the ball somewhere else; give it the room to get there.
            restTilesLeft = 1;
        }
    }

    /** Whether a style blocks a given lane. */
    private boolean blocksLane(GateStyle style, int lane, int lanes, float diff) {
        switch (style) {
            case EDGES:
                return lane == 0 || lane == lanes - 1;
            case CENTRE:
                return lane > 0 && lane < lanes - 1;
            case ALL_BUT_OPEN:
                return true;
            default:
                return rng.nextFloat() < (0.45f + (0.35f * diff));
        }
    }

    /** Nudges the open lane off a hole, since standing on nothing is not an option. */
    private int avoidHole(Tile tile, int open, int lanes, float laneWidth) {
        if (!tile.hasHole()) {
            return open;
        }

        for (int offset = 0; offset < lanes; offset++) {
            for (int dir = -1; dir <= 1; dir += 2) {
                int lane = open + (offset * dir);

                if (lane < 0 || lane >= lanes) {
                    continue;
                }

                float c = laneCentre(tile, lane, laneWidth);

                if (c <= tile.holeFrom || c >= tile.holeTo) {
                    return lane;
                }

                if (offset == 0) {
                    break;
                }
            }
        }

        return open;
    }

    /** How many lanes this tile's width supports. */
    private int laneCount(Tile tile) {
        int lanes = MAX_BLOCKS_PER_TILE + 1;
        float width = tile.halfWidth * 2f;

        while (lanes > 1 && (width / lanes) < MIN_LANE_WIDTH) {
            lanes--;
        }

        return lanes;
    }

    private float laneCentre(Tile tile, int lane, float laneWidth) {
        return -tile.halfWidth + (laneWidth * (lane + 0.5f));
    }

    /** The lane whose centre sits nearest a point across the road. */
    private int laneNearest(Tile tile, float lateral, int lanes, float laneWidth) {
        int best = 0;
        float bestDist = Float.MAX_VALUE;

        for (int lane = 0; lane < lanes; lane++) {
            float d = Math.abs(laneCentre(tile, lane, laneWidth) - lateral);

            if (d < bestDist) {
                bestDist = d;
                best = lane;
            }
        }

        return best;
    }

    /** The current line, pulled back onto this tile's road if it has narrowed. */
    private float holdLine(Tile tile) {
        float limit = Math.max(0f, tile.halfWidth - 4f);
        lastOpenLateral = Math.max(-limit, Math.min(limit, lastOpenLateral));
        return lastOpenLateral;
    }

    /**
     * Drops a power-up on a tile. Coins are not placed here any more: they come in trails
     * along the open lane, where they mark the route. This is only for the boost and the
     * slow-mo, which were showing up in well under half of all runs when they had to
     * share these rolls with coins.
     */
    private void maybeAddPickup(Tile tile, float centre, float diff, boolean generous) {
        float chance = generous ? 0.30f : 0.22f;

        if (rng.nextFloat() >= chance) {
            return;
        }

        // Keep pickups inside the road and off the void.
        float half = Math.max(2f, tile.halfWidth - 4f);
        float lateral = (rng.nextFloat() * 2f - 1f) * half;

        if (tile.hasHole() && lateral > tile.holeFrom && lateral < tile.holeTo) {
            // Sitting over a hole would be a trap rather than a reward.
            lateral = (tile.holeFrom > -half) ? tile.holeFrom - 2f : tile.holeTo + 2f;

            if (Math.abs(lateral) > half) {
                return;
            }
        }

        // Slow-mo is the rarer of the two, and is most useful where the hill is worst.
        Pickup.Type type = (rng.nextFloat() < 0.62f) ? Pickup.Type.BOOST : Pickup.Type.SLOW;
        tile.pickups.add(new Pickup(type, centre, lateral));
    }

    // ---- Section structure ----

    /**
     * Which tier the next obstacle comes from, following the original's shape: a run is
     * made of sections, a section is one set of obstacles per unlocked tier, and the sets
     * grow as the run goes on. The first section is a single easy obstacle; by the fourth
     * the player is getting four of everything, late obstacles included.
     *
     * <p>A section ends with a breather, which is what the tunnel is for.
     */
    private Tier nextTier() {
        if (obstaclesLeftInSet <= 0) {
            tierCursor++;

            if (tierCursor >= unlockedTiers()) {
                // Section complete: take a breather, then start the next one.
                tierCursor = 0;
                sectionIndex++;
                breatherDue = true;
                breatherBoostDue = true;
            }

            obstaclesLeftInSet = Math.min(MAX_SET_SIZE, sectionIndex + 1);
        }

        obstaclesLeftInSet--;
        return Tier.values()[Math.min(tierCursor, unlockedTiers() - 1)];
    }

    private int unlockedTiers() {
        return Math.min(Tier.values().length, sectionIndex + 1);
    }

    /**
     * A run of coins along one lane. Dense enough to read as a line to follow rather than
     * as scattered collectables: a single floating cube per tile was passed four times a
     * run and collected once, which is not a reward system anybody can see.
     */
    private void layCoinTrail(Tile tile, float centre, float lateral) {
        if (rng.nextFloat() >= COIN_TRAIL_CHANCE) {
            return;
        }

        int count = COIN_TRAIL_MIN + rng.nextInt(COIN_TRAIL_MAX - COIN_TRAIL_MIN + 1);
        float spread = tileSpan * 0.7f;

        for (int i = 0; i < count; i++) {
            float t = (count == 1) ? 0.5f : (i / (float) (count - 1));
            tile.pickups.add(new Pickup(Pickup.Type.COIN,
                    centre + ((t - 0.5f) * spread), lateral));
            coinsPlaced++;
        }
    }


    private static int clampInt(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    // ---- Surface queries ----

    /** Surface height at the given distance, measured from the start of the run. */
    public float heightAt(float s) {
        Tile tile = tileAt(s);
        float local = s - tileStart(tile.index);

        // Outside the window the surface simply stops changing, so a ball that ran past
        // the generated road still has something finite to fall away from.
        local = Math.max(0f, Math.min(tileSpan, local));

        return tile.heightAtStart + (tile.slope * local);
    }

    /** Rise over run of the road at the given distance. */
    public float slopeAt(float s) {
        return tileAt(s).slope;
    }

    /**
     * Cosine of the road's pitch, which converts a height difference into a distance
     * measured perpendicular to the surface.
     */
    public float normalCosAt(float s) {
        float slope = slopeAt(s);
        return (float) (1.0 / Math.sqrt(1.0 + (slope * slope)));
    }

    /** Half the road width at the given distance. */
    public float halfWidthAt(float s) {
        return tileAt(s).halfWidth;
    }

    public boolean isTunnelAt(float s) {
        return tileAt(s).tunnel;
    }

    /**
     * Whether there is floor under a point. False off the sides, in a hole, or anywhere
     * in a chasm. This is the single question the physics asks to decide if the ball is
     * still supported, so holes and edges behave identically by construction.
     */
    public boolean isSolid(float s, float lateral) {
        return isSolid(s, lateral, 0f);
    }

    /**
     * Whether a ball of the given radius is held up at this point.
     *
     * <p>An edge is still decided by the centre: a sphere touches the ground directly
     * below its middle, so it is supported until its centre passes the lip, however much
     * of it overhangs. A hole is different — a gap narrower than the ball cannot swallow
     * it, because the ball rests on both lips at once and rolls across. Testing the
     * centre alone dropped it into gaps it physically bridges.
     *
     * @param radius the ball's radius; pass 0 for the plain geometric test
     */
    public boolean isSolid(float s, float lateral, float radius) {
        Tile tile = tileAt(s);

        if (tile.kind == Kind.CHASM) {
            return false;
        }

        if (Math.abs(lateral) > tile.halfWidth) {
            return false;
        }

        if (tile.hasHole() && lateral > tile.holeFrom && lateral < tile.holeTo) {
            // Bridged if the gap is no wider than the ball.
            return (tile.holeTo - tile.holeFrom) <= (radius * 2f);
        }

        return true;
    }

    /**
     * Returns the block the ball is touching, or null. The ball and the blocks both sit
     * on the road, so this is a circle against a rectangle in track space. Only the tiles
     * either side of the ball are examined, which keeps the test flat in run length.
     */
    public Obstacle hitTest(float s, float lateral, float radius) {
        int centreTile = tileIndexAt(s);

        for (int i = centreTile - 1; i <= centreTile + 1; i++) {
            Tile tile = tileOrNull(i);

            if (tile == null) {
                continue;
            }

            for (int k = 0; k < tile.obstacles.size(); k++) {
                Obstacle o = tile.obstacles.get(k);

                float dx = Math.max(0f, Math.abs(s - o.distance) - o.halfLength);
                float dy = Math.max(0f, Math.abs(lateral - o.lateral) - o.halfWidth);

                if ((dx * dx) + (dy * dy) < radius * radius) {
                    return o;
                }
            }
        }

        return null;
    }

    /**
     * The block directly beneath a point, or null. Support uses the centre, the same way
     * an edge does: a sphere rests on whatever is under its middle, so this is what
     * decides whether the ball is standing on a block rather than beside it.
     */
    public Obstacle blockUnder(float s, float lateral) {
        int centreTile = tileIndexAt(s);

        for (int i = centreTile - 1; i <= centreTile + 1; i++) {
            Tile tile = tileOrNull(i);

            if (tile == null) {
                continue;
            }

            for (int k = 0; k < tile.obstacles.size(); k++) {
                Obstacle o = tile.obstacles.get(k);

                if (Math.abs(s - o.distance) <= o.halfLength
                        && Math.abs(lateral - o.lateral) <= o.halfWidth) {
                    return o;
                }
            }
        }

        return null;
    }

    /**
     * Collects any pickup the ball is on top of, marking it taken. Returns null when
     * there is nothing there.
     */
    public Pickup collect(float s, float lateral, float radius) {
        int centreTile = tileIndexAt(s);

        for (int i = centreTile - 1; i <= centreTile + 1; i++) {
            Tile tile = tileOrNull(i);

            if (tile == null) {
                continue;
            }

            for (int k = 0; k < tile.pickups.size(); k++) {
                Pickup p = tile.pickups.get(k);

                if (p.taken) {
                    continue;
                }

                float dx = s - p.distance;
                float dy = lateral - p.lateral;
                float reach = radius + 3.5f; // pickups are generous on purpose

                if ((dx * dx) + (dy * dy) < reach * reach) {
                    p.taken = true;
                    return p;
                }
            }
        }

        return null;
    }

    // ---- Tile access, for the renderer ----

    public int getFirstTileIndex() {
        return firstIndex;
    }

    public int getLastTileIndex() {
        return firstIndex + tiles.size() - 1;
    }

    /** The tile with the given absolute index, or null when it is outside the window. */
    public Tile tileOrNull(int index) {
        int offset = index - firstIndex;
        return (offset < 0 || offset >= tiles.size()) ? null : tiles.get(offset);
    }

    /** Distance of a tile's centre; tile 0 sits behind the start. */
    public float getTileCentre(int index) {
        return tileSpan * (index - 1);
    }

    /** Uphill edge of a tile, as a distance from the start. */
    public float tileStart(int index) {
        return tileSpan * (index - 1.5f);
    }

    public int tileIndexAt(float s) {
        return (int) Math.floor((s / tileSpan) + 1.5f);
    }

    public float getTileSpan() {
        return tileSpan;
    }

    public float getFullHalfWidth() {
        return fullHalfWidth;
    }

    public long getSeed() {
        return seed;
    }

    public int getCoinsPlaced() {
        return coinsPlaced;
    }

    /** The tile covering s, clamped to the generated window. */
    private Tile tileAt(float s) {
        int index = tileIndexAt(s);
        int offset = clampInt(index - firstIndex, 0, tiles.size() - 1);
        return tiles.get(offset);
    }
}
