package com.slope.game;

import org.joml.Quaternionf;
import org.joml.Vector3f;

/**
 * The ball, and what the hill does to it.
 *
 * <p>The simulation runs in world space, but every question about the road is asked in
 * track space: {@code s} is horizontal distance down the hill and {@code lateral} is the
 * offset across it. The road is a {@link Track}, so the surface under the ball is
 * whatever the generator decided, including the places where there is no surface at all.
 *
 * <p>A fixed plane is still supported for when no track has been built; that path is kept
 * so the renderer and camera have something sane to sit on before the first run.
 */
public class Physics {
    /**
     * The simulation the game is running. Everything used to be static, which meant one
     * ball, one world, and no way to run a second: no replay, no ghost, no preview, and
     * no test that could drive the physics without disturbing the live game. The state is
     * per-instance now; this is simply the instance the game plays.
     */
    private static Physics active = new Physics();

    public static Physics active() {
        return active;
    }

    /** Points the game at a different simulation. Mainly for tests and replays. */
    public static void setActive(Physics simulation) {
        active = simulation;
    }

    // Gravity is scaled to the world, not to metres: tiles are ~74 across and the ball
    // rolls at 40-150, so earth gravity left it floating over every crest it crossed.
    private static final float GRAVITY = -55.0f;
    private static final float RADIUS = 3.0f;
    private static final float RESTITUTION = 0.2f;   // 0=no bounce, 1=perfect
    private static final float FRICTION = 2.0f;      // downhill damping (≈ m/s^2)
    private static final Vector3f GRAVITY_VEC = new Vector3f(0f, GRAVITY, 0f);

    // Steering and pacing. These are the knobs that decide how a run feels.
    private static final float STEER_ACCEL = 70.0f;        // lateral accel while steering
    private static final float LATERAL_FRICTION = 18.0f;   // lateral damping when not steering
    private static final float MAX_LATERAL_SPEED = 40.0f;
    private static final float BOOST_ACCEL = 3.0f;         // extra downhill accel
    private static final float MAX_DOWNHILL_SPEED = 100.0f;

    // Top speed climbs the further the run goes, so a long run keeps getting harder
    // rather than settling at one pace. Capped, or it ends up outrunning the camera.
    //
    // Tuned against how far runs actually get: a typical run ends around 1300 and a good
    // one around 2500, so the ramp has to bite inside that range to be felt at all. It
    // reaches its cap at about 2900, which is deeper than most runs ever go.
    private static final float SPEED_RAMP = 0.018f;     // extra top speed per unit run
    private static final float SPEED_RAMP_CAP = 100.0f; // most the ramp can ever add
    private static final float FALL_DEATH_DEPTH = 25.0f;   // how far below the road is dead
    private static final float DRAG = 0.08f;               // gentle; the ceiling sets cruise speed
    private static final float SNAP_GAP = 2.5f;            // stay glued within this of the road
    private static final float SNAP_SIN = 0.31f;           // ...unless rising steeper than ~18 deg

    /**
     * How far the contact solver may lift the ball back onto the road in one step.
     *
     * <p>Without a limit the solver applied the whole penetration, however deep: a ball
     * that had dropped into a gap and travelled under the road was teleported back up
     * onto the surface the moment solid ground passed over it. That made every gap less
     * lethal the faster you were going, because speed shortened the fall before the far
     * lip rescued you — exactly backwards, since the run only gets faster.
     *
     * <p>Legitimate penetration in one step is small: the ball lands at well under a
     * unit per tick, and a steep tile joint lifts the road about one unit under it. A
     * limit of one ball radius leaves room for both and still catches a real fall.
     */
    private static final float MAX_CONTACT_CORRECTION = RADIUS;

    /** Once the ball is under the road with nothing holding it up, how long until the run ends. */
    private static final float FALL_GRACE = 0.6f;

    /**
     * How far below a block's top face the ball may be and still count as landing on it
     * rather than driving into its side. One radius: any deeper and the ball's middle is
     * below the lip, which is a side-on hit however it got there.
     */
    private static final float BLOCK_TOP_TOLERANCE = RADIUS;

    /** While airborne the player keeps this share of their steering authority. */
    private static final float AIR_STEER_FACTOR = 0.35f;

    // Jumping.
    private static final float JUMP_SPEED = 34.0f;
    private static final float AIR_SPIN_DECAY = 0.25f;  // how fast spin bleeds off in flight
    private static final float SPIN_GRIP = 18.0f;       // how fast spin catches the ground
    private static final float COYOTE_TIME = 0.12f;   // grace after leaving the ground
    private static final float JUMP_BUFFER = 0.12f;   // grace before touching down

    /**
     * How long after a jump the ground-snap is ignored. The snap decides whether the
     * ball is leaving by the angle it is rising at, which is the right question for a
     * crest but the wrong one for a jump: at speed the same upward push is a far
     * shallower angle, so a jump at 100 was grabbed back after three frames.
     */
    private static final float JUMP_LOCKOUT = 0.20f;

    // Hitting a block.
    /** Closing speed into a block above which the run ends rather than bounces. */
    private static final float FATAL_CLOSING_SPEED = 20.0f;
    private static final float GLANCE_SPEED_SCRUB = 0.62f;  // speed kept after a brush
    private static final float GLANCE_INVULN = 0.25f;

    // Pickups.
    private static final float COIN_SCORE = 25f;
    private static final float BOOST_DURATION = 2.5f;
    private static final float BOOST_IMPULSE = 20.0f;
    private static final float BOOST_EXTRA_ACCEL = 34.0f;
    private static final float BOOST_SPEED_BONUS = 35.0f;
    private static final float SLOW_DURATION = 3.0f;

    /** How far ahead the road is kept generated. Must exceed the camera's reach. */
    private static final float STREAM_AHEAD = 900f;

    /** Tunnel walls shove the ball back rather than letting it leave the road. */
    private static final float WALL_RESTITUTION = 0.45f;

    public enum DeathCause { NONE, FELL, CRASHED }

    private final Vector3f spherePos = new Vector3f(0f, 5f, 0f);
    private final Vector3f velocity = new Vector3f();
    private final Quaternionf orientation = new Quaternionf();

    // How the ball is spinning right now. Held across a jump rather than recomputed
    // from the road, because a ball in the air keeps the spin it took off with.
    private final Vector3f spinAxis = new Vector3f(0f, 0f, 1f);
    private float spinRate = 0f;

    // World layout when a track is set: the road runs along -X, across is -Z, up is +Y.
    private static final Vector3f TRACK_RIGHT = new Vector3f(0f, 0f, -1f);

    // Scratch for the step. The simulation runs on one thread at a fixed rate, so a
    // handful of reused vectors beats allocating a fresh one per call; the old code
    // churned a few kilobytes every tick, all of it immediately garbage. Each has one
    // owner to make aliasing impossible: contact, spin and jump never share a slot.
    private final Vector3f sPrevVelocity = new Vector3f();
    private final Vector3f sNormal = new Vector3f();
    private final Vector3f sForward = new Vector3f();
    private final Vector3f sTangent = new Vector3f();
    private final Vector3f sSpinNormal = new Vector3f();
    private final Vector3f sSpinPlanar = new Vector3f();
    private final Vector3f sEventNormal = new Vector3f();

    private Track track = null;
    private final Vector3f trackOrigin = new Vector3f();
    private float steerInput = 0f;
    private float distance = 0f;
    private boolean alive = true;
    private boolean onGround = false;

    // Latched once the ball has dropped below the road with nothing under it. A ball
    // that has fallen past a lip is beneath the surface, not resting on it, so the
    // contact solver must leave it alone rather than pulling it back up.
    private boolean fellThrough = false;
    private float fallTimer = 0f;

    // Height above the road of whatever is currently holding the ball up: zero on the
    // road itself, a block's height while standing on one.
    private float supportHeight = 0f;

    // Jump and air state.
    private float timeSinceGround = 999f;
    private float jumpRequestedAt = -999f;
    private float clock = 0f;
    private boolean jumpArmed = true;

    // Power-ups and penalties.
    private float boostTimer = 0f;
    private float slowTimer = 0f;
    private float jumpLockout = 0f;
    private float invulnTimer = 0f;
    private int coins = 0;
    private float bonusScore = 0f;

    private DeathCause deathCause = DeathCause.NONE;

    // One-shot events, drained by the game layer for audio and effects.
    private Track.Pickup lastPickup = null;
    private boolean jumpEvent = false;
    private boolean landEvent = false;
    private boolean hitEvent = false;
    private boolean deathEvent = false;
    private float landImpact = 0f;

    // ---- Fixed-plane configuration (fallback, used before a track exists) ----

    public Vector3f getPosition() {
        return spherePos;
    }

    // ---- Track ----

    /**
     * Hands the simulation the road to run on. The origin is the world point the run
     * starts from, sitting on the surface.
     */
    public void setTrack(Track value, Vector3f originOnSurface) {
        track = value;
        trackOrigin.set(originOnSurface);
    }

    public Track getTrack() {
        return track;
    }

    /** World point on the road surface at the given spot in track space. */
    public void surfacePoint(float s, float lateral, Vector3f dest) {
        if (track == null) {
            // Only reachable before the first run is built; the camera asks early.
            dest.set(trackOrigin.x - s, trackOrigin.y, trackOrigin.z - lateral);
            return;
        }

        dest.set(trackOrigin.x - s, trackOrigin.y + track.heightAt(s), trackOrigin.z - lateral);
    }

    /** Upward surface normal at the given distance. */
    public void surfaceNormal(float s, Vector3f dest) {
        if (track == null) {
            dest.set(0f, 1f, 0f);
            return;
        }

        dest.set(track.slopeAt(s), 1f, 0f).normalize();
    }

    /** Downhill tangent at the given distance; on a kicker this points upwards. */
    public void surfaceForward(float s, Vector3f dest) {
        if (track == null) {
            dest.set(-1f, 0f, 0f);
            return;
        }

        dest.set(-1f, track.slopeAt(s), 0f).normalize();
    }

    /** Downhill tangent under the ball, written into dest. */
    public Vector3f forwardHere(Vector3f dest) {
        surfaceForward(getTravelled(), dest);
        return dest;
    }

    /** Surface normal under the ball, written into dest. */
    public Vector3f normalHere(Vector3f dest) {
        surfaceNormal(getTravelled(), dest);
        return dest;
    }

    /** Distance from the ball to the road, measured perpendicular to the surface. */
    private float surfaceDistance() {
        if (track == null) {
            return spherePos.y - trackOrigin.y;
        }

        float s = getTravelled();
        return (spherePos.y - (trackOrigin.y + track.heightAt(s))) * track.normalCosAt(s);
    }

    /**
     * How far the ball has travelled downhill from where the run started. With a track
     * this is a horizontal distance, so it stays meaningful over rises and drops.
     */
    public float getTravelled() {
        return trackOrigin.x - spherePos.x;
    }

    /** Signed offset from the road's centreline, positive to the right of the screen. */
    public float getLateralOffset() {
        return trackOrigin.z - spherePos.z;
    }

    /** Starts the ball moving downhill, so a run begins at speed. */
    public void launch(float speed) {
        velocity.set(forwardHere(sForward)).mul(speed);
    }

    /** Steering from the player: -1 is full left, +1 is full right. */
    public void setSteerInput(float input) {
        steerInput = input < -1f ? -1f : (input > 1f ? 1f : input);
    }

    /**
     * Asks for a jump. The request is remembered briefly, so pressing just before
     * touching down still jumps on landing rather than being swallowed.
     */
    public void requestJump() {
        jumpRequestedAt = clock;
    }

    public void reset(Vector3f pos, Vector3f vel) {
        spherePos.set(pos);

        // Sit the ball on the surface so it starts resting on it instead of dropping in.
        if (track != null) {
            float s0 = trackOrigin.x - spherePos.x;
            spherePos.y = trackOrigin.y + track.heightAt(s0) + (RADIUS / track.normalCosAt(s0));
        }

        velocity.set(vel);
        orientation.identity();
        spinAxis.set(0f, 0f, 1f);
        spinRate = 0f;

        steerInput = 0f;
        distance = 0f;
        alive = true;
        onGround = true;
        fellThrough = false;
        fallTimer = 0f;
        supportHeight = 0f;

        clock = 0f;
        timeSinceGround = 0f;
        jumpRequestedAt = -999f;
        jumpArmed = true;

        boostTimer = 0f;
        slowTimer = 0f;
        jumpLockout = 0f;
        invulnTimer = 0f;
        coins = 0;
        bonusScore = 0f;
        deathCause = DeathCause.NONE;

        lastPickup = null;
        jumpEvent = false;
        landEvent = false;
        hitEvent = false;
        deathEvent = false;
        landImpact = 0f;
    }

    /** Advances the simulation by one step. Called from the update tick, not from render. */
    public void update(float dt) {
        clock += dt;

        boostTimer = Math.max(0f, boostTimer - dt);
        slowTimer = Math.max(0f, slowTimer - dt);
        jumpLockout = Math.max(0f, jumpLockout - dt);
        invulnTimer = Math.max(0f, invulnTimer - dt);

        // Keep the road built where the ball and the camera are about to look.
        if (track != null) {
            track.streamTo(getTravelled(), STREAM_AHEAD);
        }

        final boolean wasOnGround = onGround;
        final Vector3f prevVelocity = sPrevVelocity.set(velocity);

        velocity.fma(dt, GRAVITY_VEC);   // v += g*dt
        spherePos.fma(dt, velocity);     // p += v*dt

        if (track != null) {
            resolveTrack(dt);
        }

        // Jump, taken after contact is resolved so a ground frame is already known.
        trackAirTime(dt, wasOnGround, prevVelocity);
        tryJump();

        spinBall(dt);

        if (alive) {
            scoreAndHazards(dt);
        }
    }

    /** Watches the transition between ground and air, and reports it. */
    private void trackAirTime(float dt, boolean wasOnGround, Vector3f prevVelocity) {
        if (onGround) {
            timeSinceGround = 0f;

            if (!wasOnGround) {
                // Landing. How hard is worth knowing, for the sound and the camera.
                Vector3f n = normalHere(sEventNormal);
                landImpact = Math.max(0f, -prevVelocity.dot(n));
                landEvent = true;
                jumpArmed = true;
            }
        } else {
            timeSinceGround += dt;
        }
    }

    /**
     * Turns a buffered jump request into upward speed. Coyote time means a request just
     * after running off a lip still counts, which stops kickers feeling like they ate
     * the input.
     */
    private void tryJump() {
        if (!alive) {
            return;
        }

        boolean grounded = onGround || (timeSinceGround <= COYOTE_TIME);
        boolean requested = (clock - jumpRequestedAt) <= JUMP_BUFFER;

        if (!grounded || !requested || !jumpArmed) {
            return;
        }

        Vector3f n = normalHere(sEventNormal);

        // Replace any downward component so a jump off a drop still gets full height.
        float vn = velocity.dot(n);

        if (vn < 0f) {
            velocity.fma(-vn, n);
        }

        velocity.fma(JUMP_SPEED, n);

        onGround = false;
        jumpLockout = JUMP_LOCKOUT;
        jumpArmed = false;
        jumpRequestedAt = -999f;
        timeSinceGround = COYOTE_TIME + 1f; // spent; no second jump from the same lip
        jumpEvent = true;
    }

    /**
     * Rolls the ball. On the ground the spin is rolling without slipping, so it follows
     * the road. In the air it is held: the ball keeps the axis and rate it left the
     * ground with, instead of chasing a surface it is no longer touching. Recomputing it
     * mid-flight made the ball race and jerk its axis every time the slope changed under
     * it, which is what made a jump look wrong.
     */
    private void spinBall(float dt) {
        if (onGround) {
            Vector3f surfN = normalHere(sSpinNormal);
            Vector3f planarVel = sSpinPlanar.set(velocity).fma(-velocity.dot(surfN), surfN);
            float speed = planarVel.length();

            if (speed > 1e-5f) {
                spinAxis.set(planarVel).cross(surfN);

                if (spinAxis.lengthSquared() > 1e-12f) {
                    spinAxis.normalize();

                    // Ease onto the rolling rate rather than snapping to it. Landing
                    // from a jump otherwise doubles the spin in a single frame, which
                    // reads as the ball flicking rather than touching down.
                    float target = speed / RADIUS;
                    spinRate += (target - spinRate) * Math.min(1f, SPIN_GRIP * dt);
                }
            } else {
                spinRate -= spinRate * Math.min(1f, SPIN_GRIP * dt);
            }
        } else {
            // A touch of decay, so a very long flight does not land still spinning at
            // takeoff speed when the ball has clearly slowed.
            spinRate -= spinRate * AIR_SPIN_DECAY * dt;
        }

        if (Math.abs(spinRate) > 1e-5f) {
            orientation.rotateAxis(spinRate * dt, spinAxis.x, spinAxis.y, spinAxis.z);
            orientation.normalize();
        }
    }

    /** Scores the ground covered, collects pickups, and decides whether the run ends. */
    private void scoreAndHazards(float dt) {
        final float travelled = getTravelled();
        final float lateral = getLateralOffset();

        // Score is ground covered. Only count it while the ball is supported, so falling
        // off does not keep paying out on the way down.
        if ((onGround || timeSinceGround < 1.5f) && travelled > distance) {
            distance = travelled;
        }

        // Two ways to be gone. The absolute depth catches a ball dropped into open air;
        // the timer catches one that fell past a lip and is now sliding along under the
        // road, which the depth test alone can miss on a shallow stretch.
        if (fellThrough) {
            fallTimer += dt;
        }

        if (surfaceDistance() < -FALL_DEATH_DEPTH || (fellThrough && fallTimer >= FALL_GRACE)) {
            die(DeathCause.FELL);
            return;
        }

        if (track == null) {
            return;
        }

        Track.Pickup picked = track.collect(travelled, lateral, RADIUS);

        if (picked != null) {
            applyPickup(picked);
        }

        resolveObstacle(travelled, lateral);
    }

    private void applyPickup(Track.Pickup picked) {
        lastPickup = picked;

        switch (picked.type) {
            case COIN -> {
                coins++;
                bonusScore += COIN_SCORE;
            }
            case BOOST -> {
                boostTimer = BOOST_DURATION;

                // An immediate shove as well as the sustained push, so it is felt at once.
                Vector3f forward = forwardHere(sForward);
                velocity.fma(BOOST_IMPULSE, forward);
            }
            case SLOW -> slowTimer = SLOW_DURATION;
        }
    }

    /**
     * Resolves the ball against a block. A block is a box standing on the road, so a ball
     * that is above it has cleared it; that is what makes jumping over one work. A hit is
     * only fatal when the ball drove into it hard. A brush along the side costs speed and
     * shoves the ball clear, which is survivable and reads as a scrape.
     */
    private void resolveObstacle(float s, float lateral) {
        Track.Obstacle o = track.hitTest(s, lateral, RADIUS);

        if (o == null || invulnTimer > 0f) {
            return;
        }

        // Over the top of it: either cleared in flight or standing on it. The same
        // threshold the contact solver uses to call a block a surface, so the two can
        // never disagree about whether the ball is on a block or in it.
        float ballBottom = surfaceDistance() - RADIUS;

        if (ballBottom > o.height - BLOCK_TOP_TOLERANCE) {
            return;
        }

        // Closest point on the block to the ball, in track space.
        float nearestS = clamp(s, o.distance - o.halfLength, o.distance + o.halfLength);
        float nearestLat = clamp(lateral, o.lateral - o.halfWidth, o.lateral + o.halfWidth);

        float dS = s - nearestS;
        float dLat = lateral - nearestLat;
        float dist = (float) Math.sqrt((dS * dS) + (dLat * dLat));

        if (dist < 1e-4f) {
            // Dead centre inside the box: no usable direction, so call it a crash.
            die(DeathCause.CRASHED);
            return;
        }

        float nS = dS / dist;
        float nLat = dLat / dist;

        // Track-space velocity. s grows as -x, lateral grows as -z.
        float vS = -velocity.x;
        float vLat = -velocity.z;
        float closing = -((vS * nS) + (vLat * nLat));

        if (closing > FATAL_CLOSING_SPEED) {
            die(DeathCause.CRASHED);
            return;
        }

        // Survivable. Push the ball out of the block and scrub its speed.
        float push = (RADIUS - dist) + 0.05f;

        if (push > 0f) {
            spherePos.x -= push * nS;   // s grows as -x
            spherePos.z -= push * nLat; // lateral grows as -z
        }

        if (closing > 0f) {
            // Remove the closing component, then give a little of it back as a bounce.
            float restitution = 0.25f;
            velocity.x += (1f + restitution) * closing * -nS;
            velocity.z += (1f + restitution) * closing * -nLat;
        }

        velocity.mul(GLANCE_SPEED_SCRUB);
        invulnTimer = GLANCE_INVULN;
        hitEvent = true;
    }

    private void die(DeathCause cause) {
        if (!alive) {
            return;
        }

        alive = false;
        deathCause = cause;
        deathEvent = true;
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    // ---- Contact resolution ----

    /**
     * Rolls the ball on the generated road. The surface pitches from tile to tile, so the
     * contact frame is rebuilt at the ball rather than taken from a fixed plane.
     */
    private void resolveTrack(float dt) {
        final float s = getTravelled();
        final float lateral = getLateralOffset();

        // A tunnel is walled, so the ball is turned back instead of leaving the road.
        if (track.isTunnelAt(s)) {
            holdInsideTunnel(s, lateral);
        }

        // A jump is an explicit push, not a surface interaction, so the snap below gets
        // no say for a moment: long enough for the ball to climb clear of it. Kept after
        // the tunnel hold, or a jump inside one would carry the ball through a wall.
        if (jumpLockout > 0f) {
            onGround = false;
            steerInAir(dt);
            return;
        }

        final Vector3f normal = sNormal;
        surfaceNormal(s, normal);

        // A block standing on the road is a surface in its own right. Without this the
        // only thing a block could do was be driven into: a ball coming down onto one had
        // no horizontal closing speed, so it was read as a harmless brush and squirted
        // out of the side. Now it is either landed on or hit, and nothing in between.
        supportHeight = 0f;

        Track.Obstacle under = track.blockUnder(s, lateral);

        if (under != null) {
            float heightOverRoad = surfaceDistance() - RADIUS;

            if (heightOverRoad > under.height - BLOCK_TOP_TOLERANCE) {
                supportHeight = under.height;
            }
        }

        // Positive means the ball is off the surface, negative means it is sunk into it.
        final float gap = surfaceDistance() - supportHeight - RADIUS;

        // Already under the road and falling. Nothing up there is going to catch it, so
        // the solver stays out of the way until the run ends.
        if (fellThrough) {
            onGround = false;
            steerInAir(dt);
            return;
        }

        if (gap > 0f) {
            // Where the road steepens it drops away from under the ball. Stay glued
            // across those crests; only leave the ground on a real launch, meaning well
            // clear of the surface or rising off a kicker.
            float speed = velocity.length();
            boolean launching = gap > SNAP_GAP
                    || (speed > 1e-3f && (velocity.dot(normal) / speed) > SNAP_SIN);

            if (launching) {
                onGround = false;
                steerInAir(dt);
                return;
            }
        }

        // Nothing underneath — off the side, over a hole, or out in a chasm. All three
        // are the same question, so they behave identically by construction.
        if (supportHeight <= 0f && !track.isSolid(s, lateral, RADIUS)) {
            onGround = false;
            steerInAir(dt);
            return;
        }

        // There is road here, but the ball is far below it: it fell past an edge and is
        // now underneath, rather than having landed on top. Landing never looks like
        // this, so the only honest reading is that the run is over.
        if (-gap > MAX_CONTACT_CORRECTION) {
            fellThrough = true;
            onGround = false;
            steerInAir(dt);
            return;
        }

        onGround = true;

        final Vector3f forward = sForward;
        surfaceForward(s, forward);

        // Sit the ball exactly on whatever is holding it up, whether it was sunk into
        // it or hovering just above a crest.
        spherePos.fma(-gap + 1e-4f, normal);

        float vn = velocity.dot(normal);

        if (vn < 0f) {
            velocity.fma(-(1f + RESTITUTION) * vn, normal);
        } else if (gap > 0f) {
            // Kill the drift that would otherwise lift the ball off the crest again.
            velocity.fma(-vn, normal);
        }

        // Split the in-plane velocity into along-track and lateral parts so steering
        // survives the slide, then rebuild the velocity from the pieces.
        float vNormal = velocity.dot(normal);
        Vector3f vTangent = sTangent.set(velocity).fma(-vNormal, normal);
        float vAlong = vTangent.dot(forward);
        float vLat = vTangent.dot(TRACK_RIGHT);

        float drop = Math.min(Math.abs(vAlong), FRICTION * dt);
        vAlong = Math.signum(vAlong) * (Math.abs(vAlong) - drop);

        vLat += steerInput * steerAccelHere() * dt;
        float latDrop = Math.min(Math.abs(vLat),
                LATERAL_FRICTION * dt * (1f - Math.abs(steerInput)));
        vLat = Math.signum(vLat) * (Math.abs(vLat) - latDrop);

        float latCeiling = lateralCeiling();
        vLat = clamp(vLat, -latCeiling, latCeiling);

        // Keep the run accelerating, harder and further while a boost is running.
        float accel = BOOST_ACCEL + (boostTimer > 0f ? BOOST_EXTRA_ACCEL : 0f);
        float ceiling = topSpeedHere() + (boostTimer > 0f ? BOOST_SPEED_BONUS : 0f);

        // Linear drag as well as the ceiling, so the ball settles at a cruising speed
        // set by how steep the road is instead of pinning to the cap everywhere.
        vAlong += accel * dt;
        vAlong -= vAlong * DRAG * dt;
        vAlong = Math.min(vAlong, ceiling);

        velocity.set(normal).mul(vNormal)
                .fma(vAlong, forward)
                .fma(vLat, TRACK_RIGHT);
    }

    /**
     * Partial steering while off the ground. Full authority in the air would make kickers
     * trivial; none at all makes a long flight feel broken.
     */
    private void steerInAir(float dt) {
        if (Math.abs(steerInput) < 1e-4f) {
            return;
        }

        float vLat = velocity.dot(TRACK_RIGHT);
        float delta = steerInput * steerAccelHere() * AIR_STEER_FACTOR * dt;
        float latCeiling = lateralCeiling();
        float target = clamp(vLat + delta, -latCeiling, latCeiling);

        velocity.fma(target - vLat, TRACK_RIGHT);
    }

    /** Keeps the ball between a tunnel's walls, bouncing it off them. */
    private void holdInsideTunnel(float s, float lateral) {
        float limit = track.halfWidthAt(s) - RADIUS;

        if (limit <= 0f || Math.abs(lateral) <= limit) {
            return;
        }

        float clamped = lateral > 0f ? limit : -limit;

        // lateral = origin.z - z, so pushing the ball in means moving z the other way.
        spherePos.z = trackOrigin.z - clamped;

        float vLat = velocity.dot(TRACK_RIGHT);
        boolean intoWall = (lateral > 0f && vLat > 0f) || (lateral < 0f && vLat < 0f);

        if (intoWall) {
            velocity.fma(-(1f + WALL_RESTITUTION) * vLat, TRACK_RIGHT);
        }
    }

    // ---- State ----

    // Copies into a caller-owned destination instead of allocating. The renderer reads
    // these every frame, so the defensive copies above were turning into steady
    // short-lived garbage on the render path.

    public Quaternionf getOrientation(Quaternionf dest) {
        return dest.set(orientation);
    }

    public Vector3f getVelocity(Vector3f dest) {
        return dest.set(velocity);
    }

    public float getDistance() {
        return distance;
    }

    /** Ground covered plus what the pickups were worth. */
    public int getScore() {
        return (int) (distance + bonusScore);
    }

    public int getCoins() {
        return coins;
    }

    public float getSpeed() {
        return velocity.length();
    }

    /** Top speed for how far this run has already gone. */
    public float topSpeedHere() {
        return MAX_DOWNHILL_SPEED + Math.min(SPEED_RAMP_CAP, distance * SPEED_RAMP);
    }

    /** The fastest this run could ever go, ignoring boost. The HUD scales against this. */
    public static float absoluteTopSpeed() {
        return MAX_DOWNHILL_SPEED + SPEED_RAMP_CAP;
    }

    /**
     * How hard the ball can be steered right now. This tracks the top speed: covering a
     * tile in half the time needs twice the lateral speed to make the same correction, so
     * a rising speed ceiling with a fixed steering ceiling would quietly turn into a
     * handling nerf just as the run got demanding.
     */
    public float lateralCeiling() {
        return MAX_LATERAL_SPEED * (topSpeedHere() / MAX_DOWNHILL_SPEED);
    }

    private float steerAccelHere() {
        return STEER_ACCEL * (topSpeedHere() / MAX_DOWNHILL_SPEED);
    }

    /**
     * Speed as a share of the fastest this run could go. Measured against the absolute
     * top rather than the current ceiling, so the meter climbs through the run instead of
     * sitting pinned at full the moment the ball reaches whatever today's cap is.
     */
    public float getSpeedFraction() {
        float f = getSpeed() / absoluteTopSpeed();
        return f < 0f ? 0f : (f > 1f ? 1f : f);
    }

    public boolean isAlive() {
        return alive;
    }

    public boolean isOnGround() {
        return onGround;
    }

    public boolean isInTunnel() {
        return track != null && track.isTunnelAt(getTravelled());
    }

    public float getBoostTimer() {
        return boostTimer;
    }

    public float getSlowTimer() {
        return slowTimer;
    }

    public boolean isBoosting() {
        return boostTimer > 0f;
    }

    public DeathCause getDeathCause() {
        return deathCause;
    }

    public float getAirTime() {
        return onGround ? 0f : timeSinceGround;
    }

    // ---- One-shot events, drained by the game layer ----

    /** The pickup collected since the last call, or null. */
    public Track.Pickup consumePickup() {
        Track.Pickup p = lastPickup;
        lastPickup = null;
        return p;
    }

    public boolean consumeJumpEvent() {
        boolean v = jumpEvent;
        jumpEvent = false;
        return v;
    }

    /** True once per landing. The impact speed is available until the next landing. */
    public boolean consumeLandEvent() {
        boolean v = landEvent;
        landEvent = false;
        return v;
    }

    public float getLandImpact() {
        return landImpact;
    }

    public boolean consumeHitEvent() {
        boolean v = hitEvent;
        hitEvent = false;
        return v;
    }

    public boolean consumeDeathEvent() {
        boolean v = deathEvent;
        deathEvent = false;
        return v;
    }
}
