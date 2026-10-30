package com.slope.game;

import java.util.ArrayList;
import java.util.List;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import com.slope.game.utils.PropModel;

/**
 * Draws the streamed road. The geometry for a tile, a block, a wall and a pickup is
 * uploaded once each; everything on screen is a pooled {@link PropModel} pointed at one
 * of those uploads and given a transform. Running the hill therefore costs no buffer
 * creation at all, which is what makes an endless track affordable.
 *
 * <p>Placement is analytic. Measuring a mesh's bounds per tile per frame — which is what
 * a naive port of the one-shot builder would do — walks every vertex several times over;
 * instead each mesh is measured once into a {@link MeshBox} and the offsets are derived
 * from that box under the tile's rotation and scale.
 */
public final class TrackView {
    /** How far down the road props are drawn. Beyond this the far plane hides them. */
    private static final float VIEW_DISTANCE = 620f;

    /** Tiles kept behind the camera, so the road does not vanish under the ball. */
    private static final int TILES_BEHIND = 3;

    /**
     * Props a tile can need at once. A tile is now up to three gates of blocks plus a
     * trail of coins along the open lane, so the old budget of seven would have quietly
     * truncated the back of every busy tile.
     */
    private static final int PROPS_PER_TILE = 20;

    /** How solid a block's faces are. The road and the scenery stay wireframe. */
    private static final float BLOCK_FILL = 0.34f;

    private static final float WALL_HEIGHT = 26f;
    private static final float WALL_HALF_THICKNESS = 2.0f;
    private static final float ROOF_CLEARANCE = 30f;
    private static final float ROOF_HALF_THICKNESS = 1.5f;

    private static final float PICKUP_SIZE = 2.6f;
    private static final float PICKUP_HOVER = 5.5f;

    /** Across-track points to -Z, matching the physics track space. */
    private static final Vector3f RIGHT_AXIS = new Vector3f(0f, 0f, -1f);


    /** A mesh's local axis-aligned bounds, measured once. */
    private static final class MeshBox {
        final Vector3f centre;
        final Vector3f halfExtents;

        MeshBox(Vector3f centre, Vector3f halfExtents) {
            this.centre = centre;
            this.halfExtents = halfExtents;
        }
    }

    private final ObjectLoader loader;
    private final RenderManager renderer;

    private PropModel roadMesh;
    private PropModel cubeMesh;
    private MeshBox roadBox;
    private MeshBox cubeBox;

    private int roadVao = -1;
    private int cubeVao = -1;
    private int texIndex = 0;

    private float tileSpan = 1f;
    private float fullHalfWidth = 1f;

    private final List<PropModel> roadPool = new ArrayList<>();
    private final List<PropModel> propPool = new ArrayList<>();

    private int roadUsed = 0;
    private int propUsed = 0;

    // Scratch, reused every frame so the render path allocates nothing. Kept separate
    // rather than shared: placeBox is called with `lifted` as its target, so a single
    // scratch vector would have the box maths overwrite the position it was handed.
    private final Vector3f spot = new Vector3f();
    private final Vector3f normal = new Vector3f();
    private final Vector3f forward = new Vector3f();
    private final Matrix4f rs = new Matrix4f();
    private final Vector3f lifted = new Vector3f();
    private final Vector3f boxCentre = new Vector3f();

    public TrackView(ObjectLoader loader, RenderManager renderer) {
        this.loader = loader;
        this.renderer = renderer;
    }

    /**
     * Loads and uploads the meshes the road is built from, and sizes the pools.
     *
     * @param roadModelPath the flat slab one tile is made of
     * @param cubeModelPath the box used for blocks, walls, roofs and pickups
     * @param textureIndex the texture every road prop samples; the theme recolours it
     */
    public void init(String roadModelPath, String cubeModelPath, int textureIndex) {
        this.texIndex = textureIndex;

        roadMesh = loader.loadGLTFModel(1, textureIndex, roadModelPath);
        cubeMesh = loader.loadGLTFModel(1, textureIndex, cubeModelPath);

        roadBox = measure(roadMesh);
        cubeBox = measure(cubeMesh);

        // One upload each. Every pooled model points at these.
        loader.loadVertexObject(roadMesh, 3);
        roadVao = roadMesh.getIndex();

        loader.loadVertexObject(cubeMesh, 3);
        cubeVao = cubeMesh.getIndex();

        // A tile's footprint, taken from the untilted mesh.
        tileSpan = roadBox.halfExtents.x * 2f;
        fullHalfWidth = roadBox.halfExtents.z;

        if (tileSpan <= 1e-3f) {
            tileSpan = 1f;
        }

        if (fullHalfWidth <= 1e-3f) {
            fullHalfWidth = 1f;
        }

        int tilesVisible = (int) Math.ceil(VIEW_DISTANCE / tileSpan) + TILES_BEHIND + 2;

        for (int i = 0; i < tilesVisible; i++) {
            roadPool.add(pooled(roadMesh, roadVao));
        }

        for (int i = 0; i < tilesVisible * PROPS_PER_TILE; i++) {
            propPool.add(pooled(cubeMesh, cubeVao));
        }

        renderer.setTrackModels(roadPool, propPool);

        System.out.println("TrackView: tile span " + tileSpan
                + ", road width " + (fullHalfWidth * 2f)
                + ", pool " + roadPool.size() + " tiles / " + propPool.size() + " props");
    }

    /** A model that shares an already-uploaded mesh rather than uploading its own. */
    private PropModel pooled(PropModel source, int vao) {
        PropModel m = new PropModel(texIndex,
                source.getVertices(), source.getIndices(),
                source.getTexCoord(), source.getColorArray(), 1);

        // The vertex arrays carried here are never uploaded again; the index is what the
        // renderer actually draws, and it points at the shared VAO.
        m.setIndex(vao);
        return m;
    }

    /** Measures a mesh's local bounds, once, so placement can stay analytic. */
    private static MeshBox measure(PropModel model) {
        float[] v = model.getVertices();

        float minX = Float.POSITIVE_INFINITY;
        float minY = Float.POSITIVE_INFINITY;
        float minZ = Float.POSITIVE_INFINITY;
        float maxX = Float.NEGATIVE_INFINITY;
        float maxY = Float.NEGATIVE_INFINITY;
        float maxZ = Float.NEGATIVE_INFINITY;

        for (int i = 0; i + 2 < v.length; i += 3) {
            minX = Math.min(minX, v[i]);
            maxX = Math.max(maxX, v[i]);
            minY = Math.min(minY, v[i + 1]);
            maxY = Math.max(maxY, v[i + 1]);
            minZ = Math.min(minZ, v[i + 2]);
            maxZ = Math.max(maxZ, v[i + 2]);
        }

        if (v.length < 3) {
            return new MeshBox(new Vector3f(), new Vector3f(1f, 1f, 1f));
        }

        return new MeshBox(
                new Vector3f((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (minZ + maxZ) * 0.5f),
                new Vector3f((maxX - minX) * 0.5f, (maxY - minY) * 0.5f, (maxZ - minZ) * 0.5f));
    }

    public float getTileSpan() {
        return tileSpan;
    }

    /** How many road models were used last frame, for the debug readout. */
    public int getRoadDrawn() {
        return roadUsed;
    }

    /** How many props (blocks, walls, pickups) were used last frame. */
    public int getPropsDrawn() {
        return propUsed;
    }

    public float getFullHalfWidth() {
        return fullHalfWidth;
    }

    /**
     * Lays out everything visible for this frame. Tiles outside the window keep their
     * pooled models, which simply go unused and undrawn.
     *
     * @param track the road to draw
     * @param s where the ball is, as a distance down the hill
     * @param theme the palette to tint the world with
     * @param time run clock, used to animate the pickups
     */
    public void update(Track track, float s, Theme theme, float time) {
        roadUsed = 0;
        propUsed = 0;

        if (track == null) {
            renderer.setTrackCounts(0, 0);
            return;
        }

        int first = Math.max(track.getFirstTileIndex(), track.tileIndexAt(s) - TILES_BEHIND);
        int last = Math.min(track.getLastTileIndex(), track.tileIndexAt(s + VIEW_DISTANCE));

        for (int i = first; i <= last; i++) {
            Track.Tile tile = track.tileOrNull(i);

            if (tile == null) {
                continue;
            }

            drawTile(track, tile, theme);
        }

        for (int i = first; i <= last; i++) {
            Track.Tile tile = track.tileOrNull(i);

            if (tile == null) {
                continue;
            }

            drawProps(track, tile, theme, time);
        }

        renderer.setTrackCounts(roadUsed, propUsed);
    }

    /** The road surface for one tile, pitched and scaled onto its stretch of hill. */
    private void drawTile(Track track, Track.Tile tile, Theme theme) {
        if (tile.kind == Track.Kind.CHASM || roadUsed >= roadPool.size()) {
            return; // a chasm has no floor to draw, which is the whole point of it
        }

        final float centre = track.getTileCentre(tile.index);
        final float pitch = (float) Math.atan(tile.slope);

        Physics.active().surfacePoint(centre, 0f, spot);
        Physics.active().surfaceNormal(centre, normal);
        Physics.active().surfaceForward(centre, forward);

        if (tile.hasHole()) {
            // A hole is drawn as the two surviving strips either side of it, so the gap
            // the physics reports is the gap the player sees.
            float leftFrom = -tile.halfWidth;
            float leftTo = tile.holeFrom;
            float rightFrom = tile.holeTo;
            float rightTo = tile.halfWidth;

            drawStrip(tile, centre, pitch, leftFrom, leftTo, theme);
            drawStrip(tile, centre, pitch, rightFrom, rightTo, theme);
            return;
        }

        PropModel m = roadPool.get(roadUsed++);
        tintRoad(m, tile, theme);

        // placeBox wants world half-extents, not scale factors. Passing ratios here drew
        // the road at about a seventieth of its size, which is why the hill was invisible.
        // The 1/cos stretches a tilted tile so it still covers one span horizontally.
        float halfLength = (tileSpan * 0.5f) / (float) Math.cos(pitch);
        placeBox(m, roadBox, (float) -Math.toDegrees(pitch),
                halfLength, roadBox.halfExtents.y, tile.halfWidth,
                spot, forward, normal, 1f);
    }

    /** One side of a holed tile: a narrowed slab shifted across to its own centre. */
    private void drawStrip(Track.Tile tile, float centre, float pitch,
                           float from, float to, Theme theme) {
        float width = to - from;

        if (width < 1.5f || roadUsed >= roadPool.size()) {
            return;
        }

        float lateral = (from + to) * 0.5f;

        Physics.active().surfacePoint(centre, lateral, spot);
        Physics.active().surfaceNormal(centre, normal);
        Physics.active().surfaceForward(centre, forward);

        PropModel m = roadPool.get(roadUsed++);
        tintRoad(m, tile, theme);

        float halfLength = (tileSpan * 0.5f) / (float) Math.cos(pitch);
        placeBox(m, roadBox, (float) -Math.toDegrees(pitch),
                halfLength, roadBox.halfExtents.y, width * 0.5f,
                spot, forward, normal, 1f);
    }

    private void tintRoad(PropModel m, Track.Tile tile, Theme theme) {
        Vector3f c = (tile.kind == Track.Kind.KICKER) ? theme.getKicker() : theme.getRoad();
        m.setTint(c.x, c.y, c.z, 1f);

        // setTint clears this, so it goes on after the colour.
        m.setRung(1f);
    }

    /** Blocks, tunnel walls and roof, and pickups for one tile. */
    private void drawProps(Track track, Track.Tile tile, Theme theme, float time) {
        final float centre = track.getTileCentre(tile.index);
        final float pitch = (float) Math.atan(tile.slope);
        final float pitchDeg = (float) -Math.toDegrees(pitch);

        for (int i = 0; i < tile.obstacles.size(); i++) {
            Track.Obstacle o = tile.obstacles.get(i);

            if (propUsed >= propPool.size()) {
                return;
            }

            Physics.active().surfacePoint(o.distance, o.lateral, spot);
            Physics.active().surfaceNormal(o.distance, normal);
            Physics.active().surfaceForward(o.distance, forward);

            PropModel m = propPool.get(propUsed++);
            Vector3f c = theme.getBlock();
            m.setTint(c.x, c.y, c.z, 1f);
            m.setFill(BLOCK_FILL);

            placeBox(m, cubeBox, pitchDeg,
                    o.halfLength, o.height * 0.5f, o.halfWidth,
                    spot, forward, normal, -1f);
        }

        if (tile.tunnel) {
            drawTunnel(tile, centre, pitchDeg, theme);
        }

        for (int i = 0; i < tile.pickups.size(); i++) {
            Track.Pickup p = tile.pickups.get(i);

            if (p.isTaken() || propUsed >= propPool.size()) {
                continue;
            }

            Physics.active().surfacePoint(p.distance, p.lateral, spot);
            Physics.active().surfaceNormal(p.distance, normal);

            PropModel m = propPool.get(propUsed++);
            Vector3f c = switch (p.type) {
                case COIN -> theme.getAccent();
                case BOOST -> theme.getKicker();
                case SLOW -> theme.getWall();
            };
            m.setTint(c.x, c.y, c.z, 1f);

            // Hover above the road and spin, so a pickup reads as collectable rather than
            // as one more block to dodge.
            float bob = (float) Math.sin((time * 3.0) + (p.distance * 0.08)) * 1.2f;
            lifted.set(spot).fma(PICKUP_HOVER + bob, normal);

            float spin = (time * 140f) % 360f;
            float size = PICKUP_SIZE * (p.type == Track.Pickup.Type.COIN ? 0.85f : 1.15f);

            m.rotate(0f, spin, pitchDeg);
            m.scale(size / cubeBox.halfExtents.x,
                    size / cubeBox.halfExtents.y,
                    size / cubeBox.halfExtents.z);
            m.setPosition(
                    lifted.x - cubeBox.centre.x,
                    lifted.y - cubeBox.centre.y,
                    lifted.z - cubeBox.centre.z);
            m.update();
        }
    }

    /** The walls and roof that make a tunnel a tunnel. */
    private void drawTunnel(Track.Tile tile, float centre, float pitchDeg, Theme theme) {
        Vector3f c = theme.getWall();
        float halfLength = tileSpan * 0.5f;

        for (int side = -1; side <= 1; side += 2) {
            if (propUsed >= propPool.size()) {
                return;
            }

            float lateral = side * (tile.halfWidth + WALL_HALF_THICKNESS);

            Physics.active().surfacePoint(centre, lateral, spot);
            Physics.active().surfaceNormal(centre, normal);
            Physics.active().surfaceForward(centre, forward);

            PropModel m = propPool.get(propUsed++);
            m.setTint(c.x, c.y, c.z, 1f);

            placeBox(m, cubeBox, pitchDeg,
                    halfLength, WALL_HEIGHT * 0.5f, WALL_HALF_THICKNESS,
                    spot, forward, normal, -1f);
        }

        if (propUsed >= propPool.size()) {
            return;
        }

        Physics.active().surfacePoint(centre, 0f, spot);
        Physics.active().surfaceNormal(centre, normal);
        Physics.active().surfaceForward(centre, forward);

        PropModel roof = propPool.get(propUsed++);
        roof.setTint(c.x * 0.7f, c.y * 0.7f, c.z * 0.7f, 1f);

        lifted.set(spot).fma(ROOF_CLEARANCE, normal);
        placeBox(roof, cubeBox, pitchDeg,
                halfLength, ROOF_HALF_THICKNESS,
                tile.halfWidth + (WALL_HALF_THICKNESS * 2f),
                lifted, forward, normal, -1f);
    }

    /**
     * Sits a box-shaped mesh on the road.
     *
     * <p>The mesh is rotated about Z by {@code rotZdeg} and scaled so its world
     * half-extents become the ones asked for, then positioned so it is centred on
     * {@code spot} along and across the road. {@code anchor} says what touches the spot
     * vertically: {@code 1} the top face, {@code -1} the bottom face, {@code 0} the centre.
     *
     * <p>No vertices are read: the mesh's local box is already known, so the world extent
     * along the surface normal is the usual sum of the scaled, rotated axes.
     */
    private void placeBox(PropModel m, MeshBox box, float rotZdeg,
                          float halfX, float halfY, float halfZ,
                          Vector3f at, Vector3f fwd, Vector3f nrm, float anchor) {
        // Convert the half-extents asked for into a scale on this particular mesh.
        float sx = halfX / Math.max(1e-4f, box.halfExtents.x);
        float sy = halfY / Math.max(1e-4f, box.halfExtents.y);
        float sz = halfZ / Math.max(1e-4f, box.halfExtents.z);

        m.rotate(0f, 0f, rotZdeg);
        m.scale(sx, sy, sz);

        // The rotation-and-scale part of the model matrix, without translation.
        rs.identity().rotateZ((float) Math.toRadians(rotZdeg)).scale(sx, sy, sz);

        // Where the mesh's own centre lands once rotated and scaled about the origin.
        boxCentre.set(box.centre);
        rs.transformPosition(boxCentre);

        // Extent along the surface normal: the scaled, rotated local axes projected onto
        // it. JOML is column-major, so column i is (m[i]0, m[i]1, m[i]2).
        float extN =
                (Math.abs((nrm.x * rs.m00()) + (nrm.y * rs.m01()) + (nrm.z * rs.m02()))
                        * box.halfExtents.x)
              + (Math.abs((nrm.x * rs.m10()) + (nrm.y * rs.m11()) + (nrm.z * rs.m12()))
                        * box.halfExtents.y)
              + (Math.abs((nrm.x * rs.m20()) + (nrm.y * rs.m21()) + (nrm.z * rs.m22()))
                        * box.halfExtents.z);

        // Decompose the wanted position in the road's orthonormal frame. forward, the
        // surface normal and the across-track axis are mutually perpendicular, so the
        // three dot products are the position.
        float a = fwd.dot(at) - fwd.dot(boxCentre);
        float b = RIGHT_AXIS.dot(at) - RIGHT_AXIS.dot(boxCentre);
        float cN = nrm.dot(at) - nrm.dot(boxCentre) - (anchor * extN);

        m.setPosition(
                (fwd.x * a) + (RIGHT_AXIS.x * b) + (nrm.x * cN),
                (fwd.y * a) + (RIGHT_AXIS.y * b) + (nrm.y * cN),
                (fwd.z * a) + (RIGHT_AXIS.z * b) + (nrm.z * cN));
        m.update();
    }

    /** Frees nothing of its own: the pools share the renderer's uploaded meshes. */
    public void destroy() {
        roadPool.clear();
        propPool.clear();
    }
}
