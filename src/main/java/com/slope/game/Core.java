package com.slope.game;

import java.util.ArrayList;
import java.util.List;

import org.joml.Vector3f;
import org.lwjgl.opengl.GL21;

import com.slope.game.utils.PropModel;
import com.slope.game.utils.TransformUtils;

public class Core implements IComponentManager {
    protected final ObjectLoader loader;
    protected final RenderManager renderer;
    protected final FrameBuffer frameBuffer;
    protected final CameraMatrices camMatrices;

    /** Draws the streamed road from a recycled pool of models. */
    protected final TrackView trackView;

    /** The software-drawn overlay: score, prompts, menus, debug readout. */
    protected final HudCanvas hudCanvas;

    /** Fixed background props. Kept so the theme can tint them along with the road. */
    private final List<PropModel> scenery = new ArrayList<>();

    /** The instanced tower field, moved each frame to stay with the run. */
    private PropModel towerField;

    // The field is laid out on a lattice by the tower shader: a row every 150 along the
    // run and 75 down. Moving it by whole lattice steps keeps it with the player without
    // any of it appearing to slide, because every tower lands where another one was.
    private static final float TOWER_STEP_X = 150f;
    private static final float TOWER_STEP_Y = 75f;
    /**
     * How much taller the towers stand than the mesh was authored. The mesh is centred on
     * its own origin, so stretching it grows the tower in both directions; the base is
     * lifted by the half it gained so the extra height all goes upwards and the feet stay
     * where they were.
     */
    private static final float TOWER_HEIGHT_SCALE = 1.2f;
    private static final float TOWER_MESH_HALF_HEIGHT = 225f;
    private static final float TOWER_LIFT =
            TOWER_MESH_HALF_HEIGHT * (TOWER_HEIGHT_SCALE - 1f);

    private static final Vector3f TOWER_BASE =
            new Vector3f(-250f, -400f + TOWER_LIFT, -250f);

    private static final String FLAT_MODEL = "src/main/resources/models/flat.glb";
    private static final String RAMP_MODEL = "src/main/resources/models/ramp.glb";
    private static final String TOWER_MODEL = "src/main/resources/models/tower.glb";
    private static final String CUBE_MODEL = "src/main/resources/models/cube.glb";

    /** How much of the road mesh's width the playable road actually uses. */
    private static final float ROAD_WIDTH_SHARE = 0.68f;

    public Core() {
        camMatrices = new CameraMatrices();
        loader = new ObjectLoader();
        renderer = new RenderManager(camMatrices);
        frameBuffer = new FrameBuffer();
        trackView = new TrackView(loader, renderer);
        hudCanvas = new HudCanvas();
    }

    @Override
    public void init() {
        {
            final int width = Engine.getMain().getPrimaryWindow().getFramebufferWidth();
            final int height = Engine.getMain().getPrimaryWindow().getFramebufferHeight();

            frameBuffer.init(width, height);
        }

        createGreenTowers();

        {
            PropModel screen = renderer.setScreenModel(loader.createScreen(
        frameBuffer.getTextureID() | (frameBuffer.getDepthTexture() << ObjectLoader.BIT_16_CAPACITY)
            ));
            loader.loadVertexObject(screen, 3);
            //screen.scale(0.5f, 0.5f, 0.5f);
            screen.update();
        }

        camMatrices.init();
        renderer.init();

        // The road's own texture. Index 0 stays the scenery texture, because the screen
        // pass samples it for the ball.
        final int trackTex = loader.getTextureCount();
        loader.loadTexture("textures/Object.png");

        trackView.init(FLAT_MODEL, CUBE_MODEL, trackTex);

        hudCanvas.init();
        renderer.setHudTexture(hudCanvas.getTexture());

        loader.unbind();
    }

    @Override
    public IComponent addComponent(IComponent component, Class<IGraphics> handler) {
        return null; // TODO: Make this work, meaning implement.
    }

    @Override
    public void render() {
        final int width = Engine.getMain().getPrimaryWindow().getFramebufferWidth();
        final int height = Engine.getMain().getPrimaryWindow().getFramebufferHeight();

        GL21.glViewport(0, 0, width, height);

        frameBuffer.bind();
        graphicsPass();
        FrameBuffer.unbind();

        // Any drawing the game layer did this tick goes to the GPU now, just before the
        // pass that composites it.
        hudCanvas.upload();

        renderer.begin();
        renderer.renderScreen(1, null, loader);
        renderer.end();
    }

    @Override
    public void update() {
        camMatrices.updateViewMat();
    }

    @Override
    public void destroy() {
        trackView.destroy();
        hudCanvas.destroy();
        frameBuffer.destroy();
        renderer.destroy();
        loader.destroy();
    }

    @Override
    public void onWindowResize(int width, int height) {
        frameBuffer.onWindowResize(width, height);
        renderer.onWindowResize(width, height);
    }

    public void graphicsPass() {
        renderer.begin();
        renderer.renderInstances(loader);
        renderer.end();
    }


    /**
     * Starts a fresh hill. The road generates itself as the ball runs, so this only has
     * to create the generator and tell the physics where the run begins; nothing is built
     * up front and nothing has a length.
     *
     * @param origin world point where the run starts, on the road surface
     * @param seed fixes the layout, so the same seed replays the same run
     */
    protected Track newTrack(Vector3f origin, long seed) {
        // The road is a share of the mesh width, sized so a dodge is possible within
        // roughly one tile at cruising speed: at 100 the ball can cover about eighteen
        // units sideways in the time a tile passes. The tiles are scaled to match, so
        // what is drawn is what is solid.
        final float halfWidth = trackView.getFullHalfWidth() * ROAD_WIDTH_SHARE;
        final Track track = new Track(trackView.getTileSpan(), halfWidth, seed);

        Physics.active().setTrack(track, origin);

        // Build the opening stretch before the first frame, so the ball has something to
        // rest on and the camera has road to look down.
        track.streamTo(0f, 900f);

        System.out.println("Track: seed " + seed
                + ", tile span " + track.getTileSpan()
                + ", width " + (track.getFullHalfWidth() * 2f));

        return track;
    }

    /** Lays out the visible stretch of road for this frame. */
    protected void updateTrackView(Theme theme, float time) {
        trackView.update(Physics.active().getTrack(), Physics.active().getTravelled(), theme, time);

        // Keep the towers with the run. They were fixed world scenery from when the hill
        // was a short fixed ramp; on an endless road the ball descends out of them and the
        // horizon goes empty. Snapped to the field's own lattice, so it reads as standing
        // still rather than trailing the player around.
        if (towerField != null) {
            Vector3f ball = Physics.active().getPosition();

            towerField.setPosition(
                    TOWER_BASE.x + (Math.round(ball.x / TOWER_STEP_X) * TOWER_STEP_X),
                    TOWER_BASE.y + (Math.round(ball.y / TOWER_STEP_Y) * TOWER_STEP_Y),
                    TOWER_BASE.z);
            towerField.update();
        }

        // The towers and slabs are the same colour as everything else, just dimmer, so
        // a zone change washes over the whole world rather than only the road.
        Vector3f c = theme.getWall();

        for (int i = 0; i < scenery.size(); i++) {
            scenery.get(i).setTint(c.x, c.y, c.z, 1f);
        }
    }

    protected int getRoadDrawn() {
        return trackView.getRoadDrawn();
    }

    protected int getPropsDrawn() {
        return trackView.getPropsDrawn();
    }


    // These are background objects that can't be interacted with the game in any way.
    // That's why they are in the Core Class rather than the Game Class.
    private void createGreenTowers() {
        {
            loader.loadTexture("textures/Object.png");

            // Scenery slabs, set off to the side of the track. The physics surface is
            // configured by the game layer, not fitted to this mesh.
            for (int i = 0; i < 3; i++) {
                PropModel ramp = loader.loadGLTFModel(1, 0, RAMP_MODEL);
                ramp.rotate(0.0f, 180.0f, 180.0f);
                ramp.scale(1.0f, 1.0f, 1.0f);
                TransformUtils.centerModelAtWorld(ramp, new Vector3f(250f - (i * 250f), 0f, -220f));
                renderer.addPropModel(ramp);
                scenery.add(ramp);
                loader.loadVertexObject(ramp, 3);
            }

            loader.loadTexture("textures/Object.png");
            PropModel n = loader.loadGLTFModel(1,0, FLAT_MODEL);
            n.setPosition(0.0f, -107.5f, 250.0f);
            n.update();
            renderer.addPropModel(n);
            scenery.add(n);

            loader.loadVertexObject(n, 3);

            loader.loadTexture("textures/Object.png");
            PropModel m = loader.loadGLTFModel(300,0, TOWER_MODEL);

            m.scale(1f, TOWER_HEIGHT_SCALE, 1f);
            m.setPosition(TOWER_BASE.x, TOWER_BASE.y, TOWER_BASE.z);
            m.update();
            renderer.addPropModel(m);
            scenery.add(m);
            towerField = m;

            loader.loadVertexObject(m, 3);
        }
    }
}