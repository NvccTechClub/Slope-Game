package com.slope.game;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.joml.Vector2f;
import org.joml.Vector3f;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL21;
import org.lwjgl.opengl.GL30;
import org.lwjgl.opengl.GL31;
import static org.lwjgl.opengl.GL31.GL_UNIFORM_BUFFER;
import org.lwjgl.opengl.GL33;

import com.slope.game.utils.PropModel;

public final class RenderManager {
    // Nice utils to have

    public static int maxGLBindings(int target) {
        return switch(target) {
            case GL_UNIFORM_BUFFER -> GL21.glGetInteger(GL31.GL_MAX_UNIFORM_BUFFER_BINDINGS);
            default -> throw new IllegalStateException("Invalid Target: 0x" + Integer.toHexString(target).toUpperCase(Locale.ROOT));
        };
    }


    // How the renderer works at runtime.

    private PropModel screen;
    private ArrayList<PropModel> backgroundModels;

    // The streamed road. These lists are owned by TrackView and handed over once; only
    // the active counts change per frame, so nothing is allocated while running.
    private List<PropModel> trackRoadModels = java.util.Collections.emptyList();
    private List<PropModel> trackPropModels = java.util.Collections.emptyList();
    private int trackRoadCount = 0;
    private int trackPropCount = 0;
    private UniformBlockState uniformBlockState;
    private ShaderManager shaderManager;
    private Vector2f resolution;
    private int __dirtyLink = -1;

    // HUD state pushed in by the game layer each tick. The numbers themselves are drawn
    // on the overlay canvas now, so only what the screen pass actually uses is kept.
    private int hudState = 0;
    private float hudTime = 0f;
    private boolean hudDebug = false;

    // The software-drawn overlay, composited over the scene by the screen pass.
    private int hudTexture = 0;

    // Theme and full-screen effects, pushed in by the game layer each tick.
    private final Vector3f skyTop = new Vector3f(0.04f, 0.10f, 0.24f);
    private final Vector3f skyBottom = new Vector3f(0.12f, 0.25f, 0.43f);
    private final Vector3f fogColor = new Vector3f(0.09f, 0.20f, 0.36f);
    private float speedFx = 0f;
    private float flashAmount = 0f;

    // Reused each frame rather than rebuilt: the screen pass runs every frame, and a
    // fresh quaternion and vector per frame is garbage for no gain.
    private final org.joml.Quaternionf ballOrientation = new org.joml.Quaternionf();
    private final org.joml.Vector4f ballOrientationVec = new org.joml.Vector4f();

    // Sphere Stuff
    private final SizedShaderBlock<Sphere> sphereBlock;

    // Camera Stuff.
    private final SizedShaderBlock<CameraMatrices> camBlock;
    private final CameraMatrices camMatrices;

    public RenderManager(CameraMatrices camMatrices) {
        this.camMatrices = camMatrices;
        this.camBlock = new SizedShaderBlock<>(this, GL_UNIFORM_BUFFER, CameraMatrices.SIZE, CameraMatrices::write);
        this.sphereBlock = new SizedShaderBlock<>(this, GL_UNIFORM_BUFFER, Sphere.SIZE, Sphere::write);
        this.backgroundModels = new ArrayList<>();

        {
            this.resolution = new Vector2f(0, 0);
        }
    }

    public void init() {
        shaderManager = new ShaderManager();
        uniformBlockState = new UniformBlockState(shaderManager);

        {
            final int width = Engine.getMain().getPrimaryWindow().getFramebufferWidth();
            final int height = Engine.getMain().getPrimaryWindow().getFramebufferHeight();
            this.resolution = new Vector2f(width, height);
        }

        try {
            shaderManager.createShaderProgram();
            shaderManager.createVertexShader(ResourceLoader.loadShader("shaders/main-vertex.glsl"));
            shaderManager.createFragmentShader(ResourceLoader.loadShader("shaders/main-fragment.glsl"));
            shaderManager.createVertexShader(1, ResourceLoader.loadShader("shaders/sphere-vertex.glsl"));
            shaderManager.createFragmentShader(1, ResourceLoader.loadShader("shaders/sphere-fragment.glsl"));

            shaderManager.createShaderProgram();
            shaderManager.createVertexShader(2, ResourceLoader.loadShader("shaders/tower-vertex.glsl"));
            shaderManager.createFragmentShader(2, ResourceLoader.loadShader("shaders/tower-fragment.glsl"));

            createGameUniforms();
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    public void renderInstances(ObjectLoader loader) {
        shaderManager.bind(0);
        camMatrices.update(CameraMatrices.Z_NEAR, CameraMatrices.Z_FAR);
        renderCamera();

        shaderManager.bind(2);

        for (int i = 0; i < backgroundModels.size(); i++) {
            drawProp(loader, backgroundModels.get(i));
        }

        // The road, then everything standing on it. Drawing the surface first keeps the
        // depth buffer doing the work for the blocks and walls rather than the other way.
        for (int i = 0; i < trackRoadCount; i++) {
            drawProp(loader, trackRoadModels.get(i));
        }

        for (int i = 0; i < trackPropCount; i++) {
            drawProp(loader, trackPropModels.get(i));
        }

        unbind(this.camBlock);
    }

    /**
     * Draws one prop with the currently bound program. Shared by the scenery and by every
     * piece of the streamed road, so the two cannot drift apart.
     */
    private void drawProp(ObjectLoader loader, PropModel m) {
        int ID = loader.getID(m.getIndex());
        int indicesCount = loader.getIndicesCount(m.getIndex());
        int textureID = loader.getTextures(m.getTexIndex());

        shaderManager.setMatrixUniform("model", m.getModelMatrix());
        shaderManager.setIntUniform("textureSampler", 0);

        // Only the tower field uses the shader's built-in instance spread.
        shaderManager.setFloatUniform("instanceSpread", m.getAmount() > 1 ? 1.0f : 0.0f);

        // The theme recolours the world through this, so one mesh and one texture cover
        // road, block, wall, kicker and pickup.
        shaderManager.setVec4Uniform("propTint", m.getTint());
        shaderManager.setFloatUniform("propFill", m.getFill());
        shaderManager.setFloatUniform("rungAmount", m.getRung());

        GL30.glBindVertexArray(ID);
        GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, loader.getEBO(m.getIndex()));

        GL20.glEnableVertexAttribArray(0);
        GL20.glEnableVertexAttribArray(1);
        GL20.glEnableVertexAttribArray(2);

        GL13.glActiveTexture(GL13.GL_TEXTURE0);
        GL21.glBindTexture(GL21.GL_TEXTURE_2D, textureID);

        GL33.glDrawElementsInstanced(GL11.GL_TRIANGLES, indicesCount,
                GL11.GL_UNSIGNED_INT, 0, m.getAmount());

        GL20.glDisableVertexAttribArray(0);
        GL20.glDisableVertexAttribArray(1);
        GL20.glDisableVertexAttribArray(2);

        GL30.glBindVertexArray(0);
        GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);
        GL21.glBindTexture(GL15.GL_TEXTURE_2D, 0);
    }

    public void renderScreen(int programIndex, Sphere sphere, ObjectLoader loader) {
        if(screen == null) {
            return;
        }

        shaderManager.bind(programIndex);
        int ID = loader.getID(screen.getIndex());
        int textureID = screen.getTexIndex() & 0xFF;
        int depthTextureID = screen.getTexIndex() >> ObjectLoader.BIT_16_CAPACITY;

        // Bind VAO
        GL30.glBindVertexArray(ID);

        // Add model matrix
        camMatrices.update(CameraMatrices.Z_NEAR, CameraMatrices.Z_FAR);
        renderCamera();

        // Add resolution vector
        {
            shaderManager.setVec2Uniform("iResolution", resolution);
            shaderManager.setVec3Uniform("camPosition", camMatrices.getPosition());
            shaderManager.setVec3Uniform("position", Physics.active().getPosition());
            shaderManager.setIntUniform("gameState", hudState);
            shaderManager.setFloatUniform("hudTime", hudTime);
            shaderManager.setIntUniform("debugHud", hudDebug ? 1 : 0);
            shaderManager.setVec3Uniform("skyTop", skyTop);
            shaderManager.setVec3Uniform("skyBottom", skyBottom);
            shaderManager.setVec3Uniform("fogColor", fogColor);
            shaderManager.setFloatUniform("speedFx", speedFx);
            shaderManager.setFloatUniform("flashAmount", flashAmount);
            Physics.active().getOrientation(ballOrientation);
            ballOrientationVec.set(ballOrientation.x, ballOrientation.y,
                    ballOrientation.z, ballOrientation.w);
            shaderManager.setVec4Uniform("sphereOrientation", ballOrientationVec);
        }

        // Update uniform texture sampler
        shaderManager.setIntUniform("textureSampler0", 0);
        shaderManager.setIntUniform("textureSampler1", 1);
        shaderManager.setIntUniform("textureSampler2", 2);
        shaderManager.setIntUniform("hudTexture", 3);

        // Enable the vertex attribute array.
        GL20.glEnableVertexAttribArray(0);
        GL20.glEnableVertexAttribArray(1);

        // Active our texture 0
        GL13.glActiveTexture(GL13.GL_TEXTURE0);
        GL21.glBindTexture(GL21.GL_TEXTURE_2D, textureID);

        // Active our texture 1
        GL13.glActiveTexture(GL13.GL_TEXTURE1);
        GL21.glBindTexture(GL21.GL_TEXTURE_2D, loader.getTextures(0));

        // Active our texture 2
        GL13.glActiveTexture(GL13.GL_TEXTURE2);
        GL21.glBindTexture(GL21.GL_TEXTURE_2D, depthTextureID);

        // Active our texture 3: the software-drawn overlay.
        GL13.glActiveTexture(GL13.GL_TEXTURE3);
        GL21.glBindTexture(GL21.GL_TEXTURE_2D, hudTexture);
        //System.out.println(depthTextureID);

        // Draw the vertices as triangles.
        GL21.glDrawArrays(GL21.GL_TRIANGLES, 0, screen.getVertices().length);

        // Disable our attributes
        GL20.glDisableVertexAttribArray(0);

        // Unbind the texture.
        GL21.glBindTexture(GL21.GL_TEXTURE_2D, 0);

        // Unbind the VAO to avoid any accidental changes.
        GL30.glBindVertexArray(0);

        unbind(this.camBlock);
        GL20.glDisableVertexAttribArray(0);
        GL20.glDisableVertexAttribArray(1);
    }

    /**
     * What the screen pass needs about the game's state. The score and the best are no
     * longer passed: those are text, and text is drawn on the overlay canvas now.
     *
     * @param phase 0 menu, 1 ready, 2 playing, 3 paused, 4 dead
     * @param time run clock, for animated effects
     * @param debug whether to draw the developer overlays
     */
    public void setHudState(int phase, float time, boolean debug) {
        this.hudState = phase;
        this.hudTime = time;
        this.hudDebug = debug;
    }

    public void addPropModel(PropModel model) { backgroundModels.add(model); }

    /** Hands over the streamed road's pools. Called once, at load. */
    public void setTrackModels(List<PropModel> roadModels, List<PropModel> propModels) {
        this.trackRoadModels = roadModels;
        this.trackPropModels = propModels;
    }

    /** How much of each pool is live this frame. */
    public void setTrackCounts(int roadCount, int propCount) {
        this.trackRoadCount = Math.min(roadCount, trackRoadModels.size());
        this.trackPropCount = Math.min(propCount, trackPropModels.size());
    }

    public void setHudTexture(int texture) {
        this.hudTexture = texture;
    }

    /** Pushes the live palette to the screen pass, which paints the sky and the fog. */
    public void setTheme(Theme theme) {
        skyTop.set(theme.getSkyTop());
        skyBottom.set(theme.getSkyBottom());
        fogColor.set(theme.getFog());
    }

    /**
     * Full-screen effects.
     *
     * @param speed 0..1, how hard to push the speed vignette and the edge streaks
     * @param flash 0..1, a white-out on landing or a red-out on death
     */
    public void setScreenEffects(float speed, float flash) {
        this.speedFx = speed;
        this.flashAmount = flash;
    }

    public PropModel setScreenModel(PropModel value) {
        return screen = value;
    }

    // Assigns the specified block to the next available binding slot.
    public void bind(CharSequence name, SizedShaderBlock<?> block) {
        uniformBlockState.bind(name, block);
    }

    // Releases the specified block and frees its occupied binding slot.
    public void unbind(SizedShaderBlock<?> block) {
        uniformBlockState.unbind(block);
    }

    public void begin() {
        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);
    }

    public void destroy() {
        backgroundModels.clear();
        shaderManager.destroy();
    }

    private void renderCamera() {
        camBlock.set(camMatrices);
        bind("CameraMatrices", this.camBlock);
    }

    public void renderSphere(Sphere object) {
        sphereBlock.set(object);
        bind("SphereBlock", this.sphereBlock);
    }

    public void onWindowResize(int width, int height) {
        resolution.set(width, height);
    }

    public void end() {
        shaderManager.unbind();
    }

    private void createGameUniforms() throws Exception {
        link(0);
        link(1);
        link(2);

        shaderManager.bind(0);
        shaderManager.createUniform("textureSampler");
        shaderManager.createUniform("model");

        shaderManager.bind(1);
        shaderManager.createUniform(1,"iResolution");
        shaderManager.createUniform(1, "camPosition");
        shaderManager.createUniform(1, "textureSampler0");
        shaderManager.createUniform(1, "textureSampler1");
        shaderManager.createUniform(1, "textureSampler2");
        shaderManager.createUniform(1, "position");
        shaderManager.createUniform(1, "sphereOrientation");
        shaderManager.createUniform(1, "gameState");
        // hudTime is not registered: nothing in the fragment shader samples it since
        // the speed streaks were removed, so the compiler strips it and lookup fails.
        // Setting it stays harmless; re-add this line if an animated effect returns.
        shaderManager.createUniform(1, "debugHud");
        shaderManager.createUniform(1, "hudTexture");
        shaderManager.createUniform(1, "skyTop");
        shaderManager.createUniform(1, "skyBottom");
        shaderManager.createUniform(1, "fogColor");
        shaderManager.createUniform(1, "speedFx");
        shaderManager.createUniform(1, "flashAmount");

        shaderManager.bind(2);
        shaderManager.createUniform(2, "textureSampler");
        shaderManager.createUniform(2, "model");
        shaderManager.createUniform(2, "instanceSpread");
        shaderManager.createUniform(2, "propTint");
        shaderManager.createUniform(2, "propFill");
        shaderManager.createUniform(2, "rungAmount");

    }

    private void link(int index) {
        if(__dirtyLink == index) {
            return;
        }

        __dirtyLink = index;

        try {
            shaderManager.link(index);
        }catch (Exception e) {
            // Swallowing this silently turns a shader problem into a confusing
            // "could not find uniform" failure further down, so say what broke.
            System.err.println("Shader program " + index + " failed to link: " + e.getMessage());
            return;
        }
    }
}