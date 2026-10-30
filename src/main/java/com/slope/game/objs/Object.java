package com.slope.game.objs;

import com.slope.game.ObjectLoader;
import com.slope.game.utils.PropModel;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;

public abstract class Object {

    // Position, scale, and rotation
    private Vector3f position;
    private Vector3f scale;
    private Vector3f rotation;

    // Multiplied over the mesh colour at draw time. The theme recolours the world
    // through this, so one mesh can be road, wall or kicker depending on its tint.
    private final Vector4f tint;

    // How much of the face is filled in rather than left to the texture. The texture is
    // a grid on black, which reads as a wireframe: fine for the road and the scenery,
    // useless for something you have to spot and dodge.
    private float fill;

    // 1 on the road surface. Rungs are drawn across the thing you travel over and
    // nothing else, so blocks, walls and scenery leave this alone.
    private float rung;

    // Model Matrix
    private Matrix4f modelMatrix;

    public Object() {
        this.position = new Vector3f(0f, 0f, 0f);
        this.scale = new Vector3f(1f, 1f, 1f);
        this.rotation = new Vector3f(0.0f, 0.0f, 0.0f);
        this.tint = new Vector4f(1f, 1f, 1f, 1f);
        this.fill = 0f;
        this.rung = 0f;
        this.modelMatrix = new Matrix4f();
    }

    public Vector4f getTint() {
        return tint;
    }

    public float getFill() {
        return fill;
    }

    public float getRung() {
        return rung;
    }

    public void setRung(float value) {
        rung = value;
    }

    public void setFill(float value) {
        fill = value;
    }

    /**
     * Sets the tint and clears the fill. Models are pooled and reused for whatever the
     * next frame needs, so the fill resets here rather than lingering from whatever this
     * one happened to be last time. Anything wanting a filled face sets it after.
     */
    public void setTint(float r, float g, float b, float a) {
        tint.set(r, g, b, a);
        fill = 0f;
        rung = 0f;
    }

    public void setPosition(float x, float y, float z) {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    public void rotate(float x, float y, float z) {
        rotation.x = x;
        rotation.y = y;
        rotation.z = z;
    }

    public void scale(float x, float y, float z) {
        scale.x = x;
        scale.y = y;
        scale.z = z;
    }

    public void update() {
        modelMatrix.identity()
                .translate(position)
                .rotateX((float)Math.toRadians(rotation.x))
                .rotateY((float)Math.toRadians(rotation.y))
                .rotateZ((float)Math.toRadians(rotation.z))
                .scale(scale);
    }

    // Getting the transformation matrix back
    public Matrix4f getModelMatrix() {
        return modelMatrix;
    }
}