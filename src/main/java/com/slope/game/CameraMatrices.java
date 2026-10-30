package com.slope.game;

import org.joml.*;
import org.joml.Math;

import java.nio.ByteBuffer;


public class CameraMatrices {
    public static final float Z_NEAR = 0.05f;
    public static final float Z_FAR = 960.0f;

    private static final float COS_X = Math.cos(0);
    private static final float SIN_X = Math.sin(0);

    /** Resting field of view. Speed widens it from here. */
    public static final float BASE_FOV = Math.toRadians(45);

    public static final Vector3f LOOK_UP = new Vector3f(0.0f, 1.0f, 0.0f);

    // std140 layout of the CameraMatrices block. The inverses are uploaded alongside
    // the forward matrices so the fullscreen pass can build view rays without calling
    // inverse() per fragment.
    public static final int SIZE =
            Float.BYTES * 16 + // projection
            Float.BYTES * 16 + // view
            Float.BYTES * 16 + // inverse projection
            Float.BYTES * 16 + // inverse view
            Float.BYTES * 2 +  // near and far plane
            Float.BYTES * 2;   // std140 rounds a block up to a multiple of 16 bytes

    // Our data sent to the GPU.
    protected final Matrix4f projectionMatrix;
    protected final Matrix4f viewMatrix;
    protected final Matrix4f invProjectionMatrix;
    protected final Matrix4f invViewMatrix;
    protected final Vector3f position;
    protected final Vector3f center;
    protected float nearPlane;
    protected float farPlane;

    // Our data that stays.
    protected float verticalAngle;
    protected float horizontalAngle;
    protected Vector3f lookAt;

    // The projection only changes when the clip planes, the window aspect or the field
    // of view change, so it is rebuilt on demand rather than once per pass.
    private float cachedAspect;
    private float cachedFov;

    // Widened by speed, so going fast feels fast rather than just reading as a number.
    private float fov = BASE_FOV;

    // Bumped whenever anything the GPU copy depends on changes. The renderer compares
    // it against the version it last uploaded, so the two passes in a frame share one
    // buffer upload instead of each doing their own.
    private int version;

    public CameraMatrices() {
        this.projectionMatrix = new Matrix4f();
        this.viewMatrix = new Matrix4f();
        this.invProjectionMatrix = new Matrix4f();
        this.invViewMatrix = new Matrix4f();
        this.position = new Vector3f(0.0f, 0.0f, -3.0f);
        this.nearPlane = 0.0f;
        this.farPlane = 0.0f;
        this.cachedAspect = 0.0f;
        this.cachedFov = 0.0f;

        this.lookAt = new Vector3f(0.0f, 0.0f, 0.0f);

        this.horizontalAngle = Math.atan2(lookAt.x, lookAt.z);
        this.verticalAngle = Math.asin(lookAt.y);

        this.center = new Vector3f(0.0f, 0.0f, 0.0f);
    }

    public void init() {
        updateRotationMat();
        updateViewMat();
    }

    public void write(ByteBuffer buffer) {
        this.projectionMatrix.get(0, buffer);
        this.viewMatrix.get(Float.BYTES * 16, buffer);
        this.invProjectionMatrix.get(Float.BYTES * 32, buffer);
        this.invViewMatrix.get(Float.BYTES * 48, buffer);
        buffer.putFloat(Float.BYTES * 64, this.nearPlane);
        buffer.putFloat(Float.BYTES * 65, this.farPlane);
    }

    // @param zFar (datatype: float) -> The far clipping plane of the camera.
    // @param zNear (datatype: float) -> The near clipping plane of the camera.

    public void update(float zNear, float zFar) {
        final float aspect = Engine.getMain().getPrimaryWindow().getAspectRatio();

        // Nothing that feeds the projection has moved, so keep the matrix we already have.
        if (zNear == this.nearPlane && zFar == this.farPlane
                && aspect == this.cachedAspect && this.fov == this.cachedFov) {
            return;
        }

        this.nearPlane = zNear;
        this.farPlane = zFar;
        this.cachedAspect = aspect;
        this.cachedFov = this.fov;

        projectionMatrix.identity();
        projectionMatrix.perspective(this.fov, aspect, zNear, zFar, false);

        // invertPerspective is the closed form for a perspective matrix, not a general
        // 4x4 inversion.
        projectionMatrix.invertPerspective(invProjectionMatrix);
        this.version++;
    }

    public void updateViewMat() {
        center.zero();
        position.add(lookAt, center);
        viewMatrix.identity();
        viewMatrix.lookAt(position, center, LOOK_UP);

        // lookAt is affine, so the cheap affine inverse is exact here.
        viewMatrix.invertAffine(invViewMatrix);
        this.version++;
    }

    public void updateRotationMat() {
        float cos_y = Math.cos(verticalAngle);
        float sin_y = Math.sin(verticalAngle);

        lookAt.set(SIN_X * cos_y, sin_y, cos_y * COS_X);
    }

    public int getVersion() { return version; }

    /**
     * Sets the field of view, in radians. The projection is rebuilt on the next update
     * only if this actually changed, so calling it every tick is free.
     */
    public void setFov(float radians) {
        this.fov = radians;
    }

    public float getFov() {
        return fov;
    }

    public Matrix4f getProjectionMatrix() {
        return projectionMatrix;
    }

    public Matrix4f getViewMatrix() { return viewMatrix; }

    public Vector3f getPosition() { return position; }
}
