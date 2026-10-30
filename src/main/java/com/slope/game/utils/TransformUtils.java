package com.slope.game.utils;

import org.joml.Vector3f;

/** Utility transforms for positioning models precisely in world space. */
public final class TransformUtils {
    private TransformUtils() {}

    /** Computes the axis-aligned bounding box center in the model's local space. */
    public static Vector3f computeLocalAabbCenter(PropModel model) {
        float[] v = model.getVertices();
        float minX = Float.POSITIVE_INFINITY, minY = Float.POSITIVE_INFINITY, minZ = Float.POSITIVE_INFINITY;
        float maxX = Float.NEGATIVE_INFINITY, maxY = Float.NEGATIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;
        for (int i = 0; i < v.length; i += 3) {
            float x = v[i], y = v[i + 1], z = v[i + 2];
            if (x < minX) minX = x; if (x > maxX) maxX = x;
            if (y < minY) minY = y; if (y > maxY) maxY = y;
            if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
        }
        return new Vector3f((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (minZ + maxZ) * 0.5f);
    }

    /**
     * Projects the model's world-space vertices onto the given axis and returns
     * {min, max}. Returns {0, 0} if the model or axis is unusable.
     */
    public static float[] computeWorldRangeAlong(PropModel model, Vector3f axis) {
        float[] v = model.getVertices();
        Vector3f dir = new Vector3f(axis);

        if (dir.lengthSquared() < 1e-9f || v.length < 3) {
            return new float[] { 0f, 0f };
        }

        dir.normalize();
        float min = Float.POSITIVE_INFINITY;
        float max = Float.NEGATIVE_INFINITY;
        Vector3f local = new Vector3f();
        Vector3f world = new Vector3f();

        for (int i = 0; i + 2 < v.length; i += 3) {
            local.set(v[i], v[i + 1], v[i + 2]);
            model.getModelMatrix().transformPosition(local, world);
            float t = world.dot(dir);
            if (t < min) min = t;
            if (t > max) max = t;
        }

        return new float[] { min, max };
    }

    /**
     * Computes half the extent of the model's world-space bounds along the given axis.
     * Used to derive the playable track width from the road geometry itself.
     */
    public static float computeWorldHalfExtentAlong(PropModel model, Vector3f axis) {
        float[] range = computeWorldRangeAlong(model, axis);
        return (range[1] - range[0]) * 0.5f;
    }

    /**
     * Centers the given model so its local AABB center ends up at the specified world-space position.
     * Rotation and scale should be set before calling. This method preserves them.
     */
    public static void centerModelAtWorld(PropModel model, Vector3f targetWorld) {
        Vector3f localCenter = computeLocalAabbCenter(model);

        // Temporarily zero the translation to extract RS transform via the model matrix.
        model.setPosition(0f, 0f, 0f);
        model.update();

        Vector3f centerWorldFromOrigin = new Vector3f();
        model.getModelMatrix().transformPosition(localCenter, centerWorldFromOrigin);

        // Compute the translation that places the (rotated+scaled) center at the requested target.
        Vector3f translation = new Vector3f(targetWorld).sub(centerWorldFromOrigin);
        model.setPosition(translation.x, translation.y, translation.z);
        model.update();
    }
}

