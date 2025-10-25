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

