package com.slope.game;

import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import com.slope.game.utils.PropModel;

public class Physics {
    private static final float GRAVITY = -9.81f;
    private static final float RADIUS = 3.0f;
    private static final float RESTITUTION = 0.2f;   // 0=no bounce, 1=perfect
    private static final float FRICTION = 3.0f;      // tangential damping (≈ m/s^2)
    private static final Vector3f GRAVITY_VEC = new Vector3f(0f, GRAVITY, 0f);

    private static final Vector3f spherePos = new Vector3f(0f, 5f, 0f);
    private static final Vector3f velocity = new Vector3f();
    private static final Vector3f acceleration = new Vector3f();
    private static final Quaternionf orientation = new Quaternionf();

    // Ramp plane: n · x + d = 0, |n| = 1
    private static final Vector3f rampN = new Vector3f(0f, 1f, 0f);
    private static float rampD = 0f;
    // Preferred downhill axis in world space (normalized). Default: +Z.
    private static final Vector3f preferredAxis = new Vector3f(0f, 0f, 1f);

    public static void setRampByNormalAndPoint(Vector3f normal, Vector3f pointOnPlane) {
        rampN.set(normal).normalize();
        rampD = -rampN.dot(pointOnPlane);
    }
    public static org.joml.Vector3f getPosition() {
        return updateAndGetPosition();
    }

    // Convenience: ramp rising along +Z by 'degrees'; plane passes through (0, heightAtOrigin, 0)
    public static void setSimpleRampAlongZ(float degrees, float heightAtOrigin) {
        float k = (float)Math.tan(Math.toRadians(degrees)); // y = k*z + heightAtOrigin (rises with +Z)
        // For plane y - k*z - y0 = 0, a valid normal is (0, 1, -k)
        Vector3f n = new Vector3f(0f, 1f, -k).normalize();
        setRampByNormalAndPoint(n, new Vector3f(0f, heightAtOrigin, 0f));
    }

    // Convenience: ramp rising along +X; plane y = k*x + y0
    public static void setSimpleRampAlongX(float degrees, float heightAtOrigin) {
        float k = (float)Math.tan(Math.toRadians(degrees)); // y = k*x + y0
        Vector3f n = new Vector3f(-k, 1f, 0f).normalize();
        setRampByNormalAndPoint(n, new Vector3f(0f, heightAtOrigin, 0f));
    }

    // Build a world-space ramp plane from a model matrix whose local ramp rises along +X by `degrees`.
    public static void setRampAlongXFromModel(float degrees, Matrix4f modelMatrix) {
        float k = (float)Math.tan(Math.toRadians(degrees));
        Vector3f nLocal = new Vector3f(-k, 1f, 0f).normalize();
        Vector3f nWorld = new Vector3f();
        Vector3f pWorld = new Vector3f();
        modelMatrix.transformDirection(nLocal, nWorld).normalize();
        modelMatrix.transformPosition(new Vector3f(0f, 0f, 0f), pWorld);
        setRampByNormalAndPoint(nWorld, pWorld);
    }

    // Compute a best-fit ramp plane from a rendered model (uses transformed triangle normals)
    public static void setRampFromModel(PropModel model) {
        float[] verts = model.getVertices();
        int[] idx = model.getIndices();
        Matrix4f M = new Matrix4f(model.getModelMatrix());

        Vector3f normalSum = new Vector3f();
        Vector3f pointSum = new Vector3f();
        int triCount = 0;

        for (int i = 0; i + 2 < idx.length; i += 3) {
            int i0 = idx[i] * 3;
            int i1 = idx[i + 1] * 3;
            int i2 = idx[i + 2] * 3;

            Vector3f a = new Vector3f(verts[i0], verts[i0 + 1], verts[i0 + 2]);
            Vector3f b = new Vector3f(verts[i1], verts[i1 + 1], verts[i1 + 2]);
            Vector3f c = new Vector3f(verts[i2], verts[i2 + 1], verts[i2 + 2]);

            Vector3f aw = new Vector3f();
            Vector3f bw = new Vector3f();
            Vector3f cw = new Vector3f();
            M.transformPosition(a, aw);
            M.transformPosition(b, bw);
            M.transformPosition(c, cw);

            Vector3f ab = new Vector3f(bw).sub(aw);
            Vector3f ac = new Vector3f(cw).sub(aw);
            Vector3f n = ab.cross(ac);
            if (n.lengthSquared() > 1e-9f) {
                normalSum.add(n);
                pointSum.add(new Vector3f(aw).add(bw).add(cw).mul(1f / 3f));
                triCount++;
            }
        }

        if (triCount > 0 && normalSum.lengthSquared() > 1e-9f) {
            normalSum.normalize();
            pointSum.mul(1f / triCount);
            setRampByNormalAndPoint(normalSum, pointSum);
        }
    }

    // Select the axis we constrain sliding to while in contact (e.g., (1,0,0) or (0,0,1))
    public static void setPreferredDownhillAxis(Vector3f axis) {
        preferredAxis.set(axis).normalize();
    }

    // Optional: define plane from three points on the ramp (world-space)
    public static void setRampBy3Points(Vector3f a, Vector3f b, Vector3f c) {
        Vector3f ab = new Vector3f(b).sub(a);
        Vector3f ac = new Vector3f(c).sub(a);
        Vector3f n = ab.cross(ac).normalize();
        setRampByNormalAndPoint(n, a);
    }

    public static void reset(Vector3f pos, Vector3f vel) {
        spherePos.set(pos);
        velocity.set(vel);
        acceleration.set(0f, 0f, 0f);
        orientation.identity();
    }

    public static Vector3f updateAndGetPosition() {
        float dt = Engine.getMain().getFrameArea();

        Vector3f prevVelocity = new Vector3f(velocity);

        velocity.fma(dt, GRAVITY_VEC);   // v += g*dt
        spherePos.fma(dt, velocity);     // p += v*dt

        resolveSphereRamp(dt);

        // Compute acceleration including effects of collisions/friction this frame
        if (dt > 0f) {
            acceleration.set(velocity).sub(prevVelocity).div(dt);
        } else {
            acceleration.set(0f, 0f, 0f);
        }

        // Integrate orientation from planar velocity (approx rolling without slipping)
        Vector3f planarVel = new Vector3f(velocity).fma(-velocity.dot(rampN), rampN);
        float speed = planarVel.length();
        if (speed > 1e-5f) {
            Vector3f axis = new Vector3f(planarVel).cross(rampN).normalize();
            float radius = RADIUS;
            float angle = (speed / radius) * dt; // radians
            orientation.rotateAxis(angle, axis.x, axis.y, axis.z);
            orientation.normalize();
        }
        return spherePos;
    }

    public static Quaternionf getOrientation() {
        return new Quaternionf(orientation);
    }

    public static Vector3f getVelocity() {
        return new Vector3f(velocity);
    }

    public static Vector3f getAcceleration() {
        return new Vector3f(acceleration);
    }

    private static void resolveSphereRamp(float dt) {
        float dist = rampN.dot(spherePos) + rampD;          // signed distance to plane
        float penetration = RADIUS - dist;
        if (penetration > 0f) {
            spherePos.fma(penetration + 1e-4f, rampN);      // positional correction

            float vn = velocity.dot(rampN);                 // normal vel
            if (vn < 0f) {
                velocity.fma(-(1f + RESTITUTION) * vn, rampN);
            }

            // Compute downhill direction (project gravity onto the plane)
            Vector3f gTangent = new Vector3f(GRAVITY_VEC).fma(-GRAVITY_VEC.dot(rampN), rampN);
            float gTLen = gTangent.length();

            // Tangential velocity component
            Vector3f vTangent = new Vector3f(velocity).fma(-velocity.dot(rampN), rampN);

            if (gTLen > 1e-6f) {
                // Constrain tangential motion to chosen world axis direction
                float comp = gTangent.dot(preferredAxis);
                Vector3f downhillDir;
                if (Math.abs(comp) > 1e-6f) {
                    downhillDir = new Vector3f(preferredAxis).mul(Math.signum(comp));
                } else {
                    // Fallback to actual steepest descent if nearly orthogonal
                    downhillDir = gTangent.div(gTLen);
                }
                float vAlong = vTangent.dot(downhillDir);

                // Apply simple Coulomb-like friction as a speed reduction along downhill
                float drop = Math.min(Math.abs(vAlong), FRICTION * dt);
                float sign = Math.signum(vAlong);
                vAlong = sign * (Math.abs(vAlong) - drop);

                vTangent.set(downhillDir.mul(vAlong));
            } else {
                // If plane is nearly horizontal, just damp the existing tangential velocity
                float vtLen = vTangent.length();
                if (vtLen > 1e-6f) {
                    float drop = Math.min(vtLen, FRICTION * dt);
                    vTangent.mul((vtLen - drop) / vtLen);
                }
            }

            Vector3f nComp = new Vector3f(rampN).mul(velocity.dot(rampN));
            velocity.set(nComp.add(vTangent));
        }
    }
}