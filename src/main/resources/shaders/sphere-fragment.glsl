#version 410 core

#define MAX_STEPS 80
#define PI 3.14159265
#define TAU (2*PI)

in vec3 color;
in vec2 fragTexCoords;
in vec4 outColor;
out vec4 fragColor;

uniform sampler2D textureSampler0;
uniform sampler2D textureSampler1;
uniform sampler2D textureSampler2;
uniform vec2 iResolution;
uniform vec3 camPosition;
uniform vec3 position;
uniform vec4 sphereOrientation; // quaternion (x,y,z,w)

layout(std140) uniform CameraMatrices {
    mat4 projectionMatrix;
    mat4 viewMatrix;
    float nearPlane;
    float farPlane;
} CamMatrix;

vec3 rotateZ(vec3 p, float angle) {
    float cosT = cos(angle);
    float sinT = sin(angle);

    return vec3(
        p.x * cosT - p.y * sinT,
        p.x * sinT + p.y * cosT,
        p.z
    );
}

vec3 rotateY(vec3 p, float angle) {
    float cosT = cos(angle);
    float sinT = sin(angle);

    return vec3(
        p.x * cosT + p.z * sinT,
        p.y,
        p.z * cosT - p.x * sinT
    );
}

vec3 rotateX(vec3 p, float angle) {
    float cosT = cos(angle);
    float sinT = sin(angle);

    return vec3(
        p.y * sinT + p.x * cosT,
        p.y * cosT - p.x * sinT,
        p.z
    );
}

float atan2(in float y, in float x) {
    return y > 0.0 ? atan(y, x) + PI : -atan(y, -x);
}

vec3 quatRotate(vec4 q, vec3 v) {
    // q = (x,y,z,w) with w as scalar
    vec3 t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

vec2 sphereUV(vec3 p) {
    p = rotateX(p, PI / 4);
    p = rotateY(p, 0);
    p = rotateZ(p, 0);

    float r = length(p);
    float phi = atan2(p.z, p.x);
    return vec2(phi / TAU, acos(p.y / r) / PI);
}

// Source: https://iquilezles.org/articles/distfunctions/
float sdSphere(vec3 p, float s) {
    return length(p-position)-s;
}


// Raymarching Algorithm made by Diego Fonseca + various online videos
// Source for linear view transformation: https://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
bool raymarched(vec2 uv, vec2 ndc, inout vec3 p) {

    // Get World View
    vec4 clipSpacePos = vec4(ndc, -1.0, 1.0);
    vec4 viewSpacePos = inverse(CamMatrix.projectionMatrix) * clipSpacePos;
    viewSpacePos /= viewSpacePos.w;

    // Ray Marching
    vec3 ro = camPosition;
    vec3 rd = normalize((inverse(CamMatrix.viewMatrix) * vec4(viewSpacePos.xyz, 0.0)).xyz);

    // Total distance
    float t = 0.0;

    // Raymarching
    // TODO: Instead of MIN and MAX distance use z-near and z-far.
    for(int i=0; i<MAX_STEPS; i++) {
        p = ro + rd * t;
        float d = sdSphere(p, 3.0);
        t += d;

        if(d < CamMatrix.nearPlane) {
            return true;
        }

        if(d > CamMatrix.farPlane) {
            return false;
        }
    }

    return false;
}

float LinearizeDepth(float d)  {
    float z = d * 2.0 - 1.0;
    return (2.0 * CamMatrix.nearPlane * CamMatrix.farPlane)
    / (CamMatrix.farPlane + CamMatrix.nearPlane - z * (CamMatrix.farPlane - CamMatrix.nearPlane));
}

// Distance from point p to segment ab (2D)
float sdSegment(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h);
}

void main() {
    vec3 p = vec3(0.0);
    vec2 uv = (2.0 * gl_FragCoord.xy - iResolution.xy) / iResolution.y;
    vec2 ndc = (gl_FragCoord.xy / iResolution) * 2.0 - 1.0;
    bool hit = raymarched(uv, ndc, p);
    vec4 col = vec4(0.0);
    float depth = texture(textureSampler2, fragTexCoords).r;
    vec4 tex = texture(textureSampler0, fragTexCoords);


    if(hit) {
        vec3 nor = normalize(p - position);
        // Apply sphere orientation to normal for texturing highlight (optional)
        nor = quatRotate(sphereOrientation, nor);
        vec2 spTexCoord = sphereUV(nor);
        col = texture(textureSampler1, spTexCoord * 6);

        float sphDepth = length((p - position) - camPosition);
        float ndcDepth = (sphDepth - CamMatrix.nearPlane) / (CamMatrix.farPlane - CamMatrix.nearPlane);
        ndcDepth = clamp(ndcDepth, 0.0, 1.0);

        if(ndcDepth < depth) {
            fragColor = vec4(0.0, col.g, 0.0, col.a);
            return;
        }
    }

    fragColor = tex;

    // --- Axis overlay (bottom-left corner) ---
    // Size and placement (in pixels)
    vec2 anchor = vec2(70.0, 70.0);
    float radius = 40.0;
    float thickness = 2.0;

    // Only compute inside the overlay box to keep it cheap
    if (gl_FragCoord.x <= anchor.x + radius + 2.0 && gl_FragCoord.y <= anchor.y + radius + 2.0) {
        // Map current fragment to overlay-local coordinates
        vec2 c = gl_FragCoord.xy - anchor;

        // Camera-space directions of world axes using view matrix (w=0 for directions)
        vec3 xCam = (CamMatrix.viewMatrix * vec4(1.0, 0.0, 0.0, 0.0)).xyz;
        vec3 yCam = (CamMatrix.viewMatrix * vec4(0.0, 1.0, 0.0, 0.0)).xyz;
        vec3 zCam = (CamMatrix.viewMatrix * vec4(0.0, 0.0, 1.0, 0.0)).xyz;

        // 2D projected directions (X-right, Y-up in camera space)
        vec2 x2 = normalize(vec2(xCam.x, xCam.y));
        vec2 y2 = normalize(vec2(yCam.x, yCam.y));
        vec2 z2 = normalize(vec2(zCam.x, zCam.y));

        // Draw circle background faintly
        float dCircle = abs(length(c) - radius);
        float ring = smoothstep(1.5, 0.5, dCircle);
        vec4 overlay = vec4(0.0);
        overlay.rgb += ring * 0.15;

        // Draw axis lines
        float dx = sdSegment(c, vec2(0.0), x2 * radius);
        float dy = sdSegment(c, vec2(0.0), y2 * radius);
        float dz = sdSegment(c, vec2(0.0), z2 * radius);

        overlay.rgb = mix(overlay.rgb, vec3(1.0, 0.2, 0.2), smoothstep(thickness+0.5, thickness-0.5, dx)); // X - red
        overlay.rgb = mix(overlay.rgb, vec3(0.2, 1.0, 0.2), smoothstep(thickness+0.5, thickness-0.5, dy)); // Y - green
        overlay.rgb = mix(overlay.rgb, vec3(0.2, 0.6, 1.0), smoothstep(thickness+0.5, thickness-0.5, dz)); // Z - blue

        // Composite over the current color
        float a = 0.9 * step(length(c), radius + 2.0);
        fragColor = mix(fragColor, vec4(overlay.rgb, 1.0), a);
    }
}