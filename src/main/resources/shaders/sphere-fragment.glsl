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
uniform vec3 velocity;
uniform vec3 acceleration;
uniform vec4 sphereOrientation; // quaternion (x,y,z,w)

layout(std140) uniform CameraMatrices {
    mat4 projectionMatrix;
    mat4 viewMatrix;
    float nearPlane;
    float farPlane;
} CamMatrix;

// ---- Minimal helpers for HUD digits (7-segment) ----
float sdBox(vec2 p, vec2 b) {
    vec2 d = abs(p) - b;
    return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0);
}

// Draw a single 7-segment digit at origin, size = scale pixels
// Returns coverage mask in [0,1]
float drawDigit(vec2 p, int digit, float scale) {
    // Segment layout (A,B,C,D,E,F,G) bitmask per digit (0-9)
    // A: top, B: top-right, C: bottom-right, D: bottom, E: bottom-left, F: top-left, G: middle
    int segMask[10] = int[](0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F);
    int mask = segMask[clamp(digit, 0, 9)];

    float w = 1.4 * scale;  // segment thickness
    float L = 5.0 * scale;  // segment half-length horizontally
    float H = 8.0 * scale;  // vertical spacing

    float m = 0.0;

    // A (top)
    if ((mask & 1) != 0) {
        float d = sdBox(p - vec2(0.0,  H), vec2(L, w));
        m = max(m, 1.0 - smoothstep(0.5, 1.5, d));
    }
    // B (top-right)
    if ((mask & 2) != 0) {
        float d = sdBox(p - vec2( L,  H*0.5), vec2(w, H*0.5 - w*0.3));
        m = max(m, 1.0 - smoothstep(0.5, 1.5, d));
    }
    // C (bottom-right)
    if ((mask & 4) != 0) {
        float d = sdBox(p - vec2( L, -H*0.5), vec2(w, H*0.5 - w*0.3));
        m = max(m, 1.0 - smoothstep(0.5, 1.5, d));
    }
    // D (bottom)
    if ((mask & 8) != 0) {
        float d = sdBox(p - vec2(0.0, -H), vec2(L, w));
        m = max(m, 1.0 - smoothstep(0.5, 1.5, d));
    }
    // E (bottom-left)
    if ((mask & 16) != 0) {
        float d = sdBox(p - vec2(-L, -H*0.5), vec2(w, H*0.5 - w*0.3));
        m = max(m, 1.0 - smoothstep(0.5, 1.5, d));
    }
    // F (top-left)
    if ((mask & 32) != 0) {
        float d = sdBox(p - vec2(-L,  H*0.5), vec2(w, H*0.5 - w*0.3));
        m = max(m, 1.0 - smoothstep(0.5, 1.5, d));
    }
    // G (middle)
    if ((mask & 64) != 0) {
        float d = sdBox(p - vec2(0.0, 0.0), vec2(L, w));
        m = max(m, 1.0 - smoothstep(0.5, 1.5, d));
    }
    return m;
}

// Draw an integer value (0..999) using 7-seg digits, left-to-right
float drawInt3(vec2 p, int value, float scale) {
    value = clamp(value, 0, 999);
    int hundreds = value / 100;
    int tens = (value / 10) % 10;
    int ones = value % 10;

    float spacing = 14.0 * scale;
    float m = 0.0;

    if (hundreds > 0) {
        m = max(m, drawDigit(p + vec2(0.0, 0.0), hundreds, scale));
        m = max(m, drawDigit(p + vec2(spacing, 0.0), tens, scale));
        m = max(m, drawDigit(p + vec2(spacing*2.0, 0.0), ones, scale));
    } else if (tens > 0) {
        m = max(m, drawDigit(p + vec2(0.0, 0.0), tens, scale));
        m = max(m, drawDigit(p + vec2(spacing, 0.0), ones, scale));
    } else {
        m = max(m, drawDigit(p + vec2(0.0, 0.0), ones, scale));
    }
    return m;
}

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

    // Only compute inside the overlay box to keep it cheap (expanded to fit HUD bars)
    if (gl_FragCoord.x <= anchor.x + radius + 90.0 && gl_FragCoord.y <= anchor.y + radius + 90.0) {
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

        // Composite axis ring over the current color
        float a = 0.9 * step(length(c), radius + 2.0);
        fragColor = mix(fragColor, vec4(overlay.rgb, 1.0), a);

    }

    // --- Right-side HUD: speed and acceleration bars + MPH numbers ---
    {
        float thicknessBar = 2.0;
        float maxLen = 100.0;                 // max bar length in pixels
        float speedLen = clamp(length(velocity) * 6.0, 0.0, maxLen);
        float accelLen = clamp(length(acceleration) * 12.0, 0.0, maxLen);

        vec3 speedCol = vec3(1.0, 0.6, 0.2);
        vec3 accelCol = vec3(0.2, 0.9, 1.0);

        float margin = 70.0;                 // distance from right/bottom edges
        vec2 rAnchor = vec2(iResolution.x - margin, margin);

        // Limit work to a small region in the bottom-right
        if (gl_FragCoord.x >= (iResolution.x - (margin + 100.0)) && gl_FragCoord.y <= (margin + 100.0)) {
            vec2 r = gl_FragCoord.xy - rAnchor;

            // Bars extend leftwards from the anchor
            vec2 speedA = vec2(0.0, 0.0);
            vec2 speedB = vec2(-speedLen, 0.0);
            vec2 accelA = vec2(0.0, -12.0);
            vec2 accelB = vec2(-accelLen, -12.0);

            float dSpeed = sdSegment(r, speedA, speedB);
            float dAccel = sdSegment(r, accelA, accelB);
            float sMask = smoothstep(thicknessBar + 0.5, thicknessBar - 0.5, dSpeed);
            float aMask = smoothstep(thicknessBar + 0.5, thicknessBar - 0.5, dAccel);
            fragColor = mix(fragColor, vec4(speedCol, 1.0), sMask * 0.9);
            fragColor = mix(fragColor, vec4(accelCol, 1.0), aMask * 0.9);

            // MPH readouts (rounded integer)
            float speedMPH = length(velocity) * 2.2369363;
            float accelMPHps = length(acceleration) * 2.2369363;
            int spVal = int(floor(speedMPH + 0.5));
            int acVal = int(floor(accelMPHps + 0.5));

            // Place numbers slightly above and below bars, left of the anchor
            float scale = 1.0;
            vec2 numPosSpeed = vec2(-68.0, 10.0);
            vec2 numPosAccel = vec2(-68.0, -26.0);

            float spMask = drawInt3(r - numPosSpeed, spVal, scale);
            float acMask = drawInt3(r - numPosAccel, acVal, scale);
            fragColor = mix(fragColor, vec4(speedCol, 1.0), spMask * 0.95);
            fragColor = mix(fragColor, vec4(accelCol, 1.0), acMask * 0.95);
        }
    }

    // --- Screen-space marker at sphere position (small crosshair) ---
    {
        vec4 clip = CamMatrix.projectionMatrix * (CamMatrix.viewMatrix * vec4(position, 1.0));
        if (clip.w != 0.0) {
            vec2 ndcPos = clip.xy / clip.w;
            vec2 scr = (ndcPos * 0.5 + 0.5) * iResolution;

            float crossSize = 6.0;
            float dH = sdSegment(gl_FragCoord.xy, scr + vec2(-crossSize, 0.0), scr + vec2(crossSize, 0.0));
            float dV = sdSegment(gl_FragCoord.xy, scr + vec2(0.0, -crossSize), scr + vec2(0.0, crossSize));
            float ch = max(smoothstep(1.5, 0.5, dH), smoothstep(1.5, 0.5, dV));
            fragColor = mix(fragColor, vec4(1.0, 1.0, 1.0, 1.0), ch * 0.75);
        }
    }
}