#version 410 core

#define SPHERE_RADIUS 3.0

// Low enough that the markings stay under the Nyquist limit at any speed the ball
// can reach: at 225 units/s it turns 0.099 of a revolution per frame, so anything
// above about five repeats around the sphere aliases into a standstill.
#define BALL_TEX_REPEATS 2.0

const vec3 BALL_LIGHT = vec3(0.45, 1.00, 0.55);
const vec3 BALL_DARK = vec3(0.03, 0.26, 0.10);
const vec3 BALL_LIGHT_DIR = normalize(vec3(0.35, 0.82, -0.45));
#define PI 3.14159265
#define TAU (2*PI)

// Bloom. Taps per ring and ring count decide the cost: this is TAPS*RINGS samples of the
// scene buffer per pixel, which is the whole budget of the effect.
#define BLOOM_TAPS 8
#define BLOOM_RINGS 3
#define BLOOM_STEP 2.6        // pixels between rings; a tight halo, not a smear
#define BLOOM_THRESHOLD 0.39  // only the bright cores of the lines glow
#define BLOOM_STRENGTH 2.10

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
uniform int gameState;          // 0 = menu, 1 = ready, 2 = playing, 3 = paused, 4 = dead
uniform float hudTime;          // seconds elapsed, for animated effects
uniform int debugHud;           // 1 = draw the developer overlays (axis gizmo, ball marker)

// The overlay: score, prompts, menus and the debug readout, drawn on the CPU into an
// RGBA canvas. Text of any kind lives there now, because a fragment shader can only
// draw glyphs it has been taught one expression at a time.
uniform sampler2D hudTexture;

// Theme, which shifts with distance so a long run reads as progress.
uniform vec3 skyTop;
uniform vec3 skyBottom;
uniform vec3 fogColor;

// Full-screen effects: speed vignette and streaks, and a flash on impact.
uniform float speedFx;
uniform float flashAmount;

layout(std140) uniform CameraMatrices {
    mat4 projectionMatrix;
    mat4 viewMatrix;
    mat4 invProjectionMatrix;
    mat4 invViewMatrix;
    float nearPlane;
    float farPlane;
} CamMatrix;

// ---------------------------------------------------------------------------
// HUD: a single 5x7 pixel typeface drawn in pixel space. Every readout is built
// from this plus a couple of boxes, so the overlay reads as one kit.
// ---------------------------------------------------------------------------

// Glyph slots: 0-9 are the digits, then the handful of letters the HUD needs.
#define G_A 10
#define G_B 11
#define G_C 12
#define G_E 13
#define G_P 14
#define G_S 15
#define G_T 16

// Five-bit rows of a glyph, top row first: x holds rows 0-3, y holds rows 4-6.
// A switch rather than a lookup table: a const array this size is dynamically
// indexed, which some drivers refuse to link.
ivec2 glyphRows(int g) {
    switch (g) {
        case  0: return ivec2(0x74675, 0x662E); // 0
        case  1: return ivec2(0x23084, 0x108E); // 1
        case  2: return ivec2(0x74422, 0x111F); // 2
        case  3: return ivec2(0xF8882, 0x062E); // 3
        case  4: return ivec2(0x11952, 0x7C42); // 4
        case  5: return ivec2(0xFC3C1, 0x062E); // 5
        case  6: return ivec2(0x3221E, 0x462E); // 6
        case  7: return ivec2(0xF8444, 0x2108); // 7
        case  8: return ivec2(0x7462E, 0x462E); // 8
        case  9: return ivec2(0x7462F, 0x044C); // 9
        case 10: return ivec2(0x7463F, 0x4631); // A
        case 11: return ivec2(0xF463E, 0x463E); // B
        case 12: return ivec2(0x74610, 0x422E); // C
        case 13: return ivec2(0xFC21E, 0x421F); // E
        case 14: return ivec2(0xF463E, 0x4210); // P
        case 15: return ivec2(0x7C20E, 0x043E); // S
        case 16: return ivec2(0xF9084, 0x1084); // T
        default: return ivec2(0, 0);
    }
}

int fontRow(int g, int r) {
    ivec2 rows = glyphRows(g);

    return (r < 4) ? (rows.x >> (5 * (3 - r))) & 31
                   : (rows.y >> (5 * (6 - r))) & 31;
}

// Is grid cell (q.x, q.y) of glyph g lit? The grid is 5 wide, 7 tall, y up.
float glyphBit(vec2 q, int g) {
    if (q.x < 0.0 || q.x >= 5.0 || q.y < 0.0 || q.y >= 7.0) {
        return 0.0;
    }

    int cx = int(q.x);
    int cy = int(q.y);

    return float((fontRow(g, 6 - cy) >> (4 - cx)) & 1);
}

// Coverage of one glyph centred on the origin, `cell` pixels per grid cell.
// Four samples per fragment keep the pixel edges from crawling.
float glyph(vec2 p, int g, float cell) {
    if (g < 0 || abs(p.x) > cell * 3.0 || abs(p.y) > cell * 4.0) {
        return 0.0;
    }

    vec2 q = p / cell + vec2(2.5, 3.5);
    float e = 0.25 / cell;

    return 0.25 * (glyphBit(q + vec2(-e, -e), g)
                 + glyphBit(q + vec2( e, -e), g)
                 + glyphBit(q + vec2(-e,  e), g)
                 + glyphBit(q + vec2( e,  e), g));
}

int digitCount(int v) {
    int n = 1;

    while (v >= 10 && n < 9) {
        v /= 10;
        n++;
    }

    return n;
}

// Glyphs advance six cells: five of ink, one of air.
#define ADVANCE 6.0

// An integer centred on p, leading zeros suppressed.
float numberMask(vec2 p, int value, float cell) {
    value = max(value, 0);

    int n = digitCount(value);
    int v = value;
    float m = 0.0;

    // Walk the digits least-significant first, placing them right to left.
    for (int i = n - 1; i >= 0; i--) {
        int d = v % 10;
        v /= 10;
        m = max(m, glyph(p - vec2((float(i) - 0.5 * float(n - 1)) * ADVANCE * cell, 0.0), d, cell));
    }

    return m;
}

// The two words the HUD needs, centred on p. Spelling them out keeps every
// glyph index a literal.
float wordBest(vec2 p, float cell) {
    float adv = ADVANCE * cell;
    float m = glyph(p + vec2(1.5 * adv, 0.0), G_B, cell);

    m = max(m, glyph(p + vec2(0.5 * adv, 0.0), G_E, cell));
    m = max(m, glyph(p - vec2(0.5 * adv, 0.0), G_S, cell));
    m = max(m, glyph(p - vec2(1.5 * adv, 0.0), G_T, cell));

    return m;
}

float wordSpace(vec2 p, float cell) {
    float adv = ADVANCE * cell;
    float m = glyph(p + vec2(2.0 * adv, 0.0), G_S, cell);

    m = max(m, glyph(p + vec2(adv, 0.0), G_P, cell));
    m = max(m, glyph(p, G_A, cell));
    m = max(m, glyph(p - vec2(adv, 0.0), G_C, cell));
    m = max(m, glyph(p - vec2(2.0 * adv, 0.0), G_E, cell));

    return m;
}

// "BEST 1234" as one centred line: the label, a gap, then the value.
float bestLine(vec2 p, int value, float cell) {
    value = max(value, 0);

    int nd = digitCount(value);
    float total = 4.0 * ADVANCE + 2.0 + float(nd) * ADVANCE - 1.0;
    float left = -0.5 * total;

    float m = wordBest(p - vec2((left + 11.5) * cell, 0.0), cell);
    float numLeft = left + 4.0 * ADVANCE + 2.0;

    return max(m, numberMask(p - vec2((numLeft + 3.0 * float(nd) - 0.5) * cell, 0.0), value, cell));
}

float sdBox(vec2 p, vec2 b) {
    vec2 d = abs(p) - b;
    return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0);
}

// Axis-aligned bar with a one-pixel soft edge.
float barMask(vec2 p, vec2 centre, vec2 halfSize) {
    return smoothstep(0.75, -0.25, sdBox(p - centre, halfSize));
}

// Filled right-pointing triangle centred on the origin, for the "ready" prompt.
float playTriangle(vec2 p, float s) {
    vec2 a = vec2(-0.55 * s, -s);
    vec2 b = vec2(-0.55 * s,  s);
    vec2 c = vec2( 0.85 * s,  0.0);

    float d1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    float d2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    float d3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);

    bool inside = (d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0)
               || (d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0);

    return inside ? 1.0 : 0.0;
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

// The ball is one analytic sphere, so it is intersected in closed form rather than
// marched. A quadratic replaces what used to be up to MAX_STEPS distance evaluations
// per pixel, and it is exact instead of epsilon-limited.
//
// The inverse camera matrices arrive through the uniform block. They are constant for
// the whole frame, so inverting them here -- once per fragment, twice over -- was by
// far the most expensive thing this shader did.
bool raymarched(vec2 uv, vec2 ndc, inout vec3 p) {

    // Get World View
    vec4 clipSpacePos = vec4(ndc, -1.0, 1.0);
    vec4 viewSpacePos = CamMatrix.invProjectionMatrix * clipSpacePos;
    viewSpacePos /= viewSpacePos.w;

    vec3 ro = camPosition;
    vec3 rd = normalize((CamMatrix.invViewMatrix * vec4(viewSpacePos.xyz, 0.0)).xyz);

    // Ray/sphere intersection: |ro + rd*t - centre| = radius, solved for the near root.
    vec3 oc = ro - position;
    float b = dot(oc, rd);
    float c = dot(oc, oc) - SPHERE_RADIUS * SPHERE_RADIUS;
    float h = b * b - c;

    // Negative discriminant means the ray passes the ball entirely.
    if(h < 0.0) {
        return false;
    }

    h = sqrt(h);
    float t = -b - h;

    // Near root behind the eye: we are inside the ball, so take the far one.
    if(t < 0.0) {
        t = -b + h;
    }

    // Still behind the eye, or past the far plane.
    if(t < 0.0 || t > CamMatrix.farPlane) {
        return false;
    }

    p = ro + rd * t;
    return true;
}

float LinearizeDepth(float d)  {
    float z = d * 2.0 - 1.0;
    return (2.0 * CamMatrix.nearPlane * CamMatrix.farPlane)
    / (CamMatrix.farPlane + CamMatrix.nearPlane - z * (CamMatrix.farPlane - CamMatrix.nearPlane));
}

// Neon bloom. The world is thin bright lines on near-black, which is exactly the case
// where a little light bleed reads as glow rather than as blur. Gathered straight out of
// the scene buffer in rings: a proper separable blur would need its own passes and target,
// and at this tap count the difference is not worth the plumbing.
//
// Only what is already brighter than the threshold contributes. The threshold has to sit
// above the flat surfaces or this stops being a glow: with it low, whole tower faces and
// road panels clear it, and adding a blurred copy of a large bright surface to itself
// just reads as the thing being out of focus. High enough, and only the bright cores of
// the grid lines bleed, which is what looks lit.
vec3 gatherBloom(vec2 uv) {
    vec3 sum = vec3(0.0);
    float total = 0.0;

    for (int ring = 1; ring <= BLOOM_RINGS; ring++) {
        float radius = float(ring) * BLOOM_STEP;

        // Weight falls off with distance, so the core stays tight and the halo is soft.
        float weight = 1.0 / float(ring * ring);

        for (int i = 0; i < BLOOM_TAPS; i++) {
            // Offset each ring so the taps do not line up into spokes.
            float a = ((float(i) + (0.5 * float(ring))) / float(BLOOM_TAPS)) * TAU;
            vec2 off = vec2(cos(a), sin(a)) * radius / iResolution;

            vec3 c = texture(textureSampler0, uv + off).rgb;
            sum += max(vec3(0.0), c - BLOOM_THRESHOLD) * weight;
            total += weight;
        }
    }

    return sum / max(total, 1e-4);
}

// Distance from point p to segment ab (2D)
float sdSegment(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h);
}

void main() {
    bool ballDrawn = false;
    vec3 p = vec3(0.0);
    vec2 uv = (2.0 * gl_FragCoord.xy - iResolution.xy) / iResolution.y;
    vec2 ndc = (gl_FragCoord.xy / iResolution) * 2.0 - 1.0;
    bool hit = raymarched(uv, ndc, p);
    vec4 col = vec4(0.0);
    float depth = texture(textureSampler2, fragTexCoords).r;
    float sceneDepth = depth;
    vec4 tex = texture(textureSampler0, fragTexCoords);


    if(hit) {
        // Two different normals. The world one shades the ball; the body one is the
        // same direction carried into the ball's own frame, so markings painted with it
        // turn as the ball turns.
        vec3 nor = normalize(p - position);
        vec3 bodyNor = quatRotate(sphereOrientation, nor);

        vec2 spTexCoord = sphereUV(bodyNor);
        col = texture(textureSampler1, spTexCoord * BALL_TEX_REPEATS);

        // Depth computed the way the rasterised scene computes it, so comparing against
        // the depth buffer is like for like. The old version measured from the world
        // origin rather than the camera, so the ball vanished once the run had travelled
        // far enough away from it.
        vec4 sphereClip = CamMatrix.projectionMatrix * (CamMatrix.viewMatrix * vec4(p, 1.0));
        float ndcDepth = (sphereClip.z / sphereClip.w) * 0.5 + 0.5;

        if(ndcDepth < depth) {
            // Eight big wedges painted on the body. Whatever axis the ball is spinning
            // about, a boundary is always crossing the silhouette, and a shape this
            // large cannot alias: the old six-times-tiled grid moved more than half a
            // cycle per frame above about 188 units/s, so it strobed and read as
            // standing still exactly when the ball was going quickest.
            float wedge = step(0.0, bodyNor.x * bodyNor.y * bodyNor.z);
            vec3 paint = mix(BALL_DARK, BALL_LIGHT, wedge) * (0.55 + (0.45 * col.g));

            // Plain diffuse and a rim, so the thing reads as a sphere at all. Unlit, it
            // was a flat disc, and a flat disc has no visible rotation however fast it
            // is actually turning.
            vec3 viewDir = normalize(p - camPosition);
            float diffuse = max(0.0, dot(nor, BALL_LIGHT_DIR));
            float rim = pow(1.0 - max(0.0, dot(nor, -viewDir)), 3.0);

            vec3 lit = paint * (0.30 + (0.70 * diffuse));
            lit += BALL_LIGHT * rim * 0.45;

            fragColor = vec4(lit, 1.0);
            ballDrawn = true;
            sceneDepth = ndcDepth;
        }
    }

    if (!ballDrawn) {
        fragColor = tex;
    }

    // ---- Sky, fog and full-screen effects ---------------------------------
    vec2 fc = gl_FragCoord.xy;
    vec2 res = iResolution;
    vec2 fromCentre = (fc / res) * 2.0 - 1.0;

    // Nothing was rasterised here, so this fragment is sky. Painting it from the theme's
    // two sky colours is what makes a zone change read as a change of place.
    if (!ballDrawn && depth >= 0.999999) {
        fragColor = vec4(mix(skyBottom, skyTop, clamp(fc.y / res.y, 0.0, 1.0)), 1.0);
    } else {
        // Distance fog towards the theme's fog colour, so the far end of the road fades
        // into the sky instead of stopping at a hard edge.
        float lin = clamp(LinearizeDepth(sceneDepth) / CamMatrix.farPlane, 0.0, 1.0);
        fragColor.rgb = mix(fragColor.rgb, fogColor, smoothstep(0.30, 1.0, lin));
    }

    // Glow. Added rather than mixed, because neon is light on top of the scene, not a
    // tint of it. The ball is excluded: it is raymarched into this pass rather than drawn
    // into the scene buffer, so a gather here would smear the road over it.
    fragColor.rgb += gatherBloom(fragTexCoords) * BLOOM_STRENGTH;

    // Speed. The vignette closes in at the edges. The raking streaks that used to go
    // with it were removed: they washed out the road rather than selling velocity.
    if (speedFx > 0.01) {
        float r = length(fromCentre);

        fragColor.rgb = mix(fragColor.rgb, fogColor * 0.35,
                smoothstep(0.40, 1.30, r) * speedFx * 0.55);
    }

    // A dim behind the ready prompt and the menus, so the overlay stays readable. The
    // menu backdrop is drawn on the canvas itself, so only the ready state needs this.
    if (gameState == 1) {
        fragColor.rgb = mix(fragColor.rgb, vec3(0.02, 0.02, 0.03), 0.35);
    }

    // Dead: a red wash that deepens towards the edges.
    if (gameState == 4) {
        float vignette = clamp(length(fromCentre) * 0.8, 0.0, 1.0);
        fragColor.rgb = mix(fragColor.rgb, vec3(0.22, 0.02, 0.02), 0.22 + 0.28 * vignette);
    }

    // Impact flash: red for a crash, near-white for a hard landing.
    if (flashAmount > 0.001) {
        vec3 flashColor = (gameState == 4) ? vec3(0.8, 0.08, 0.06) : vec3(0.9, 0.95, 1.0);
        fragColor.rgb = mix(fragColor.rgb, flashColor, clamp(flashAmount, 0.0, 1.0) * 0.55);
    }

    // ---- Overlay -----------------------------------------------------------
    // The canvas has its origin at the top left; the screen quad's V runs the other way,
    // so it is flipped here rather than uploading the canvas upside down.
    vec4 ui = texture(hudTexture, vec2(fragTexCoords.x, 1.0 - fragTexCoords.y));
    fragColor.rgb = mix(fragColor.rgb, ui.rgb, ui.a);
    fragColor.a = 1.0;


    // ---- Developer overlays, shown with the F1 free camera ------------------
    if (debugHud != 0) {
        // Axis gizmo, bottom-left.
        vec2 anchor = vec2(70.0, 70.0);
        float radius = 40.0;
        float thickness = 2.0;

        if (fc.x <= anchor.x + radius + 4.0 && fc.y <= anchor.y + radius + 4.0) {
            vec2 c = fc - anchor;

            // Camera-space directions of the world axes (w = 0 for directions).
            vec3 xCam = (CamMatrix.viewMatrix * vec4(1.0, 0.0, 0.0, 0.0)).xyz;
            vec3 yCam = (CamMatrix.viewMatrix * vec4(0.0, 1.0, 0.0, 0.0)).xyz;
            vec3 zCam = (CamMatrix.viewMatrix * vec4(0.0, 0.0, 1.0, 0.0)).xyz;

            vec2 x2 = normalize(vec2(xCam.x, xCam.y));
            vec2 y2 = normalize(vec2(yCam.x, yCam.y));
            vec2 z2 = normalize(vec2(zCam.x, zCam.y));

            vec4 overlay = vec4(vec3(smoothstep(1.5, 0.5, abs(length(c) - radius)) * 0.15), 1.0);

            overlay.rgb = mix(overlay.rgb, vec3(1.0, 0.2, 0.2), smoothstep(thickness + 0.5, thickness - 0.5, sdSegment(c, vec2(0.0), x2 * radius)));
            overlay.rgb = mix(overlay.rgb, vec3(0.2, 1.0, 0.2), smoothstep(thickness + 0.5, thickness - 0.5, sdSegment(c, vec2(0.0), y2 * radius)));
            overlay.rgb = mix(overlay.rgb, vec3(0.2, 0.6, 1.0), smoothstep(thickness + 0.5, thickness - 0.5, sdSegment(c, vec2(0.0), z2 * radius)));

            fragColor = mix(fragColor, overlay, 0.9 * step(length(c), radius + 2.0));
        }

        // Where the ball projects to on screen.
        vec4 clip = CamMatrix.projectionMatrix * (CamMatrix.viewMatrix * vec4(position, 1.0));

        if (clip.w != 0.0) {
            vec2 scr = (clip.xy / clip.w * 0.5 + 0.5) * res;
            float size = 6.0;
            float dH = sdSegment(fc, scr + vec2(-size, 0.0), scr + vec2(size, 0.0));
            float dV = sdSegment(fc, scr + vec2(0.0, -size), scr + vec2(0.0, size));

            fragColor = mix(fragColor, vec4(1.0), max(smoothstep(1.5, 0.5, dH), smoothstep(1.5, 0.5, dV)) * 0.75);
        }
    }
}
