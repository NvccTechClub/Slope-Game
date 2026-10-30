#version 410 core

in vec2 fragTexCoords;
in vec4 outColor;
in vec3 worldPos;
in float viewDepth;
out vec4 fragColor;

uniform sampler2D textureSampler;

// Multiplied over the mesh colour. The theme recolours the whole world through this, so
// the road, the blocks, the tunnel walls, the kickers and the pickups are all one mesh
// and one texture wearing different paint.
uniform vec4 propTint;

// Floor under the texture, before the tint. Zero leaves the grid-on-black wireframe
// look alone; higher fills the dark gaps so the prop reads as a solid object.
uniform float propFill;

// 1 on the road surface, 0 on everything else. Rungs only belong on the thing you are
// travelling over.
uniform float rungAmount;

// Lines drawn across the road, every RUNG_SPACING units along it. Transverse lines are
// the cue that actually reads as speed, because they sweep underneath the player rather
// than past the sides of the screen, and the eye measures its own motion against them.
#define RUNG_SPACING 12.0
#define RUNG_WIDTH 0.10

void main() {
    vec4 tex = texture(textureSampler, fragTexCoords);
    vec4 base = tex * outColor;

    // Lift the gaps between the grid lines. The lines are still the brightest part, so a
    // filled prop keeps its outline and gains a face instead of being a hollow cage.
    base.rgb = max(base.rgb, vec3(propFill));

    vec4 colour = base * propTint;

    if (rungAmount > 0.0) {
        // The road runs along -X, so position along it is simply the world x.
        float along = worldPos.x / RUNG_SPACING;
        float edge = abs(fract(along) - 0.5);

        // Widen the line with distance in step with how fast it is shrinking on screen,
        // so far-off rungs blur into an even wash instead of flickering as they cross
        // pixel boundaries.
        float footprint = max(fwidth(along), 0.0012);
        float line = 1.0 - smoothstep(RUNG_WIDTH, RUNG_WIDTH + footprint, edge);

        // Strongest just in front of the player, gone by the time the road is far away:
        // near rungs are the ones moving fast enough to be worth drawing.
        float near = 1.0 - smoothstep(40.0, 320.0, viewDepth);

        colour.rgb += propTint.rgb * line * near * rungAmount * 0.85;
    }

    fragColor = colour;
}