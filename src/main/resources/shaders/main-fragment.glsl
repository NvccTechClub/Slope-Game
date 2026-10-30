#version 410 core

in vec3 color;
in vec2 fragTexCoords;
in vec2 projSpace;
in vec4 outColor;
in float depth;

out vec4 fragColor;

uniform sampler2D textureSampler;

float LinearizeDepth(float d)  {
    float z = d * 2.0 - 1.0;
    return (2.0 * projSpace.x * projSpace.y)
    / (projSpace.y + projSpace.x - z * (projSpace.y - projSpace.x));
}

void main() {
    // No gl_FragDepth write here. Writing depth from the fragment shader switches off
    // early-Z and hierarchical-Z on essentially every GPU, so each covered fragment
    // runs the full shader even when something nearer has already been drawn over it.
    // The rasteriser's own interpolated depth is what we want anyway.
    vec4 tex = texture(textureSampler, fragTexCoords);
    fragColor = vec4(tex.r * outColor.r, tex.g * outColor.g, tex.b * outColor.b, tex.a * outColor.a);
}