#version 410 core

#define TOWERS_PER_ROW 7

layout(location = 0) in vec3 pos;
layout(location = 1) in vec2 texCoord;
layout(location = 2) in vec4 inColor;

layout(std140) uniform CameraMatrices {
    mat4 projectionMatrix;
    mat4 viewMatrix;
    mat4 invProjectionMatrix;
    mat4 invViewMatrix;
    float nearPlane;
    float farPlane;
} CamMatrix;

uniform mat4 model;

// 1.0 for the instanced tower field, 0.0 for props placed by their model matrix.
uniform float instanceSpread;

out vec2 fragTexCoords;
out vec4 outColor;

// Where this fragment sits along the road, and how far it is from the eye. The rungs
// are drawn from world position rather than from the mesh UVs, because the road tiles
// are scaled across their width and UV-derived lines would stretch with them.
out vec3 worldPos;
out float viewDepth;

void main() {
    float gap_offset = (gl_InstanceID % TOWERS_PER_ROW >= 3) ? 270.0f : 0.0;
    float x_offset = (gl_InstanceID / TOWERS_PER_ROW) * 75.0;

    vec3 instanceOffset = vec3(-x_offset * 2.0, (sin(float(gl_InstanceID)) * 50.0f) - x_offset, (float(gl_InstanceID % TOWERS_PER_ROW) * 150.0) - 266.0f + gap_offset);

    mat4 instanceModel = model;
    instanceModel[3].xyz += instanceOffset * instanceSpread;

    vec4 world = instanceModel * vec4(pos, 1.0);
    vec4 view = CamMatrix.viewMatrix * world;

    gl_Position = CamMatrix.projectionMatrix * view;

    worldPos = world.xyz;
    viewDepth = -view.z;

    fragTexCoords = texCoord;
    outColor = inColor;
}