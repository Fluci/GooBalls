#version 330 core

layout(location = 0) in vec2 center;
layout(location = 1) in float radius;
layout(location = 2) in vec3 color;

out VS_OUT {
    flat vec2 center;
    flat float radius;
    flat vec3 color;
} vs_out;

void main() {
    vs_out.center = center;
    vs_out.radius = radius;
    vs_out.color = color;

    gl_Position = vec4(center, 0.0, 1.0);
}
