#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

in VS_OUT {
    flat vec2 center;
    flat float radius;
    flat vec3 color;
} gs_in[];

flat out vec2 f_center;
flat out float f_radius;
flat out vec3 f_color;
out vec2 f_position;

void main()
{
    f_center = gs_in[0].center;
    f_radius = gs_in[0].radius;
    f_color = gs_in[0].color;

    vec4 vertPos;

    vertPos = gl_in[0].gl_Position + vec4(gs_in[0].radius, -gs_in[0].radius, 0.0, 0.0);
    gl_Position = vertPos;
    f_position = vertPos.xy;
    EmitVertex();

    vertPos = gl_in[0].gl_Position + vec4(-gs_in[0].radius, -gs_in[0].radius, 0.0, 0.0);
    gl_Position = vertPos;
    f_position = vertPos.xy;
    EmitVertex();

    vertPos = gl_in[0].gl_Position + vec4(gs_in[0].radius, gs_in[0].radius, 0.0, 0.0);
    gl_Position = vertPos;
    f_position = vertPos.xy;
    EmitVertex();

    vertPos = gl_in[0].gl_Position + vec4(-gs_in[0].radius, gs_in[0].radius, 0.0, 0.0);
    gl_Position = vertPos;
    f_position = vertPos.xy;
    EmitVertex();

    EndPrimitive();
}
