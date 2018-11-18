#version 330 core

out vec4 color;

flat in vec2 f_center;
flat in float f_radius;
flat in vec3 f_color;

in vec2 f_position;

void main() {
    vec2 diffVec = f_position - f_center;
    float radSq = f_radius * f_radius;
    float diffSq = dot(diffVec, diffVec);
    vec3 lightVec = normalize(vec3(1, 1, 1));
    if(diffSq < radSq) {
        vec3 normal = normalize(vec3(diffVec, sqrt(radSq - diffSq)));
        color = vec4(dot(lightVec, normal) * f_color, 1);
    } else {
        color = vec4(0, 0, 0, 0);
    }
}
