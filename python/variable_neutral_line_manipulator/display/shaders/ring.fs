#version 330
in vec3 mColor;
out vec4 color_out;

void main() {
    color_out = vec4(mColor, 1.0);
}