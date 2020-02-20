#version 330
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;

uniform mat4 view;
uniform mat4 projection;
uniform mat4 model;

out vec3 mColor;

void main() {
    gl_Position = projection * view * model * vec4(aPosition, 1.0f);
    mColor = aColor; 
}