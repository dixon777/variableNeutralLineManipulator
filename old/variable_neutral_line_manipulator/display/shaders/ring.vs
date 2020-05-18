#version 330
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
layout(location = 2) in mat4 aTransform;

uniform mat4 view;
uniform mat4 projection;

out vec3 mColor;

void main() {
    gl_Position = (projection*view)*aTransform*vec4(aPosition, 1.0f);
    mColor = aColor; 
}