#version 410

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

in vec3 vertex;
in vec3 color;

out vec4 vertColor;

void main() {
  gl_Position = projection * view * model * vec4(vertex, 1.0);
  gl_PointSize = 1.0;
  vertColor = vec4(color, 1.0);
  //vertColor = vec4(0.5, 0.5, 0.5, 0.9);
}