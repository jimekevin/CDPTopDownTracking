#version 410

uniform mat4 view;
uniform mat4 projection;

in vec3 vertex;
//in vec3 color;
in vec2 texCoord;

out vec2 vertTexCoord;

void main() {
  gl_Position = projection * view * vec4(vertex, 1.0);
  gl_PointSize = 1.5;
  //vertColor = vec4(color.x / 255, color.y / 255, color.y / 255, 1.0);
  vertTexCoord = texCoord;
}