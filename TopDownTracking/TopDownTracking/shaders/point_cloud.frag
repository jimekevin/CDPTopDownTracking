#version 410

in vec2 vertTexCoord;

out vec4 fragColor;

uniform sampler2D ourTexture;

void main() {
  fragColor = texture(ourTexture, vertTexCoord);
  //fragColor = vec4(1.0, 0.0, 0.0, 1.0);
}