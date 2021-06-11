#version 330 core

in vec3 vertex_normal;
in vec3 vertex_pos;
in vec2 vertex_tex;

uniform sampler2D tex;
uniform sampler2D tex2;
uniform vec3 camPos;

out vec4 fragCol;

void main()
{
	fragCol = vec4(1,0,0,1);
	vec4 tcol = texture(tex, vertex_tex);
	fragCol.rgb = tcol.rgb;
}


