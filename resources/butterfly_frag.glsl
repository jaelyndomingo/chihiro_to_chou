#version 410 core
out vec4 color;
in vec3 vertex_normal;
in vec3 vertex_pos;
in vec2 vertex_tex;
uniform vec3 campos;

uniform vec2 texoff;
uniform vec2 texoff_next;
uniform float t;

uniform sampler2D tex;
uniform sampler2D tex2;

void main()
{
//vec3 n = normalize(vertex_normal);
//vec3 lp=vec3(10,-20,-100);
//vec3 ld = normalize(vertex_pos - lp);
//float diffuse = dot(n,ld);

vec4 tcol = texture(tex, vec2(vertex_tex.x/12 + texoff.x/12, vertex_tex.y/3 + texoff.y/3));

vec4 tcol2 = texture(tex, vec2(vertex_tex.x/12 + texoff_next.x/12, vertex_tex.y/3 + texoff_next.y/3));
/*vec4 tcol = texture(tex2, vec2(vertex_tex.x/10 + texoff.x/10, vertex_tex.y));

vec4 tcol2 = texture(tex2, vec2(vertex_tex.x/10 + texoff_next.x/10, vertex_tex.y));*/

color = (tcol * (1 - t)) + (tcol2 * t);


}
