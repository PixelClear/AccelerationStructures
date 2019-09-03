#version 420

layout(location = 0) in vec3 in_Vertex;
layout(location = 1) in vec3 in_Normal;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

out vec3 v_WorldNormal;

void main()
{
    gl_Position = u_Projection * u_View * u_Model * vec4(in_Vertex, 1.0f);
    mat3 normalMatrix = transpose(inverse(mat3(u_Model)));
    v_WorldNormal = normalMatrix * in_Normal;  
}