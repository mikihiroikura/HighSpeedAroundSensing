#version 460 core
//
// graphics.vert
//
layout (location = 0) in vec3 position; // the position variable has attribute position 0
out vec4 vertexColor; // specify a color output to the fragment shader
uniform mat4 MVP;


void main()
{
    vec4 v = vec4(position, 1.0);
    gl_Position = MVP * v;
    if(vertices.y<0.5 && vertices.y>-0.5) vertexColor = vec4(1.0, 0.0, 0.0, 1.0);
    else if(vertices.y>=0.5) vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
    else vertexColor = vec4(0.0, 0.0, 1.0, 1.0);
}