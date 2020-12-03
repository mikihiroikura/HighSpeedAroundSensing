#version 460 core
//
// graphics.vert
//
layout (location = 0) in vec3 position; // the position variable has attribute position 0
layout (location = 1) in vec3 color;

uniform mat4 MVP;
out vec4 vertexColor;

void main()
{
    vec4 v = vec4(position, 1.0);
    gl_Position = MVP * v;
    /*if(position.y<0.5 && position.y>-0.5) {
        vertexColor = vec4(1.0, 0.0, 0.0, 1.0);
    }
    else if(vertices.y>=0.5) {
        vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
    }
    else {
        vertexColor = vec4(0.0, 0.0, 1.0, 1.0);
    }*/
    vertexColor = vec4(color, 1.0);
}