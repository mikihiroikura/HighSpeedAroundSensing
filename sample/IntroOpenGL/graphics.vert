#version 460 core
//
// graphics.vert
//　gl_Positionはbuilt in変数
//指定Indexの変数
layout (location = 0) in vec3 position; // the position variable has attribute position 0
layout (location = 1) in vec3 color;

//shader program内のGlobal変数
uniform mat4 MVP;
//outの変数と.fragのinが名前が一致していれば引き継がれる
out vec4 vertexColor;

void main()
{
    vec4 v = vec4(position, 1.0);
    gl_Position = MVP * v;
    vertexColor = vec4(color, 1.0);
}