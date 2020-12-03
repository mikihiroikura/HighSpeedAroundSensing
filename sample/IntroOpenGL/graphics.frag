#version 460
//
// simple.frag
//
in vec4 vertexColor;
 
void main(void)
{
  gl_FragColor = vertexColor;
  //gl_FragColor = vec4(1,0,0,1);
}