#include "graphics.h"
#include <iostream>

using namespace std;

// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;
GLFWwindow* window;

//vbo
GLuint vbo;

//imgui



//OpenGL�̏�����
int initGL() {
    //GLFW�̏�����
    if (glfwInit() == GL_FALSE)
    {
        cerr << "Can't initilize GLFW" << endl;
        return 1;
    }

    //Window�̍쐬
    window = glfwCreateWindow(window_width, window_height, "Cuda GL Interop (VBO)", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
        return 1;
    }


    //Window��OpenGL�̑Ώۂɂ���
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrent�̌�ɍs��Ȃ��Ǝ��s����炵��
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
        return 1;
    }

    glClearColor(0.0, 0.0, 0.0, 1.0);   //�w�i�F�̎w��
    glDisable(GL_DEPTH_TEST);


	return 0;
}