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



//OpenGL‚Ì‰Šú‰»
int initGL() {
    //GLFW‚Ì‰Šú‰»
    if (glfwInit() == GL_FALSE)
    {
        cerr << "Can't initilize GLFW" << endl;
        return 1;
    }

    //Window‚Ìì¬
    window = glfwCreateWindow(window_width, window_height, "Cuda GL Interop (VBO)", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
        return 1;
    }


    //Window‚ğOpenGL‚Ì‘ÎÛ‚É‚·‚é
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrent‚ÌŒã‚És‚í‚È‚¢‚Æ¸”s‚·‚é‚ç‚µ‚¢
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
        return 1;
    }

    glClearColor(0.0, 0.0, 0.0, 1.0);   //”wŒiF‚Ìw’è
    glDisable(GL_DEPTH_TEST);


	return 0;
}