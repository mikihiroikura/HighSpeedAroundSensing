#include "graphics.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;
GLFWwindow* window;

//vbo
GLuint vbo, cbo;

//imgui
float pointsize = 2.5;
bool hide_red, hide_geen, hide_blue;
float rotate_x = 0.0, rotate_y = 0.0;
double mouse_x, mouse_y, mouse_x_old, mouse_y_old;
double horiz_angle = -M_PI, vert_angle = 0.0;
double mouse_speed = 0.01;
double dx = 0.0, dy = 0.0;
float init_fov = 60, fov = init_fov;
float hovered;
glm::vec3 position(0, 0, -1), up(0, -1, 0), direction(0, 0, 0);


static void setfov(GLFWwindow* window, double x, double y);


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
    glPointSize(pointsize);
    glDisable(GL_DEPTH_TEST);


    //���_�o�b�t�@�I�u�W�F�N�g��Bind
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    //glBufferData(GL_ARRAY_BUFFER, )
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    //�F�o�b�t�@�I�u�W�F�N�g��Bind
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    //glBufferData(GL_ARRAY_BUFFER, )
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    //viewpoint
    glViewport(0, 0, window_width, window_height);

    //Projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (GLfloat)window_width/(GLfloat)window_height, 0.1, 100);
    glfwSetScrollCallback(window, setfov);

    //imgui�̏�����
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
    hovered = ImGui::IsWindowHovered();

    //�����Ƀ��[�v����������
    while (glfwWindowShouldClose(window) == GL_FALSE)
    {
        //�_�Q�̈ʒu�X�V
    }



	return 0;
}

static void setfov(GLFWwindow* window, double x, double y) {
    fov -= static_cast<GLfloat>(y);
}