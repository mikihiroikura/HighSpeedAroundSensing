#include "graphics.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;
GLFWwindow* window;

//shader object
static GLuint vertShader, fragShader, gl2Program;

//vbo
GLuint vbo, cbo, vao;

//imgui
float pointsize = 2.5;
bool hide_red, hide_green, hide_blue;
float rotate_x = 0.0, rotate_y = 0.0;
float translate_x = 0.0, translate_y = 0.0, translate_z = -.0;
double mouse_x, mouse_y, mouse_x_old, mouse_y_old;
double horiz_angle = -M_PI, vert_angle = 0.0;
double mouse_speed = 0.01;
double dx = 0.0, dy = 0.0;
float init_fov = 60, fov = init_fov;
float hovered;
glm::vec3 position(0, 0, -1), up(0, -1, 0), direction(0, 0, 0);
glm::mat4 mvp, Model, View, Projection;
GLint matlocation;

//�v���g�^�C�v�錾
static void setfov(GLFWwindow* window, double x, double y);
int readShaderSource(GLuint shader, const char* file);


//OpenGL�̏�����
void drawGL(bool* flg) {
    //GLFW�̏�����
    if (glfwInit() == GL_FALSE)
    {
        cerr << "Can't initilize GLFW" << endl;
    }

    //Window�̍쐬
    window = glfwCreateWindow(window_width, window_height, "Cuda GL Interop (VBO)", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
    }

    //Window��OpenGL�̑Ώۂɂ���
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrent�̌�ɍs��Ȃ��Ǝ��s����炵��
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
    }

    glClearColor(0.0, 0.0, 0.0, 1.0);   //�w�i�F�̎w��
    glPointSize(pointsize);
    glDisable(GL_DEPTH_TEST);

    //shader�I�u�W�F�N�g�̍쐬
    vertShader = glCreateShader(GL_VERTEX_SHADER);
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    if (readShaderSource(vertShader, "graphics.vert")) exit(1);
    if (readShaderSource(fragShader, "graphics.frag")) exit(1);

    //Shader compile
    glCompileShader(vertShader);
    glCompileShader(fragShader);

    //�v���O�����I�u�W�F�N�g�쐬
    gl2Program = glCreateProgram();
    glAttachShader(gl2Program, vertShader);
    glDeleteShader(vertShader);
    glAttachShader(gl2Program, fragShader);
    glDeleteShader(fragShader);

    //�V�F�[�_�v���O�����̃����N
    glLinkProgram(gl2Program);

    matlocation = glGetUniformLocation(gl2Program, "MVP");//�V�F�[�_�v���O�������"MVP" uniform�̈ʒu�̌���

    //VAO�̃o�C���h
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    //���_�o�b�t�@�I�u�W�F�N�g
    glGenBuffers(1, &vbo);//vbp�쐬
    glBindBuffer(GL_ARRAY_BUFFER, vbo);//vbo�̃o�C���h�C���ꂩ��̏����̑Ώ�
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);//vbo�̃f�[�^�̈�̊m��
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);//vertex shader���̈����̎w��index�ɍ����悤�ɕύX����
    glEnableVertexAttribArray(0);//index�̒l��attribute�ϐ��̗L����
    //glEnableVertexArrayAttrib(vao, 0); //��̊֐��̑���ɂ���ł�����

    //�F�o�b�t�@�I�u�W�F�N�g
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(colors), colors, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    //glEnableVertexArrayAttrib(vao, 1);

    glBindBuffer(GL_ARRAY_BUFFER, 0); //EnableVertexAttribArray�̌�ɍs��
    glBindVertexArray(0);//VAO�ɏ��VBO�̏������܂Ƃ߂�C���[�v�ň�x������Ăׂ�vertexattrib,enablevertexattrib�͎��s�����

    //�X�N���[������Callback����֐��̎w��
    glfwSetScrollCallback(window, setfov);

    //imgui�̏�����
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    //�����Ƀ��[�v����������
    while (glfwWindowShouldClose(window) == GL_FALSE && *flg)
    {
        //�_�Q�̈ʒu�X�V

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //�J�[�\���ʒu����ړ��ω��ʂ��v�Z
        glfwGetCursorPos(window, &mouse_x, &mouse_y);
        dx = mouse_x - mouse_x_old;
        dy = mouse_y - mouse_y_old;

        //���N���b�N���Ă���΂���IMGUI���Window�ɂ��Ȃ���΁C�ړ��ω��ʂ���Ɋp�x�X�V
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !hovered)
        {
            horiz_angle += mouse_speed * dx;
            vert_angle += mouse_speed * dy;
        }
        mouse_x_old = mouse_x;
        mouse_y_old = mouse_y;

        //�X�y�[�X�L�[�������Ă���΁C�p�����[�^���Z�b�g
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        {
            horiz_angle = -M_PI;
            vert_angle = 0.0;
            rotate_x = 0.0, rotate_y = 0.0;
            translate_x = 0.0, translate_y = 0.0, translate_z = -.0;
            fov = init_fov;
            hide_red = false;
            hide_green = false;
            hide_blue = false;
        }

        //Model view�s��̌v�Z
        position = glm::vec3(cos(vert_angle) * sin(horiz_angle), sin(vert_angle), cos(vert_angle) * cos(horiz_angle));
        Projection = glm::perspective(glm::radians(fov), (GLfloat)window_width / (GLfloat)window_height, 0.1f, 100.0f);
        View = glm::lookAt(position, direction, up);
        Model = glm::translate(glm::mat4(1.0), glm::vec3(translate_x, translate_y, translate_z))
            * glm::rotate(glm::radians(rotate_x), glm::vec3(1, 0, 0))
            * glm::rotate(glm::radians(rotate_y), glm::vec3(0, 1, 0));
        mvp = Projection * View * Model;

        //�V�F�[�_�v���O�����̊J�n
        glUseProgram(gl2Program);
        glUniformMatrix4fv(matlocation, 1, GL_FALSE, &mvp[0][0]); //�V�F�[�_�v���O�����̊J�n�̌�ɃV�F�[�_�v���O��������MVP�s����X�V
        
        //�_�Q�̈ʒu�ƐF���X�V
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices); //VBO���̓_�Q�̈ʒu�̍X�V
        glBindBuffer(GL_ARRAY_BUFFER, cbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colors), colors);//VBO���̐F���X�V
        glBindVertexArray(vao);//VBO�ł̓_�Q�ʒu�ƐF�X�V���܂Ƃ߂�VAO���o�C���h���Ď��s
        glDrawArrays(GL_POINTS, 0, 100 * 100);//���ۂ̕`��
        glBindVertexArray(0);//VBO�̃A���o�C���h

        glfwPollEvents(); //�}�E�X�C�x���g�����o���L�^

        //start imgui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_Once);
        ImGui::Begin("hello world");
        ImGui::Text("This is useful text");
        hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem); //IMGUI���Window�ł̃J�[�\���������̃t���O�𗧂Ă�
        ImGui::Checkbox("Hide Red", &hide_red);
        ImGui::Checkbox("Hide Green", &hide_green);
        ImGui::Checkbox("Hide Blue", &hide_blue);
        ImGui::DragFloat("rotate x", &rotate_x);
        ImGui::DragFloat("rotate y", &rotate_y);
        ImGui::DragFloat("trans x", &translate_x);
        ImGui::DragFloat("trans y", &translate_y);
        ImGui::DragFloat("trans z", &translate_z);
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwSwapBuffers(window);//�_�u���o�b�t�@�̓���ւ��C������s�����Ƃŉ�ʂ��ꕔ�X�V�������̂ňꕔ�X�V����Ă��Ȃ��Ȃǂ̂��������Ȃ�����
    }

    //vao,vbo�̏���
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &cbo);
}

static void setfov(GLFWwindow* window, double x, double y) {
    fov -= static_cast<GLfloat>(y);
}

/*
** �V�F�[�_�[�̃\�[�X�v���O�������������ɓǂݍ���
*/
int readShaderSource(GLuint shader, const char* file)
{
    FILE* fp;
    const GLchar* source;
    GLsizei length;
    int ret;

    /* �t�@�C�����J�� */
    fp = fopen(file, "rb");
    if (fp == NULL) {
        perror(file);
        return -1;
    }

    /* �t�@�C���̖����Ɉړ������݈ʒu (�܂�t�@�C���T�C�Y) �𓾂� */
    fseek(fp, 0L, SEEK_END);
    length = ftell(fp);

    /* �t�@�C���T�C�Y�̃��������m�� */
    source = (GLchar*)malloc(length);
    if (source == NULL) {
        fprintf(stderr, "Could not allocate read buffer.\n");
        return -1;
    }

    /* �t�@�C����擪����ǂݍ��� */
    fseek(fp, 0L, SEEK_SET);
    ret = fread((void*)source, 1, length, fp) != (size_t)length;
    fclose(fp);

    /* �V�F�[�_�̃\�[�X�v���O�����̃V�F�[�_�I�u�W�F�N�g�ւ̓ǂݍ��� */
    if (ret)
        fprintf(stderr, "Could not read file: %s.\n", file);
    else
        glShaderSource(shader, 1, &source, &length);

    /* �m�ۂ����������̊J�� */
    free((void*)source);

    return ret;
}