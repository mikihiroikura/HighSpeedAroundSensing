#include <iostream>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtx/transform.hpp>

#pragma warning(disable:4996)

using namespace std;

// constants
const unsigned int window_width = 1280;
const unsigned int window_height = 720;
GLFWwindow* window;

//shader object
static GLuint vertShader, fragShader, gl2Program;

//vbo
GLuint vbo, cbo, vao;
float vertices[100][100][3];
float colors[100][100][3];

//imgui
float rotate_x = 0.0, rotate_y = 0.0;
float translate_x = 0.0, translate_y = 0.0, translate_z = -.0;
double mouse_x, mouse_y, mouse_x_old, mouse_y_old;
double horiz_angle = -M_PI, vert_angle = 0.0;
double mouse_speed = 0.01;
double dx = 0.0, dy = 0.0;
float init_fov = 60;
float fov = init_fov;
glm::vec3 position(0, 0, -1);
glm::vec3 up(0, -1, 0);
glm::vec3 direction(0, 0, 0);
bool hovered;
bool hide_red;
bool hide_green;
bool hide_blue;
float time = 0;
float pointsize = 2.5;
glm::mat4 mvp, View, Projection;
GLint matlocation;

static void setfov(GLFWwindow* window, double x, double y);
int readShaderSource(GLuint shader, const char* file);


int main() {
    //GLFWの初期化
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << "Can't initilize GLFW" << std::endl;
        return 1;
    }

    //Windowの作成
    window = glfwCreateWindow(window_width, window_height, "Cuda GL Interop (VBO)", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
        return 1;
    }

    //WindowをOpenGLの対象にする
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrentの後に行わないと失敗するらしい
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
        return 1;
    }

    glClearColor(0.0, 0.0, 0.0, 1.0);   //背景色の指定
    glPointSize(pointsize);
    glDisable(GL_DEPTH_TEST);

    //shaderオブジェクトの作成
    vertShader = glCreateShader(GL_VERTEX_SHADER);
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    if (readShaderSource(vertShader, "graphics.vert")) exit(1);
    if (readShaderSource(fragShader, "graphics.frag")) exit(1);

    //Shader compile
    glCompileShader(vertShader);
    glCompileShader(fragShader);

    //プログラムオブジェクト作成
    gl2Program = glCreateProgram();
    glAttachShader(gl2Program, vertShader);
    glDeleteShader(vertShader);
    glAttachShader(gl2Program, fragShader);
    glDeleteShader(fragShader);

    matlocation = glGetUniformLocation(gl2Program, "MVP");

    //glGenVertexArrays(1, &vao);
    //glBindVertexArray(vao);


    //positionの初期化
    for (size_t i = 0; i < 100; i++)
    {
        for (size_t j = 0; j < 100; j++)
        {
            vertices[i][j][0] = (float)i * 0.01-100*0.01/2;
            vertices[i][j][1] = 0;
            vertices[i][j][2] = (float)j * 0.01 - 100 * 0.01 / 2;
            colors[i][j][0] = 1;
            colors[i][j][1] = 0;
            colors[i][j][2] = 0.0;
        }
    }

    //頂点バッファオブジェクト
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    //色バッファオブジェクト
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(colors), colors, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (GLfloat)window_width / (GLfloat)window_height, 0.1, 100.0);
    glfwSetScrollCallback(window, setfov);

    //imguiの初期化
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
    hovered = ImGui::IsWindowHovered();

    //ここにループを書く
    while (glfwWindowShouldClose(window) == GL_FALSE)
    {
        //点群の位置更新
        for (size_t i = 0; i < 100; i++)
        {
            for (size_t j = 0; j < 100; j++)
            {
                vertices[i][j][0] = (float)i * 0.01 - 100 * 0.01 / 2;
                vertices[i][j][1] = sin((float)i + time) * cos((float)j + time);
                vertices[i][j][2] = (float)j * 0.01 - 100 * 0.01 / 2;
                //if (vertices[i][j][1]<0.5 && vertices[i][j][1] > -0.5)
                //{
                //    colors[i][j][1] = 0;
                //    colors[i][j][2] = 0.0;
                //    if (hide_red) colors[i][j][0] = 0;
                //    else colors[i][j][0] = 1;
                //}
                //else if (vertices[i][j][1] > 0.5)
                //{
                //    colors[i][j][0] = 0;
                //    colors[i][j][2] = 0.0;
                //    if (hide_green) colors[i][j][1] = 0.0;
                //    else colors[i][j][1] = 1.0;
                //}
                //else
                //{
                //    colors[i][j][0] = 0;
                //    colors[i][j][1] = 0;
                //    if (hide_blue) colors[i][j][2] = 0.0;
                //    else colors[i][j][2] = 1.0;
                //}
            }
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //glMatrixMode(GL_PROJECTION);
        //glLoadIdentity();
        //gluPerspective(fov, (GLfloat)window_width / (GLfloat)window_height, 0.1, 100.0);
        //
        //glMatrixMode(GL_MODELVIEW);
        //glLoadIdentity();//視野変換，モデリング変換行列の初期化
        //glPushMatrix();
        glfwGetCursorPos(window, &mouse_x, &mouse_y);
        dx = mouse_x - mouse_x_old;
        dy = mouse_y - mouse_y_old;

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !hovered)
        {
            horiz_angle += mouse_speed * dx;
            vert_angle += mouse_speed * dy;
        }
        mouse_x_old = mouse_x;
        mouse_y_old = mouse_y;
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
        position = glm::vec3(cos(vert_angle) * sin(horiz_angle), sin(vert_angle), cos(vert_angle) * cos(horiz_angle));
        //gluLookAt(position.x, position.y, position.z, direction.x, direction.y, direction.z, up.x, up.y, up.z);
        Projection = glm::perspective(glm::radians(fov), (GLfloat)window_width / (GLfloat)window_height, 0.1f, 100.0f);
        View = glm::lookAt(position, direction, up);
        mvp = Projection * View;
        glUseProgram(gl2Program);
        matlocation = glGetUniformLocation(gl2Program, "MV");
        glUniformMatrix4fv(matlocation, 1, GL_FALSE, &mvp[0][0]);

        glRotated(rotate_x, 1, 0, 0);
        glRotated(rotate_y, 0, 1, 0);
        glTranslatef(translate_x, translate_y, translate_z);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
        //glVertexPointer(3, GL_FLOAT, 0, 0);
        //glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, cbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colors), colors);
        glColorPointer(3, GL_FLOAT, 0, 0);
        //glColorPointer(3, GL_FLOAT, 0, colors);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        //glColor3f(0.0, 1.0, 0.0);
        glDrawArrays(GL_POINTS, 0, 100*100);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
        //glPopMatrix();      

        glfwPollEvents();

        //start imgui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_Once);
        ImGui::Begin("hello world");
        ImGui::Text("This is useful text");
        hovered = ImGui::IsWindowHovered();
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


        glfwSwapBuffers(window);

        time += 0.01;
    }

    glBindBuffer(1, vbo);
    glDeleteBuffers(1, &vbo);

    return 0;
}

static void setfov(GLFWwindow* window, double x, double y) {
    fov -= static_cast<GLfloat>(y);
}

/*
** シェーダーのソースプログラムをメモリに読み込む
*/
int readShaderSource(GLuint shader, const char* file)
{
    FILE* fp;
    const GLchar* source;
    GLsizei length;
    int ret;

    /* ファイルを開く */
    fp = fopen(file, "rb");
    if (fp == NULL) {
        perror(file);
        return -1;
    }

    /* ファイルの末尾に移動し現在位置 (つまりファイルサイズ) を得る */
    fseek(fp, 0L, SEEK_END);
    length = ftell(fp);

    /* ファイルサイズのメモリを確保 */
    source = (GLchar*)malloc(length);
    if (source == NULL) {
        fprintf(stderr, "Could not allocate read buffer.\n");
        return -1;
    }

    /* ファイルを先頭から読み込む */
    fseek(fp, 0L, SEEK_SET);
    ret = fread((void*)source, 1, length, fp) != (size_t)length;
    fclose(fp);

    /* シェーダのソースプログラムのシェーダオブジェクトへの読み込み */
    if (ret)
        fprintf(stderr, "Could not read file: %s.\n", file);
    else
        glShaderSource(shader, 1, &source, &length);

    /* 確保したメモリの開放 */
    free((void*)source);

    return ret;
}