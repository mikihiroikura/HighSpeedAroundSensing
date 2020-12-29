#include "graphics.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vector>

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

//出力点群に関するパラメータ
queue<cv::Mat> vertices;
queue<vector<float>> colors;
vector<float> color;
vector<cv::Mat> vertices_vec;
const int maxvertsize = 100;
const int maxpointsize = 432-104;
double dist;
double verts[maxvertsize][maxpointsize][3] = {0};
float colos[maxvertsize][maxpointsize][3] = {0};
int savecnt = 0;

//時間計測用変数
LARGE_INTEGER glstart, glend, glfreq;
double gltime = 0;

cv::Mutex mutex_gl;

//プロトタイプ宣言
static void setfov(GLFWwindow* window, double x, double y);
static int readShaderSource(GLuint shader, const char* file);

#pragma warning(disable:4996)

static void setfov(GLFWwindow* window, double x, double y) {
    fov -= static_cast<GLfloat>(y);
}

/*
** シェーダーのソースプログラムをメモリに読み込む
*/
static int readShaderSource(GLuint shader, const char* file)
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

//OpenGLの初期化のみ
void initGL() {
    //時間計測用変数の初期化
    QueryPerformanceFrequency(&glfreq);

    //GLFWの初期化
    if (glfwInit() == GL_FALSE)
    {
        cerr << "Can't initilize GLFW" << endl;
    }

    //Windowの作成
    window = glfwCreateWindow(window_width, window_height, "Around Sensing Point Cloud", NULL, NULL);
    if (window == nullptr)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        glfwTerminate();
    }

    //WindowをOpenGLの対象にする
    glfwMakeContextCurrent(window);
    //MakeCOntextcurrentの後に行わないと失敗するらしい
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Can't initilize GLEW" << std::endl;
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

    //シェーダプログラムのリンク
    glLinkProgram(gl2Program);

    matlocation = glGetUniformLocation(gl2Program, "MVP");//シェーダプログラム上の"MVP" uniformの位置の検索

    //VAOのバインド
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    //頂点バッファオブジェクト
    glGenBuffers(1, &vbo);//vbp作成
    glBindBuffer(GL_ARRAY_BUFFER, vbo);//vboのバインド，これからの処理の対象
    std::cout << sizeof(verts) << endl;
    //glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_example), nullptr, GL_DYNAMIC_DRAW);//vboのデータ領域の確保
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, 0);//vertex shader内の引数の指定indexに合うように変更する
    glEnableVertexAttribArray(0);//indexの値のattribute変数の有効化
    //glEnableVertexArrayAttrib(vao, 0); //上の関数の代わりにこれでもいい

    //色バッファオブジェクト
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(colors_example), nullptr, GL_DYNAMIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, sizeof(colos), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    //glEnableVertexArrayAttrib(vao, 1);

    glBindBuffer(GL_ARRAY_BUFFER, 0); //EnableVertexAttribArrayの後に行う
    glBindVertexArray(0);//VAOに上のVBOの処理をまとめる，ループで一度これを呼べばvertexattrib,enablevertexattribは実行される

    //スクロール時にCallbackする関数の指定
    glfwSetScrollCallback(window, setfov);

    //imguiの初期化
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
}

//OpenGLの描画
void drawGL_one(double *pts, int *lsmshowid) {
    //時間計測開始
    QueryPerformanceCounter(&glstart);

    //点群の位置更新
    for (size_t i = 0; i < maxpointsize; i++)
    {
        //vertices.emplace(last_pts[i]*0.001);
        verts[savecnt][i][0] = *(pts + *lsmshowid * i * 3 + 3 * i + 0) * 0.001;
        verts[savecnt][i][1] = *(pts + *lsmshowid * i * 3 + 3 * i + 1) * 0.001;
        verts[savecnt][i][2] = *(pts + *lsmshowid * i * 3 + 3 * i + 2) * 0.001;
        dist = hypot(verts[savecnt][i][0], verts[savecnt][i][1]);
        if (dist > safe_area) {
            colos[savecnt][i][0] = 0.0;
            colos[savecnt][i][1] = 0.0;
            if (hide_blue) colos[savecnt][i][2] = 0.0;
            else colos[savecnt][i][2] = 1.0;
        }
        else if (dist <= safe_area && dist > danger_area) {
            colos[savecnt][i][0] = 0.0;
            colos[savecnt][i][2] = 0.0;
            if (hide_green) colos[savecnt][i][1] = 0.0;
            else colos[savecnt][i][1] = 1.0;
        }
        else if (dist <= danger_area && dist > zero_area) {
            colos[savecnt][i][1] = 0.0;
            colos[savecnt][i][2] = 0.0;
            if (hide_red) colos[savecnt][i][0] = 0.0;
            else colos[savecnt][i][0] = 1.0;
        }
        else if (dist <= zero_area)
        {
            colos[savecnt][i][0] = 0.0;
            colos[savecnt][i][1] = 0.0;
            colos[savecnt][i][2] = 0.0;
        }
        else
        {
            colos[savecnt][i][0] = 1.0;
            colos[savecnt][i][1] = 1.0;
            colos[savecnt][i][2] = 1.0;
        }
    }
    savecnt = (savecnt + 1) % maxvertsize;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //カーソル位置から移動変化量を計算
    glfwGetCursorPos(window, &mouse_x, &mouse_y);
    dx = mouse_x - mouse_x_old;
    dy = mouse_y - mouse_y_old;

    //左クリックしていればかつIMGUI上のWindowにいなければ，移動変化量を基に角度更新
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !hovered)
    {
        horiz_angle += mouse_speed * dx;
        vert_angle += mouse_speed * dy;
    }
    mouse_x_old = mouse_x;
    mouse_y_old = mouse_y;

    //スペースキーを押していれば，パラメータリセット
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

    //Model view行列の計算
    position = glm::vec3(cos(vert_angle) * sin(horiz_angle), sin(vert_angle), cos(vert_angle) * cos(horiz_angle));
    Projection = glm::perspective(glm::radians(fov), (GLfloat)window_width / (GLfloat)window_height, 0.1f, 100.0f);
    View = glm::lookAt(position, direction, up);
    Model = glm::translate(glm::mat4(1.0), glm::vec3(translate_x, translate_y, translate_z))
        * glm::rotate(glm::radians(rotate_x), glm::vec3(1, 0, 0))
        * glm::rotate(glm::radians(rotate_y), glm::vec3(0, 1, 0));
    mvp = Projection * View * Model;

    //シェーダプログラムの開始
    glUseProgram(gl2Program);
    glUniformMatrix4fv(matlocation, 1, GL_FALSE, &mvp[0][0]); //シェーダプログラムの開始の後にシェーダプログラム内のMVP行列を更新

    //点群の位置と色を更新
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    //glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices_example), vertices_example); //VBO内の点群の位置の更新
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colos), colos);//VBO内の色を更新
    glBindVertexArray(vao);//VBOでの点群位置と色更新をまとめたVAOをバインドして実行
    glDrawArrays(GL_POINTS, 0, maxvertsize * maxpointsize);//実際の描画
    glBindVertexArray(0);//VBOのアンバインド

    glfwPollEvents(); //マウスイベントを取り出し記録

    //ここまでの時間計測
    //QueryPerformanceCounter(&glend);
    //gltime = (double)(glend.QuadPart - glstart.QuadPart) / glfreq.QuadPart;
    //std::cout << "openGL time: " << gltime << endl;

    //start imgui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize(ImVec2(320, 300), ImGuiCond_Once);
    ImGui::Begin("Logs and Parameters");
    //ImGui::Text("Processing time %.3f ms", gltime * 1000);
    ImGui::Text("DDMotor mode  :  %c", mode);
    ImGui::Text("Rotation speed:  %d", rpm);
    hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem); //IMGUI上のWindowでのカーソル処理時のフラグを立てる
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


    glfwSwapBuffers(window);//ダブルバッファの入れ替え，これを行うことで画面が一部更新したもので一部更新されていないなどのがたつきをなくせる
    /*QueryPerformanceCounter(&glend);
    gltime = (double)(glend.QuadPart - glstart.QuadPart) / glfreq.QuadPart;
    std::cout << "openGL time: " << gltime << endl;*/
}

void finishGL() {
    //vao,vboの消去
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &cbo);
}