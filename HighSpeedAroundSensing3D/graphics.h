#pragma once
#ifndef GRAPHICS_H
#define GRAPHICS_H


#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <imgui/imgui.h>
#include <imgui/imgui_impl_opengl3.h>
#include <imgui/imgui_impl_glfw.h>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtx/transform.hpp>
#include <queue>
#include <Windows.h>
#include "params.h"

extern void drawGL_one(double* pts, int* lsmshowid);
extern void finishGL();
extern void initGL();

const double safe_area = 1.5, danger_area = 1.5, zero_area = 0.01;
const int rstart = 104, rends = 432;

//表示したい情報
extern char rotdir; //DDMotorの制御モード
extern int rpm; //DDMotorの回転数


#endif // GRAPHICS_H

