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

extern void drawGL(LSM* lsm, Logs* logs, bool* flg);
extern void update_pts(vector<vector<double>> pts);
extern void drawGL_part(bool* flg);
extern void drawGL_one(vector<vector<double>> pts);
extern void finishGL();
extern void initGL();


#endif // GRAPHICS_H

