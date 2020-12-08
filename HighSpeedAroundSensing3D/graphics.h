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
#include "params.h"

extern void drawGL(Logs* logs, bool* flg);


#endif // GRAPHICS_H

