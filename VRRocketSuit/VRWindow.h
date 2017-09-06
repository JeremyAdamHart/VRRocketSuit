#pragma once

#ifdef _WIN32
#define APIENTRY __stdcall
#endif

// GLAD
#include <glad/glad.h>

// confirm that GLAD didn't include windows.h
#ifdef _WINDOWS_
#error windows.h was included!
#endif

// GLFW
#include <GLFW/glfw3.h>

#include <openvr.h>

#include <string>
#include <glm/glm.hpp>

#include "VRCamera.h"


class WindowManager {
protected:
	GLFWwindow *window;
	vr::IVRSystem *vrDisplay;
	void initGL();
	
	int window_width, window_height;


public:
	WindowManager();
	WindowManager(int width, int height, std::string name, 
		glm::vec4 color = glm::vec4(1.f));

	void mainLoop();
};

vr::IVRSystem *initVR();
void initGlad();
GLFWwindow *createWindow(int width, int height, std::string name);