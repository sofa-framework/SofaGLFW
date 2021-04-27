cmake_minimum_required(VERSION 3.12)


option(SOFA_ENABLE_GLFW "Build the SofaGLFW app+lib for SOFA" OFF)
if(SOFA_ENABLE_GLFW)
	add_subdirectory(lib)
	add_subdirectory(exe)
endif()