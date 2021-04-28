cmake_minimum_required(VERSION 3.12)
project(SofaGLFW)

find_package(SofaSimulation REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(glfw3 QUIET)
option(SOFA_GLFW_BUILD "Build GLFW library for SofaGLFW" OFF)

if(NOT glfw3_FOUND AND SOFA_GLFW_BUILD)
	set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
	set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
	set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

	add_subdirectory(extlibs/glfw-3.3.4)
else()
	message(SEND_ERROR "SofaGLFW needs GLFW but it was not detected and the option to build it is OFF")
endif()

set(HEADER_FILES
    SofaGLFW.h)

set(SOURCE_FILES
    SofaGLFW.cpp)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} Sofa.GL SofaSimulationGraph glfw)
target_compile_definitions(${PROJECT_NAME} PUBLIC SOFA_BUILD_SOFA_GLFW)
target_include_directories(${PROJECT_NAME} PUBLIC .)
