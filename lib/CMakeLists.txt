cmake_minimum_required(VERSION 3.12)
project(SofaGLFW)

find_package(SofaSimulation REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(SofaBaseVisual REQUIRED)
find_package(SofaGuiCommon REQUIRED)
find_package(glfw3 QUIET)
option(SOFA_GLFW_BUILD "Build GLFW library for SofaGLFW" OFF)

if(NOT glfw3_FOUND AND SOFA_GLFW_BUILD)
	include(FetchContent)

	FetchContent_Declare(glfw
	        GIT_REPOSITORY https://github.com/glfw/glfw
	        GIT_TAG        3.3.4
	)

	FetchContent_GetProperties(glfw)
	if(NOT glfw_POPULATED)
	    FetchContent_Populate(glfw)

	    set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
	    set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
	    set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
	    set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")

	    add_subdirectory(${glfw_SOURCE_DIR} ${glfw_BINARY_DIR})
	endif()
else()
	message(SEND_ERROR "SofaGLFW needs GLFW but it was not detected and the option to build it is OFF")
endif()

set(SOFAGLFW_SOURCE_DIR src/SofaGLFW)

set(HEADER_FILES
	${SOFAGLFW_SOURCE_DIR}/config.h
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.h
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.h
)

set(SOURCE_FILES
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.cpp
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} Sofa.GL SofaSimulationGraph SofaBaseVisual glfw)
target_link_libraries(${PROJECT_NAME} SofaGuiCommon)
target_compile_definitions(${PROJECT_NAME} PUBLIC BUILD_SOFA_GLFW)
target_include_directories(${PROJECT_NAME} PUBLIC src/)
