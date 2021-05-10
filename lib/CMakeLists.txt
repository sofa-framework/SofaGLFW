cmake_minimum_required(VERSION 3.12)
project(SofaGLFW LANGUAGES CXX)

find_package(SofaSimulation REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(SofaBaseVisual REQUIRED)
find_package(SofaGuiCommon REQUIRED)

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

set(SOFAGLFW_SOURCE_DIR src/SofaGLFW)

set(HEADER_FILES
	${SOFAGLFW_SOURCE_DIR}/config.h.in
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.h
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.h
)

set(SOURCE_FILES
    ${SOFAGLFW_SOURCE_DIR}/initSofaGLFW.cpp
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.cpp
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.GL SofaSimulationGraph SofaBaseVisual SofaGuiCommon)
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
