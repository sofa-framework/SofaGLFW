cmake_minimum_required(VERSION 3.12)
project(SofaGLFW LANGUAGES CXX)

find_package(Sofa.Config REQUIRED)

sofa_find_package(SofaSimulation REQUIRED)
sofa_find_package(Sofa.GL REQUIRED)
sofa_find_package(SofaBaseVisual REQUIRED)
sofa_find_package(SofaGuiCommon QUIET)

set(SOFA_BUILD_SOFAGLFWIMGUI OFF CACHE BOOL "Build a GUI based on Dear ImGui")

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
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWImgui.h
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.h
	${SOFAGLFW_SOURCE_DIR}/config.h.in
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.h
)

set(SOURCE_FILES
    ${SOFAGLFW_SOURCE_DIR}/initSofaGLFW.cpp
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.cpp
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.cpp
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWImgui.cpp
)

if (SOFA_BUILD_SOFAGLFWIMGUI)
    add_subdirectory(SofaGLFWImgui)
    list(APPEND HEADER_FILES ${SOFAGLFW_SOURCE_DIR}/CustomImGuiFileDialogConfig.h)
    add_definitions(-DCUSTOM_IMGUIFILEDIALOG_CONFIG=<SofaGLFW/CustomImGuiFileDialogConfig.h>)
else()
    set(SOFAGLFW_HAS_IMGUI_VALUE 0)
endif()


if(SofaGuiCommon_FOUND)
    LIST(APPEND HEADER_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.h)
    LIST(APPEND SOURCE_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${IMGUI_HEADER_FILES} ${IMGUI_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${IMGUI_SOURCE_DIR} "${glfw_SOURCE_DIR}/include")
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.GL SofaSimulationGraph SofaBaseVisual)
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

if (SOFAGLFW_HAS_IMGUI_VALUE)
    target_link_libraries(${PROJECT_NAME} PRIVATE nfd)
endif()

if(SofaGuiCommon_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC SofaGuiCommon)
endif()

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
