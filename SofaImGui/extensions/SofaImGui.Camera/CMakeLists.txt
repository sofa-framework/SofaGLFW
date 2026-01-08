# CMakeLists.txt for SofaImGui.Camera plugin
cmake_minimum_required(VERSION 3.12)
project(SofaImGui.Camera VERSION 1.0 LANGUAGES CXX)

find_package(Sofa.Config REQUIRED)
sofa_find_package(SofaImGui REQUIRED)

sofa_fetch_dependency(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG        11b3a7c8ca23201294464c7f368614a9106af2a1 # v1.91.8-docking
        DONT_BUILD
)

set(HEADER_FILES
    src/SofaImGui.Camera/config.h.in
    src/SofaImGui.Camera/init.h

    src/SofaImGui.Camera/CameraGUI.h
)

set(SOURCE_FILES
    src/SofaImGui.Camera/init.cpp

    src/SofaImGui.Camera/CameraGUI.cpp
)

set(IMGUI_SOURCE_DIR ${imgui_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${IMGUI_SOURCE_DIR}>")
target_link_libraries(${PROJECT_NAME} PUBLIC SofaImGui)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
