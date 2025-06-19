# CMakeLists.txt for SofaImGui.Camera plugin
cmake_minimum_required(VERSION 3.12)
project(SofaImGui.Camera VERSION 1.0 LANGUAGES CXX)

find_package(Sofa.Config REQUIRED)
sofa_find_package(SofaImGui REQUIRED)

set(HEADER_FILES
    src/SofaImGui.Camera/config.h.in
    src/SofaImGui.Camera/init.h

    src/SofaImGui.Camera/CameraGUI.h
)

set(SOURCE_FILES
    src/SofaImGui.Camera/init.cpp

    src/SofaImGui.Camera/CameraGUI.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaImGui)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
