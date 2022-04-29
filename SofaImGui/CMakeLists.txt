cmake_minimum_required(VERSION 3.12)
project(SofaImGui LANGUAGES CXX)

find_package(SofaGLFW REQUIRED)

option(SOFAIMGUI_FORCE_OPENGL2 "Force OpenGL2 mode (mandatory for MacOS). If false, OpenGL3 mode is activated (default)." OFF)
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(SOFAGLFWIMGUI_FORCE_OPENGL2 ON CACHE BOOL "" FORCE)
endif()

include(FetchContent)

FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG        fc198fe1dbae2e045b7fb31d53466a54f5368414
)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(nfd
        GIT_REPOSITORY https://github.com/btzy/nativefiledialog-extended
)
FetchContent_MakeAvailable(nfd)

FetchContent_Declare(ImPlot
        GIT_REPOSITORY https://github.com/epezent/implot
        GIT_TAG        b47c8bacdbc78bc521691f70666f13924bb522ab
)
FetchContent_MakeAvailable(ImPlot)

FetchContent_Declare(IconFontCppHeaders
        GIT_REPOSITORY https://github.com/juliettef/IconFontCppHeaders
        GIT_TAG        1856754666dff0db6c1af489a977f0ea2269f520
)
FetchContent_MakeAvailable(IconFontCppHeaders)

set(IMGUI_HEADER_FILES
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.h
    ${imgui_SOURCE_DIR}/imconfig.h
    ${imgui_SOURCE_DIR}/imgui.h
    ${imgui_SOURCE_DIR}/imgui_internal.h
    ${implot_SOURCE_DIR}/implot.h
    ${implot_SOURCE_DIR}/implot_internal.h
    ${iconfontcppheaders_SOURCE_DIR}/IconsFontAwesome5.h
    resources/fa-regular-400.h
    resources/fa-solid-900.h
    resources/Roboto-Medium.h
    resources/Style.h
)
set(IMGUI_SOURCE_FILES
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    resources/Style.cpp
)

set(SOFAIMGUI_SOURCE_DIR src/SofaImGui)

set(HEADER_FILES
    ${SOFAIMGUI_SOURCE_DIR}/config.h.in
    ${SOFAIMGUI_SOURCE_DIR}/CustomImGuiFileDialogConfig.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiDataWidget.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUI.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUIEngine.h
    ${SOFAIMGUI_SOURCE_DIR}/ObjectColor.h
)

set(SOURCE_FILES
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiDataWidget.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUI.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUIEngine.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ObjectColor.cpp
    ${SOFAIMGUI_SOURCE_DIR}/initSofaImGui.cpp
)


set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources ${imgui_SOURCE_DIR} ${implot_SOURCE_DIR} ${iconfontcppheaders_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${IMGUI_HEADER_FILES} ${IMGUI_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${IMGUI_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaGLFW)
target_link_libraries(${PROJECT_NAME} PRIVATE nfd glfw)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
