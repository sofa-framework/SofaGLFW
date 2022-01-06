cmake_minimum_required(VERSION 3.12)
project(SofaGLFWImgui)

include(FetchContent)

FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG        fc198fe1dbae2e045b7fb31d53466a54f5368414
)

FetchContent_MakeAvailable(imgui)

FetchContent_Declare(ImGuiFileDialog
        GIT_REPOSITORY https://github.com/aiekick/ImGuiFileDialog
        GIT_TAG        9be0b2cfc57a415b7d46b33f02cd04d512bb9c69
)

FetchContent_MakeAvailable(ImGuiFileDialog)

set(IMGUI_HEADER_FILES
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.h
    ${imgui_SOURCE_DIR}/imconfig.h
    ${imgui_SOURCE_DIR}/imgui.h
    ${imgui_SOURCE_DIR}/imgui_internal.h
    ${imguifiledialog_SOURCE_DIR}/ImGuiFileDialog.h
    ${imguifiledialog_SOURCE_DIR}/ImGuiFileDialogConfig.h
    PARENT_SCOPE
)
set(IMGUI_SOURCE_FILES
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imguifiledialog_SOURCE_DIR}/ImGuiFileDialog.cpp
    PARENT_SCOPE
)
set(IMGUI_SOURCE_DIR ${imgui_SOURCE_DIR} ${imguifiledialog_SOURCE_DIR} PARENT_SCOPE)
set(SOFAGLFW_HAS_IMGUI_VALUE 1 PARENT_SCOPE)
