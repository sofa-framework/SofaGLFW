cmake_minimum_required(VERSION 3.12)
project(SofaGLFWImgui)

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

set(IMGUI_HEADER_FILES
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.h
    ${imgui_SOURCE_DIR}/imconfig.h
    ${imgui_SOURCE_DIR}/imgui.h
    ${imgui_SOURCE_DIR}/imgui_internal.h
    ${implot_SOURCE_DIR}/implot.h
    ${implot_SOURCE_DIR}/implot_internal.h
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
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    PARENT_SCOPE
)
set(IMGUI_SOURCE_DIR ${imgui_SOURCE_DIR} ${implot_SOURCE_DIR} PARENT_SCOPE)
set(SOFAGLFW_HAS_IMGUI_VALUE 1 PARENT_SCOPE)
