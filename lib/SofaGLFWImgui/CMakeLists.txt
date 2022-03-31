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
    ${CMAKE_CURRENT_SOURCE_DIR}/fa-regular-400.h
    ${CMAKE_CURRENT_SOURCE_DIR}/fa-solid-900.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Roboto-Medium.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Style.h
    PARENT_SCOPE
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
    ${CMAKE_CURRENT_SOURCE_DIR}/Style.cpp
    PARENT_SCOPE
)
set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} ${imgui_SOURCE_DIR} ${implot_SOURCE_DIR} ${iconfontcppheaders_SOURCE_DIR} PARENT_SCOPE)
set(SOFAGLFW_HAS_IMGUI_VALUE 1 PARENT_SCOPE)
