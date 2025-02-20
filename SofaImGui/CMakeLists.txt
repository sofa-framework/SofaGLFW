cmake_minimum_required(VERSION 3.12)
project(SofaImGui LANGUAGES CXX)

# needed to find SofaGLFWConfig.cmake
# if building both of them out-of-tree
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/lib/cmake)

sofa_find_package(SofaGLFW REQUIRED)
sofa_find_package(Sofa.GL.Component.Rendering3D REQUIRED)

option(SOFAIMGUI_FORCE_OPENGL2 "Force OpenGL2 mode (mandatory for MacOS). If false, OpenGL3 mode is activated (default)." OFF)
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(SOFAIMGUI_FORCE_OPENGL2 ON CACHE BOOL "" FORCE)
endif()

include(FetchContent)

FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG        11b3a7c8ca23201294464c7f368614a9106af2a1 # v1.91.8-docking
)
FetchContent_MakeAvailable(imgui)

find_package(nfd CONFIG QUIET)

if(NOT TARGET nfd::nfd)
    message("nativefiledialog-extended not found, fetching source code...")
    FetchContent_Declare(nfd
            GIT_REPOSITORY https://github.com/btzy/nativefiledialog-extended
            GIT_TAG        d4df2b6ad5420f5300c00f418bf28d86291fa675                # v1.0.0
    )
    FetchContent_MakeAvailable(nfd)
    set_property(TARGET nfd  PROPERTY POSITION_INDEPENDENT_CODE ON)
    set_target_properties(nfd PROPERTIES LINKER_LANGUAGE CXX)
    add_library(nfd::nfd ALIAS nfd) # introduced in nfd >= v1.2.1
endif()


FetchContent_Declare(ImPlot
        GIT_REPOSITORY https://github.com/epezent/implot
        GIT_TAG        18c72431f8265e2b0b5378a3a73d8a883b2175ff # v0.16
)
FetchContent_MakeAvailable(ImPlot)

FetchContent_Declare(IconFontCppHeaders
        GIT_REPOSITORY https://github.com/juliettef/IconFontCppHeaders
        GIT_TAG        bf610efb2e5d4361df799d0dfb5ae3977d7bba2e
)
FetchContent_MakeAvailable(IconFontCppHeaders)

FetchContent_Declare(simpleini
        GIT_REPOSITORY https://github.com/brofield/simpleini
        GIT_TAG        09c21bda1dc1b578fa55f4a005d79b0afd481296 # v4.22
)
FetchContent_MakeAvailable(simpleini)

set(IMGUI_HEADER_FILES
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.h
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.h
    ${imgui_SOURCE_DIR}/imconfig.h
    ${imgui_SOURCE_DIR}/imgui.h
    ${imgui_SOURCE_DIR}/imgui_internal.h
    ${implot_SOURCE_DIR}/implot.h
    ${implot_SOURCE_DIR}/implot_internal.h
    ${iconfontcppheaders_SOURCE_DIR}/IconsFontAwesome4.h
    ${iconfontcppheaders_SOURCE_DIR}/IconsFontAwesome6.h
    ${simpleini_SOURCE_DIR}/SimpleIni.h
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
    ${SOFAIMGUI_SOURCE_DIR}/init.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiDataWidget.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUI.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUIEngine.h
    ${SOFAIMGUI_SOURCE_DIR}/ObjectColor.h
    ${SOFAIMGUI_SOURCE_DIR}/UIStrings.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/Performances.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/Log.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/Profiler.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/SceneGraph.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/DisplayFlags.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/Plugins.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/Components.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/Settings.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/ViewPort.h
    ${SOFAIMGUI_SOURCE_DIR}/AppIniFile.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/WindowState.h
)

set(SOURCE_FILES
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiDataWidget.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUI.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUIEngine.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ObjectColor.cpp
    ${SOFAIMGUI_SOURCE_DIR}/initSofaImGui.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/Performances.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/Log.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/Profiler.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/SceneGraph.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/DisplayFlags.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/Plugins.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/Components.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/Settings.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/ViewPort.cpp
    ${SOFAIMGUI_SOURCE_DIR}/AppIniFile.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/WindowState.cpp

)



set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources ${imgui_SOURCE_DIR} ${implot_SOURCE_DIR} ${iconfontcppheaders_SOURCE_DIR} ${simpleini_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${IMGUI_HEADER_FILES} ${IMGUI_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${IMGUI_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC SofaGLFW Sofa.GL.Component.Rendering3D ${CMAKE_DL_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE nfd::nfd)

find_package(SofaPython3 QUIET)
if(SofaPython3_FOUND)
    add_subdirectory(bindings)
endif()

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
