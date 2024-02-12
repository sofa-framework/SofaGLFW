cmake_minimum_required(VERSION 3.12)
project(SofaImGui LANGUAGES CXX)

# needed to find SofaGLFWConfig.cmake
# if building both of them out-of-tree
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/lib/cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

sofa_find_package(SofaGLFW REQUIRED)
sofa_find_package(Sofa.GL.Component.Rendering3D REQUIRED)
sofa_find_package(Sofa.Component.Playback REQUIRED)

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
        GIT_TAG        d4df2b6ad5420f5300c00f418bf28d86291fa675                # v1.0.0
)
FetchContent_MakeAvailable(nfd)

FetchContent_Declare(ImPlot
        GIT_REPOSITORY https://github.com/epezent/implot
        GIT_TAG        b47c8bacdbc78bc521691f70666f13924bb522ab
)
FetchContent_MakeAvailable(ImPlot)

FetchContent_Declare(IconFontCppHeaders
        GIT_REPOSITORY https://github.com/juliettef/IconFontCppHeaders
        GIT_TAG        8886c5657bac22b8fee34354871e3ade2a596433
)
FetchContent_MakeAvailable(IconFontCppHeaders)

FetchContent_Declare(simpleini
        GIT_REPOSITORY https://github.com/brofield/simpleini
        GIT_TAG        9b3ed7ec815997bc8c5b9edf140d6bde653e1458
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
    ${iconfontcppheaders_SOURCE_DIR}/IconsFontAwesome5.h
    ${simpleini_SOURCE_DIR}/SimpleIni.h
    resources/fa-regular-400.h
    resources/fa-solid-900.h
    resources/OpenSans-Regular.h
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
    ${SOFAIMGUI_SOURCE_DIR}/windows/BaseWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/IOWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/MyRobotWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/MoveWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/SceneGraphWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/StateWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/ProgramWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/ViewportWindow.h
    ${SOFAIMGUI_SOURCE_DIR}/windows/WorkspaceWindow.h

    ${SOFAIMGUI_SOURCE_DIR}/widgets/Buttons.h

    ${SOFAIMGUI_SOURCE_DIR}/menus/FileMenu.h
    ${SOFAIMGUI_SOURCE_DIR}/menus/ViewMenu.h

    ${SOFAIMGUI_SOURCE_DIR}/models/Action.h
    ${SOFAIMGUI_SOURCE_DIR}/models/Move.h
    ${SOFAIMGUI_SOURCE_DIR}/models/Program.h
    ${SOFAIMGUI_SOURCE_DIR}/models/Track.h

    ${SOFAIMGUI_SOURCE_DIR}/ImGuiDataWidget.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUI.h
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUIEngine.h
    ${SOFAIMGUI_SOURCE_DIR}/ObjectColor.h
    ${SOFAIMGUI_SOURCE_DIR}/config.h.in
    ${SOFAIMGUI_SOURCE_DIR}/init.h
)

set(SOURCE_FILES
    ${SOFAIMGUI_SOURCE_DIR}/windows/BaseWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/IOWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/MyRobotWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/MoveWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/SceneGraphWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/StateWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/ProgramWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/ViewportWindow.cpp
    ${SOFAIMGUI_SOURCE_DIR}/windows/WorkspaceWindow.cpp

    ${SOFAIMGUI_SOURCE_DIR}/widgets/Buttons.cpp

    ${SOFAIMGUI_SOURCE_DIR}/menus/FileMenu.cpp
    ${SOFAIMGUI_SOURCE_DIR}/menus/ViewMenu.cpp

    ${SOFAIMGUI_SOURCE_DIR}/models/Move.cpp
    ${SOFAIMGUI_SOURCE_DIR}/models/Program.cpp
    ${SOFAIMGUI_SOURCE_DIR}/models/Track.cpp

    ${SOFAIMGUI_SOURCE_DIR}/ImGuiDataWidget.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUI.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ImGuiGUIEngine.cpp
    ${SOFAIMGUI_SOURCE_DIR}/ObjectColor.cpp
    ${SOFAIMGUI_SOURCE_DIR}/initSofaImGui.cpp
)

find_package(ROS QUIET)
set(SOFAIMGUI_WITH_ROS ${ROS_FOUND})
message("SOFAIMGUI_WITH_ROS = " ${SOFAIMGUI_WITH_ROS})

set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources ${imgui_SOURCE_DIR} ${implot_SOURCE_DIR} ${iconfontcppheaders_SOURCE_DIR} ${simpleini_SOURCE_DIR})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${IMGUI_HEADER_FILES} ${IMGUI_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${IMGUI_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC SofaGLFW Sofa.GL.Component.Rendering3D Sofa.Component.Playback ${CMAKE_DL_LIBS})
set_target_properties(nfd PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} PRIVATE nfd glfw tinyxml2)

if(ROS_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${ROS_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${ROS_LIBRARIES})
endif(ROS_FOUND)

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
