project(SofaImGuiBindings)

set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_MoveWindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_MoveWindow_doc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_MyRobotWindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_MyRobotWindow_doc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_SimulationState.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_SimulationState_doc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_PlottingWindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_PlottingWindow_doc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_ProgramWindow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_ProgramWindow_doc.h
)

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_MoveWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_MyRobotWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_SimulationState.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_PlottingWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_ProgramWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Module_SofaImGui.cpp
)

if (NOT TARGET SofaPython3::Plugin)
    find_package(SofaPython3 REQUIRED COMPONENTS SofaGLFW SofaImGui Bindings.Sofa Sofa.Simulation.Common SoftRobots SoftRobots.Inverse)
endif()

set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources ${imgui_SOURCE_DIR} ${implot_SOURCE_DIR} ${iconfontcppheaders_SOURCE_DIR} ${simpleini_SOURCE_DIR})
include_directories(${IMGUI_SOURCE_DIR} ${ROS_INCLUDE_DIRS} ${QPOASES_INCLUDE_DIR})

SP3_add_python_module(
    TARGET       ${PROJECT_NAME}
    PACKAGE      SofaImGui
    MODULE       ImGui
    DESTINATION  Sofa
    HEADERS      ${HEADER_FILES}
    SOURCES      ${SOURCE_FILES}
    DEPENDS      SofaPython3::Plugin SofaPython3::Bindings.Sofa SofaGLFW SofaImGui Sofa.Simulation.Common SoftRobots SoftRobots.Inverse
)
message("-- SofaPython3 bindings for SofaImGui will be created.")
