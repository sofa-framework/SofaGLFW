project(Bindings_SofaImGui)

set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_AdditionalGUI.h
)

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Module_SofaImGui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Binding_AdditionalGUI.cpp
)

if (NOT TARGET SofaPython3::Plugin)
    find_package(SofaPython3 REQUIRED COMPONENTS SofaImGui Bindings.Sofa)
endif()

SP3_add_python_module(
    TARGET       ${PROJECT_NAME}
    PACKAGE      SofaImGui
    MODULE       SofaImGui
    DESTINATION  .
    SOURCES      ${SOURCE_FILES}
    DEPENDS      SofaPython3::Plugin SofaPython3::Bindings.Sofa SofaImGui

)
message("-- SofaPython3 bindings for SofaImGui will be created.")
