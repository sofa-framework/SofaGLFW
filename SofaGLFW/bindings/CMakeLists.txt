project(Bindings_SofaGLFW)

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Module_SofaGLFW.cpp
)

if (NOT TARGET SofaPython3::Plugin)
    find_package(SofaPython3 REQUIRED COMPONENTS SofaGLFW Bindings.Sofa)
endif()

SP3_add_python_module(
    TARGET       ${PROJECT_NAME}
    PACKAGE      SofaGLFW
    MODULE       SofaGLFW
    DESTINATION  /
    SOURCES      ${SOURCE_FILES}
    DEPENDS      SofaPython3::Plugin SofaPython3::Bindings.Sofa SofaGLFW

)
message("-- SofaPython3 bindings for SofaGLFW will be created.")
