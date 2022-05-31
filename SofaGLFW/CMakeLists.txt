cmake_minimum_required(VERSION 3.12)
project(SofaGLFW LANGUAGES CXX)

include(cmake/environment.cmake)

find_package(Sofa.Config REQUIRED)

sofa_find_package(Sofa.Simulation REQUIRED)
sofa_find_package(Sofa.GL REQUIRED)
sofa_find_package(Sofa.Component.Visual REQUIRED)
sofa_find_package(SofaGuiCommon QUIET)

include(FetchContent)

FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw
        GIT_TAG        3.3.4
)

FetchContent_GetProperties(glfw)
if(NOT glfw_POPULATED)
    FetchContent_Populate(glfw)

    set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
    set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
    set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
    set(GLFW_INSTALL ON CACHE INTERNAL "Generate installation target")
    set(BUILD_SHARED_LIBS ON CACHE INTERNAL "Build GLFW as a shared library")

    add_subdirectory(${glfw_SOURCE_DIR} ${glfw_BINARY_DIR})
endif()

set(SOFAGLFW_SOURCE_DIR src/SofaGLFW)

set(HEADER_FILES
    ${SOFAGLFW_SOURCE_DIR}/config.h.in
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.h
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.h
    ${SOFAGLFW_SOURCE_DIR}/BaseGUIEngine.h
    ${SOFAGLFW_SOURCE_DIR}/NullGUIEngine.h
)

set(SOURCE_FILES
    ${SOFAGLFW_SOURCE_DIR}/initSofaGLFW.cpp
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.cpp
	${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.cpp
    ${SOFAGLFW_SOURCE_DIR}/NullGUIEngine.cpp
)

if(SofaGuiCommon_FOUND)
    LIST(APPEND HEADER_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.h)
    LIST(APPEND SOURCE_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.GL Sofa.Simulation.Graph Sofa.Simulation.Core Sofa.Component.Visual)
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${glfw_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
)

if(SofaGuiCommon_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC SofaGuiCommon)
endif()

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)

include(cmake/packaging.cmake)
