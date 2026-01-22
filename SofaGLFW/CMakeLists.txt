cmake_minimum_required(VERSION 3.12)
project(SofaGLFW LANGUAGES CXX)

include(cmake/environment.cmake)

find_package(Sofa.Config REQUIRED)
sofa_find_package(Sofa.Simulation.Graph REQUIRED)
sofa_find_package(Sofa.GL REQUIRED)
sofa_find_package(Sofa.Component.Visual REQUIRED)
sofa_find_package(Sofa.GUI.Common QUIET)

if( UNIX AND NOT APPLE )
    option(SOFAGLFW_USEX11 "Flag to force x11 for windows management" ON)
    if(SOFAGLFW_USEX11)
        set(SOFAGLFW_USEX11_INTERNAL ON)
    endif ()
endif ()

sofa_find_package(glfw3 3.4 CONFIG QUIET)

if(TARGET glfw)
    if(CMAKE_SYSTEM_NAME STREQUAL Windows)
        sofa_install_libraries(TARGETS glfw)
    endif()
elseif( (DEFINED SOFA_ALLOW_FETCH_DEPENDENCIES AND SOFA_ALLOW_FETCH_DEPENDENCIES) OR (NOT DEFINED SOFA_ALLOW_FETCH_DEPENDENCIES))
    # Download if 
    # 1) SOFA_ALLOW_FETCH_DEPENDENCIES is defined, meaning a in-tree project AND its value is TRUE
    # 2) SOFA_ALLOW_FETCH_DEPENDENCIES is not defined, which means it is a out-of-tree project
    message("${PROJECT_NAME}: glfw3 not found and SOFA_ALLOW_FETCH_DEPENDENCIES is ON, fetching source code...")
    set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
    set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
    set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
    set(GLFW_INSTALL ON CACHE INTERNAL "Generate installation target")
    set(BUILD_SHARED_LIBS ON CACHE INTERNAL "Build GLFW as a shared library")

    sofa_fetch_dependency(glfw
            GIT_REPOSITORY https://github.com/glfw/glfw
            GIT_TAG        3.4
    )

endif()

set(SOFAGLFW_SOURCE_DIR src/SofaGLFW)

set(HEADER_FILES
    ${SOFAGLFW_SOURCE_DIR}/config.h.in
    ${SOFAGLFW_SOURCE_DIR}/init.h
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.h
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.h
    ${SOFAGLFW_SOURCE_DIR}/BaseGUIEngine.h
    ${SOFAGLFW_SOURCE_DIR}/NullGUIEngine.h
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWMouseManager.h
)

set(SOURCE_FILES
    ${SOFAGLFW_SOURCE_DIR}/initSofaGLFW.cpp
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWWindow.cpp
    ${SOFAGLFW_SOURCE_DIR}/NullGUIEngine.cpp
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWBaseGUI.cpp
    ${SOFAGLFW_SOURCE_DIR}/SofaGLFWMouseManager.cpp
)

if(Sofa.GUI.Common_FOUND)
    LIST(APPEND HEADER_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.h)
    LIST(APPEND SOURCE_FILES ${SOFAGLFW_SOURCE_DIR}/SofaGLFWGUI.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.GL Sofa.Simulation.Graph Sofa.Component.Visual)
target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${glfw_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
)

if(Sofa.GUI.Common_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.GUI.Common)
endif()

sofa_find_package(SofaPython3 QUIET)
if(SofaPython3_FOUND)
    add_subdirectory(bindings)
endif()

if(TARGET PkgConfig::FFMPEG)
    target_link_libraries(${PROJECT_NAME} PRIVATE PkgConfig::FFMPEG)
    set(SOFAGLFW_HAVE_FFMPEG 1)
endif()

add_definitions("-DSOFAGLFW_RESOURCES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/resources\"")

# FFMPEG_exec
sofa_find_package(FFMPEG_exec BOTH_SCOPES)
# FFMPEG
if(FFMPEG_EXEC_FOUND)
    install(PROGRAMS "${FFMPEG_EXEC_FILE}" DESTINATION ${CMAKE_INSTALL_PREFIX}/bin COMPONENT applications)
endif()

# Create build and install versions of .ini file for resources finding
#### For build tree
set(FFMPEG_EXEC_PATH "${FFMPEG_EXEC_FILE}") # absolute path for build dir, see .ini file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/etc/${PROJECT_NAME}.ini.in "${CMAKE_BINARY_DIR}/etc/${PROJECT_NAME}.ini")

#### For install tree
get_filename_component(FFMPEG_EXEC_FILENAME "${FFMPEG_EXEC_FILE}" NAME)
if(FFMPEG_EXEC_FILENAME)
    set(FFMPEG_EXEC_PATH "../bin/${FFMPEG_EXEC_FILENAME}")
else()
    #If ffmpeg hasn't been found, it'll not be shipped but we still want to offer the possibility for the user to use it
    #easily by putting the executable alongside runSofa
    set(FFMPEG_EXEC_PATH "../bin/ffmpeg")
    if(WIN32)
        set(FFMPEG_EXEC_PATH "${FFMPEG_EXEC_PATH}.exe")
    endif()
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/etc/${PROJECT_NAME}.ini.in "${CMAKE_BINARY_DIR}/etc/installed${PROJECT_NAME}.ini")
#Hack relative tree to ../../etc because of Windows:NSIS that doesn't allow absolute install path.  
install(FILES "${CMAKE_BINARY_DIR}/etc/installed${PROJECT_NAME}.ini" DESTINATION ../../etc  RENAME ${PROJECT_NAME}.ini COMPONENT applications)



sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)

include(cmake/packaging.cmake)
