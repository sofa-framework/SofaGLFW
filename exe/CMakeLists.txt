cmake_minimum_required(VERSION 3.12)
project(runSofaGLFW LANGUAGES CXX)

find_package(Sofa.Config REQUIRED)

# temporary fix for out-of-tree builds
# real fix would be to set cmake_module_path with the good dir
if(NOT TARGET SofaGLFW)
    find_package(SofaGLFW REQUIRED)
endif()

if(NOT TARGET Sofa.Helper) # temporary fix about aliases problem
    sofa_find_package(Sofa.Simulation.Core REQUIRED)
endif()

set(SOURCE_FILES
    Main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} Sofa.Simulation.Core SofaGLFW)
target_include_directories(${PROJECT_NAME} PRIVATE extlibs/cxxopts-2.2.1/include)

sofa_add_targets_to_package(
    PACKAGE_NAME SofaGui
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
    INCLUDE_INSTALL_DIR "SofaGui/runSofaGLFW"
    )
