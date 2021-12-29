cmake_minimum_required(VERSION 3.12)
project(runSofaGLFW LANGUAGES CXX)

find_package(Sofa.Config REQUIRED)

# temporary fix for out-of-tree builds
# real fix would be to set cmake_module_path with the good dir
if(NOT TARGET SofaGLFW)
    find_package(SofaGLFW REQUIRED)
endif()

if(NOT TARGET Sofa.Helper) # temporary fix about aliases problem
    sofa_find_package(Sofa.SimulationCore REQUIRED)
endif()

set(SOURCE_FILES
    Main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} Sofa.SimulationCore SofaGLFW)
target_include_directories(${PROJECT_NAME} PUBLIC extlibs/cxxopts-2.2.1/include)
