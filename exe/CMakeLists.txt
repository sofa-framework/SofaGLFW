cmake_minimum_required(VERSION 3.12)
project(runSofaGLFW)

#find_package(SofaGLFW REQUIRED)
find_package(SofaBase REQUIRED)

set(SOURCE_FILES
    Main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaBase SofaGLFW)
target_include_directories(${PROJECT_NAME} PUBLIC extlibs/cxxopts-2.2.1/include)