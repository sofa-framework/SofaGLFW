cmake_minimum_required(VERSION 3.12)
project(SofaImGui_test VERSION 1.0)
find_package(Sofa.Testing REQUIRED)
sofa_find_package(Sofa.SimpleApi REQUIRED)

set(SOURCE_FILES
    windows/ProgramWindow_test.cpp
    )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${imgui_SOURCE_DIR} ${QPOASES_INCLUDE_DIR})
if(ROS_FOUND AND SOFAIMGUI_WITH_ROS)
    target_include_directories(${PROJECT_NAME} PRIVATE ${ROS_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${ROS_LIBRARIES})
endif(ROS_FOUND AND SOFAIMGUI_WITH_ROS)

target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Testing SofaImGui Sofa.SimpleApi)
add_definitions("-DSOFAIMGUI_TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

