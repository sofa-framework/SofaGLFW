cmake_minimum_required(VERSION 3.12)

find_package(Sofa.Config REQUIRED)

sofa_add_subdirectory(plugin SofaGLFW SofaGLFW ON)
sofa_add_subdirectory(plugin SofaImGui SofaImGui ON)
sofa_add_subdirectory(application exe runSofaGLFW OFF)
