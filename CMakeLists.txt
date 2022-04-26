cmake_minimum_required(VERSION 3.12)

find_package(Sofa.Config REQUIRED)

sofa_add_plugin(SofaGLFW SofaGLFW)
sofa_add_plugin(SofaImGui SofaImGui)
sofa_add_application(exe runSofaGLFW OFF)
