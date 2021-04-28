/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaGLFW.h>

#include <sofa/helper/logging/Messaging.h>

namespace sofa::glfw
{

bool SofaGLFW::s_glfwIsInitialized = false;
bool SofaGLFW::s_glewIsInitialized = false;
unsigned int SofaGLFW::s_nbInstances = 0;


SofaGLFW::SofaGLFW()
    : m_glfwWindow(nullptr)
{
}

SofaGLFW::~SofaGLFW()
{
    terminate();
}

bool SofaGLFW::init()
{
    if (s_glfwIsInitialized)
        return true;

    if (glfwInit() == GLFW_TRUE)
    {
        s_glfwIsInitialized = true;
        setErrorCallback();
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFW::setErrorCallback()
{
    glfwSetErrorCallback(error_callback);
}

bool SofaGLFW::createWindow(int width, int height, const char* title)
{
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    m_glfwWindow = glfwCreateWindow(width, height, title, NULL, NULL);
    if (m_glfwWindow)
    {
        glfwSetKeyCallback(m_glfwWindow, key_callback);
        s_nbInstances++;
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFW::destroyWindow()
{
    glfwDestroyWindow(m_glfwWindow);
}

void SofaGLFW::makeCurrentContext()
{
    glfwMakeContextCurrent(m_glfwWindow);
    if (!s_glewIsInitialized)
    {
        glewInit();
        s_glewIsInitialized = true;
    }
}

void SofaGLFW::runLoop()
{
    while (!glfwWindowShouldClose(m_glfwWindow))
    {
        // Keep running
        //...

        glfwSwapBuffers(m_glfwWindow);
        glfwPollEvents();
    }

}


void SofaGLFW::terminate()
{
    if (!s_glfwIsInitialized)
        return;

    s_nbInstances--;

    glfwDestroyWindow(m_glfwWindow);

    if (s_nbInstances < 1)
    {
        glfwTerminate();
    }
}

void SofaGLFW::error_callback(int error, const char* description)
{
    msg_error("SofaGLFW") << "Error: " << description << ".";
}
 
void SofaGLFW::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

} // namespace sofa::glfw
