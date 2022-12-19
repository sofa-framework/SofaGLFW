/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaGLFW/config.h>
#include <SofaGLFW/NullGUIEngine.h>
#include <sofa/core/visual/VisualParams.h>
#include <GLFW/glfw3.h>

namespace sofaglfw
{
    
void NullGUIEngine::init()
{
    m_startTime = glfwGetTime();
}
void NullGUIEngine::initBackend(GLFWwindow* window)
{
    m_window = window;
}
void NullGUIEngine::startFrame(SofaGLFWBaseGUI*)
{
}
void NullGUIEngine::endFrame()
{
    constexpr double refreshTime = 1.0;

    m_currentTime = glfwGetTime();

    const auto diffTime = m_currentTime - m_startTime;
    if (diffTime > refreshTime || m_nbFrames == 0)
    {
        char title_string[32];
        double fps = static_cast<double>(m_nbFrames) / diffTime;
        std::snprintf(title_string, sizeof(title_string), "FPS: %.1f", fps);
        glfwSetWindowTitle(m_window, title_string);
        m_startTime = m_currentTime;
        m_nbFrames = 0;
    }
    m_nbFrames++;
}

void NullGUIEngine::beforeDraw(GLFWwindow* window)
{
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    sofa::core::visual::VisualParams::defaultInstance()->viewport() = {0, 0, width, height};
}

void NullGUIEngine::terminate()
{

}

bool NullGUIEngine::dispatchMouseEvents()
{
    return true;
}

void NullGUIEngine::resetCounter()
{

}

sofa::type::Vec2i NullGUIEngine::getFrameBufferPixels(std::vector<uint8_t>& pixels)
{
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    pixels.resize(viewport[2] * viewport[3] * 4);
    glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    
    return {viewport[2], viewport[3]};
}

} // namespace sofaglfw
