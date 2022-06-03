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
#pragma once
#include <SofaGLFW/config.h>

#include <sofa/simulation/fwd.h>
#include <sofa/component/visual/BaseCamera.h>

struct GLFWwindow;

namespace sofaglfw
{

class SOFAGLFW_API SofaGLFWWindow
{
public:
    SofaGLFWWindow(GLFWwindow* glfwWindow, sofa::component::visual::BaseCamera::SPtr camera);
    virtual ~SofaGLFWWindow() = default;

    void draw(sofa::simulation::NodeSPtr groot, sofa::core::visual::VisualParams* vparams);
    void close();

    void mouseMoveEvent(int xpos, int ypos);
    void mouseButtonEvent(int button, int action, int mods);
    void scrollEvent(double xoffset, double yoffset);
    void setBackgroundColor(const sofa::type::RGBAColor& newColor);

    void setCamera(sofa::component::visual::BaseCamera::SPtr newCamera);
    void centerCamera(sofa::simulation::NodeSPtr node, sofa::core::visual::VisualParams* vparams) const;

private:
    GLFWwindow* m_glfwWindow{nullptr};
    sofa::component::visual::BaseCamera::SPtr m_currentCamera;
    int m_currentButton{ -1 };
    int m_currentAction{ -1 };
    int m_currentMods{ -1 };
    int m_currentXPos{ -1 };
    int m_currentYPos{ -1 };
    sofa::type::RGBAColor m_backgroundColor{ sofa::type::RGBAColor::black() };
};

} // namespace sofaglfw
