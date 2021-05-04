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
#include <SofaGLFW/config.h>

#include <sofa/simulation/fwd.h>
#include <SofaBaseVisual/BaseCamera.h>

namespace sofa::glfw
{

class SOFA_GLFW_API SofaGLFWWindow
{
public:
    SofaGLFWWindow(GLFWwindow* glfwWindow, sofa::component::visualmodel::BaseCamera::SPtr camera);
    virtual ~SofaGLFWWindow();

    void draw(sofa::simulation::NodeSPtr groot, sofa::core::visual::VisualParams* vparams);
    void close();

    void mouseMoveEvent(int xpos, int ypos);
    void mouseButtonEvent(int button, int action, int mods);
    void scrollEvent(double xoffset, double yoffset);

private:
    GLFWwindow* m_glfwWindow;
    sofa::component::visualmodel::BaseCamera::SPtr m_currentCamera;
    int m_currentButton = -1;
    int m_currentAction = -1;
    int m_currentMods = -1;
    int m_currentXPos = -1;
    int m_currentYPos = -1;
};

} // namespace sofa::glfw
