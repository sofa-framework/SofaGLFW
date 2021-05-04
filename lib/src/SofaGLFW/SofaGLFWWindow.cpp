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
#include <SofaGLFW/SofaGLFWWindow.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>
#include <sofa/gl/gl.h>

namespace sofa::glfw
{

SofaGLFWWindow::SofaGLFWWindow(GLFWwindow* glfwWindow, sofa::component::visualmodel::BaseCamera::SPtr camera)
    : m_glfwWindow(glfwWindow)
    , m_currentCamera(camera)
{
}


SofaGLFWWindow::~SofaGLFWWindow()
{
}

void SofaGLFWWindow::close()
{
    glfwDestroyWindow(m_glfwWindow);
}


void SofaGLFWWindow::draw(sofa::simulation::NodeSPtr groot, sofa::core::visual::VisualParams* vparams)
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);

    // draw the scene
    if (!m_currentCamera)
    {
        msg_error("SofaGLFWGUI") << "No camera defined.";
        return;
    }

    int width, height;
    glfwGetFramebufferSize(m_glfwWindow, &width, &height);
    if (groot->f_bbox.getValue().isValid())
    {
        vparams->sceneBBox() = groot->f_bbox.getValue();
        m_currentCamera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
    }
    m_currentCamera->computeZ();
    m_currentCamera->p_widthViewport.setValue(width);
    m_currentCamera->p_heightViewport.setValue(height);

    // matrices
    double projectionMatrix[16];
    double mvMatrix[16];
    m_currentCamera->getOpenGLProjectionMatrix(projectionMatrix);
    m_currentCamera->getOpenGLModelViewMatrix(mvMatrix);

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixd(projectionMatrix);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixd(mvMatrix);

    // Update the visual params
    vparams->viewport() = { 0, 0, width, height };
    vparams->zNear() = m_currentCamera->getZNear();
    vparams->zFar() = m_currentCamera->getZFar();
    vparams->setProjectionMatrix(projectionMatrix);
    vparams->setModelViewMatrix(mvMatrix);

    simulation::getSimulation()->draw(vparams, groot.get());

}

void SofaGLFWWindow::mouseMoveEvent(int xpos, int ypos)
{
    switch (m_currentAction)
    {
    case GLFW_PRESS:
    {
        sofa::core::objectmodel::MouseEvent* mEvent = nullptr;
        if (m_currentButton == GLFW_MOUSE_BUTTON_LEFT)
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::LeftPressed, xpos, ypos);
        else if (m_currentButton == GLFW_MOUSE_BUTTON_RIGHT)
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::RightPressed, xpos, ypos);
        else if (m_currentButton == GLFW_MOUSE_BUTTON_MIDDLE)
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::MiddlePressed, xpos, ypos);
        else {
            // A fallback event to rule them all...
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::AnyExtraButtonPressed, xpos, ypos);
        }
        m_currentCamera->manageEvent(mEvent);
        m_currentXPos = xpos;
        m_currentYPos = ypos;
        break;
    }
    case GLFW_RELEASE:
    {
        sofa::core::objectmodel::MouseEvent* mEvent = nullptr;
        if (m_currentButton == GLFW_MOUSE_BUTTON_LEFT)
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::LeftReleased, xpos, ypos);
        else if (m_currentButton == GLFW_MOUSE_BUTTON_RIGHT)
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::RightReleased, xpos, ypos);
        else if (m_currentButton == GLFW_MOUSE_BUTTON_MIDDLE)
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::MiddleReleased, xpos, ypos);
        else {
            // A fallback event to rules them all...
            mEvent = new sofa::core::objectmodel::MouseEvent(sofa::core::objectmodel::MouseEvent::AnyExtraButtonReleased, xpos, ypos);
        }
        m_currentCamera->manageEvent(mEvent);
        m_currentXPos = xpos;
        m_currentYPos = ypos;
        break;
    }

    default:
    {
        sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::Move, xpos, ypos);
        m_currentCamera->manageEvent(&me);
        break;
    }
    }

    m_currentButton = -1;
    m_currentAction = -1;
    m_currentMods = -1;
}

void SofaGLFWWindow::mouseButtonEvent(int button, int action, int mods)
{
    m_currentButton = button;
    m_currentAction = action;
    m_currentMods = mods;
}


void SofaGLFWWindow::scrollEvent(double xoffset, double yoffset)
{
    SOFA_UNUSED(xoffset);
    const double yFactor = 10.f;
    sofa::core::objectmodel::MouseEvent me(sofa::core::objectmodel::MouseEvent::Wheel, yoffset * yFactor);
    m_currentCamera->manageEvent(&me);
}

} // namespace sofa::glfw
