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
#include <sofa/gui/common/BaseViewer.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/gui/common/PickHandler.h>


#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>
#include <sofa/gl/gl.h>

using namespace sofa;
namespace sofaglfw
{
    SofaGLFWWindow::SofaGLFWWindow(GLFWwindow* glfwWindow, sofa::component::visual::BaseCamera::SPtr camera)
            : m_glfwWindow(glfwWindow)
            , m_currentCamera(camera)
    {

    }

    void SofaGLFWWindow::close()
    {
        glfwDestroyWindow(m_glfwWindow);
    }


    void SofaGLFWWindow::draw(sofa::simulation::NodeSPtr groot, sofa::core::visual::VisualParams* vparams, double lastModelviewMatrix [16], double lastProjectionMatrix [16]){
    glClearColor(m_backgroundColor.r(), m_backgroundColor.g(), m_backgroundColor.b(), m_backgroundColor.a());
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

    if (groot->f_bbox.getValue().isValid())
    {
        vparams->sceneBBox() = groot->f_bbox.getValue();
        m_currentCamera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
    }
    m_currentCamera->computeZ();
    m_currentCamera->d_widthViewport.setValue(vparams->viewport()[2]);
    m_currentCamera->d_heightViewport.setValue(vparams->viewport()[3]);

        m_currentCamera->getModelViewMatrix( lastModelviewMatrix );
        vparams->setModelViewMatrix( lastModelviewMatrix );
        // matrices
        double projectionMatrix[16];
        double mvMatrix[16];
        m_currentCamera->getOpenGLProjectionMatrix(lastProjectionMatrix);
        m_currentCamera->getOpenGLModelViewMatrix(lastModelviewMatrix);

        glViewport(0, 0, vparams->viewport()[2], vparams->viewport()[3]);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glMultMatrixd(lastProjectionMatrix);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMultMatrixd(lastModelviewMatrix);

        // Update the visual params
        vparams->zNear() = m_currentCamera->getZNear();
        vparams->zFar() = m_currentCamera->getZFar();
        vparams->setProjectionMatrix(lastProjectionMatrix);
        vparams->setModelViewMatrix(lastModelviewMatrix);

        simulation::node::draw(vparams, groot.get());
    }

    void SofaGLFWWindow::setBackgroundColor(const type::RGBAColor& newColor)
    {
        m_backgroundColor = newColor;
    }

    void SofaGLFWWindow::setCamera(component::visual::BaseCamera::SPtr newCamera)
    {
        m_currentCamera = newCamera;
    }

    void SofaGLFWWindow::centerCamera(simulation::NodeSPtr node, core::visual::VisualParams* vparams) const
    {
        if (m_currentCamera)
        {
            int width, height;
            glfwGetFramebufferSize(m_glfwWindow, &width, &height);
            if (node->f_bbox.getValue().isValid())
            {
                vparams->sceneBBox() = node->f_bbox.getValue();
                m_currentCamera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
            }

            // Update the visual params
            vparams->viewport() = { 0, 0, width, height };
            vparams->zNear() = m_currentCamera->getZNear();
            vparams->zFar() = m_currentCamera->getZFar();

            m_currentCamera->fitBoundingBox(node->f_bbox.getValue().minBBox(), node->f_bbox.getValue().maxBBox());
        }
    }

    void SofaGLFWWindow::mouseMoveEvent(int xpos, int ypos, SofaGLFWBaseGUI* gui)
    {
        m_currentXPos = xpos;
        m_currentYPos = ypos;
        switch (m_currentAction)
        {
            case GLFW_PRESS:
            {
                core::objectmodel::MouseEvent* mEvent = nullptr;
                if (m_currentButton == GLFW_MOUSE_BUTTON_LEFT)
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::LeftPressed, xpos, ypos);
                else if (m_currentButton == GLFW_MOUSE_BUTTON_RIGHT)
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::RightPressed, xpos, ypos);
                else if (m_currentButton == GLFW_MOUSE_BUTTON_MIDDLE)
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::MiddlePressed, xpos, ypos);
                else
                {
                    // A fallback event to rule them all...
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::AnyExtraButtonPressed, xpos, ypos);
                }
                m_currentCamera->manageEvent(mEvent);

                auto rootNode = gui->getRootNode();

                rootNode->propagateEvent(core::execparams::defaultInstance(), mEvent);

                break;
            }
            case GLFW_RELEASE:
            {
                core::objectmodel::MouseEvent* mEvent = nullptr;
                if (m_currentButton == GLFW_MOUSE_BUTTON_LEFT)
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::LeftReleased, xpos, ypos);
                else if (m_currentButton == GLFW_MOUSE_BUTTON_RIGHT)
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::RightReleased, xpos, ypos);
                else if (m_currentButton == GLFW_MOUSE_BUTTON_MIDDLE)
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::MiddleReleased, xpos, ypos);
                else
                {
                    // A fallback event to rules them all...
                    mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::AnyExtraButtonReleased, xpos, ypos);
                }
                m_currentCamera->manageEvent(mEvent);

                auto rootNode = gui->getRootNode();

                rootNode->propagateEvent(core::execparams::defaultInstance(), mEvent);

                break;
            }
            default:
            {
                core::objectmodel::MouseEvent me(core::objectmodel::MouseEvent::Move, xpos, ypos);
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
        // Only change state on button press; release resets state to neutral
            m_currentButton = button;
            m_currentAction = action;
            m_currentMods = mods;
    }

    bool SofaGLFWWindow::mouseEvent(GLFWwindow* window, int width, int height,int button, int action, int mods, double xpos, double ypos) {

        if (!m_currentCamera)
            return true;

        SofaGLFWBaseGUI *gui = static_cast<SofaGLFWBaseGUI *>(glfwGetWindowUserPointer(window));

        gui::common::MousePosition mousepos;
        mousepos.screenWidth = width;
        mousepos.screenHeight = height;
        mousepos.x = static_cast<int>(xpos);
        mousepos.y = static_cast<int>(ypos);
        auto rootNode = gui->getRootNode();

        if (GLFW_MOD_SHIFT)
        {
            gui->getPickHandler()->activateRay(width, height, rootNode.get());
            gui->getPickHandler()->updateMouse2D(mousepos);

            if (action == GLFW_PRESS)
            {
                if (button == GLFW_MOUSE_BUTTON_LEFT)
                {
                    gui->getPickHandler()->handleMouseEvent(gui::common::PRESSED, gui::common::LEFT);
                }
                else if (button == GLFW_MOUSE_BUTTON_RIGHT)
                {
                    gui->getPickHandler()->handleMouseEvent(gui::common::PRESSED, gui::common::RIGHT);
                }
                else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
                {
                    gui->getPickHandler()->handleMouseEvent(gui::common::PRESSED, gui::common::MIDDLE);
                }
            }
            else if (action == GLFW_RELEASE)
            {
                if (action == GLFW_RELEASE)
                {
                    if (button == GLFW_MOUSE_BUTTON_LEFT)
                    {
                        gui->getPickHandler()->handleMouseEvent(gui::common::RELEASED, gui::common::LEFT);
                        gui->getPickHandler()->deactivateRay();
                    }
                    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
                    {
                        gui->getPickHandler()->handleMouseEvent(gui::common::RELEASED, gui::common::RIGHT);
                    }
                    else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
                    {
                        gui->getPickHandler()->handleMouseEvent(gui::common::RELEASED, gui::common::MIDDLE);
                    }
                }
            }
            gui->moveRayPickInteractor(xpos, ypos);
        }
        else
        {
            gui->getPickHandler()->activateRay(width, height, rootNode.get());
        }
        return true;
    }

    void SofaGLFWWindow::scrollEvent(double xoffset, double yoffset)
    {
        SOFA_UNUSED(xoffset);
        const double yFactor = 10.f;
        core::objectmodel::MouseEvent me(core::objectmodel::MouseEvent::Wheel, static_cast<int>(yoffset * yFactor));
        m_currentCamera->manageEvent(&me);
    }

} // namespace sofaglfw
