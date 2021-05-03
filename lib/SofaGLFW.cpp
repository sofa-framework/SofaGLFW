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
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/MouseEvent.h>

#include <SofaBaseVisual/InteractiveCamera.h>
#include <SofaBaseVisual/VisualStyle.h>


namespace sofa::glfw
{

std::map< GLFWwindow*, SofaGLFWWindow*> SofaGLFWGUI::s_mapWindows;

SofaGLFWGUI::SofaGLFWGUI()
    : m_glfwIsInitialized(false)
    , m_glewIsInitialized(false)
    , m_groot()
    , m_firstWindow(NULL)
{
}

SofaGLFWGUI::~SofaGLFWGUI()
{
    terminate();
}

bool SofaGLFWGUI::init()
{
    if (m_glfwIsInitialized)
        return true;

    if (glfwInit() == GLFW_TRUE)
    {
        m_glfwIsInitialized = true;
        setErrorCallback();
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFWGUI::setErrorCallback()
{
    glfwSetErrorCallback(error_callback);
}

void SofaGLFWGUI::setSimulation(sofa::simulation::NodeSPtr groot)
{
    m_groot = groot;
}

sofa::core::visual::DrawTool* SofaGLFWGUI::getDrawTool()
{
    return &m_glDrawTool;
}


sofa::component::visualmodel::BaseCamera::SPtr SofaGLFWGUI::findCamera(sofa::simulation::NodeSPtr groot)
{
    sofa::component::visualmodel::BaseCamera::SPtr camera;
    groot->get(camera);
    if (!camera)
    {
        camera = sofa::core::objectmodel::New<component::visualmodel::InteractiveCamera>();
        camera->setName(core::objectmodel::Base::shortName(camera.get()));
        m_groot->addObject(camera);
        //currentCamera->p_position.forceSet();
        //currentCamera->p_orientation.forceSet();
        camera->bwdInit();
    }

    camera->setBoundingBox(m_groot->f_bbox.getValue().minBBox(), m_groot->f_bbox.getValue().maxBBox());

    return camera;
}

bool SofaGLFWGUI::createWindow(int width, int height, const char* title)
{
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    auto glfwWindow = glfwCreateWindow(width, height, title, NULL, m_firstWindow);
    if (!m_firstWindow)
        m_firstWindow = glfwWindow;

    if (glfwWindow)
    {
        glfwSetKeyCallback(glfwWindow, key_callback);
        glfwSetCursorPosCallback(glfwWindow, cursor_position_callback);
        glfwSetMouseButtonCallback(glfwWindow, mouse_button_callback);
        glfwSetScrollCallback(glfwWindow, scroll_callback);
        glfwSetWindowCloseCallback(glfwWindow, close_callback);
        makeCurrentContext(glfwWindow);

        auto camera = findCamera(m_groot);
        
        SofaGLFWWindow* sofaWindow = new SofaGLFWWindow(glfwWindow, camera);

        s_mapWindows[glfwWindow] = sofaWindow;
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFWGUI::destroyWindow()
{
}

void SofaGLFWGUI::makeCurrentContext(GLFWwindow* glfwWindow)
{
    glfwMakeContextCurrent(glfwWindow);
    if (!m_glewIsInitialized)
    {
        glewInit();
        m_glewIsInitialized = true;
    }
}

void SofaGLFWGUI::runLoop()
{
    m_vparams = sofa::core::visual::VisualParams::defaultInstance();

    while (!s_mapWindows.empty())
    {
        // Keep running
        runStep();

        for (auto& window : s_mapWindows)
        {
            if (window.second)
            {
                makeCurrentContext(window.first);
                window.second->draw(m_groot, m_vparams);
                glfwSwapBuffers(window.first);
            }
        }

        glfwPollEvents();
    }
}

void SofaGLFWGUI::initVisual()
{
    sofa::simulation::getSimulation()->initTextures(m_groot.get());

    component::visualmodel::VisualStyle::SPtr visualStyle = nullptr;
    m_groot->get(visualStyle);
    if (!visualStyle)
    {
        visualStyle = sofa::core::objectmodel::New<component::visualmodel::VisualStyle>();
        visualStyle->setName(sofa::helper::NameDecoder::getShortName<decltype(visualStyle.get())>());

        core::visual::DisplayFlags* displayFlags = visualStyle->displayFlags.beginEdit();
        displayFlags->setShowVisualModels(sofa::core::visual::tristate::true_value);
        visualStyle->displayFlags.endEdit();

        m_groot->addObject(visualStyle);
        visualStyle->init();
    }

    //init gl states
    glDepthFunc(GL_LEQUAL);
    glClearDepth(1.0);
    glEnable(GL_NORMALIZE);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // Setup 'light 0'
    float lightAmbient[4] = { 0.5f, 0.5f, 0.5f,1.0f };
    float lightDiffuse[4] = { 0.9f, 0.9f, 0.9f,1.0f };
    float lightSpecular[4] = { 1.0f, 1.0f, 1.0f,1.0f };
    float lightPosition[4] = { -0.7f, 0.3f, 0.0f,1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
   
    // Enable color tracking
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    // All materials hereafter have full specular reflectivity with a high shine
    float materialSpecular[4] = { 1.0f, 1.0f, 1.0f,1.0f };
    glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
    glMateriali(GL_FRONT, GL_SHININESS, 128);

    glShadeModel(GL_SMOOTH);

    glEnable(GL_LIGHT0);
}

void SofaGLFWGUI::runStep()
{
    sofa::helper::AdvancedTimer::begin("Animate");

    simulation::getSimulation()->animate(m_groot.get(), m_groot->getDt());
    simulation::getSimulation()->updateVisual(m_groot.get());

    sofa::helper::AdvancedTimer::end("Animate");
}

void SofaGLFWGUI::terminate()
{
    if (!m_glfwIsInitialized)
        return;

    glfwTerminate();
}

void SofaGLFWGUI::error_callback(int error, const char* description)
{
    msg_error("SofaGLFWGUI") << "Error: " << description << ".";
}
 
void SofaGLFWGUI::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void SofaGLFWGUI::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseMoveEvent(xpos, ypos);
    }
}

void SofaGLFWGUI::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseButtonEvent(button, action, mods);
    }
}

void SofaGLFWGUI::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->scrollEvent(xoffset, yoffset);
    }
}

void SofaGLFWGUI::close_callback(GLFWwindow* window)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->close();
        s_mapWindows.erase(window);

        delete currentSofaWindow->second;
    }
}




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
