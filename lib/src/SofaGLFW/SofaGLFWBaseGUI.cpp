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
#include <SofaGLFW/SofaGLFWBaseGUI.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <SofaGLFW/SofaGLFWWindow.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/core/visual/VisualParams.h>

#include <SofaBaseVisual/InteractiveCamera.h>
#include <SofaBaseVisual/VisualStyle.h>


namespace sofa::glfw
{

std::map< GLFWwindow*, SofaGLFWWindow*> SofaGLFWBaseGUI::s_mapWindows;
std::map< GLFWwindow*, SofaGLFWBaseGUI*> SofaGLFWBaseGUI::s_mapGUIs;

SofaGLFWBaseGUI::SofaGLFWBaseGUI()
{
}

SofaGLFWBaseGUI::~SofaGLFWBaseGUI()
{
    terminate();
}

bool SofaGLFWBaseGUI::init()
{
    if (m_bGlfwIsInitialized)
        return true;

    if (glfwInit() == GLFW_TRUE)
    {
        m_glDrawTool = new sofa::gl::DrawToolGL();
        m_bGlfwIsInitialized = true;
        setErrorCallback();
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFWBaseGUI::setErrorCallback() const
{
    glfwSetErrorCallback(error_callback);
}

void SofaGLFWBaseGUI::setSimulation(sofa::simulation::NodeSPtr groot, const std::string& filename)
{
    m_groot = groot;
    m_filename = filename;

    sofa::core::visual::VisualParams::defaultInstance()->drawTool() = m_glDrawTool;
}

void SofaGLFWBaseGUI::setSimulationIsRunning(bool running)
{
    if (m_groot)
    {
        m_groot->setAnimate(running);
    }
}


bool SofaGLFWBaseGUI::simulationIsRunning() const
{
    if (m_groot)
    {
        return m_groot->getAnimate();
    }

    return false;
}

sofa::component::visualmodel::BaseCamera::SPtr SofaGLFWBaseGUI::findCamera(sofa::simulation::NodeSPtr groot)
{
    sofa::component::visualmodel::BaseCamera::SPtr camera;
    groot->get(camera);
    if (!camera)
    {
        camera = sofa::core::objectmodel::New<component::visualmodel::InteractiveCamera>();
        camera->setName(core::objectmodel::Base::shortName(camera.get()));
        m_groot->addObject(camera);
        camera->bwdInit();
    }

    camera->setBoundingBox(m_groot->f_bbox.getValue().minBBox(), m_groot->f_bbox.getValue().maxBBox());

    return camera;
}

bool SofaGLFWBaseGUI::createWindow(int width, int height, const char* title)
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
        s_mapGUIs[glfwWindow] = this;
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFWBaseGUI::destroyWindow()
{
}

void SofaGLFWBaseGUI::makeCurrentContext(GLFWwindow* glfwWindow)
{
    glfwMakeContextCurrent(glfwWindow);
    if (!m_bGlewIsInitialized)
    {
        glewInit();
        m_bGlewIsInitialized = true;
    }
}

void SofaGLFWBaseGUI::runLoop()
{
    if (!m_groot)
    {
        return;
    }

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

void SofaGLFWBaseGUI::initVisual()
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

void SofaGLFWBaseGUI::runStep()
{
    if(simulationIsRunning())
    {
        sofa::helper::AdvancedTimer::begin("Animate");

        simulation::getSimulation()->animate(m_groot.get(), m_groot->getDt());
        simulation::getSimulation()->updateVisual(m_groot.get());

        sofa::helper::AdvancedTimer::end("Animate");
    }
}

void SofaGLFWBaseGUI::terminate()
{
    if (!m_bGlfwIsInitialized)
        return;

    glfwTerminate();
}

void SofaGLFWBaseGUI::error_callback(int error, const char* description)
{
    msg_error("SofaGLFWBaseGUI") << "Error: " << description << ".";
}
 
void SofaGLFWBaseGUI::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key)
    {
        case GLFW_KEY_ESCAPE:
            if (action == GLFW_PRESS)
            {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            break;
        case GLFW_KEY_SPACE:
            if (action == GLFW_PRESS)
            {
                auto currentGUI = s_mapGUIs.find(window);
                if (currentGUI != s_mapGUIs.end() && currentGUI->second)
                {
                    currentGUI->second->setSimulationIsRunning(!currentGUI->second->simulationIsRunning());
                }
            }
            break;
        default:
            break;
    }
}

void SofaGLFWBaseGUI::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseMoveEvent(xpos, ypos);
    }
}

void SofaGLFWBaseGUI::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseButtonEvent(button, action, mods);
    }
}

void SofaGLFWBaseGUI::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->scrollEvent(xoffset, yoffset);
    }
}

void SofaGLFWBaseGUI::close_callback(GLFWwindow* window)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->close();
        s_mapWindows.erase(window);

        delete currentSofaWindow->second;
    }
}

} // namespace sofa::glfw
