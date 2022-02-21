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

#include <SofaGLFW/SofaGLFWImgui.h>

namespace sofa::glfw
{

SofaGLFWBaseGUI::~SofaGLFWBaseGUI()
{
    terminate();
}


sofa::core::sptr<sofa::simulation::Node> SofaGLFWBaseGUI::getRootNode() const
{
    return m_groot;
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

void SofaGLFWBaseGUI::changeCamera(sofa::component::visualmodel::BaseCamera::SPtr newCamera)
{
    for (auto& w : s_mapWindows)
    {
        w.second->setCamera(newCamera);
    }
}

bool SofaGLFWBaseGUI::createWindow(int width, int height, const char* title, bool fullscreenAtStartup)
{
    imgui::imguiInit();

    GLFWwindow* glfwWindow = nullptr;
    if (fullscreenAtStartup)
    {
        GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);

        m_lastWindowWidth = width;
        m_lastWindowHeight = height;
        m_lastWindowPositionX = 100;
        m_lastWindowPositionY = 100;

        glfwWindow = glfwCreateWindow(mode->width, mode->height, title, primaryMonitor, m_firstWindow);
    }
    else
    {
        glfwWindow = glfwCreateWindow(width, height, title, nullptr, m_firstWindow);
    }


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

        imgui::imguiInitBackend(glfwWindow);

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


bool SofaGLFWBaseGUI::isFullScreen(GLFWwindow* glfwWindow) const
{
    if (hasWindow())
    {
        glfwWindow = (!glfwWindow) ? m_firstWindow : glfwWindow;
        return glfwGetWindowMonitor(glfwWindow) != nullptr;
    }
    return false;
}

void SofaGLFWBaseGUI::switchFullScreen(GLFWwindow* glfwWindow, unsigned int /* screenID */)
{
    if (hasWindow())
    {
        // only manage the first window for now
        // and the main screen
        glfwWindow = (!glfwWindow) ? m_firstWindow : glfwWindow;

        bool isFullScreen = glfwGetWindowMonitor(glfwWindow) != nullptr;

        if (!isFullScreen)
        {
            // backup window position and window size
            glfwGetWindowPos(glfwWindow, &m_lastWindowPositionX, &m_lastWindowPositionY);
            glfwGetWindowSize(glfwWindow, &m_lastWindowWidth, &m_lastWindowHeight);

            GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
            const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);

            glfwSetWindowMonitor(glfwWindow, primaryMonitor, 0, 0, mode->width, mode->height, GLFW_DONT_CARE);
        }
        else
        {
            glfwSetWindowAttrib(glfwWindow, GLFW_DECORATED, GLFW_TRUE);
            glfwSetWindowMonitor(glfwWindow, nullptr, m_lastWindowPositionX, m_lastWindowPositionY, m_lastWindowWidth, m_lastWindowHeight, GLFW_DONT_CARE);
        }
    }
    else
    {
        msg_error("SofaGLFWBaseGUI") << "No window to set fullscreen"; // can happen with runSofa/BaseGUI
    }
}

void SofaGLFWBaseGUI::setBackgroundColor(const sofa::type::RGBAColor& newColor, unsigned int /* windowID */)
{
    // only manage the first window for now
    if (hasWindow())
    {
        s_mapWindows[m_firstWindow]->setBackgroundColor(newColor);
    }
    else
    {
        msg_error("SofaGLFWBaseGUI") << "No window to set the background in";// can happen with runSofa/BaseGUI
    }
}


void SofaGLFWBaseGUI::setBackgroundImage(const std::string& /* filename */, unsigned int /* windowID */)
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

        for (auto& [glfwWindow, sofaGlfwWindow] : s_mapWindows)
        {
            if (sofaGlfwWindow)
            {
                // while user did not request to close this window (i.e press escape), draw
                if (!glfwWindowShouldClose(glfwWindow))
                {
                    makeCurrentContext(glfwWindow);
                    sofaGlfwWindow->draw(m_groot, m_vparams);

                    imgui::imguiDraw(this);

                    glfwSwapBuffers(glfwWindow);

                }
                else
                {
                    // otherwise close this window
                    close_callback(glfwWindow);
                }
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

    m_vparams = sofa::core::visual::VisualParams::defaultInstance();
    for (auto& [glfwWindow, sofaGlfwWindow] : s_mapWindows)
    {
        sofaGlfwWindow->centerCamera(m_groot, m_vparams);
    }
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

    imgui::imguiTerminate();

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
        case GLFW_KEY_F:
            if (action == GLFW_PRESS)
            {
                auto currentGUI = s_mapGUIs.find(window);
                if (currentGUI != s_mapGUIs.end() && currentGUI->second)
                {
                    currentGUI->second->switchFullScreen(window);
                }
            }
         break;
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
    if (!imgui::dispatchMouseEvents())
        return;
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseMoveEvent(static_cast<int>(xpos), static_cast<int>(ypos));
    }
}

void SofaGLFWBaseGUI::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    if (!imgui::dispatchMouseEvents())
        return;
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseButtonEvent(button, action, mods);
    }
}

void SofaGLFWBaseGUI::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    if (!imgui::dispatchMouseEvents())
        return;
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
