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

#include <sofa/component/visual/InteractiveCamera.h>
#include <sofa/component/visual/VisualStyle.h>

#include <sofa/helper/io/STBImage.h>

#include <algorithm>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/simulation/SimulationLoop.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
using namespace sofa;

namespace sofaglfw
{

SofaGLFWBaseGUI::SofaGLFWBaseGUI()
{
    m_guiEngine = std::make_shared<NullGUIEngine>();
}

SofaGLFWBaseGUI::~SofaGLFWBaseGUI()
{
    terminate();
}

sofa::core::sptr<sofa::simulation::Node> SofaGLFWBaseGUI::getRootNode() const
{
    return m_groot;
}

bool SofaGLFWBaseGUI::init(int nbMSAASamples)
{
    if (m_bGlfwIsInitialized)
        return true;

    setErrorCallback();

    if (glfwInit() == GLFW_TRUE)
    {
        // defined samples for MSAA
        // min = 0  (no MSAA Anti-aliasing)
        // max = 32 (MSAA with 32 samples)
        glfwWindowHint(GLFW_SAMPLES, std::clamp(nbMSAASamples, 0, 32) );
        
        m_glDrawTool = new sofa::gl::DrawToolGL();
        m_bGlfwIsInitialized = true;
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

sofa::component::visual::BaseCamera::SPtr SofaGLFWBaseGUI::findCamera(sofa::simulation::NodeSPtr groot)
{
    sofa::component::visual::BaseCamera::SPtr camera;
    groot->get(camera);
    if (!camera)
    {
        camera = sofa::core::objectmodel::New<component::visual::InteractiveCamera>();
        camera->setName(core::objectmodel::Base::shortName(camera.get()));
        m_groot->addObject(camera);
        camera->bwdInit();
    }

    camera->setBoundingBox(m_groot->f_bbox.getValue().minBBox(), m_groot->f_bbox.getValue().maxBBox());

    return camera;
}

void SofaGLFWBaseGUI::changeCamera(sofa::component::visual::BaseCamera::SPtr newCamera)
{
    for (auto& w : s_mapWindows)
    {
        w.second->setCamera(newCamera);
    }
}

void SofaGLFWBaseGUI::restoreCamera(sofa::component::visual::BaseCamera::SPtr camera)
{
    if (camera)
    {
        const std::string viewFileName = this->getFilename() + std::string(this->getCameraFileExtension());
        if (camera->importParametersFromFile(viewFileName))
        {
            msg_info("GUI") << "Current camera parameters have been imported from " << viewFileName << " .";
        }
        else
        {
            msg_error("GUI") << "Could not import camera parameters from " << viewFileName << " .";
        }
    }
}

void SofaGLFWBaseGUI::setWindowIcon(GLFWwindow* glfwWindow)
{
    //STBImage relies on DataRepository to find files: it must be extended with the resource files from this plugin
    sofa::helper::system::DataRepository.addFirstPath(SOFAGLFW_RESOURCES_DIR);

    sofa::helper::io::STBImage img;
    if (img.load("SOFA.png"))
    {
        GLFWimage images[1];
        images[0].height = img.getHeight();
        images[0].width = img.getWidth();
        images[0].pixels = img.getPixels();
        glfwSetWindowIcon(glfwWindow, 1, images);
    }
    sofa::helper::system::DataRepository.removePath(SOFAGLFW_RESOURCES_DIR);
}

bool SofaGLFWBaseGUI::createWindow(int width, int height, const char* title, bool fullscreenAtStartup)
{
    m_guiEngine->init();

    if (m_groot == nullptr)
    {
        msg_error("SofaGLFWBaseGUI") << "No simulation root has been defined. Quitting.";
        return false;
    }

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
        glfwWindow = glfwCreateWindow(width > 0 ? width : 100, height > 0 ? height : 100, title, nullptr, m_firstWindow);
    }

    setWindowIcon(glfwWindow);

    if (!m_firstWindow)
        m_firstWindow = glfwWindow;

    if (glfwWindow)
    {
        glfwSetKeyCallback(glfwWindow, key_callback);
        glfwSetCursorPosCallback(glfwWindow, cursor_position_callback);
        glfwSetMouseButtonCallback(glfwWindow, mouse_button_callback);
        glfwSetScrollCallback(glfwWindow, scroll_callback);
        glfwSetWindowCloseCallback(glfwWindow, close_callback);

        // this set empty callbacks
        // solve a crash when glfw is quitting and tries to use nullptr callbacks
        // could be potentially useful in the future anyway
        glfwSetWindowFocusCallback(glfwWindow, window_focus_callback);
        glfwSetCursorEnterCallback(glfwWindow, cursor_enter_callback);
        glfwSetMonitorCallback(monitor_callback);
        glfwSetCharCallback(glfwWindow, character_callback);

        makeCurrentContext(glfwWindow);

        m_guiEngine->initBackend(glfwWindow);

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

void SofaGLFWBaseGUI::resizeWindow(int width, int height)
{
    if (hasWindow())
    {
        glfwSetWindowSize(m_firstWindow, width, height);
    }
}

void SofaGLFWBaseGUI::destroyWindow()
{
}

GLFWmonitor* SofaGLFWBaseGUI::getCurrentMonitor(GLFWwindow *glfwWindow)
{
    int monitorsCount, i;
    int windowsX, windowsY, windowsWidth, windowsHeight;
    int monitorX, monitorY, monitorWidth, monitorHeight;
    int overlap, bestOverlap;
    GLFWmonitor *bestMonitor;
    GLFWmonitor **monitors;
    const GLFWvidmode *mode;

    bestOverlap = 0;
    bestMonitor = nullptr;

    glfwGetWindowPos(glfwWindow, &windowsX, &windowsY);
    glfwGetWindowSize(glfwWindow, &windowsWidth, &windowsHeight);
    monitors = glfwGetMonitors(&monitorsCount);

    for (i=0; i<monitorsCount; i++)
    {
        mode = glfwGetVideoMode(monitors[i]);
        glfwGetMonitorPos(monitors[i], &monitorX, &monitorY);
        monitorWidth = mode->width;
        monitorHeight = mode->height;

        overlap = std::max(0, std::min(windowsX + windowsWidth, monitorX + monitorWidth) - std::max(windowsX, monitorX)) *
                  std::max(0, std::min(windowsY + windowsHeight, monitorY + monitorHeight) - std::max(windowsY, monitorY));

        if (bestOverlap < overlap)
        {
            bestOverlap = overlap;
            bestMonitor = monitors[i];
        }
    }

    return bestMonitor;
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

            GLFWmonitor* monitor = getCurrentMonitor(glfwWindow);
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);

            glfwSetWindowMonitor(glfwWindow, monitor, 0, 0, mode->width, mode->height, GLFW_DONT_CARE);
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
    glfwSwapInterval( 0 ); //request disabling vsync
    if (!m_bGlewIsInitialized)
    {
        glewInit();
        m_bGlewIsInitialized = true;
    }
}

std::size_t SofaGLFWBaseGUI::runLoop(std::size_t targetNbIterations)
{
    if (!m_groot)
    {
        msg_error("SofaGLFWBaseGUI") << "Cannot start main loop: root node is invalid";
        return 0;
    }

    m_vparams = sofa::core::visual::VisualParams::defaultInstance();

    bool running = true;
    std::size_t currentNbIterations = 0;
    std::stringstream tmpStr;
    while (!s_mapWindows.empty() && running)
    {
        SIMULATION_LOOP_SCOPE

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

                    m_guiEngine->beforeDraw(glfwWindow);
                    sofaGlfwWindow->draw(m_groot, m_vparams);
                    m_guiEngine->afterDraw();

                    m_guiEngine->startFrame(this);
                    m_guiEngine->endFrame();

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

        currentNbIterations++;
        running = (targetNbIterations > 0) ? currentNbIterations < targetNbIterations : true;
    }

    return currentNbIterations;
}

void SofaGLFWBaseGUI::initVisual()
{
    sofa::simulation::node::initTextures(m_groot.get());

    component::visual::VisualStyle::SPtr visualStyle = nullptr;
    m_groot->get(visualStyle);
    if (!visualStyle)
    {
        visualStyle = sofa::core::objectmodel::New<component::visual::VisualStyle>();
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

        simulation::node::animate(m_groot.get(), m_groot->getDt());
        simulation::node::updateVisual(m_groot.get());

        sofa::helper::AdvancedTimer::end("Animate");
    }
}

void SofaGLFWBaseGUI::terminate()
{
    if (!m_bGlfwIsInitialized)
        return;

    m_guiEngine->terminate();

    glfwTerminate();
}

void SofaGLFWBaseGUI::error_callback(int error, const char* description)
{
    SOFA_UNUSED(error);
    msg_error("SofaGLFWBaseGUI") << "Error: " << description << ".";
}

int SofaGLFWBaseGUI::handleArrowKeys(int key)
{
    // Handling arrow keys with custom codes
    switch (key)
    {
            case GLFW_KEY_UP: return 19;   // Custom code for up
            case GLFW_KEY_DOWN: return 21; // Custom code for down
            case GLFW_KEY_LEFT: return 18; // Custom code for left
            case GLFW_KEY_RIGHT: return 20; // Custom code for right
    }
    // Default case return the given value as GLFW handle it
    return key;
}

void SofaGLFWBaseGUI::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    const char keyName = SofaGLFWBaseGUI::handleArrowKeys(key);
    const bool isCtrlKeyPressed = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS;

    auto currentGUI = s_mapGUIs.find(window);
    if (currentGUI == s_mapGUIs.end() || currentGUI->second == nullptr)
    {
        return;
    }

    auto rootNode = currentGUI->second->getRootNode();
    if (!rootNode)
    {
        return;
    }

    if (isCtrlKeyPressed)
    {
        if (action == GLFW_PRESS)
        {
            dmsg_info_when(key == GLFW_KEY_LEFT_CONTROL, "SofaGLFWBaseGUI") << "KeyPressEvent, CONTROL pressed";

            sofa::core::objectmodel::KeypressedEvent keyPressedEvent(keyName);
            rootNode->propagateEvent(sofa::core::ExecParams::defaultInstance(), &keyPressedEvent);
        }
        else if (action == GLFW_RELEASE)
        {
            sofa::core::objectmodel::KeyreleasedEvent keyReleasedEvent(keyName);
            rootNode->propagateEvent(sofa::core::ExecParams::defaultInstance(), &keyReleasedEvent);
        }
    }

    // Handle specific keys for additional functionality
    switch (key)
    {
        case GLFW_KEY_F:
            if (action == GLFW_PRESS && (mods & GLFW_MOD_CONTROL))
            {
                currentGUI->second->switchFullScreen(window);
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
                const bool isRunning = currentGUI->second->simulationIsRunning();
                currentGUI->second->setSimulationIsRunning(!isRunning);
            }
            break;
    }
}

void SofaGLFWBaseGUI::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    auto currentGUI = s_mapGUIs.find(window);
    if (currentGUI != s_mapGUIs.end() && currentGUI->second)
    {
        if (!currentGUI->second->getGUIEngine()->dispatchMouseEvents())
            return;
    }

    if (!currentGUI->second->m_isMouseInteractionEnabled)
    {
        auto currentSofaWindow = s_mapWindows.find(window);
        if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
        {
            currentSofaWindow->second->mouseMoveEvent(static_cast<int>(xpos), static_cast<int>(ypos));
        }
    }
}

void SofaGLFWBaseGUI::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    auto currentGUI = s_mapGUIs.find(window);
    if (currentGUI != s_mapGUIs.end() && currentGUI->second)
    {
        if (!currentGUI->second->getGUIEngine()->dispatchMouseEvents())
            return;
    }
    
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseButtonEvent(button, action, mods);
    }
}

void SofaGLFWBaseGUI::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto currentGUI = s_mapGUIs.find(window);
    if (currentGUI != s_mapGUIs.end() && currentGUI->second)
    {
        if (!currentGUI->second->getGUIEngine()->dispatchMouseEvents())
            return;
    }
    
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->scrollEvent(xoffset, yoffset);
    }
}

void SofaGLFWBaseGUI::close_callback(GLFWwindow* window)
{
    auto currentSofaWindow = s_mapWindows.find(window);
    if (currentSofaWindow != s_mapWindows.end())
    {
        if (SofaGLFWWindow* glfwWindow = currentSofaWindow->second)
        {
            glfwWindow->close();
            delete glfwWindow;
        }
        s_mapWindows.erase(window);
    }
}

void SofaGLFWBaseGUI::window_focus_callback(GLFWwindow* window, int focused)
{
    SOFA_UNUSED(window);
    SOFA_UNUSED(focused);
    //if (focused)
    //{
    //    // The window gained input focus
    //}
    //else
    //{
    //    // The window lost input focus
    //}
}
void SofaGLFWBaseGUI::cursor_enter_callback(GLFWwindow* window, int entered)
{
    SOFA_UNUSED(window);
    SOFA_UNUSED(entered);

    //if (entered)
    //{
    //    // The cursor entered the content area of the window
    //}
    //else
    //{
    //    // The cursor left the content area of the window
    //}
}
void SofaGLFWBaseGUI::monitor_callback(GLFWmonitor* monitor, int event)
{
    SOFA_UNUSED(monitor);
    SOFA_UNUSED(event);

    //if (event == GLFW_CONNECTED)
    //{
    //    // The monitor was connected
    //}
    //else if (event == GLFW_DISCONNECTED)
    //{
    //    // The monitor was disconnected
    //}
}

void SofaGLFWBaseGUI::character_callback(GLFWwindow* window, unsigned int codepoint)
{
    SOFA_UNUSED(window);
    SOFA_UNUSED(codepoint);

    // The callback function receives Unicode code points for key events
    // that would have led to regular text input and generally behaves as a standard text field on that platform.
}

bool SofaGLFWBaseGUI::centerWindow(GLFWwindow* window)
{
    if (hasWindow())
    {
        // only manage the first window for now
        // and the main screen
        window = (!window) ? m_firstWindow : window;
    }

    int sx = 0, sy = 0;
    int px = 0, py = 0;
    int mx = 0, my = 0;
    int monitor_count = 0;
    int best_area = 0;
    int final_x = 0, final_y = 0;

    glfwGetWindowSize(window, &sx, &sy);
    glfwGetWindowPos(window, &px, &py);

    // Iterate throug all monitors
    GLFWmonitor** m = glfwGetMonitors(&monitor_count);
    if (!m)
        return false;

    for (int j = 0; j < monitor_count; ++j)
    {

        glfwGetMonitorPos(m[j], &mx, &my);
        const GLFWvidmode* mode = glfwGetVideoMode(m[j]);
        if (!mode)
            continue;

        // Get intersection of two rectangles - screen and window
        const int minX = std::max(mx, px);
        const int minY = std::max(my, py);

        const int maxX = std::min(mx + mode->width, px + sx);
        const int maxY = std::min(my + mode->height, py + sy);

        // Calculate area of the intersection
        const int area = std::max(maxX - minX, 0) * std::max(maxY - minY, 0);

        // If its bigger than actual (window covers more space on this monitor)
        if (area > best_area)
        {
            // Calculate proper position in this monitor
            final_x = mx + (mode->width - sx) / 2;
            final_y = my + (mode->height - sy) / 2;

            best_area = area;
        }

    }

    // We found something
    if (best_area)
        glfwSetWindowPos(window, final_x, final_y);

    // Something is wrong - current window has NOT any intersection with any monitors. Move it to the default one.
    else
    {
        GLFWmonitor* primary = glfwGetPrimaryMonitor();
        if (primary)
        {
            const GLFWvidmode* desktop = glfwGetVideoMode(primary);

            if (desktop)
                glfwSetWindowPos(window, (desktop->width - sx) / 2, (desktop->height - sy) / 2);
            else
                return false;
        }
        else
            return false;
    }

    return true;
}

} // namespace sofaglfw
