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
#include <SofaGLFW/config.h>

#include <SofaGLFW/SofaGLFWMouseManager.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/io/STBImage.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/FileSystem.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/BaseClassNameHelper.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/SimulationLoop.h>
#include <sofa/component/visual/InteractiveCamera.h>
#include <sofa/component/visual/VisualStyle.h>
#include <sofa/gui/common/BaseViewer.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/gui/common/PickHandler.h>

#include <sofa/helper/ScopedAdvancedTimer.h>

#include <algorithm>

using namespace sofa;
using namespace sofa::gui::common;

using std::endl;
using namespace sofa::type;
using namespace sofa::defaulttype;
using namespace sofa::gl;
using simulation::getSimulation;
using namespace sofa::simulation;
using namespace sofa::gui::common;
using namespace  core::visual;
using namespace component::visual;
using namespace core::objectmodel;

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

core::sptr<Node> SofaGLFWBaseGUI::getRootNode() const
{
    return this->groot;
}

bool SofaGLFWBaseGUI::init(int nbMSAASamples)
{
    if (m_bGlfwIsInitialized)
        return true;

    setErrorCallback();

    // on macOS, glfwInit change the current working directory...
    // giving this hint avoids doing the change
#if defined(__APPLE__)
    glfwInitHint(GLFW_COCOA_CHDIR_RESOURCES, GLFW_FALSE);
#endif

    // Wayland is not fully supported in GLFW
    // this will force using X11 on wayland (XWayland)
#if defined(SOFAGLFW_USEX11_INTERNAL)
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif

    if (glfwInit() == GLFW_TRUE)
    {
        // defined samples for MSAA
        // min = 0  (no MSAA Anti-aliasing)
        // max = 32 (MSAA with 32 samples)
        glfwWindowHint(GLFW_SAMPLES, std::clamp(nbMSAASamples, 0, 32) );

        m_glDrawTool = new DrawToolGL();
        m_bGlfwIsInitialized = true;
        return true;
    }
    else
    {
        msg_error("SofaGLFWBaseGUI") << "Cannot initialize GLFW";
        return false;
    }
}

void SofaGLFWBaseGUI::setErrorCallback() const
{
    glfwSetErrorCallback(error_callback);
}

void SofaGLFWBaseGUI::setSimulation(NodeSPtr groot, const std::string& filename)
{
    this->groot = groot;
    this->sceneFileName = filename;

    VisualParams::defaultInstance()->drawTool() = m_glDrawTool;
    sofa::core::visual::VisualParams::defaultInstance()->setSupported(sofa::core::visual::API_OpenGL);

    if (this->groot)
    {
        // Initialize the pick handler
        this->pick->init(this->groot.get());
        m_sofaGLFWMouseManager.setPickHandler(getPickHandler());
    }
}

void SofaGLFWBaseGUI::setSimulationIsRunning(bool running)
{
    if (this->groot)
    {
        this->groot->setAnimate(running);
    }
}


bool SofaGLFWBaseGUI::simulationIsRunning() const
{
    if (this->groot)
    {
        return this->groot->getAnimate();
    }

    return false;
}

void SofaGLFWBaseGUI::setSizeW(int width)
{
    m_windowWidth = width;
}

void SofaGLFWBaseGUI::setSizeH(int height)
{
    m_windowHeight = height;
}

int SofaGLFWBaseGUI::getWidth()
{
    return m_windowWidth;
}

int SofaGLFWBaseGUI::getHeight()
{
    return m_windowHeight;
}

void SofaGLFWBaseGUI::redraw()
{
}

void SofaGLFWBaseGUI::drawScene()
{
}

void SofaGLFWBaseGUI::viewAll()
{
}

void SofaGLFWBaseGUI::saveView()
{
}

void SofaGLFWBaseGUI::changeCamera(BaseCamera::SPtr newCamera)
{
    for (auto& w : s_mapWindows)
    {
        w.second->setCamera(newCamera);
    }
}

void SofaGLFWBaseGUI::restoreCamera(BaseCamera::SPtr camera)
{
    if (camera)
    {
        const std::string viewFileName = this->getSceneFileName() + std::string(this->getCameraFileExtension());
        if (helper::system::FileSystem::isFile(viewFileName))
        {
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
}

void SofaGLFWBaseGUI::setWindowIcon(GLFWwindow* glfwWindow)
{
    //STBImage relies on DataRepository to find files: it must be extended with the resource files from this plugin
    helper::system::DataRepository.addFirstPath(SOFAGLFW_RESOURCES_DIR);

    helper::io::STBImage img;
    if (img.load("SOFA.png"))
    {
        GLFWimage images[1];
        images[0].height = img.getHeight();
        images[0].width = img.getWidth();
        images[0].pixels = img.getPixels();
        glfwSetWindowIcon(glfwWindow, 1, images);
    }
    helper::system::DataRepository.removePath(SOFAGLFW_RESOURCES_DIR);
}

bool SofaGLFWBaseGUI::createWindow(int width, int height, const char* title, bool fullscreenAtStartup)
{
    m_guiEngine->init();

    if (this->groot == nullptr)
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

        setWindowHeight(height);
        setWindowWidth(width);

        glfwWindow = glfwCreateWindow(mode->width, mode->height, title, primaryMonitor, m_firstWindow);
    }
    else
    {
        glfwWindow = glfwCreateWindow(width > 0 ? width : 100, height > 0 ? height : 100, title, nullptr, m_firstWindow);
    }
    assert(glfwWindow);
    s_numberOfActiveWindows++;

#ifndef __APPLE__ // Apple implies Cocoa and Cocoa does not support icon for the window
    {
        setWindowIcon(glfwWindow);
    }
#endif
    
    if (!m_firstWindow)
        m_firstWindow = glfwWindow;

    if (glfwWindow)
    {
        glfwSetKeyCallback(glfwWindow, key_callback);
        glfwSetCursorPosCallback(glfwWindow, cursor_position_callback);
        glfwSetMouseButtonCallback(glfwWindow, mouse_button_callback);
        glfwSetScrollCallback(glfwWindow, scroll_callback);
        glfwSetWindowCloseCallback(glfwWindow, close_callback);
        glfwSetWindowPosCallback(glfwWindow, window_pos_callback);
        // this set empty callbacks
        // solve a crash when glfw is quitting and tries to use nullptr callbacks
        // could be potentially useful in the future anyway
        glfwSetWindowFocusCallback(glfwWindow, window_focus_callback);
        glfwSetCursorEnterCallback(glfwWindow, cursor_enter_callback);
        glfwSetMonitorCallback(monitor_callback);
        glfwSetCharCallback(glfwWindow, character_callback);

        glfwSetWindowUserPointer(glfwWindow, this);

        makeCurrentContext(glfwWindow);

        m_guiEngine->initBackend(glfwWindow);

        SofaGLFWWindow* sofaWindow = new SofaGLFWWindow(glfwWindow, this->currentCamera);

        s_mapWindows[glfwWindow] = sofaWindow;
        s_mapGUIs[glfwWindow] = this;

        return true;
    }

    return false;

}
void SofaGLFWBaseGUI::updateViewportPosition(const float viewportPositionX, const float viewportPositionY)
{
    m_viewPortPosition = { viewportPositionX, viewportPositionY };
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

void SofaGLFWBaseGUI::setWindowBackgroundColor(const RGBAColor& newColor, unsigned int /* windowID */)
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


void SofaGLFWBaseGUI::setWindowBackgroundImage(const std::string& filename, unsigned int /* windowID */)
{
    if (hasWindow())
    {
        s_mapWindows[m_firstWindow]->setBackgroundImage(filename);
    }
    else
    {
        msg_error("SofaGLFWBaseGUI") << "No window to set the background in";// can happen with runSofa/BaseGUI
    }
}

void SofaGLFWBaseGUI::setWindowTitle(GLFWwindow* window, const char* title)
{
    if(hasWindow())
    {
        auto* glfwWindow = (window) ? window : m_firstWindow ;
        glfwSetWindowTitle(glfwWindow, title);
    }
    else
    {
        msg_error("SofaGLFWBaseGUI") << "No window to set the title on";// can happen with runSofa/BaseGUI
    }
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
    if (!this->groot)
    {
        msg_error("SofaGLFWBaseGUI") << "Cannot start main loop: root node is invalid";
        return 0;
    }

    m_vparams = VisualParams::defaultInstance();
    m_viewPortWidth = m_vparams->viewport()[2];
    m_viewPortHeight = m_vparams->viewport()[3];

    bool running = true;
    std::size_t currentNbIterations = 0;
    std::stringstream tmpStr;
    while (s_numberOfActiveWindows > 0 && running)
    {
        SIMULATION_LOOP_SCOPE

        // Keep running
        runStep();
        sofa::type::vector<std::pair<GLFWwindow*, SofaGLFWWindow*>> closedWindows;
        
        for (auto& [glfwWindow, sofaGlfwWindow] : s_mapWindows)
        {
            if (glfwWindow && sofaGlfwWindow)
            {
                // while user did not request to close this window (i.e press escape), draw
                if (!glfwWindowShouldClose(glfwWindow) && !m_guiEngine->isTerminated())
                {
                    makeCurrentContext(glfwWindow);
                    
                    m_guiEngine->beforeDraw(glfwWindow);
                    sofaGlfwWindow->draw(this->groot, m_vparams);

                    drawSelection(m_vparams);

                    m_guiEngine->afterDraw();

                    m_guiEngine->startFrame(this);
                    m_guiEngine->endFrame();

                    glfwSwapBuffers(glfwWindow);


                    m_viewPortHeight = m_vparams->viewport()[3];
                    m_viewPortWidth = m_vparams->viewport()[2];
                }
                else
                {
                    // otherwise close this window
                    closedWindows.emplace_back(glfwWindow, sofaGlfwWindow);
                }
            }
        }

        glfwPollEvents();

        // the engine must be terminated before the window
        if (s_numberOfActiveWindows == closedWindows.size())
        {
            // could be not necessary if m_guiEngine already terminated but we may need it if GLFW closed itself. (typically escape key)
            m_guiEngine->terminate();
            m_guiEngine.reset();
        }

        for (auto& [glfwWindow, sofaGlfwWindow] : closedWindows)
        {
            sofaGlfwWindow->close();

            auto currentSofaWindow = s_mapWindows.find(glfwWindow);
            if (currentSofaWindow != s_mapWindows.end())
            {
                s_numberOfActiveWindows--;
                s_mapWindows.erase(currentSofaWindow);
            }
        }

        currentNbIterations++;
        running = (targetNbIterations > 0) ? currentNbIterations < targetNbIterations : true;
    }

    return currentNbIterations;
}

void SofaGLFWBaseGUI::initVisual()
{
    node::initTextures(this->groot.get());

    VisualStyle::SPtr visualStyle = nullptr;
    this->groot->get(visualStyle);
    if (!visualStyle)
    {
        visualStyle = sofa::core::objectmodel::New<VisualStyle>();
        visualStyle->setName(sofa::core::objectmodel::BaseClassNameHelper::getShortName<decltype(visualStyle.get())>());

        DisplayFlags* displayFlags = visualStyle->d_displayFlags.beginEdit();
        displayFlags->setShowVisualModels(tristate::true_value);
        visualStyle->d_displayFlags.endEdit();

        this->groot->addObject(visualStyle);
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

    m_vparams = VisualParams::defaultInstance();
    for (auto& [glfwWindow, sofaGlfwWindow] : s_mapWindows)
    {
        sofaGlfwWindow->centerCamera(this->groot, m_vparams);
    }
    
    setWindowBackgroundImage("textures/SOFA_logo.bmp", 0);
}

void SofaGLFWBaseGUI::runStep()
{
    if(simulationIsRunning())
    {
        SCOPED_TIMER("Animate");

        node::animate(this->groot.get(), this->groot->getDt());
        node::updateVisual(this->groot.get());
    }
}

void SofaGLFWBaseGUI::terminate()
{
    if (!m_bGlfwIsInitialized)
        return;

    if (m_guiEngine)
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
    SOFA_UNUSED(scancode);

    const char keyName = handleArrowKeys(key);
    const bool isCtrlKeyPressed = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS;

    const bool foundGUI = s_mapGUIs.contains(window);
    if (!foundGUI)
    {
        return;
    }
    auto currentGUI = s_mapGUIs[window];
    if (!currentGUI)
    {
        return;
    }

    auto rootNode = currentGUI->getRootNode();
    if (!rootNode)
    {
        return;
    }

    if (isCtrlKeyPressed)
    {
        if (action == GLFW_PRESS)
        {
            KeypressedEvent keyPressedEvent(keyName);
            rootNode->propagateEvent(core::ExecParams::defaultInstance(), &keyPressedEvent);
        }
        else if (action == GLFW_RELEASE)
        {
            KeyreleasedEvent keyReleasedEvent(keyName);
            rootNode->propagateEvent(core::ExecParams::defaultInstance(), &keyReleasedEvent);
        }
    }

    // Handle specific keys for additional functionality
    switch (key)
    {
        case GLFW_KEY_B:
            if (action == GLFW_PRESS)
            {
                currentGUI->m_backgroundID = (currentGUI->m_backgroundID + 1) % 4;
                switch (currentGUI->m_backgroundID)
                {
                    case 0:
                        currentGUI->setWindowBackgroundImage("textures/SOFA_logo.bmp", 0);
                        break;
                    case 1:
                        currentGUI->setWindowBackgroundImage("textures/SOFA_logo_white.bmp", 0);
                        break;
                    case 2:
                        currentGUI->setWindowBackgroundColor(sofa::type::RGBAColor::black());
                        break;
                    case 3:
                        currentGUI->setWindowBackgroundColor(sofa::type::RGBAColor::white());
                        break;
                }
                break;
            }
            break;
        case GLFW_KEY_F:
            if (action == GLFW_PRESS && (mods & GLFW_MOD_CONTROL))
            {
                currentGUI->switchFullScreen(window);
            }
            break;
        case GLFW_KEY_F11:
            if (action == GLFW_PRESS)
            {
                currentGUI->switchFullScreen(window);
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
                const bool isRunning = currentGUI->simulationIsRunning();
                currentGUI->setSimulationIsRunning(!isRunning);
            }
            break;
        case GLFW_KEY_LEFT_SHIFT:
            if (currentGUI->getPickHandler())
            {
                if (action == GLFW_PRESS)
                {
                    currentGUI->getPickHandler()->activateRay(0, 0, rootNode.get());
                }
                else if (action == GLFW_RELEASE)
                {
                    currentGUI->getPickHandler()->deactivateRay();
                }
            }
            break;
        case GLFW_KEY_R:
            if (action == GLFW_PRESS && isCtrlKeyPressed)
            {
                // Reload using CTRL + R
                sofa::simulation::NodeSPtr groot = currentGUI->groot;
                std::string filename = currentGUI->getSceneFileName();

                if (!filename.empty() && helper::system::FileSystem::exists(filename))
                {
                    msg_info("GUI") << "Reloading file " << filename;
                    sofa::simulation::node::unload(groot);

                    groot = sofa::simulation::node::load(filename.c_str());
                    if( !groot )
                        groot = sofa::simulation::getSimulation()->createNewGraph("");

                    currentGUI->setSimulation(groot, filename);
                    currentGUI->load();
                    currentGUI->setWindowTitle(nullptr, std::string("SOFA - " + filename).c_str());

                    sofa::simulation::node::initRoot(groot.get());
                    if (currentGUI->currentCamera)
                    {
                        currentGUI->currentCamera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
                        currentGUI->changeCamera(currentGUI->currentCamera);
                    }

                    node::initTextures(groot.get());

                    currentGUI->m_guiEngine->resetCounter();

                    // update camera if a sidecar file is present
                    currentGUI->restoreCamera(currentGUI->currentCamera);
                }
            }


        default:
            break;
    }
}

void SofaGLFWBaseGUI::moveRayPickInteractor(int eventX, int eventY)
{
    const VisualParams::Viewport& viewport = m_vparams->viewport();

    double lastProjectionMatrix[16];
    double lastModelviewMatrix[16];

    m_vparams->getProjectionMatrix(lastProjectionMatrix);
    m_vparams->getModelViewMatrix(lastModelviewMatrix);

    Vec3d p0;
    Vec3d px;
    Vec3d py;
    Vec3d pz;
    Vec3d px1;
    Vec3d py1;
    gluUnProject(eventX,   viewport[3]-1-(eventY),   0,   lastModelviewMatrix, lastProjectionMatrix, viewport.data(), &(p0[0]),  &(p0[1]),  &(p0[2]));
    gluUnProject(eventX+1, viewport[3]-1-(eventY),   0,   lastModelviewMatrix, lastProjectionMatrix, viewport.data(), &(px[0]),  &(px[1]),  &(px[2]));
    gluUnProject(eventX,   viewport[3]-1-(eventY+1), 0,   lastModelviewMatrix, lastProjectionMatrix, viewport.data(), &(py[0]),  &(py[1]),  &(py[2]));
    gluUnProject(eventX,   viewport[3]-1-(eventY),   0.1, lastModelviewMatrix, lastProjectionMatrix, viewport.data(), &(pz[0]),  &(pz[1]),  &(pz[2]));
    gluUnProject(eventX+1, viewport[3]-1-(eventY),   0.1, lastModelviewMatrix, lastProjectionMatrix, viewport.data(), &(px1[0]), &(px1[1]), &(px1[2]));
    gluUnProject(eventX,   viewport[3]-1-(eventY+1), 0,   lastModelviewMatrix, lastProjectionMatrix, viewport.data(), &(py1[0]), &(py1[1]), &(py1[2]));

    px1 -= pz;
    py1 -= pz;
    px -= p0;
    py -= p0;
    pz -= p0;

    const double r0 = sqrt(px.norm2() + py.norm2());
    double r1 = sqrt(px1.norm2() + py1.norm2());
    r1 = r0 + (r1 - r0) / pz.norm();
    px.normalize();
    py.normalize();
    pz.normalize();

    Mat4x4d transform;
    transform.identity();
    transform[0][0] = px[0];
    transform[1][0] = px[1];
    transform[2][0] = px[2];
    transform[0][1] = py[0];
    transform[1][1] = py[1];
    transform[2][1] = py[2];
    transform[0][2] = pz[0];
    transform[1][2] = pz[1];
    transform[2][2] = pz[2];
    transform[0][3] = p0[0];
    transform[1][3] = p0[1];
    transform[2][3] = p0[2];

    Mat3x3d mat;
    mat = transform;
    Quat<SReal> q;
    q.fromMatrix(mat);

    Vec3d position, direction;
    position = transform * Vec4d(0, 0, 0, 1);
    direction = transform * Vec4d(0, 0, 1, 0);
    direction.normalize();
    getPickHandler()->updateRay(position, direction);
}

void SofaGLFWBaseGUI::window_pos_callback(GLFWwindow* window, int xpos, int ypos)
{
    SofaGLFWBaseGUI* gui = static_cast<SofaGLFWBaseGUI*>(glfwGetWindowUserPointer(window));
    gui->m_windowPosition[0] = static_cast<float>(xpos);
    gui->m_windowPosition[1] = static_cast<float>(ypos);
}

void SofaGLFWBaseGUI::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    auto currentGUI = s_mapGUIs.find(window);
    if (currentGUI == s_mapGUIs.end() || !currentGUI->second) {
        return;
    }

    SofaGLFWBaseGUI* gui = currentGUI->second;

    auto rootNode = currentGUI->second->getRootNode();
    if (!rootNode) {
        return;
    }

    const bool shiftPressed = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    if (shiftPressed )
    {
        // Check if the animation is running
        if (!currentGUI->second->simulationIsRunning())
        {
            msg_info("SofaGLFWBaseGUI") << "Animation is not running. Ignoring mouse interaction.";
            return;
        }

        const auto currentSofaWindow = s_mapWindows.find(window);
        if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
        {
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            translateToViewportCoordinates(gui,xpos,ypos);

            currentSofaWindow->second->mouseEvent(
                window, gui->m_viewPortWidth, gui->m_viewPortHeight, button,
                action, mods,
                gui->m_translatedCursorPos[0],
                gui->m_translatedCursorPos[1]);
        }
    }
    else
    {
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
}

void SofaGLFWBaseGUI::translateToViewportCoordinates (SofaGLFWBaseGUI* gui,double xpos, double ypos)
{
    gui->m_translatedCursorPos = Vec2d{xpos, ypos} - (gui->m_viewPortPosition - gui->m_windowPosition);
}

void SofaGLFWBaseGUI::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    auto currentGUI = s_mapGUIs.find(window);

    if (currentGUI != s_mapGUIs.end() && currentGUI->second)
    {
        if (!currentGUI->second->getGUIEngine()->dispatchMouseEvents())
            return;
    }
    SofaGLFWBaseGUI* gui = currentGUI->second;

    translateToViewportCoordinates(gui,xpos,ypos);

    bool shiftPressed = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;


    auto currentSofaWindow = s_mapWindows.find(window);

    if (shiftPressed)
    {
        for (const auto button : {GLFW_MOUSE_BUTTON_LEFT, GLFW_MOUSE_BUTTON_MIDDLE, GLFW_MOUSE_BUTTON_RIGHT})
        {
            if (glfwGetMouseButton(window, button) == GLFW_PRESS)
            {
                currentSofaWindow->second->mouseEvent(window,gui->m_viewPortWidth,gui->m_viewPortHeight, button, 1, 1, gui->m_translatedCursorPos[0], gui->m_translatedCursorPos[1]);
            }
        }
    }

    if (currentSofaWindow != s_mapWindows.end() && currentSofaWindow->second)
    {
        currentSofaWindow->second->mouseMoveEvent(static_cast<int>(xpos), static_cast<int>(ypos), currentGUI->second);
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
    SOFA_UNUSED(window);
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
    if (window == nullptr)
    {
            return false;
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
