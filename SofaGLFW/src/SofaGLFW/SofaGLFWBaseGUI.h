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

#include <sofa/simulation/Simulation.h>
#include <sofa/gl/DrawToolGL.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/simulation/Node.h>

#include <SofaGLFW/BaseGUIEngine.h>
#include <SofaGLFW/NullGUIEngine.h>
#include <sofa/gui/common/BaseViewer.h>
#include <memory>

#include <SofaGLFW/SofaGLFWMouseManager.h>

struct GLFWwindow;
struct GLFWmonitor;
using namespace sofa::type;

namespace sofaglfw
{

class SofaGLFWWindow;
class SOFAGLFW_API SofaGLFWBaseGUI : public BaseViewer
{
public:

    SofaGLFWBaseGUI();

    virtual ~SofaGLFWBaseGUI();

    bool init(int nbMSAASamples = 0);
    void setErrorCallback() const;
    void setSimulation(sofa::simulation::NodeSPtr groot, const std::string& filename = std::string());
    void setSimulationIsRunning(bool running);
    bool simulationIsRunning() const;

    bool createWindow(int width, int height, const char* title, bool fullscreenAtStartup = false);
    void destroyWindow();
    void initVisual();
    std::size_t runLoop(std::size_t targetNbIterations = 0);
    void terminate();

    int getWindowWidth() const { return m_windowWidth; }
    void setWindowWidth(int width) { m_windowWidth = width; }
    int getWindowHeight() const { return m_windowHeight; }
    void setWindowHeight(int height) { m_windowHeight = height; }
    void resizeWindow(int width, int height);
    bool centerWindow(GLFWwindow* window = nullptr);
    void updateViewportPosition(float viewportPositionX, float viewportPositionY) ;

    GLFWmonitor* getCurrentMonitor(GLFWwindow *window);
    void viewAll() override;
    void saveView() override ;
    void setSizeW(int width) override;
    void setSizeH(int height) override;
    int getWidth() override;
    int getHeight() override;
    void drawScene() override ;
    void redraw() override;

    bool isFullScreen(GLFWwindow* glfwWindow = nullptr) const;
    void switchFullScreen(GLFWwindow* glfwWindow = nullptr, unsigned int screenID = 0);
    void setWindowBackgroundColor(const RGBAColor& newColor, unsigned int windowID = 0);
    void setWindowBackgroundImage(const std::string& imageFileName, unsigned int windowID = 0);
    virtual void setBackgroundColour(float r, float g, float b) override
    {
        setWindowBackgroundColor(RGBAColor{r, g, b, 1.0f}, 0);
    }
    virtual void setBackgroundImage(std::string imageFileName) override
    {
        setWindowBackgroundImage(imageFileName, 0);
    }

    sofa::core::sptr<Node> getRootNode() const;
    bool hasWindow() const { return m_firstWindow != nullptr; }

    [[nodiscard]] std::string getFilename() const { return m_filename; }

    sofa::component::visual::BaseCamera::SPtr findCamera(sofa::simulation::NodeSPtr groot);
    void changeCamera(sofa::component::visual::BaseCamera::SPtr newCamera);
    void restoreCamera(sofa::component::visual::BaseCamera::SPtr camera);
    constexpr std::string_view getCameraFileExtension() { return ".view"; }
    
    void setWindowIcon(GLFWwindow* glfwWindow);

    void setGUIEngine(std::shared_ptr<BaseGUIEngine> guiEngine)
    {
        m_guiEngine = guiEngine;
    }
    std::shared_ptr<BaseGUIEngine> getGUIEngine()
    {
        return m_guiEngine;
    }
    void moveRayPickInteractor(int eventX, int eventY) override ;

private:
    // GLFW callbacks
    static void error_callback(int error, const char* description);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static void close_callback(GLFWwindow* window);
    static void window_focus_callback(GLFWwindow* window, int focused);
    static void cursor_enter_callback(GLFWwindow* window, int entered);
    static void monitor_callback(GLFWmonitor* monitor, int event);
    static void character_callback(GLFWwindow* window, unsigned int codepoint);
    static void window_pos_callback(GLFWwindow* window, int xpos, int ypos);
    static int handleArrowKeys(int key);
    static void translateToViewportCoordinates (SofaGLFWBaseGUI* gui,double xpos, double ypos);

    void makeCurrentContext(GLFWwindow* sofaWindow);
    void runStep();

    inline static std::map<GLFWwindow*, SofaGLFWWindow*> s_mapWindows{};
    inline static std::map<GLFWwindow*, SofaGLFWBaseGUI*> s_mapGUIs{};
    inline static std::size_t s_numberOfActiveWindows = 0;

    bool m_bGlfwIsInitialized{ false };
    bool m_bGlewIsInitialized{ false };

    sofa::simulation::NodeSPtr m_groot;
    std::string m_filename;
    sofa::gl::DrawToolGL* m_glDrawTool{ nullptr };
    sofa::core::visual::VisualParams* m_vparams{ nullptr };
    GLFWwindow* m_firstWindow{ nullptr };
    int m_windowWidth{ 0 };
    int m_windowHeight{ 0 };
    int m_lastWindowPositionX{ 0 };
    int m_lastWindowPositionY{ 0 };
    int m_lastWindowWidth{ 0 };
    int m_lastWindowHeight{ 0 };
    SofaGLFWMouseManager m_sofaGLFWMouseManager;
    int m_viewPortHeight{0};
    int m_viewPortWidth {0};
    Vec2d m_translatedCursorPos;
    Vec2f m_viewPortPosition;
    Vec2f m_windowPosition;
    std::size_t m_backgroundID{0};

    std::shared_ptr<BaseGUIEngine> m_guiEngine;
};

} // namespace sofaglfw
