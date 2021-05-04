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
#include <sofa/config.h>

#ifdef SOFA_BUILD_SOFA_GLFW
#  define SOFA_TARGET @PROJECT_NAME@
#  define SOFA_GLFW_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define SOFA_GLFW_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#include <sofa/gl/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <sofa/simulation/fwd.h>
#include <sofa/gl/DrawToolGL.h>
#include <SofaBaseVisual/BaseCamera.h>
#include <sofa/gui/BaseGUI.h>

namespace sofa::glfw
{

class SofaGLFWWindow;

class SOFA_GLFW_API SofaGLFWGUI : public sofa::gui::BaseGUI
{
public:
    SofaGLFWGUI();
    virtual ~SofaGLFWGUI();

    bool init();
    void setErrorCallback();
    void setSimulation(sofa::simulation::NodeSPtr groot, const std::string& filename = std::string());
    void setSimulationIsRunning(bool running);
    bool simulationIsRunning();
    sofa::core::visual::DrawTool* getDrawTool();

    bool createWindow(int width, int height, const char* title);
    void destroyWindow();
    void initVisual();
    void runLoop();
    void terminate();

    // BaseGUI API
    int mainLoop();
    /// Update the GUI
    void redraw();
    /// Close the GUI
    int closeGUI();
    /// Register the scene in our GUI
    void setScene(sofa::simulation::NodeSPtr groot, const char* filename = nullptr, bool temporaryFile = false);
    /// Get the rootNode of the sofa scene
    sofa::simulation::Node* currentSimulation();

private:
    static void error_callback(int error, const char* description);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static void close_callback(GLFWwindow* window);

    sofa::component::visualmodel::BaseCamera::SPtr findCamera(sofa::simulation::NodeSPtr groot);
    void makeCurrentContext(GLFWwindow* sofaWindow);
    void runStep();

    // static members
    static std::map< GLFWwindow*, SofaGLFWWindow*> s_mapWindows;
    static std::map< GLFWwindow*, SofaGLFWGUI*> s_mapGUIs;

    //members 
    bool m_bGlfwIsInitialized;
    bool m_bGlewIsInitialized;

    sofa::simulation::NodeSPtr m_groot;
    std::string m_filename;
    sofa::gl::DrawToolGL m_glDrawTool;
    sofa::core::visual::VisualParams* m_vparams;
    GLFWwindow* m_firstWindow;

};

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