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
#include <sofa/gl/DrawToolGL.h>
#include <SofaBaseVisual/BaseCamera.h>

class GLFWwindow;

namespace sofa::glfw
{

class SofaGLFWWindow;

class SOFAGLFW_API SofaGLFWBaseGUI
{
public:
    SofaGLFWBaseGUI();
    virtual ~SofaGLFWBaseGUI();

    bool init();
    void setErrorCallback() const;
    void setSimulation(sofa::simulation::NodeSPtr groot, const std::string& filename = std::string());
    void setSimulationIsRunning(bool running);
    bool simulationIsRunning() const;

    bool createWindow(int width, int height, const char* title);
    void destroyWindow();
    void initVisual();
    void runLoop();
    void terminate();

    int getWindowWidth() { return m_windowWidth; }
    void setWindowWidth(int width) { m_windowWidth = width; }
    int getWindowHeight() { return m_windowHeight; }
    void setWindowHeight(int height) { m_windowHeight = height; }

    sofa::simulation::NodeSPtr getRootNode() { return m_groot; }
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
    static std::map< GLFWwindow*, SofaGLFWBaseGUI*> s_mapGUIs;

    //members 
    bool m_bGlfwIsInitialized = false;
    bool m_bGlewIsInitialized = false;

    sofa::simulation::NodeSPtr m_groot;
    std::string m_filename;
    sofa::gl::DrawToolGL* m_glDrawTool = nullptr;
    sofa::core::visual::VisualParams* m_vparams = nullptr;
    GLFWwindow* m_firstWindow = nullptr;
    int m_windowWidth = 0;
    int m_windowHeight = 0;

};

} // namespace sofa::glfw
