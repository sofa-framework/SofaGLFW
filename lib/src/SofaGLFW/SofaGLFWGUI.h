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
#include <sofa/gui/BaseGUI.h>

#include <SofaBaseVisual/BaseCamera.h>

namespace sofa::glfw
{

class SofaGLFWWindow;

class SOFA_GLFW_API SofaGLFWGUI : public sofa::gui::BaseGUI
{
public:
    SofaGLFWGUI();
    virtual ~SofaGLFWGUI();

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

    // BaseGUI API
    int mainLoop() override;
    /// Update the GUI
    void redraw() override;
    /// Close the GUI
    int closeGUI() override;
    /// Register the scene in our GUI
    void setScene(sofa::simulation::NodeSPtr groot, const char* filename = nullptr, bool temporaryFile = false) override;
    /// Get the rootNode of the sofa scene
    sofa::simulation::Node* currentSimulation() override;

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
    bool m_bGlfwIsInitialized = false;
    bool m_bGlewIsInitialized = false;

    sofa::simulation::NodeSPtr m_groot;
    std::string m_filename;
    sofa::gl::DrawToolGL* m_glDrawTool = nullptr;
    sofa::core::visual::VisualParams* m_vparams = nullptr;
    GLFWwindow* m_firstWindow = nullptr;

};

} // namespace sofa::glfw
