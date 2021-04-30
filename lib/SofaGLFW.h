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

namespace sofa::glfw
{

class SOFA_GLFW_API SofaGLFW
{
public:
    SofaGLFW();
    virtual ~SofaGLFW();

    static bool init();
    static void setErrorCallback();
    static void setSimulation(sofa::simulation::NodeSPtr groot);
    static sofa::core::visual::DrawTool* getDrawTool();

    bool createWindow(int width, int height, const char* title);
    void destroyWindow();
    void makeCurrentContext();
    void initVisual();
    void runLoop();

    void terminate();

private:
    static void error_callback(int error, const char* description);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    void runStep();
    void draw();
    void mouseMoveEvent(int xpos, int ypos);
    void mouseButtonEvent(int button, int action, int mods);
    void scrollEvent(double xoffset, double yoffset);

    //static members
    static bool s_glfwIsInitialized;
    static bool s_glewIsInitialized;
    static sofa::simulation::NodeSPtr s_groot;
    static sofa::gl::DrawToolGL s_glDrawTool;
    static unsigned int s_nbInstances;
    static std::map< GLFWwindow*, SofaGLFW*> s_mapWindows;
    
    //members 
    GLFWwindow* m_glfwWindow;
    sofa::core::visual::VisualParams* m_vparams;
    sofa::component::visualmodel::BaseCamera::SPtr m_currentCamera;
    int m_currentButton = -1;
    int m_currentAction = -1;
    int m_currentMods = -1;
    int m_currentXPos = -1;
    int m_currentYPos = -1;

};

} // namespace sofa::glfw