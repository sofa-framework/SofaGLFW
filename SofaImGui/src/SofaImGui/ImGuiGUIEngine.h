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
#include <SofaImGui/config.h>

#include <memory>
#include <SofaGLFW/BaseGUIEngine.h>
#include <sofa/gl/FrameBufferObject.h>

#include <imgui.h>
#include <sofa/simulation/Node.h>
#include <SimpleIni.h>

#include <SofaImGui/windows/WorkspaceWindow.h>
#include <SofaImGui/windows/ViewportWindow.h>
#include <SofaImGui/windows/SceneGraphWindow.h>
#include <SofaImGui/windows/IOWindow.h>
#include <SofaImGui/windows/MyRobotWindow.h>
#include <SofaImGui/windows/MoveWindow.h>
#include <SofaImGui/windows/ProgramWindow.h>

#include <SofaImGui/models/TCPTarget.h>


struct GLFWwindow;
namespace sofa::glfw
{
    class SofaGLFWBaseGUI;
}

namespace sofaimgui
{

class ImGuiGUIEngine : public sofaglfw::BaseGUIEngine
{
public:
    ImGuiGUIEngine() = default;
    ~ImGuiGUIEngine() = default;
    
    void init() override;
    void initBackend(GLFWwindow*) override;
    void startFrame(sofaglfw::SofaGLFWBaseGUI*) override;
    void endFrame() override {}
    void beforeDraw(GLFWwindow* window) override;
    void afterDraw() override;
    void terminate() override;
    bool dispatchMouseEvents() override;

    void animateBeginEvent(sofa::simulation::Node* groot) override;
    void animateEndEvent(sofa::simulation::Node* groot) override;

    void setTCPTarget(sofa::core::behavior::BaseMechanicalState::SPtr mechanical);

    // windows::WorkspaceWindow m_workspaceWindow = windows::WorkspaceWindow("Workspace", false);
    windows::ViewportWindow m_viewportWindow = windows::ViewportWindow("       Viewport", true);
    windows::SceneGraphWindow m_sceneGraphWindow = windows::SceneGraphWindow("       Scene Graph", false);
    windows::IOWindow m_IOWindow = windows::IOWindow("       Input/Output", true);
    windows::ProgramWindow m_programWindow = windows::ProgramWindow("       Program", true);
    windows::MyRobotWindow m_myRobotWindow = windows::MyRobotWindow("       My Robot", true);
    windows::MoveWindow m_moveWindow = windows::MoveWindow("       Move", true);

protected:
    std::unique_ptr<sofa::gl::FrameBufferObject> m_fbo;
    std::pair<unsigned int, unsigned int> m_currentFBOSize;

    CSimpleIniA ini;

    static const std::string& getAppIniFile();
    void showFrameOnViewport(sofaglfw::SofaGLFWBaseGUI *baseGUI);
    void initDockSpace();
    void showViewportWindow(sofaglfw::SofaGLFWBaseGUI* baseGUI);
    void showOptionWindows(sofaglfw::SofaGLFWBaseGUI* baseGUI);
    void showMainMenuBar(sofaglfw::SofaGLFWBaseGUI* baseGUI);
    void setNightLightStyle(const bool &nightStyle, sofaglfw::SofaGLFWBaseGUI* baseGUI=nullptr);

    std::shared_ptr<models::TCPTarget> m_TCPTarget;
    bool m_animate{false};
    int m_mode{0};
    bool m_nightStyle{false};
};

} // namespace sofaimgui
