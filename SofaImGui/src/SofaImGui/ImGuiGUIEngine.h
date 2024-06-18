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

#include <SofaImGui/windows/ViewportWindow.h>
#include <SofaImGui/windows/SceneGraphWindow.h>
#include <SofaImGui/windows/IOWindow.h>
#include <SofaImGui/windows/MyRobotWindow.h>
#include <SofaImGui/windows/MoveWindow.h>
#include <SofaImGui/windows/PlottingWindow.h>
#include <SofaImGui/windows/ProgramWindow.h>
#include <SofaImGui/windows/DisplayFlags.h>

#include <SofaImGui/models/IPController.h>
#include <SofaImGui/models/SimulationState.h>
#include <SoftRobots.Inverse/component/solver/QPInverseProblemSolver.h>
#include <SoftRobots.Inverse/component/constraint/PositionEffector.h>


struct GLFWwindow;
namespace sofa::glfw
{
    class SofaGLFWBaseGUI;
}

namespace sofaimgui
{

class SOFAIMGUI_API ImGuiGUIEngine : public sofaglfw::BaseGUIEngine
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

    void setIPController(sofa::simulation::Node::SPtr groot,
                         softrobotsinverse::solver::QPInverseProblemSolver::SPtr solver,
                         sofa::core::behavior::BaseMechanicalState::SPtr TCPTargetMechanical,
                         sofa::core::behavior::BaseMechanicalState::SPtr TCPMechanical,
                         softrobotsinverse::constraint::PositionEffector<sofa::defaulttype::Rigid3Types>::SPtr rotationEffector);

    bool getRobotConnection() {return m_robotConnection;}
    models::SimulationState& getSimulationState() {return m_simulationState;}

    std::shared_ptr<windows::StateWindow> m_stateWindow = std::make_shared<windows::StateWindow>("State", true);
    windows::ViewportWindow m_viewportWindow = windows::ViewportWindow("       Viewport", true, m_stateWindow);
    windows::SceneGraphWindow m_sceneGraphWindow = windows::SceneGraphWindow("       Scene Graph", false);
    windows::IOWindow m_IOWindow = windows::IOWindow("       Input/Output", false);
    windows::ProgramWindow m_programWindow = windows::ProgramWindow("       Program", true);
    windows::PlottingWindow m_plottingWindow = windows::PlottingWindow("       Plotting", true);
    windows::MyRobotWindow m_myRobotWindow = windows::MyRobotWindow("       My Robot", true);
    windows::MoveWindow m_moveWindow = windows::MoveWindow("       Move", true);
    windows::DisplayFlagsWindow m_displayFlagsWindow = windows::DisplayFlagsWindow("       Display Options", false);

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
    
    models::IPController::SPtr m_IPController;
    models::SimulationState m_simulationState;
    bool m_animate{false};
    int m_mode{0};
    bool m_nightStyle{false};
    bool m_robotConnection{false};

};

} // namespace sofaimgui
