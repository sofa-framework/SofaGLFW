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
#include <SofaImGui/windows/StateWindow.h>

#if SOFAIMGUI_WITH_ROS == 1
#include <SofaImGui/windows/ROSWindow.h>
#endif

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

protected:
    std::unique_ptr<sofa::gl::FrameBufferObject> m_fbo;
    std::pair<unsigned int, unsigned int> m_currentFBOSize;

    windows::ViewportWindow m_viewportWindow = windows::ViewportWindow("Viewport", true);
    windows::SceneGraphWindow m_sceneGraphWindow = windows::SceneGraphWindow("Scene Graph", false);
    windows::StateWindow m_stateWindow = windows::StateWindow("State", true);

#if SOFAIMGUI_WITH_ROS == 1
    windows::ROSWindow m_ROSWindow = windows::ROSWindow("ROS", true);
#endif

    CSimpleIniA ini;

    void showSceneGraph(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameSceneGraph,
                        bool& isSceneGraphWindowOpen, std::set<sofa::core::objectmodel::BaseObject*>& openedComponents,
                        std::set<sofa::core::objectmodel::BaseObject*>& focusedComponents);
    void showROS(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameROS, bool& isROSWindowOpen);

    static const std::string& getAppIniFile();

    void loadFile(sofaglfw::SofaGLFWBaseGUI* baseGUI, sofa::core::sptr<sofa::simulation::Node>& groot, std::string filePathName);
};

} // namespace sofaimgui
