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
    std::pair<float, float> m_viewportWindowSize;
    bool isMouseOnViewport { false };
    CSimpleIniA ini;

    void showViewport(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameViewport, bool& isViewportWindowOpen);
    void showSceneGraph(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameSceneGraph,
                        bool& isSceneGraphWindowOpen, std::set<sofa::core::objectmodel::BaseObject*>& openedComponents,
                        std::set<sofa::core::objectmodel::BaseObject*>& focusedComponents);
    void showROS(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameROS, bool& isROSWindowOpen);

    static const std::string& getAppIniFile();

    void loadFile(sofaglfw::SofaGLFWBaseGUI* baseGUI, sofa::core::sptr<sofa::simulation::Node>& groot, std::string filePathName);
};

} // namespace sofaimgui
