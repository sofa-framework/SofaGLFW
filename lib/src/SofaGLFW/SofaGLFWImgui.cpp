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
#include <ostream>
#include <SofaGLFW/SofaGLFWImgui.h>

#include <SofaGLFW/config.h>

#if SOFAGLFW_HAS_IMGUI
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <sofa/helper/vector.h>
#include <sofa/simulation/Node.h>
#include <SofaBaseVisual/VisualStyle.h>
#endif

namespace sofa::glfw::imgui
{

void imguiInit()
{
#if SOFAGLFW_HAS_IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
#endif
}

void imguiInitBackend(GLFWwindow* glfwWindow)
{
#if SOFAGLFW_HAS_IMGUI
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(glfwWindow, true);
    ImGui_ImplOpenGL3_Init(nullptr);
#endif
}

void imguiDraw(sofa::simulation::NodeSPtr groot)
{
#if SOFAGLFW_HAS_IMGUI
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    static bool isControlsWindowOpen = true;
    static bool isPerformancesWindowOpen = false;
    static bool isSceneGraphWindowOpen = false;
    static bool isDisplayFlagsWindowOpen = false;

    ImVec2 mainMenuBarSize;

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("Windows"))
        {
            ImGui::Checkbox("Controls", &isControlsWindowOpen);
            ImGui::Checkbox("Performances", &isPerformancesWindowOpen);
            ImGui::Checkbox("Scene Graph", &isSceneGraphWindowOpen);
            ImGui::Checkbox("Display Flags", &isDisplayFlagsWindowOpen);
            ImGui::EndMenu();
        }
        mainMenuBarSize = ImGui::GetWindowSize();
        ImGui::EndMainMenuBar();
    }

    if (isControlsWindowOpen)
    {
        ImGui::SetNextWindowPos(ImVec2(0, mainMenuBarSize.y), ImGuiCond_Once);
        ImGui::Begin("Controls", &isControlsWindowOpen);

        auto& animate = sofa::helper::getWriteAccessor(groot->animate_).wref();
        ImGui::Checkbox("Animate", &animate);

        if (ImGui::Button("Reset"))
        {
            groot->setTime(0.);
            simulation::getSimulation()->reset ( groot.get() );
        }

        ImGui::End();
    }

    if (isPerformancesWindowOpen)
    {
        static sofa::helper::vector<float> msArray;
        ImGui::SetNextWindowPos(ImVec2(0,400), ImGuiCond_Once);
        if (ImGui::Begin("Performances", &isPerformancesWindowOpen))
        {
            const ImGuiIO& io = ImGui::GetIO();
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Text("%d vertices, %d indices (%d triangles)", io.MetricsRenderVertices, io.MetricsRenderIndices, io.MetricsRenderIndices / 3);
            ImGui::Text("%d visible windows, %d active allocations", io.MetricsRenderWindows, io.MetricsActiveAllocations);

            msArray.push_back(1000.0f / io.Framerate);
            if (msArray.size() >= 2000)
            {
                msArray.erase(msArray.begin());
            }
            ImGui::PlotLines("Frame Times", msArray.data(), msArray.size(), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 100));
        }
        ImGui::End();
    }

    if (isSceneGraphWindowOpen)
    {
        ImGui::Begin("Scene Graph", &isSceneGraphWindowOpen);

        unsigned int treeDepth {};
        static core::objectmodel::BaseObject* clickedObject { nullptr };

        std::function<void(simulation::Node*)> showNode;
        showNode = [&showNode, &treeDepth](simulation::Node* node)
        {
            if (node == nullptr) return;
            if (treeDepth == 0)
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            const bool open = ImGui::TreeNode(node->getName().c_str());
            ImGui::TableNextColumn();
            ImGui::TextDisabled("Node");
            if (open)
            {
                ++treeDepth;
                for (const auto child : node->getChildren())
                {
                    showNode(dynamic_cast<simulation::Node*>(child));
                }

                for (const auto object : node->getNodeObjects())
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TreeNodeEx(object->getName().c_str(), ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanFullWidth);
                    if (ImGui::IsItemClicked())
                        clickedObject = object;
                    ImGui::TableNextColumn();
                    ImGui::Text(object->getClassName().c_str());
                }

                --treeDepth;
                ImGui::TreePop();
            }
        };

        static ImGuiTableFlags flags = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody;
        if (ImGui::BeginTable("sceneGraphTable", 2, flags))
        {
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_NoHide);
            ImGui::TableSetupColumn("Class Name", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("A").x * 12.0f);
            ImGui::TableHeadersRow();

            showNode(groot.get());

            ImGui::EndTable();
        }

        static bool areDataDisplayed;
        areDataDisplayed = clickedObject != nullptr;
        if (clickedObject != nullptr)
        {
            ImGui::Separator();
            if (ImGui::CollapsingHeader(clickedObject->getName().c_str(), &areDataDisplayed))
            {
                for (const auto* data : clickedObject->getDataFields())
                {
                    if (ImGui::CollapsingHeader(data->m_name.c_str()))
                    {
                        ImGui::TextWrapped(data->getValueString().c_str());
                    }
                }
            }
            if (!areDataDisplayed)
            {
                clickedObject = nullptr;
            }
        }

        ImGui::End();
    }

    if (isDisplayFlagsWindowOpen)
    {
        ImGui::Begin("Display Flags", &isDisplayFlagsWindowOpen);

        component::visualmodel::VisualStyle::SPtr visualStyle = nullptr;
        groot->get(visualStyle);
        if (visualStyle)
        {
            auto& displayFlags = sofa::helper::getWriteAccessor(visualStyle->displayFlags).wref();

            {
                const bool initialValue = displayFlags.getShowVisualModels();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Visual Models", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowVisualModels(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowBehaviorModels();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Behavior Models", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowBehaviorModels(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowForceFields();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Force Fields", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowForceFields(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowCollisionModels();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Collision Models", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowCollisionModels(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowBoundingCollisionModels();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Bounding Collision Models", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowBoundingCollisionModels(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowMappings();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Mappings", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowMappings(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowMechanicalMappings();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Mechanical Mappings", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowMechanicalMappings(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowWireFrame();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Wire Frame", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowWireFrame(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowNormals();
                bool changeableValue = initialValue;
                ImGui::Checkbox("Show Normals", &changeableValue);
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowNormals(changeableValue);
                }
            }

        }

        ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


#endif
}

void imguiTerminate()
{
#if SOFAGLFW_HAS_IMGUI
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
#endif
}

bool dispatchMouseEvents()
{
#if SOFAGLFW_HAS_IMGUI
    return !ImGui::GetIO().WantCaptureMouse;
#else
    return true;
#endif

}
} //namespace sofa::glfw::imgui