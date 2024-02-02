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

#include <SofaImGui/windows/ProgramWindow.h>
#include <SofaImGui/models/Action.h>
#include <SofaImGui/models/Trajectory.h>
#include <imgui_internal.h>
#include <GLFW/glfw3.h>

namespace sofaimgui::windows {

using sofa::type::Vec3;
using sofa::type::Quat;

ProgramWindow::ProgramWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void ProgramWindow::showWindow(sofa::simulation::Node* groot,
                             const ImGuiWindowFlags& windowFlags)
{

    RigidCoord wp0(Vec3(1000, 0, 0), Quat(0, 0, 0, 1));
    RigidCoord wp1(Vec3(1000, 50, 0), Quat(0, 0, 0, 1));
    RigidCoord wp2(Vec3(1000, 0, 30), Quat(0, 0, 0, 1));
    RigidCoord wp3(Vec3(1000, 0, 0), Quat(0, 0, 0, 1));
    RigidCoord wp4(Vec3(1000, -50, 0), Quat(0, 0, 0, 1));
    RigidCoord wp5(Vec3(1000, 0, -30), Quat(0, 0, 0, 1));

    auto t1 = std::make_shared<models::Trajectory>(wp0, wp1, 0, 5);
    auto t2 = std::make_shared<models::Trajectory>(wp1, wp2, 0, 3);
    auto t3 = std::make_shared<models::Trajectory>(wp2, wp3, 0, 2);
    auto t4 = std::make_shared<models::Trajectory>(wp3, wp4, 0, 5);
    auto t5 = std::make_shared<models::Trajectory>(wp4, wp5, 0, 4);

    m_program.clear();
    m_program.addAction(t1);
    m_program.addAction(t2);
    m_program.addAction(t3);
    m_program.addAction(t4);
    m_program.addAction(t5);

    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            float width = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() / 1.2;
            static float zoomCoef = 1;
            static float initSize = 100;
            float sSize = zoomCoef * initSize;

            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
            ImGui::BeginChildFrame(ImGui::GetID(m_name.c_str()), ImVec2(width, height),
                                   ImGuiWindowFlags_AlwaysHorizontalScrollbar);

            addTimeline(sSize);

            ImGui::NewLine();
            ImGui::Separator();

            ImGui::Text("");
            ImGui::SameLine();
            const auto& actions = m_program.getActions();
            ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.f, 0.f));
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(20.f, 20.f));
            for (std::shared_ptr<models::Action> action: actions)
            {
                float actionWidth = action->getDuration() * sSize - ImGui::GetStyle().ItemSpacing.x;
                float actionHeight = height - ImGui::GetTextLineHeightWithSpacing() * 3;
                ImGui::PushStyleColor(ImGuiCol_Button, action->getColor());
                ImGui::Button(action->getContentDescription(), ImVec2(actionWidth, actionHeight));
                ImGui::PopStyleColor();
                ImGui::SameLine();
            }
            ImGui::PopStyleVar(2);

            ImGui::EndChildFrame();
            ImGui::PopStyleColor();

            ImGui::SetItemUsingMouseWheel();
            if (ImGui::IsItemHovered() && !ImGui::IsKeyPressed(GLFW_KEY_LEFT_SHIFT) && ImGui::IsKeyPressed(GLFW_KEY_LEFT_CONTROL))
                zoomCoef += ImGui::GetIO().MouseWheel * 0.1f;
            zoomCoef = (zoomCoef < 1)? 1 : zoomCoef;
            zoomCoef = (zoomCoef > 4)? 4 : zoomCoef;
        }
        ImGui::End();
    }
}

void ProgramWindow::addTimeline(float sSize)
{
    float width = ImGui::GetWindowWidth() * 2;
    int nbSteps = width / sSize;

    for (int i=0 ; i<nbSteps; i++)
    {
        std::string text = std::to_string(i) + " s";
        float textSize = ImGui::CalcTextSize(text.c_str()).x;
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(sSize - textSize, 0.f));
        ImGui::Text("%s", text.c_str());
        ImGui::SameLine();
        ImGui::PopStyleVar();
    }

    ImGui::NewLine();
    ImGui::Text("");
    ImGui::SameLine();

    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(sSize / 10, 0.f));
    for (int i=0 ; i<nbSteps; i++)
    {
        for (int j=0; j<10; j++)
        {
            ImGui::PushStyleColor(ImGuiCol_Separator, (j==0) ? ImVec4(1.f, 1.f, 1.f, 1.f) : ImVec4(0.5f, 0.5f, 0.5f, 1.f));
            ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
            ImGui::PopStyleColor();
            ImGui::SameLine();
        }
    }
    ImGui::PopStyleVar();
}

} // namespace

