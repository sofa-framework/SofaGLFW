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

#include <SofaImGui/windows/StateWindow.h>
#include <SofaImGui/widgets/Buttons.h>
#include <imgui_internal.h>


namespace sofaimgui::windows {

StateWindow::StateWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void StateWindow::setSimulationState(const models::SimulationState &simulationState)
{
    m_simulationStateData = simulationState.getStateData();
}

void StateWindow::showWindow()
{
    if (m_isWindowOpen && !m_simulationStateData.empty())
    {
        static bool openstate = true;
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f, 0.20f, 0.34f, 0.05f));
        if (ImGui::Begin("ViewportChildState", &openstate,
                         ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
        {
            if(ImGui::CollapsingHeader("Simulation State       "))
            {
                if (ImGui::BeginTable("StateColumns", 2, ImGuiTableFlags_None))
                {
                    std::string groups;
                    for (const auto& d: m_simulationStateData)
                    {
                        auto* typeinfo = d.data->getValueTypeInfo();
                        auto* values = d.data->getValueVoidPtr();
                        const std::string& description = d.description;
                        const std::string& group = d.group;
                        if (groups.find(group) == std::string::npos)
                        {
                            ImGui::TableNextColumn();
                            ImGui::AlignTextToFramePadding();
                            ImGui::Text("%s     ", group.c_str()); // Group title
                            groups += group + " ";
                            ImGui::TableNextColumn();
                            ImGui::AlignTextToFramePadding();
                            ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
                            ImGui::SameLine();
                        } else
                        {
                            ImGui::SameLine();
                        }

                        ImGui::BeginGroup();
                        {
                            ImGui::AlignTextToFramePadding();
                            ImGui::Text("%s ", description.c_str()); // Value description

                            ImGui::BeginDisabled();
                            for (size_t i=0; i<typeinfo->size(); i++) // Values
                            {
                                double buffer = typeinfo->getScalarValue(values, i);
                                ImGui::LocalInputDouble("##0", &buffer, 0, 0);
                                ImGui::SameLine();
                            }
                            ImGui::EndDisabled();
                        }
                        ImGui::EndGroup();
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::End();
        }
        ImGui::PopStyleColor();
    }
}

}

