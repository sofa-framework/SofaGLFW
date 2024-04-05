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

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/type/Quat.h>

#include <imgui_internal.h>

#include <SofaImGui/windows/MyRobotWindow.h>

namespace sofaimgui::windows {

MyRobotWindow::MyRobotWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
    m_isDrivingSimulation = true;
}

void MyRobotWindow::showWindow(const ImGuiWindowFlags &windowFlags)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            ImGui::Indent();

            if (ImGui::BeginTable("StateColumns", 3, ImGuiTableFlags_None))
            {
                ImGui::Spacing();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Information");
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
                ImGui::TableNextColumn();

                ImGui::BeginDisabled();
                for (auto &information: m_informations)
                {
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("%s: ", information.description.c_str());
                    ImGui::SameLine();

                    std::istringstream iss(information.data->getValueString());
                    std::vector<std::string> values;
                    copy(std::istream_iterator<std::string>(iss),
                         std::istream_iterator<std::string>(),
                         back_inserter(values));

                    std::string uiValue;
                    for (std::string v : values)
                    {
                        std::replace(v.begin(), v.end(), '.', ',');
                        double buffer = std::stod(v);
                        ImGui::PushItemWidth(ImGui::CalcTextSize("-10000,00").x);
                        ImGui::InputDouble(("##setting" + information.description).c_str(), &buffer, 0, 0, "%.2f");
                        uiValue += std::to_string(buffer) + " ";
                        ImGui::PopItemWidth();
                    }
                }
                ImGui::EndDisabled();

                ImGui::NewLine();

                ImGui::Spacing();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Settings");
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
                ImGui::TableNextColumn();

                for (auto &setting: m_settings)
                {
                    ImGui::AlignTextToFramePadding();
                    ImGui::Text("%s: ", setting.description.c_str());
                    ImGui::SameLine();

                    std::istringstream iss(setting.data->getValueString());
                    std::vector<std::string> values;
                    copy(std::istream_iterator<std::string>(iss),
                         std::istream_iterator<std::string>(),
                         back_inserter(values));

                    std::string uiValue;
                    for (std::string v : values)
                    {
                        std::replace(v.begin(), v.end(), '.', ',');
                        double buffer = std::stod(v);
                        ImGui::PushItemWidth(ImGui::CalcTextSize("-10000,00").x);
                        ImGui::InputDouble(("##setting" + setting.description).c_str(), &buffer, 0, 0, "%.2f");
                        uiValue += std::to_string(buffer) + " ";
                        ImGui::PopItemWidth();
                    }
                    setting.data->read(uiValue);
                }

                ImGui::EndTable();
            }

            ImGui::Unindent();
            ImGui::End();
        }
    }
}

}

