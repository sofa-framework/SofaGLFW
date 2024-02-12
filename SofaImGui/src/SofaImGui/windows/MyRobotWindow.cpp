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
#include <IconsFontAwesome5.h>

#include <SofaImGui/windows/MyRobotWindow.h>

namespace sofaimgui::windows {

MyRobotWindow::MyRobotWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
    m_isDrivingSimulation = true;
}

void MyRobotWindow::showWindow(sofa::simulation::Node* groot)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, ImGuiWindowFlags_None))
        {
            ImGui::Indent();
            const auto& node = groot->getChild("UserInterface");
            if(node != nullptr)
            {
                bool unindent = false;
                const auto& data = node->getDataFields();
                std::string groups;
                std::string delimiter = "/";
                for(auto d: data)
                {
                    const std::string& value = d->getValueString();
                    std::string name = d->getName();
                    const std::string& group = d->getGroup();

                    if(group.find("myRobot") != std::string::npos)
                    {
                        std::string group = name.substr(0, name.find(delimiter));

                        if (groups.find(group) == std::string::npos)
                        {
                            ImGui::TableNextColumn();
                            ImGui::AlignTextToFramePadding();
                            ImGui::Text("%s:     ", group.c_str()); // Group title
                            groups += group + " ";
                            ImGui::TableNextColumn();
                            ImGui::AlignTextToFramePadding();
                        }
                        name.erase(0, name.find(delimiter) + delimiter.length());

                        ImGui::BeginGroup();
                        {
                            ImGui::AlignTextToFramePadding();
                            ImGui::Text("%s: ", name.c_str()); // Value name
                            ImGui::SameLine();

                            std::istringstream iss(value);
                            std::vector<std::string> values;
                            copy(std::istream_iterator<std::string>(iss),
                                 std::istream_iterator<std::string>(),
                                 back_inserter(values));

                            ImGui::BeginDisabled();
                            for (std::string v : values) // Values
                            {
                                std::replace(v.begin(), v.end(), '.', ',');
                                float buffer = std::stof(v);
                                ImGui::PushItemWidth(ImGui::CalcTextSize("-10000,00").x);
                                ImGui::InputFloat("##0", &buffer, 0, 0, "%.0f");
                                ImGui::SameLine();
                                ImGui::PopItemWidth();
                            }
                            ImGui::EndDisabled();
                        }
                        ImGui::EndGroup();
                    }
                }
            }

            ImGui::Unindent();
            ImGui::End();
        }
    }
}


}

