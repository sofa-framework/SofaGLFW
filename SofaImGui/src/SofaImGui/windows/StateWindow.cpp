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
#include <imgui_internal.h>


namespace sofaimgui::windows {

StateWindow::StateWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void StateWindow::showWindow(sofa::simulation::Node* groot)
{
    if (m_isWindowOpen)
    {
        const auto& node = groot->getChild("UserInterface");
        if(node != nullptr)
        {
            bool unindent = false;

            static bool openstate = true;
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetColorU32(ImVec4(0.f, 0.f, 0.f, 0.2f)));
            if (ImGui::Begin("ViewportChildState", &openstate,
                             ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
            {
                if(ImGui::CollapsingHeader("State           ")) // TODO fit to content
                {
                    const auto& data = node->getDataFields();
                    std::string groups;
                    std::string delimiter = "/";
                    for(auto d: data)
                    {
                        const std::string& value = d->getValueString();
                        std::string name = d->getName();
                        const std::string& group = d->getGroup();
                        if(group.find("state") != std::string::npos)
                        {
                            std::string group = name.substr(0, name.find(delimiter));

                            if (groups.find(group) == std::string::npos)
                            {
                                if (unindent)
                                {
                                    ImGui::Unindent();
                                }

                                ImGui::Text("%s:", group.c_str()); // Group title
                                groups += group + " ";

                                ImGui::Indent();
                                unindent = true;
                            } else {
                                ImGui::SameLine();
                            }
                            name.erase(0, name.find(delimiter) + delimiter.length());

                            ImGui::BeginGroup();
                            {
                                ImGui::AlignTextToFramePadding();
                                ImGui::Text("%s ", name.c_str()); // Value name

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
                                    ImGui::InputFloat("##0", &buffer, 0, 0, "%.2f");
                                    ImGui::SameLine();
                                    ImGui::PopItemWidth();
                                }
                                ImGui::EndDisabled();
                            }
                            ImGui::EndGroup();
                        }
                    }
                    if (unindent)
                    {
                        ImGui::Unindent();
                    }
                }
                ImGui::End();
            }
            ImGui::PopStyleColor();
        }
    }
}

}

