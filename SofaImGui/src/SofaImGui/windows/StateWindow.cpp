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


namespace sofaimgui::windows {

StateWindow::StateWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void StateWindow::showWindow(sofa::simulation::Node* groot,
                             const ImGuiWindowFlags& windowFlags)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            const auto& node = groot->getChild("UserInterface");
            if(node != nullptr)
            {
                const auto& data = node->getDataFields();
                ImGui::BeginTable("", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize);
                for(auto d: data)
                {
                    const std::string& value = d->getValueString();
                    const std::string& name = d->getName();
                    const std::string& group = d->getGroup();
                    if(group.find("state") != std::string::npos)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextDisabled("%s", name.c_str());
                        ImGui::TableNextColumn();
                        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(value.c_str()).x
                                             - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                        ImGui::TextDisabled("%s", value.c_str());
                    }
                }
                ImGui::EndTable();
            }
        }
        ImGui::End();
    }
}

}

