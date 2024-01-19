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

StateWindow::StateWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void StateWindow::showWindow(sofa::core::sptr<sofa::simulation::Node> groot)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen))
        {
            const auto& modelling = groot->getChild("Modelling");
            if(modelling != nullptr)
            {
                const auto& state = modelling->getChild("State");
                if(state != nullptr)
                {
                    const auto& data = state->getDataFields();
                    ImGui::BeginTable("", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize);
                    for(auto d: data)
                    {
                        auto value = d->getValueString();
                        auto name = d->getName();
                        if(name.find("effector") != std::string::npos || name.find("actuator") != std::string::npos)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableNextColumn();
                            ImGui::Text("%s", name.c_str());
                            ImGui::TableNextColumn();
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(value.c_str()).x
                                                 - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                            ImGui::Text("%s", value.c_str());
                        }
                    }
                    ImGui::EndTable();
                }
            }
        }
        ImGui::End();
    }
}

}

