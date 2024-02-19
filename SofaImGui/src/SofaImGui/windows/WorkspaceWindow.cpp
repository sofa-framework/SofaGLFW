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

#include <SofaImGui/windows/WorkspaceWindow.h>
#include <IconsFontAwesome6.h>


namespace sofaimgui::windows {

WorkspaceWindow::WorkspaceWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void WorkspaceWindow::showWindow(const ImGuiWindowFlags& windowFlags)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            ImGui::SetCursorPosY(ImGui::GetWindowHeight() / 2.5); //approximatively the center of the window

            ImGui::BeginDisabled();
            ImGui::Button(ICON_FA_PENCIL);
            ImGui::EndDisabled();

            ImGui::Button(ICON_FA_CODE);
\
            ImGui::BeginDisabled();
            ImGui::Button(ICON_FA_WAVE_SQUARE);
            ImGui::EndDisabled();
        }
        ImGui::End();
    }
}

}

