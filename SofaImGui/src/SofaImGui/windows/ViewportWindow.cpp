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

#include <SofaImGui/windows/ViewportWindow.h>
#include <imgui.h>

namespace sofaimgui::windows {

ViewportWindow::ViewportWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void ViewportWindow::showWindow(sofa::core::sptr<sofa::simulation::Node> groot)
{
    SOFA_UNUSED(groot);
    if (m_isWindowOpen)
    {
        ImVec2 pos;
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen))
        {
            pos = ImGui::GetWindowPos();

            ImGui::BeginChild("Render");
            ImVec2 wsize = ImGui::GetWindowSize();
            m_viewportWindowSize = { wsize.x, wsize.y};

            ImGui::Image((ImTextureID)m_fbo->getColorTexture(), wsize, ImVec2(0, 1), ImVec2(1, 0));
            m_isMouseOnViewport = ImGui::IsItemHovered();
            ImGui::EndChild();

        }
        ImGui::End();
    }
}

}

