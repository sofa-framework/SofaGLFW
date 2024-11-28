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
#include "IconsFontAwesome6.h"
#include "imgui_internal.h"
#include <sofa/helper/logging/Messaging.h>
#include <SofaImGui/FooterStatusBar.h>
#include <sofa/version.h>

namespace sofaimgui {

FooterStatusBar &FooterStatusBar::getInstance()
{
    static FooterStatusBar footerStatusBar;
    return footerStatusBar;
}

void FooterStatusBar::showFooterStatusBar()
{
    ImGuiViewportP* viewport = (ImGuiViewportP*)(void*)ImGui::GetMainViewport();
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;
    float height = ImGui::GetFrameHeight();

    ImGui::PushStyleColor(ImGuiCol_MenuBarBg, ImGui::GetColorU32(ImGuiCol_Header));
    if (ImGui::BeginViewportSideBar("##FooterStatusBar", viewport, ImGuiDir_Down, height, window_flags))
    {
        if (ImGui::BeginMenuBar())
        {
            // Display the software version in the right of the status bar
            std::string version = "SOFA v" + std::string(SOFA_VERSION_STR);
            float length = ImGui::CalcTextSize(version.c_str()).x;
            float right = ImGui::GetCursorPosX() + ImGui::GetWindowSize().x - length - 2 * ImGui::GetStyle().ItemSpacing.x;
            ImGui::SetCursorPosX(right); // Set the position to the middle of the bar
            ImGui::TextDisabled("%s", version.c_str());

            ImGui::EndMenuBar();
        }
        ImGui::End();
    }
    ImGui::PopStyleColor();
}

void FooterStatusBar::showTempInfoOnStatusBar()
{
    static float infoRefreshTime = 0.;

    if (!m_tempInfo.empty())
    {
        if (m_refreshTempInfo)
        {
            infoRefreshTime = (float)ImGui::GetTime();
            m_refreshTempInfo = false;
        }
        
        if((float)ImGui::GetTime() - infoRefreshTime > m_tempInfoLifeSpan)
        {
            m_tempInfo.clear();
            return;
        }

        if (ImGui::Begin("##FooterStatusBar"))
        {
            if (ImGui::BeginMenuBar())
            {
                float length = ImGui::CalcTextSize(m_tempInfo.c_str()).x;
                float center = ImGui::GetWindowPos().x + ImGui::GetWindowSize().x / 2.f - length / 2.f;
                ImGui::SetCursorPosX(center); // Set the position to the middle of the status bar
                ImGui::Text(ICON_FA_CIRCLE_INFO" %s", m_tempInfo.c_str());

                ImGui::EndMenuBar();
            }
            ImGui::End();
        }
    }
}

void FooterStatusBar::setTempInfo(const std::string &info)
{
    m_refreshTempInfo = true;
    m_tempInfo = info;
}

}
