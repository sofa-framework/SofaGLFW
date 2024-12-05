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

void FooterStatusBar::showTempMessageOnStatusBar()
{
    static float infoRefreshTime = 0.;

    if (!m_tempMessage.empty())
    {
        if (m_refreshTempMessage)
        {
            infoRefreshTime = (float)ImGui::GetTime();
            m_refreshTempMessage = false;
        }
        
        if((float)ImGui::GetTime() - infoRefreshTime > m_tempMessageLifeSpan)
        {
            m_tempMessage.clear();
            return;
        }

        if (ImGui::Begin("##FooterStatusBar"))
        {
            if (ImGui::BeginMenuBar())
            {
                float length = ImGui::CalcTextSize(m_tempMessage.c_str()).x;
                float center = ImGui::GetWindowPos().x + ImGui::GetWindowSize().x / 2.f - length / 2.f;
                ImGui::SetCursorPosX(center); // Set the position to the middle of the status bar

                std::string icon;
                switch (m_tempMessageType) {
                case MessageType::WARNING:
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.4f, 0.f, 1.f));
                    icon = ICON_FA_CIRCLE_EXCLAMATION;
                    break;
                case MessageType::ERROR:
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.f, 0.f, 1.f));
                    icon = ICON_FA_CIRCLE_EXCLAMATION;
                    break;
                default:
                    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_Text));
                    icon = ICON_FA_CIRCLE_INFO;
                    break;
                }
                ImGui::Text("%s", (icon + " " + m_tempMessage).c_str());
                ImGui::PopStyleColor();

                ImGui::EndMenuBar();
            }
            ImGui::End();
        }
    }
}

void FooterStatusBar::setTempMessage(const std::string &message, const MessageType& type)
{
    m_refreshTempMessage = true;
    m_tempMessageType = type;
    m_tempMessage = message;
}

}
