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
#include <imgui_internal.h>
#include <IconsFontAwesome5.h>

namespace sofaimgui::windows {

ViewportWindow::ViewportWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void ViewportWindow::showWindow(sofa::simulation::Node* groot,
                                const ImTextureID& texture,
                                const ImGuiWindowFlags& windowFlags)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen,
                         windowFlags | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
        {
            ImGui::BeginChild("Render");
            ImVec2 wsize = ImGui::GetWindowSize();
            m_windowSize = {wsize.x, wsize.y};

            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 p_min = ImGui::GetCursorScreenPos();
            ImVec2 p_max = ImVec2(p_min.x + wsize.x, p_min.y + wsize.y);
            ImGui::ItemAdd(ImRect(p_min, p_max), ImGui::GetID("ImageRender"));
            dl->AddImageRounded(texture, p_min, p_max,
                                ImVec2(0, 1), ImVec2(1, 0), ImGui::GetColorU32(ImVec4(1, 1, 1, 1)),
                                ImGuiStyleVar_FrameRounding * 2);

            m_isMouseOnViewport = ImGui::IsItemHovered();

            addStateWindow(groot);

            ImGui::EndChild();
        }
        ImGui::End();
    }
}

void ViewportWindow::addStateWindow(sofa::simulation::Node* groot)
{
    ImGui::SetNextWindowPos(ImGui::GetWindowPos());  // attach the state window to top left of the viewport window
    m_stateWindow.showWindow(groot);
}

bool ViewportWindow::addStepButton()
{
    ImVec2 buttonSize = ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
    bool isItemClicked = false;

    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen))
        {
            if(ImGui::BeginChild("Render"))
            {
                auto position = ImGui::GetWindowPos();
                position.x += ImGui::GetWindowWidth() / 2;
                ImGui::SetNextWindowPos(position);  // attach the button window to top middle of the viewport window

                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.0f)));
                if (ImGui::Begin("ViewportChildButtons", &m_isWindowOpen,
                                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
                {
                    ImGui::SameLine();
                    ImGui::Button(ICON_FA_STEP_FORWARD, buttonSize);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("One step of simulation");

                    if (ImGui::IsItemClicked())
                        isItemClicked = true;

                    ImGui::End();
                }
                ImGui::PopStyleColor();

                ImGui::EndChild();
            }
            ImGui::End();
        }
    }

    return isItemClicked;
}

bool ViewportWindow::addAnimateButton(bool *animate)
{
    ImVec2 buttonSize = ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
    bool isItemClicked = false;

    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen))
        {
            if(ImGui::BeginChild("Render"))
            {
                auto position = ImGui::GetWindowPos();
                position.x += ImGui::GetWindowWidth() / 2;
                ImGui::SetNextWindowPos(position);  // attach the button window to top middle of the viewport window

                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.0f)));
                if (ImGui::Begin("ViewportChildButtons", &m_isWindowOpen,
                         ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
                {
                    ImGui::SameLine();
                    ImGui::Button(*animate ? ICON_FA_PAUSE : ICON_FA_PLAY, buttonSize);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(*animate ? "Stop simulation" : "Start simulation");

                    if (ImGui::IsItemClicked())
                    {
                        *animate = !*animate;
                        isItemClicked = true;
                    }

                    ImGui::End();
                }
                ImGui::PopStyleColor();

                ImGui::EndChild();
            }
            ImGui::End();
        }
    }

    return isItemClicked;
}

bool ViewportWindow::addModeButton(int *mode, const char *listModes[], const int &sizeListModes)
{
    bool hasValueChanged = false;

    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen))
        {
            if(ImGui::BeginChild("Render"))
            {
                auto position = ImGui::GetWindowPos();
                position.x += ImGui::GetWindowWidth() / 2;
                ImGui::SetNextWindowPos(position);  // attach the button window to top middle of the viewport window

                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.0f)));
                if (ImGui::Begin("ViewportChildButtons", &m_isWindowOpen,
                                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
                {
                    ImGui::SameLine();
                    hasValueChanged = ImGui::Combo("Mode##Viewport", mode, listModes, sizeListModes);

                    ImGui::End();
                }
                ImGui::PopStyleColor();

                ImGui::EndChild();
            }
            ImGui::End();
        }
    }

    return hasValueChanged;
}

}

