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
#include <IconsFontAwesome6.h>

namespace sofaimgui::windows {

ViewportWindow::ViewportWindow(const std::string& name, const bool& isWindowOpen, std::shared_ptr<StateWindow> stateWindow)
    : m_stateWindow(stateWindow)
{
    m_name = name;
    m_isOpen = isWindowOpen;
}

void ViewportWindow::showWindow(sofa::simulation::Node* groot,
                                const ImTextureID& texture,
                                const ImGuiWindowFlags& windowFlags)
{
    if (enabled() && isOpen())
    {
        if (ImGui::Begin(m_name.c_str(), &m_isOpen, windowFlags))
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
                                ImGui::GetStyle().FrameRounding);

            m_isMouseOnViewport = ImGui::IsItemHovered();

            addStateWindow();
            addSimulationTimeAndFPS(groot);

            // Panel backgroung
            double maxItemWidth = ImGui::CalcTextSize("Input/Output").x;
            ImDrawList* drawList = ImGui::GetWindowDrawList();
            ImVec2 size(ImGui::GetFrameHeight() * 3 + ImGui::GetStyle().FramePadding.x * 8 + maxItemWidth, ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.y * 2);

            float x = ImGui::GetWindowPos().x + ImGui::GetWindowWidth() / 2.f - ImGui::GetFrameHeight() * 4.f + ImGui::GetStyle().FramePadding.x;
            float y = ImGui::GetWindowPos().y + ImGui::GetStyle().FramePadding.y;

            ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));
            { // Draw
                auto color = ImGui::GetStyle().Colors[ImGuiCol_TabActive];
                color.w = 0.6f;
                drawList->AddRectFilled(bb.Min, bb.Max,
                                        ImGui::GetColorU32(color),
                                        ImGui::GetStyle().FrameRounding,
                                        ImDrawFlags_None);
            }

            ImGui::EndChild();
        }
        ImGui::End();
    }
}

void ViewportWindow::addStateWindow()
{
    ImGui::SetNextWindowPos(ImGui::GetWindowPos());  // attach the state window to top left of the viewport window
    m_stateWindow->showWindow();
}

bool ViewportWindow::addStepButton()
{
    ImVec2 buttonSize = ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
    bool isItemClicked = false;
    
    if (m_isOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isOpen))
        {
            if(ImGui::BeginChild("Render"))
            {
                auto position = ImGui::GetWindowPos();
                position.x += ImGui::GetWindowWidth() / 2 - ImGui::GetFrameHeight() * 4.f;
                position.y += ImGui::GetStyle().FramePadding.y;
                ImGui::SetNextWindowPos(position);  // attach the button window to top middle of the viewport window

                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                if (ImGui::Begin("ViewportChildButtons", &m_isOpen,
                                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
                {
                    ImGui::SameLine();
                    ImGui::Button(ICON_FA_FORWARD_STEP, buttonSize);
                    ImGui::SetItemTooltip("One step of simulation");

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
    
    if (m_isOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isOpen))
        {
            if(ImGui::BeginChild("Render"))
            {
                auto position = ImGui::GetWindowPos();
                position.x += ImGui::GetWindowWidth() / 2.f - ImGui::GetFrameHeight() * 4.f;
                position.y += ImGui::GetStyle().FramePadding.y;
                ImGui::SetNextWindowPos(position);  // attach the button window to top middle of the viewport window

                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                if (ImGui::Begin("ViewportChildButtons", &m_isOpen,
                         ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
                {
                    ImGui::SameLine();
                    ImGui::Button(*animate ? ICON_FA_PAUSE : ICON_FA_PLAY, buttonSize);
                    ImGui::SetItemTooltip(*animate ? "Stop simulation" : "Start simulation");

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

bool ViewportWindow::addDrivingTabCombo(int *mode, const char *listModes[], const int &sizeListModes, const double &maxItemWidth)
{
    bool hasValueChanged = false;
    
    if (m_isOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isOpen))
        {
            if(ImGui::BeginChild("Render"))
            {
                auto position = ImGui::GetWindowPos();
                position.x += ImGui::GetWindowWidth() / 2.f - ImGui::GetFrameHeight() * 4.f;
                position.y += ImGui::GetStyle().FramePadding.y;
                ImGui::SetNextWindowPos(position);  // attach the button window to top middle of the viewport window

                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                if (ImGui::Begin("ViewportChildButtons", &m_isOpen,
                                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove))
                {
                    ImGui::SameLine();
                    ImGui::PushItemWidth(maxItemWidth + ImGuiStyleVar_FramePadding * 2.0f + ImGui::GetTextLineHeightWithSpacing());
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.53f, 0.54f, 0.55f, 1.00f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.53f, 0.54f, 0.55f, 1.00f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.53f, 0.54f, 0.55f, 1.00f));
                    hasValueChanged = ImGui::Combo("##DrivingTabViewport", mode, listModes, sizeListModes);
                    ImGui::PopStyleColor(3);
                    ImGui::PopItemWidth();
                    ImGui::SetItemTooltip("Choose a tab to drive the TCP target");

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

void ViewportWindow::addSimulationTimeAndFPS(sofa::simulation::Node* groot)
{
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));

    // Time
    auto position = ImGui::GetWindowWidth() - ImGui::CalcTextSize("Time: 000.000").x - ImGui::GetStyle().ItemSpacing.x;
    ImGui::SetCursorPosX(position);
    ImGui::SetCursorPosY(ImGui::GetWindowHeight() - ImGui::GetTextLineHeightWithSpacing());
    ImGui::Text("Time: %.3f", groot->getTime());

    // FPS
    if (groot->animate_.getValue())
    {
        position -= ImGui::CalcTextSize("100.0 FPS ").x;
        ImGui::SetCursorPosX(position);
        ImGui::SetCursorPosY(ImGui::GetWindowHeight() - ImGui::GetTextLineHeightWithSpacing());
        ImGui::Text("%.1f FPS", io.Framerate);
    }

    ImGui::PopStyleColor();
}

}

