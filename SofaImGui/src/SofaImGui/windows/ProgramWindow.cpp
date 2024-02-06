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

#include <SofaImGui/windows/ProgramWindow.h>
#include <SofaImGui/models/Action.h>
#include <SofaImGui/models/Move.h>
#include <SofaImGui/widgets/Buttons.h>

#include <sofa/helper/system/FileSystem.h>

#include <imgui_internal.h>
#include <GLFW/glfw3.h>

#include <nfd.h>
#include <IconsFontAwesome5.h>


namespace sofaimgui::windows {

using sofa::type::Vec3;
using sofa::type::Quat;

ProgramWindow::ProgramWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void ProgramWindow::showWindow(sofa::simulation::Node* groot,
                             const ImGuiWindowFlags& windowFlags)
{
    SOFA_UNUSED(groot);
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen,
                         windowFlags | ImGuiWindowFlags_AlwaysAutoResize
                         ))
        {
            addButtons();

            float width = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() - ImGui::GetTextLineHeightWithSpacing() * 3;
            static float zoomCoef = 1;
            static float initSize = 150;
            float sectionSize = zoomCoef * initSize;
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.4f, 0.4f, 0.4f, 1.0f));
            ImGui::BeginChildFrame(ImGui::GetID(m_name.c_str()), ImVec2(width, height),
                                   ImGuiWindowFlags_AlwaysHorizontalScrollbar);

            addTimeline(sectionSize);

            addTracks(sectionSize);

            ImGui::EndChildFrame();
            ImGui::PopStyleColor();

            if (ImGui::IsItemHovered())
                zoomCoef += ImGui::GetIO().MouseWheel * 0.1f;
            zoomCoef = (zoomCoef < 1)? 1 : zoomCoef;
            zoomCoef = (zoomCoef > 4)? 4 : zoomCoef;
        }
        ImGui::End();
    }
}

void ProgramWindow::addButtons()
{
    auto position = ImGui::GetCursorPosX() + ImGui::GetCurrentWindow()->Size.x - ImGui::CalcTextSize(ICON_FA_PLUS).x * 3 - ImGuiStyleVar_ItemSpacing * 5; // Get position for right buttons

    // Left buttons
    if (ImGui::Button("Import"))
        importProgram();

    ImGui::SameLine();
    if (ImGui::Button("Export"))
        exportProgram();

    // Right buttons
    ImGui::SameLine();
    ImGui::SetCursorPosX(position); // Set position to right of the header

    ImGui::Button(ICON_FA_PLUS"##Add");

    static bool repeat = false;
    static bool reverse = false;

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, repeat? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_REDO"##Repeat"))
    {
        reverse = false;
        repeat = !repeat;
    }
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Repeat program");

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, reverse? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_ARROWS_ALT_H"##Reverse"))
    {
        repeat = false;
        reverse = !reverse;
    }
    ImGui::PopStyleColor();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reverse and repeat program");
}

void ProgramWindow::addTimeline(float sectionSize)
{
    float width = ImGui::GetWindowWidth() + ImGui::GetScrollX();
    int nbSteps = width / sectionSize + 1;

    float indentSize = ImGui::CalcTextSize(ICON_FA_BARS).x + 2 * ImGuiStyleVar_FramePadding;
    ImGui::Indent(indentSize);

    ImGui::BeginGroup(); // Timeline's number (seconds)
    for (int i=0 ; i<nbSteps; i++)
    {
        std::string text = std::to_string(i) + " s";
        float textSize = ImGui::CalcTextSize(text.c_str()).x;
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(sectionSize - textSize, 0.f));
        ImGui::Text("%s", text.c_str());
        ImGui::SameLine();
        ImGui::PopStyleVar();
    }
    ImGui::EndGroup();

    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    float thickness = 1;
    float height = 12;

    ImGui::NewLine();
    ImGui::BeginGroup(); // Timeline's lines
    ImGui::SameLine();
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(sectionSize / 10, 0.f));
    for (int i=0 ; i<nbSteps; i++)
    {
        for (int j=0; j<10; j++)
        {
            float y1 = window->DC.CursorPos.y ;
            float y2 = window->DC.CursorPos.y + height;
            const ImRect bb(ImVec2(window->DC.CursorPos.x, y1), ImVec2(window->DC.CursorPos.x + thickness, y2));
            ImVec4 color = (j==0) ? ImVec4(1.f, 1.f, 1.f, 1.f) : ImVec4(0.5f, 0.5f, 0.5f, 1.f);

            drawList->AddLine(ImVec2(bb.Min.x, bb.Min.y), ImVec2(bb.Min.x, bb.Max.y), ImGui::GetColorU32(color));
            ImGui::Spacing();
            ImGui::SameLine();
        }
    }
    ImGui::PopStyleVar();
    ImGui::EndGroup();

    ImGui::Unindent(indentSize);
}

void ProgramWindow::addTracks(const float& sectionSize)
{
    const auto& tracks = m_program.getTracks();
    float trackHeight = 200;
    int trackID = 0;
    for (const auto& track: tracks)
    {
        // Track options menu
        std::string menuLabel = std::string("##TrackMenu" + std::to_string(trackID));
        std::string buttonLabel = std::string(ICON_FA_BARS "##TrackButton" + std::to_string(trackID));
        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::MenuItem("Clear track"))
            {
                track->clear();
            }
            if (ImGui::MenuItem("Add track", nullptr, false, false))
            {
                std::shared_ptr<models::Track> track = std::make_shared<models::Track>();
                m_program.addTrack(track);
            }
            if (ImGui::MenuItem("Remove track", nullptr, false, (trackID>0)? true : false))
            {
                m_program.removeTrack(trackID--);
            }
            ImGui::EndPopup();
        }

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2, 0.2, 0.2, 1.0)); // Color of track button
        ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5, 0)); // Align icon top middle
        if(ImGui::Button(buttonLabel.c_str(), ImVec2(ImGui::CalcTextSize(ICON_FA_BARS).x + 2 * ImGuiStyleVar_FramePadding, trackHeight)))
        {
            ImGui::PopStyleVar(); // End align icon top middle
            ImGui::OpenPopup(menuLabel.c_str());
        }
        else
        {
            ImGui::PopStyleVar(); // End align icon top middle
        }
        ImGui::PopStyleColor(); // End color of track button

        ImGui::SameLine();
        addBlocks(track, trackID, sectionSize, trackHeight);
        ImGui::NewLine();
        trackID++;
    }
}

void ProgramWindow::addBlocks(const std::shared_ptr<models::Track> &track,
                              const int& trackID,
                              const float &sectionSize,
                              const float &height)
{
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(20.f, 20.f));
    const auto& actions = track->getActions();
    int actionID = 0;

    if (actions.empty())
    {
        ImVec2 size(ImGui::GetWindowWidth(), height);
        std::string trackLabel = std::string("##Track" + std::to_string(trackID) + std::to_string(actionID++));
        const ImGuiID id = ImGui::GetID(trackLabel.c_str());
        if(!ImGui::BeginChildFrame(id, size))
            return;
        ImGui::EndChildFrame();
    }
    else
    {
        for (std::shared_ptr<models::Action> action: actions)
        {
            float actionWidth = action->getDuration() * sectionSize - ImGui::GetStyle().ItemSpacing.x;
            float actionHeight = height;
            std::string blockLabel = std::string("##Action" + std::to_string(trackID) + std::to_string(actionID++));
            ImGui::SameLine();
            action->showBlock(blockLabel.c_str(), ImVec2(actionWidth, actionHeight));
        }
    }

    ImGui::PopStyleVar();
}

void ProgramWindow::importProgram()
{
    nfdchar_t *outPath;
    std::vector<nfdfilteritem_t> nfd_filters;
    nfd_filters.push_back({"program file", "crprog"});
    nfdresult_t result = NFD_OpenDialog(&outPath, nfd_filters.data(), nfd_filters.size(), nullptr);
    if (result == NFD_OKAY)
    {
        if (sofa::helper::system::FileSystem::exists(outPath))
            m_program.importProgram(outPath);
        NFD_FreePath(outPath);
    }
}

void ProgramWindow::exportProgram()
{
    nfdchar_t *outPath;
    std::vector<nfdfilteritem_t> nfd_filters;
    nfd_filters.push_back({"program file", "crprog"});
    nfdresult_t result = NFD_SaveDialog(&outPath, nfd_filters.data(), nfd_filters.size(), nullptr, "");
    if (result == NFD_OKAY)
        m_program.exportProgram(outPath);
}

} // namespace


























