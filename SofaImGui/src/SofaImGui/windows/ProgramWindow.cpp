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
#include <IconsFontAwesome6.h>


namespace sofaimgui::windows {

float ProgramWindow::m_cursor = 0.f;
ImVec2 ProgramWindow::m_trackBeginPos = ImVec2(0.f, 0.f);
float ProgramWindow::m_timelineOneSecondSize = 0.f;
float ProgramWindow::m_time = 0.f;

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
        m_trackHeight = ImGui::GetFrameHeight() * 4;

        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen,
                         windowFlags | ImGuiWindowFlags_AlwaysAutoResize
                         ))
        {
            addButtons();

            float width = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() - ImGui::GetTextLineHeightWithSpacing() * 3;
            static float zoomCoef = 1;
            static float minSize = 100;
            m_timelineOneSecondSize = zoomCoef * minSize;
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetColorU32(ImGuiCol_WindowBg));
            if (ImGui::BeginChildFrame(ImGui::GetID(m_name.c_str()), ImVec2(width, height),
                                       ImGuiWindowFlags_AlwaysHorizontalScrollbar))
            {
                ImGui::PopStyleColor();

                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 6));
                addTimeline();
                addTracks();
                addCursorMarker();
                ImGui::PopStyleVar();

                ImGui::EndChildFrame();
            }
            else
            {
                ImGui::PopStyleColor();
            }
            if (ImGui::IsItemHovered())
                zoomCoef += ImGui::GetIO().MouseWheel * 0.1f;
            zoomCoef = (zoomCoef < 1)? 1 : zoomCoef;
            zoomCoef = (zoomCoef > 4)? 4 : zoomCoef;

            ImGui::End();
        }
    }
}

void ProgramWindow::addButtons()
{
    ImVec2 buttonSize(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
    auto position = ImGui::GetCursorPosX() + ImGui::GetCurrentWindow()->Size.x - buttonSize.x * 3 - ImGuiStyleVar_ItemSpacing * 5; // Get position for right buttons

    // Left buttons
    if (ImGui::Button(ICON_FA_FOLDER_OPEN, buttonSize))
    {
        importProgram();
    }
    ImGui::SetItemTooltip("Import program");

    ImGui::SameLine();

    if (ImGui::Button(ICON_FA_ARROW_UP_FROM_BRACKET, buttonSize))
    {
        exportProgram();
    }
    ImGui::SetItemTooltip("Export program");

    // Right buttons
    ImGui::SameLine();
    ImGui::SetCursorPosX(position); // Set position to right of the header

    ImGui::Button(ICON_FA_PLUS"##Add", buttonSize);

    static bool repeat = false;
    static bool reverse = false;

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, repeat? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_REPEAT"##Repeat", buttonSize))
    {
        reverse = false;
        repeat = !repeat;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Repeat program");

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, reverse? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_ARROWS_LEFT_RIGHT"##Reverse", buttonSize))
    {
        repeat = false;
        reverse = !reverse;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Reverse and repeat program");
}

void ProgramWindow::addCursorMarker()
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImVec4 color(0.95f, 0.f, 0.f, 1.0f);

    float thicknessRect = 2.f;
    float widthTri = ImGuiStyleVar_ItemInnerSpacing;

    m_cursor = m_time * m_timelineOneSecondSize;
    ImVec2 p0Rect(m_trackBeginPos.x + m_cursor - window->Scroll.x, m_trackBeginPos.y);
    ImVec2 p1Rect(p0Rect.x + thicknessRect,
                  p0Rect.y + m_trackHeight * m_program.getNbTracks() + ImGuiStyleVar_ItemSpacing * (m_program.getNbTracks() - 1));

    ImVec2 p0Tri(p0Rect.x + thicknessRect / 2.f, p0Rect.y);
    ImVec2 p1Tri(p0Tri.x - widthTri / 2.f, p0Tri.y - widthTri);
    ImVec2 p2Tri(p0Tri.x + widthTri / 2.f,  p0Tri.y - widthTri);

    window->DrawList->AddTriangleFilled(p0Tri, p1Tri, p2Tri, ImGui::GetColorU32(color));
    window->DrawList->AddRectFilled(p0Rect, p1Rect, ImGui::GetColorU32(color), ImGuiStyleVar_FrameRounding / 2.f);
}

void ProgramWindow::addTimeline()
{
    float width = ImGui::GetWindowWidth() + ImGui::GetScrollX();
    int nbSteps = width / m_timelineOneSecondSize + 1;

    float indentSize = ImGui::CalcTextSize(ICON_FA_BARS).x + 2 * ImGuiStyleVar_FramePadding;
    ImGui::Indent(indentSize);

    ImGui::BeginGroup(); // Timeline's number (seconds)
    for (int i=0 ; i<nbSteps; i++)
    {
        std::string text = std::to_string(i) + " s";
        float textSize = ImGui::CalcTextSize(text.c_str()).x;
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(m_timelineOneSecondSize - textSize, 0.f));
        ImGui::Text("%s", text.c_str());
        ImGui::SameLine();
        ImGui::PopStyleVar();
    }
    ImGui::EndGroup();

    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImGui::NewLine();
    ImGui::BeginGroup(); // Timeline's lines
    ImGui::SameLine();
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(m_timelineOneSecondSize / 10, 0.f));
    float height = ImGui::GetFrameHeight() / 3.f;
    float heightSpace = height / 4.f;
    float y1 = window->DC.CursorPos.y;
    float y1Space = window->DC.CursorPos.y + heightSpace;
    float y2 = window->DC.CursorPos.y + height;

    for (int i=0 ; i<nbSteps; i++)
    {
        for (int j=0; j<10; j++)
        {
            ImVec4 color = (j==0) ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : ImVec4(0.5f, 0.5f, 0.5f, 1.f);

            drawList->AddLine(ImVec2(window->DC.CursorPos.x, (j==0)? y1: y1Space), ImVec2(window->DC.CursorPos.x, y2), ImGui::GetColorU32(color));
            ImGui::Spacing();
            ImGui::SameLine();
        }
    }
    ImGui::PopStyleVar();
    ImGui::EndGroup();

    ImGui::Unindent(indentSize);
}

void ProgramWindow::addTracks()
{
    const auto& tracks = m_program.getTracks();

    int trackID = 0;

    for (const auto& track: tracks)
    {
        // Track options menu
        std::string menuLabel = std::string("##TrackMenu" + std::to_string(trackID));
        std::string buttonLabel = std::string(ICON_FA_BARS "##TrackButton" + std::to_string(trackID));
        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::MenuItem(std::string("Clear track##" + std::to_string(trackID)).c_str()))
            {
                track->clear();
            }
            if (ImGui::MenuItem(std::string("Add track##" + std::to_string(trackID)).c_str(), nullptr, false, false))
            {
                m_program.addTrack(std::make_shared<models::Track>());
            }
            if (ImGui::MenuItem(std::string("Remove track##" + std::to_string(trackID)).c_str(), nullptr, false, (trackID>0)? true : false))
            {
                m_program.removeTrack(trackID--);
            }
            ImGui::EndPopup();
        }

        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_MenuBarBg)); // Color of track button
        ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5, 0)); // Align icon top middle
        if(ImGui::Button(buttonLabel.c_str(), ImVec2(ImGui::CalcTextSize(ICON_FA_BARS).x + 2 * ImGuiStyleVar_FramePadding, m_trackHeight)))
        {
            ImGui::PopStyleVar(); // End align icon top middle
            ImGui::OpenPopup(menuLabel.c_str());
        }
        else
        {
            ImGui::PopStyleVar(); // End align icon top middle
        }
        ImGui::PopStyleColor();

        m_trackBeginPos = ImGui::GetCurrentWindow()->DC.CursorPosPrevLine;

        ImGui::SameLine();
        addBlocks(track, trackID);
        trackID++;
    }
}

void ProgramWindow::addBlocks(const std::shared_ptr<models::Track> &track,
                              const int& trackID)
{
    const std::vector<std::shared_ptr<models::Action>> &actions = track->getActions();
    int actionID = 0;

    // Action blocks
    for (const std::shared_ptr<models::Action> &action: actions)
    {
        float actionWidth = action->getDuration() * m_timelineOneSecondSize - ImGui::GetStyle().ItemSpacing.x;
        float actionHeight = m_trackHeight;
        std::string blockLabel = std::string("##Action" + std::to_string(trackID) + std::to_string(actionID++));
        ImGui::SameLine();
        action->showBlock(blockLabel.c_str(), ImVec2(actionWidth, actionHeight));
    }

    { // Empty track background
        ImGui::SameLine();
        std::string trackLabel = std::string("##Track" + std::to_string(trackID) + std::to_string(actionID));
        ImVec2 size(ImGui::GetWindowWidth() + ImGui::GetScrollX(), m_trackHeight);

        float x = ImGui::GetCurrentWindow()->DC.CursorPos.x ;
        float y = ImGui::GetCurrentWindow()->DC.CursorPos.y ;
        ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));

        ImGui::ItemSize(size);
        if (!ImGui::ItemAdd(bb, ImGui::GetID(trackLabel.c_str())))
            return;

        { // Backgroung
            ImGui::GetWindowDrawList()->AddRectFilled(bb.Min, bb.Max,
                                                      ImGui::GetColorU32(ImGuiCol_FrameBg),
                                                      ImGui::GetStyle().FrameRounding,
                                                      ImDrawFlags_None);
        }
    }
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

void ProgramWindow::animateBeginEvent(sofa::simulation::Node *groot)
{
    SOFA_UNUSED(groot);
}

void ProgramWindow::animateEndEvent(sofa::simulation::Node *groot)
{
    SOFA_UNUSED(groot);
    m_time = groot->getTime(); // time in seconds
}

} // namespace


























