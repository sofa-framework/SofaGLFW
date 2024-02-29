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
#include <SofaImGui/Utils.h>
#include <SofaImGui/widgets/Buttons.h>

#include <sofa/helper/system/FileSystem.h>

#include <imgui_internal.h>
#include <GLFW/glfw3.h>

#include <nfd.h>
#include <IconsFontAwesome6.h>


namespace sofaimgui::windows {

using sofa::type::Vec3;
using sofa::type::Quat;

ProgramWindow::ProgramWindow(const std::string& name,
                             const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void ProgramWindow::showWindow(sofaglfw::SofaGLFWBaseGUI *baseGUI,
                               const ImGuiWindowFlags& windowFlags)
{
    if (m_isWindowOpen)
    {
        if (baseGUI)
            m_baseGUI = baseGUI;
        else
            return;

        static bool firstTime = true;
        if (firstTime)
        {
            firstTime = false;
            m_program = models::Program(m_baseGUI, m_TCPTarget);
        }

        m_trackHeight = ImGui::GetFrameHeightWithSpacing() * 5;

        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen,
                         windowFlags | ImGuiWindowFlags_AlwaysAutoResize
                         ))
        {
            showProgramButtons();

            float width = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() - ImGui::GetTextLineHeightWithSpacing() * 3;
            static float zoomCoef = 1;
            static float minSize = ImGui::GetFrameHeight() * 2;
            m_timelineOneSecondSize = zoomCoef * minSize;
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetColorU32(ImGuiCol_WindowBg));
            if (ImGui::BeginChildFrame(ImGui::GetID(m_name.c_str()), ImVec2(width, height),
                                       ImGuiWindowFlags_AlwaysHorizontalScrollbar))
            {
                ImGui::PopStyleColor();

                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 6));
                showTimeline();
                showTracks();
                showCursorMarker();
                ImGui::PopStyleVar();

                ImGui::EndChildFrame();
            }
            else
            {
                ImGui::PopStyleColor();
            }
            if (ImGui::IsItemHovered())
                zoomCoef += ImGui::GetIO().MouseWheel * 0.4f;
            float coefMax = 20.f;
            zoomCoef = (zoomCoef < 1)? 1 : zoomCoef;
            zoomCoef = (zoomCoef > coefMax)? coefMax : zoomCoef;

            ImGui::End();
        }
    }
}

void ProgramWindow::showProgramButtons()
{
    ImVec2 buttonSize(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
    auto positionRight = ImGui::GetCursorPosX() + ImGui::GetWindowSize().x - buttonSize.x * 3 - ImGui::GetStyle().ItemSpacing.y * 3; // Get position for right buttons
    auto positionMiddle = ImGui::GetCursorPosX() + ImGui::GetWindowSize().x / 2.f; // Get position for middle button

            // Left buttons
    static bool successfulImport = true; // TODO: notify user on failure
    if (ImGui::Button(ICON_FA_FOLDER_OPEN, buttonSize))
    {
        successfulImport = importProgram();
    }
    ImGui::SetItemTooltip("Import program");

    ImGui::SameLine();

    if (ImGui::Button(ICON_FA_ARROW_UP_FROM_BRACKET, buttonSize))
    {
        exportProgram();
    }
    ImGui::SetItemTooltip("Export program");

            // Middle button
    ImGui::SameLine();
    ImGui::SetCursorPosX(positionMiddle); // Set position to right of the header

    if (!isDrivingSimulation())
        ImGui::BeginDisabled();

    if (ImGui::Button("Restart"))
    {
        auto groot = m_baseGUI->getRootNode().get();
        groot->setTime(0.);
        m_time = 0.f;
    }
    ImGui::SetItemTooltip("Reload the simulation and restart the program");

    if (!isDrivingSimulation())
        ImGui::EndDisabled();

            // Right buttons
    ImGui::SameLine();
    ImGui::SetCursorPosX(positionRight); // Set position to right of the header

    ImGui::PushStyleColor(ImGuiCol_Button, m_drawTrajectory? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_DRAW_POLYGON"##Draw", buttonSize))
    {
        m_drawTrajectory = !m_drawTrajectory;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Draw trajectory");

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, m_repeat? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_REPEAT"##Repeat", buttonSize))
    {
        m_reverse = false;
        m_repeat = !m_repeat;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Repeat program");

    ImGui::SameLine();

    ImGui::BeginDisabled();
    ImGui::PushStyleColor(ImGuiCol_Button, m_reverse? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_ARROWS_LEFT_RIGHT"##Reverse", buttonSize))
    {
        m_repeat = false;
        m_reverse = !m_reverse;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Reverse and repeat program");
    ImGui::EndDisabled();
}

void ProgramWindow::showCursorMarker()
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImVec4 color(0.95f, 0.f, 0.f, 1.0f);

    float thicknessRect = ImGui::GetStyle().SeparatorTextBorderSize;
    float widthTri = ImGui::GetStyle().ItemInnerSpacing.x * 2;

    m_cursor = m_time * m_timelineOneSecondSize;
    ImVec2 p0Rect(m_trackBeginPos.x + m_cursor - window->Scroll.x, m_trackBeginPos.y);
    ImVec2 p1Rect(p0Rect.x + thicknessRect,
                  p0Rect.y + m_trackHeight * m_program.getNbTracks() + ImGui::GetStyle().ItemSpacing.y * (m_program.getNbTracks() - 1));

    ImVec2 p0Tri(p0Rect.x + thicknessRect / 2.f, p0Rect.y);
    ImVec2 p1Tri(p0Tri.x - widthTri / 2.f, p0Tri.y - widthTri);
    ImVec2 p2Tri(p0Tri.x + widthTri / 2.f,  p0Tri.y - widthTri);

    window->DrawList->AddTriangleFilled(p0Tri, p1Tri, p2Tri, ImGui::GetColorU32(color));
    window->DrawList->AddRectFilled(p0Rect, p1Rect, ImGui::GetColorU32(color), ImGui::GetStyle().FrameRounding / 2.f);
}

void ProgramWindow::showTimeline()
{
    float width = ImGui::GetWindowWidth() + ImGui::GetScrollX();
    int nbSteps = width / m_timelineOneSecondSize + 1;

    float indentSize = ImGui::CalcTextSize(ICON_FA_BARS).x + 2 * ImGui::GetStyle().FramePadding.x;
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

void ProgramWindow::showTracks()
{
    const auto& tracks = m_program.getTracks();

    int trackID = 0;

    for (const auto& track: tracks)
    {
        // Track options menu
        std::string menuLabel = "##TrackMenu" + std::to_string(trackID);
        std::string buttonLabel = ICON_FA_BARS "##TrackButton" + std::to_string(trackID);
        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::MenuItem(("Clear track##" + std::to_string(trackID)).c_str()))
            {
                track->clear();
            }
            if (ImGui::MenuItem(("Add track##" + std::to_string(trackID)).c_str(), nullptr, false, false))
            {
                m_program.addTrack(std::make_shared<models::Track>(m_TCPTarget));
            }
            if (ImGui::MenuItem(("Remove track##" + std::to_string(trackID)).c_str(), nullptr, false, (trackID>0)? true : false))
            {
                m_program.removeTrack(trackID--);
            }

            ImGui::Separator();

            if (ImGui::MenuItem(("Add move##" + std::to_string(trackID)).c_str()))
            {
                track->pushMove();
            }
            ImGui::EndPopup();
        }

        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_MenuBarBg)); // Color of track button
        ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5, 0)); // Align icon top middle
        if(ImGui::Button(buttonLabel.c_str(), ImVec2(ImGui::CalcTextSize(ICON_FA_BARS).x + 2 * ImGui::GetStyle().FramePadding.x, m_trackHeight)))
        {
            ImGui::PopStyleVar(); // End align icon top middle
            ImGui::OpenPopup(menuLabel.c_str());
        }
        else
        {
            ImGui::PopStyleVar(); // End align icon top middle
        }
        ImGui::PopStyleColor();

        ImGui::SameLine();
        m_trackBeginPos = ImGui::GetCurrentWindow()->DC.CursorPos;
        showBlocks(track, trackID);
        trackID++;
    }
}

void ProgramWindow::showBlocks(const std::shared_ptr<models::Track> &track,
                               const int& trackID)
{
    const std::vector<std::shared_ptr<models::Action>> &actions = track->getActions();

            // Action blocks
    sofa::Index actionID = 0;
    while(actionID < actions.size())
    {
        const std::shared_ptr<models::Action> &action = actions[actionID];
        float actionWidth = action->getDuration() * m_timelineOneSecondSize - ImGui::GetStyle().ItemSpacing.x;
        float actionHeight = m_trackHeight;
        std::string blockLabel = "##Action" + std::to_string(trackID) + std::to_string(actionID);
        ImGui::SameLine();

        std::shared_ptr<models::Move> move = std::dynamic_pointer_cast<models::Move>(action);
        if (move)
        {
            move->drawTrajectory(m_drawTrajectory);
            showMoveBlock(track, actionID, move, blockLabel, ImVec2(actionWidth, actionHeight));
            // Options menu
            std::string menuLabel = std::string("##ActionOptionsMenu" + blockLabel);
            if (ImGui::BeginPopup(menuLabel.c_str()))
            {
                if (ImGui::MenuItem("Add left"))
                {
                    track->insertMove(actionID);
                }
                if (ImGui::MenuItem("Add right"))
                {
                    track->insertMove(actionID + 1);
                }

                ImGui::Separator();

                if (ImGui::MenuItem("Overwrite waypoint"))
                {
                    move->setWaypoint(m_TCPTarget->getPosition());
                    track->updateNextMove(actionID);
                }

                ImGui::Separator();

                if (ImGui::MenuItem("Delete"))
                {
                    track->deleteMove(actionID);
                }
                else
                    actionID++;
                ImGui::EndPopup();
            } else {
                actionID++;
            }
            if (actionWidth > ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 3.0f)
                showActionOptionButton(menuLabel, blockLabel);
        }
    }

    { // Empty track background
        ImGui::SameLine();
        std::string trackLabel = "##Track" + std::to_string(trackID) + "Empty";
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

void ProgramWindow::showMoveBlock(const std::shared_ptr<models::Track> &track,
                                  const sofa::Index &actionID,
                                  const std::shared_ptr<models::Move> &move,
                                  const std::string &label,
                                  const ImVec2 &size)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float alignWidth = ImGui::CalcTextSize("duration    ").x;
    float x = window->DC.CursorPos.x ;
    float y = window->DC.CursorPos.y ;

    ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));
    ImVec2 topRight = ImVec2(x + size.x, y);

    ImGui::ItemSize(size);
    const ImGuiID id = ImGui::GetID(label.c_str());
    if (!ImGui::ItemAdd(bb, id))
        return;

    { // Block backgroung
        drawList->AddRectFilled(bb.Min, bb.Max,
                                ImGui::GetColorU32(ImVec4(0.39f, 0.57f, 0.6f, 1.0f)),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);
    }

    if (ImGui::IsItemHovered())
        move->highlightWaypoint(true);
    else
        move->highlightWaypoint(false);

    std::string text = "Move to Way Point";
    ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
    ImVec2 padding(ImGui::GetStyle().FramePadding);

    auto rectMin = ImGui::GetItemRectMin();
    auto rectMax = ImGui::GetItemRectMax();
    rectMax.x -= padding.x;
    ImGui::PushClipRect(rectMin, rectMax, true);

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
    { // Move
        x += padding.y;
        y += padding.y;
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + size.x - padding.x * 2,
                        y + textSize.y + padding.y * 2);
        drawList->AddRectFilled(bb.Min, bb.Max,
                                ImGui::GetColorU32(ImVec4(0.29f, 0.47f, 0.5f, 1.0f)),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);

        window->DC.CursorPos.x = x;
        window->DC.CursorPos.y = y;

        auto rectMin = ImGui::GetItemRectMin();
        auto rectMax = ImGui::GetItemRectMax();
        rectMax.x -= padding.x * 2 + ImGui::GetFrameHeight(); // leave space for option button
        ImGui::PushClipRect(rectMin, rectMax, true);

        std::string id = "##comment" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0., 0., 0., 0.));
        ImGui::InputText(id.c_str(), move->getComment(), models::Action::COMMENTSIZE);
        ImGui::PopStyleColor();

        ImGui::PopClipRect();
    }
    ImGui::PopStyleColor();

    text = "WP.pos";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
    { // Way point position
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        RigidCoord waypoint = move->getWaypoint();
        for (int i=0; i<3; i++)
        {
            ImGui::PushItemWidth(ImGui::CalcTextSize("10000").x);
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.7f, 0.7f, 0.7f, 1.f));
            if (ImGui::InputFloat(id.c_str(), &waypoint[i], 0, 0, "%0.f", ImGuiInputTextFlags_CharsNoBlank))
            {
                move->setWaypoint(waypoint);
                track->updateNextMove(actionID);
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::PopItemWidth();
        }
        ImGui::PopStyleVar();
    }
    ImGui::PopStyleColor();

    text = "WP.rot";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
    { // Way point rotation
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);


        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());

        window->DC.CursorPos.x = x + alignWidth;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        RigidCoord waypoint = move->getWaypoint();
        for (int i=3; i<7; i++)
        {
            window->DC.CursorPos.y = y;
            ImGui::PushItemWidth(ImGui::CalcTextSize("10000").x);
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.7f, 0.7f, 0.7f, 1.f));
            if (ImGui::InputFloat(id.c_str(), &waypoint[i], 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
            {
                move->setWaypoint(waypoint);
                track->updateNextMove(actionID);
                move->computeSpeed();
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::PopItemWidth();
        }
        ImGui::PopStyleVar();
    }
    ImGui::PopStyleColor();

    text = "duration";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
    { // Duration
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(ImGui::CalcTextSize("10000").x);
        std::string id = "##duration" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.7f, 0.7f, 0.7f, 1.f));
        float duration = move->getDuration();
        if (ImGui::InputFloat(id.c_str(), &duration, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
        {
            move->setDuration(duration);
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }
    ImGui::PopStyleColor();

    text = "speed";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
    { // Speed
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(ImGui::CalcTextSize("10000").x);
        std::string id = "##speed" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.7f, 0.7f, 0.7f, 1.f));
        float speed = move->getSpeed();
        if (ImGui::InputFloat(id.c_str(), &speed, 0, 0, "%0.f", ImGuiInputTextFlags_CharsNoBlank))
        {
            move->setSpeed(speed);
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }
    ImGui::PopStyleColor();

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;

    ImGui::PopClipRect();
}

void ProgramWindow::showActionOptionButton(const std::string &menulabel,
                                           const std::string &label)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    auto backuppos = window->DC.CursorPosPrevLine;

    float ysize = ImGui::CalcTextSize(ICON_FA_BARS).y + ImGui::GetStyle().FramePadding.y * 2.0f;
    ImVec2 buttonSize(ysize, ysize);
    window->DC.CursorPos = window->DC.CursorPosPrevLine;
    window->DC.CursorPos.x -= buttonSize.x + ImGui::GetStyle().FramePadding.x;
    window->DC.CursorPos.y += ImGui::GetStyle().FramePadding.y;

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.29f, 0.47f, 0.50f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.42f, 0.45f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.29f, 0.47f, 0.50f, 1.0f));
    std::string buttonlabel = ICON_FA_BARS;
    buttonlabel += "##" + label;
    if (ImGui::Button(buttonlabel.c_str(), buttonSize))
    {
        ImGui::OpenPopup(menulabel.c_str());
    }
    ImGui::PopStyleColor(3);

    window->DC.CursorPosPrevLine = backuppos;
}

bool ProgramWindow::importProgram()
{
    bool successfulImport = false;
    nfdchar_t *outPath;
    std::vector<nfdfilteritem_t> nfd_filters;
    nfd_filters.push_back({"program file", "crprog"});
    nfdresult_t result = NFD_OpenDialog(&outPath, nfd_filters.data(), nfd_filters.size(), nullptr);
    if (result == NFD_OKAY)
    {
        if (sofa::helper::system::FileSystem::exists(outPath))
        {
            successfulImport = m_program.importProgram(outPath);
        }
        NFD_FreePath(outPath);
    }

    return successfulImport;
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
    if (m_isDrivingSimulation)
    {
        m_time = groot->getTime();

        for (const auto& track: m_program.getTracks())
        {
            float blockEnd = 0.f;
            float blockStart = 0.f;
            for (const auto& action: track->getActions())
            {
                blockEnd += action->getDuration();
                if (blockEnd > m_time)
                {
                    std::shared_ptr<models::Move> move = std::dynamic_pointer_cast<models::Move>(action);
                    if(move)
                    {
                        RigidCoord position = move->getInterpolatedPosition(m_time - blockStart);
                        m_TCPTarget->setPosition(position);
                        break;
                    }
                }
                blockStart = blockEnd;
            }

            if (m_time > blockEnd)
            {
                if (m_repeat)
                {
                    m_time = 0.f;
                    groot->setTime(0.);
                }
                else
                {
                    m_time -= groot->getDt();
                    groot->setTime(m_time);
                    groot->setAnimate(false);
                }
            }
        }
    }
}

void ProgramWindow::animateEndEvent(sofa::simulation::Node *groot)
{
    SOFA_UNUSED(groot);
}

} // namespace


























