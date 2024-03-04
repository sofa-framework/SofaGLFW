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
#include <SofaImGui/models/actions/Action.h>
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

        getSizes().trackHeight = ImGui::GetFrameHeightWithSpacing() * 4.55;
        getSizes().trackCollapsedHeight = ImGui::GetFrameHeightWithSpacing() * 1.9;
        getSizes().inputWidth = ImGui::CalcTextSize("10000").x;
        getSizes().alignWidth = ImGui::CalcTextSize("iterations    ").x;

        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen,
                         windowFlags | ImGuiWindowFlags_AlwaysAutoResize
                         ))
        {
            showProgramButtons();

            float width = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() - ImGui::GetTextLineHeightWithSpacing() * 3.;
            static float zoomCoef = 1.;
            static float minSize = ImGui::GetFrameHeight() * 1.5;
            m_timelineOneSecondSize = zoomCoef * minSize;
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetColorU32(ImGuiCol_WindowBg));
            if (ImGui::BeginChildFrame(ImGui::GetID(m_name.c_str()), ImVec2(width, height),
                                       ImGuiWindowFlags_AlwaysHorizontalScrollbar))
            {
                ImGui::PopStyleColor();

                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 6));
                showTimeline();
                int nbCollaspedTracks = showTracks();
                showCursorMarker(nbCollaspedTracks);
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

    ImGui::PushStyleColor(ImGuiCol_Button, m_reverse? ImVec4(0.25f, 0.25f, 0.25f, 1.00f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(ICON_FA_ARROWS_LEFT_RIGHT"##Reverse", buttonSize))
    {
        m_repeat = false;
        m_reverse = !m_reverse;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Reverse and repeat program");
}

void ProgramWindow::showCursorMarker(const int& nbCollaspedTracks)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImVec4 color(0.95f, 0.f, 0.f, 1.0f);

    float thicknessRect = ImGui::GetStyle().SeparatorTextBorderSize;
    float widthTri = ImGui::GetStyle().ItemInnerSpacing.x * 2;

    m_cursor = m_time * m_timelineOneSecondSize;
    ImVec2 p0Rect(m_trackBeginPos.x + m_cursor - window->Scroll.x, m_trackBeginPos.y);
    ImVec2 p1Rect(p0Rect.x + thicknessRect,
                  p0Rect.y + getSizes().trackHeight * (m_program.getNbTracks() - nbCollaspedTracks) + getSizes().trackCollapsedHeight * nbCollaspedTracks + ImGui::GetStyle().ItemSpacing.y * (m_program.getNbTracks() - 1));

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

    float indentSize = ImGui::GetFrameHeight();
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

int ProgramWindow::showTracks()
{
    const auto& tracks = m_program.getTracks();

    int trackIndex = 0;
    int nbCollapsedTrack = 0;

    for (const auto& track: tracks)
    {
        // Track options menu
        std::string menuLabel = "##TrackMenu" + std::to_string(trackIndex);
        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::MenuItem(("Clear track##" + std::to_string(trackIndex)).c_str()))
            {
                track->clear();
            }
            if (ImGui::MenuItem(("Add track##" + std::to_string(trackIndex)).c_str(), nullptr, false, false))
            {
                m_program.addTrack(std::make_shared<models::Track>(m_TCPTarget));
            }
            if (ImGui::MenuItem(("Remove track##" + std::to_string(trackIndex)).c_str(), nullptr, false, (trackIndex>0)? true : false))
            {
                m_program.removeTrack(trackIndex--);
            }

            ImGui::Separator();

            if (ImGui::BeginMenu(("Add action##" + std::to_string(trackIndex)).c_str()))
            {
                if (ImGui::MenuItem(("Move##" + std::to_string(trackIndex)).c_str()))
                {
                    track->pushMove();
                }
                if (ImGui::MenuItem(("Wait##" + std::to_string(trackIndex)).c_str()))
                {
                    track->pushAction(std::make_shared<models::actions::Wait>());
                }
                ImGui::EndMenu();
            }
            ImGui::EndPopup();
        }

        bool collapsed = showTrackButtons(trackIndex, menuLabel.c_str());
        if (collapsed)
            nbCollapsedTrack++;

        ImGui::SameLine();
        m_trackBeginPos = ImGui::GetCurrentWindow()->DC.CursorPos;
        showBlocks(track, trackIndex, collapsed);
        trackIndex++;
    }

    return nbCollapsedTrack;
}

bool ProgramWindow::showTrackButtons(const int &trackIndex, const char* const menuLabel)
{
    static bool collapsed = false;
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 size(ImGui::GetFrameHeight(), collapsed? getSizes().trackCollapsedHeight : getSizes().trackHeight);

    float x = window->DC.CursorPos.x ;
    float y = window->DC.CursorPos.y ;

    ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));
    ImVec2 topRight = ImVec2(x + size.x, y);

    std::string label = "##TrackButtons" + std::to_string(trackIndex);
    ImGui::ItemSize(size);
    const ImGuiID id = ImGui::GetID(label.c_str());
    if (!ImGui::ItemAdd(bb, id))
        return false;

    { // Block backgroung
        drawList->AddRectFilled(bb.Min, bb.Max,
                                ImGui::GetColorU32(ImGuiCol_MenuBarBg),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);
    }

    window->DC.CursorPos.x = x;
    window->DC.CursorPos.y = y;
    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_MenuBarBg)); // Color of track button
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_MenuBarBg)); // Color of track button
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_MenuBarBg)); // Color of track button

    std::string optionlabel = ICON_FA_BARS"##TrackOption" + std::to_string(trackIndex);
    if(ImGui::Button(optionlabel.c_str(), ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight())))
    {
        ImGui::OpenPopup(menuLabel);
    }

    window->DC.CursorPos.x = x;
    window->DC.CursorPos.y = y + (collapsed? getSizes().trackCollapsedHeight : getSizes().trackHeight) - ImGui::GetFrameHeight();
    ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5, 1)); // Align icon down middle
    std::string collapselabel = collapsed? ICON_FA_CHEVRON_DOWN : ICON_FA_CHEVRON_UP;
    collapselabel += "##TrackCollapse" + std::to_string(trackIndex);
    if (ImGui::Button(collapselabel.c_str(), ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight())))
    {
        collapsed = !collapsed;
    }
    ImGui::SetItemTooltip(collapsed? "Expend track": "Collapse track");
    ImGui::PopStyleVar(); // End align icon down middle
    ImGui::PopStyleColor(3); // Color of track button

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;

    return collapsed;
}

void ProgramWindow::showBlocks(std::shared_ptr<models::Track> track,
                               const int& trackIndex,
                               const bool &collapsed)
{
    const std::vector<std::shared_ptr<models::actions::Action>> &actions = track->getActions();

            // Action blocks
    sofa::Index actionIndex = 0;
    while(actionIndex < actions.size())
    {
        std::shared_ptr<models::actions::Action> action = actions[actionIndex];
        float blockWidth = action->getDuration() * m_timelineOneSecondSize - ImGui::GetStyle().ItemSpacing.x;
        float blockHeight = collapsed? getSizes().trackCollapsedHeight: getSizes().trackHeight;
        std::string blockLabel = "##Action" + std::to_string(trackIndex) + std::to_string(actionIndex);
        std::string menuLabel = std::string("##ActionOptionsMenu" + blockLabel);
        ImGui::SameLine();

        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::BeginMenu("Add before"))
            {
                if (ImGui::MenuItem("Move"))
                    track->insertMove(actionIndex);
                if (ImGui::MenuItem("Wait"))
                    track->insertAction(actionIndex, std::make_shared<models::actions::Wait>());
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Add after"))
            {
                if (ImGui::MenuItem("Move"))
                    track->insertMove(actionIndex + 1);
                if (ImGui::MenuItem("Wait"))
                    track->insertAction(actionIndex + 1, std::make_shared<models::actions::Wait>());
                ImGui::EndMenu();
            }

            ImGui::Separator();
            ImGui::EndPopup();
        }

        std::shared_ptr<models::actions::Move> move = std::dynamic_pointer_cast<models::actions::Move>(action);
        if (move)
        {
            move->drawTrajectory(m_drawTrajectory);
            showMoveBlock(track, actionIndex, move, blockLabel, ImVec2(blockWidth, blockHeight));
            // Options menu
            if (ImGui::BeginPopup(menuLabel.c_str()))
            {
                if (ImGui::MenuItem("Overwrite waypoint"))
                {
                    move->setWaypoint(m_TCPTarget->getPosition());
                    track->updateNextMoveInitialPoint(actionIndex, move->getWaypoint());
                }
                ImGui::Separator();
                ImGui::EndPopup();
            }
        }

        std::shared_ptr<models::actions::Wait> wait = std::dynamic_pointer_cast<models::actions::Wait>(action);
        if (wait)
        {
            showWaitBlock(wait, blockLabel, ImVec2(blockWidth, blockHeight));
        }

        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::MenuItem("Delete"))
            {
                if (move)
                    track->deleteMove(actionIndex);
                else
                    track->deleteAction(actionIndex);
            }
            else
                actionIndex++;
            ImGui::EndPopup();
        } else {
            actionIndex++;
        }

        if (blockWidth > ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 2.0f)
            showActionOptionButton(menuLabel, blockLabel);
    }

    { // Empty track background
        ImGui::SameLine();
        std::string trackLabel = "##Track" + std::to_string(trackIndex) + "Empty";
        ImVec2 size(ImGui::GetWindowWidth() + ImGui::GetScrollX(), collapsed? getSizes().trackCollapsedHeight : getSizes().trackHeight);

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


void ProgramWindow::showRepeatBlock(std::shared_ptr<models::modifiers::Repeat> repeat,
                                    const std::string &label,
                                    const ImVec2 &size,
                                    const bool& collapsed)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float alignWidth = getSizes().alignWidth;
    ImVec2 padding(ImGui::GetStyle().FramePadding);
    ImVec2 spacing(ImGui::GetStyle().ItemSpacing);
    ImVec2 innerspacing(ImGui::GetStyle().ItemInnerSpacing);
    float x = window->DC.CursorPos.x - size.x - spacing.x;
    float y = window->DC.CursorPos.y + size.y + spacing.y;

    ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + (collapsed? ImGui::GetFrameHeightWithSpacing() + innerspacing.y: size.y)));
    ImVec2 topRight = ImVec2(window->DC.CursorPosPrevLine.x, window->DC.CursorPosPrevLine.y);

    ImGui::ItemSize(size);
    const ImGuiID id = ImGui::GetID(label.c_str());
    if (!ImGui::ItemAdd(bb, id))
        return;

    { // Block backgroung
        drawList->AddRectFilled(bb.Min, bb.Max,
                                ImGui::GetColorU32(getColors().RepeatBlockBg),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);
    }

    std::string text = "Repeat";
    ImVec2 textSize = ImGui::CalcTextSize(text.c_str());

    auto rectMin = ImGui::GetItemRectMin();
    auto rectMax = ImGui::GetItemRectMax();
    rectMax.x -= padding.x;
    ImGui::PushClipRect(rectMin, rectMax, true);

    ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
    { // Reapeat
        x += padding.y;
        y += padding.y;
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + size.x - padding.x * 2,
                        y + textSize.y + padding.y * 2);
        drawList->AddRectFilled(bb.Min, bb.Max,
                                ImGui::GetColorU32(getColors().RepeatBlockTitleBg),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);

        window->DC.CursorPos.x = x;
        window->DC.CursorPos.y = y;

        auto rectMin = ImGui::GetItemRectMin();
        auto rectMax = ImGui::GetItemRectMax();
        rectMax.x -= padding.x * 2 + ImGui::GetFrameHeight(); // leave space for option button
        ImGui::PushClipRect(rectMin, rectMax, true);

        std::string id = "##comment" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0., 0., 0., 0.));
        ImGui::InputText(id.c_str(), repeat->getComment(), models::actions::Action::COMMENTSIZE);
        ImGui::PopStyleColor();

        ImGui::PopClipRect();
    }
    ImGui::PopStyleColor();

    text = "iterations";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    { // Iterations
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(getSizes().inputWidth * 2);
        std::string id = "##iterations" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
        ImGui::InputInt(id.c_str(), &repeat->getIterations());
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    text = "start time";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = spacing.y + bb.Max.y;

    { // Start time
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(getSizes().inputWidth);
        std::string id = "##starttime" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
        float startTime = repeat->getStartTime();
        if (ImGui::InputFloat(id.c_str(), &startTime, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
        {
            repeat->setStartTime(startTime);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    text = "type";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = spacing.y + bb.Max.y;

    { // Way point rotation
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth((getSizes().inputWidth + spacing.x) * 4);
        std::string id = "##speed" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
        static const char* items[]{"REPEAT", "REVERSE"};
        int type = repeat->getType();
        if (ImGui::Combo(id.c_str(), &type, items, IM_ARRAYSIZE(items)))
        {
            repeat->setType(models::modifiers::Repeat::Type(type));
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;

    ImGui::PopClipRect();
}

void ProgramWindow::showWaitBlock(std::shared_ptr<models::actions::Wait> wait,
                                  const std::string &label,
                                  const ImVec2 &size)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float alignWidth = getSizes().alignWidth;
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
                                ImGui::GetColorU32(getColors().WaitBlockBg),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);
    }

    std::string text = "Move to Way Point";
    ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
    ImVec2 padding(ImGui::GetStyle().FramePadding);

    auto rectMin = ImGui::GetItemRectMin();
    auto rectMax = ImGui::GetItemRectMax();
    rectMax.x -= padding.x;
    ImGui::PushClipRect(rectMin, rectMax, true);

    ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
    { // Wait
        x += padding.y;
        y += padding.y;
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + size.x - padding.x * 2,
                        y + textSize.y + padding.y * 2);
        drawList->AddRectFilled(bb.Min, bb.Max,
                                ImGui::GetColorU32(getColors().WaitBlockTitleBg),
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
        ImGui::InputText(id.c_str(), wait->getComment(), models::actions::Action::COMMENTSIZE);
        ImGui::PopStyleColor();

        ImGui::PopClipRect();
    }
    ImGui::PopStyleColor();

    text = "duration";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    { // Duration
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(getSizes().inputWidth);
        std::string id = "##duration" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
        float duration = wait->getDuration();
        if (ImGui::InputFloat(id.c_str(), &duration, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
        {
            wait->setDuration(duration);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;

    ImGui::PopClipRect();
}

void ProgramWindow::showMoveBlock(std::shared_ptr<models::Track> track,
                                  const sofa::Index &moveID,
                                  std::shared_ptr<models::actions::Move> move,
                                  const std::string &label,
                                  const ImVec2 &size)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float alignWidth = getSizes().alignWidth;
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
                                ImGui::GetColorU32(getColors().MoveBlockBg),
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
    ImVec2 spacing(ImGui::GetStyle().ItemSpacing);

    auto rectMin = ImGui::GetItemRectMin();
    auto rectMax = ImGui::GetItemRectMax();
    rectMax.x -= padding.x;
    ImGui::PushClipRect(rectMin, rectMax, true);

    ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
    { // Move
        x += padding.y;
        y += padding.y;
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + size.x - padding.x * 2,
                        y + textSize.y + padding.y * 2);
        drawList->AddRectFilled(bb.Min, bb.Max,
                                ImGui::GetColorU32(getColors().MoveBlockTitleBg),
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
        ImGui::InputText(id.c_str(), move->getComment(), models::actions::Action::COMMENTSIZE);
        ImGui::PopStyleColor();

        ImGui::PopClipRect();
    }
    ImGui::PopStyleColor();

    text = "duration";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    { // Duration
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(getSizes().inputWidth);
        std::string id = "##duration" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
        float duration = move->getDuration();
        if (ImGui::InputFloat(id.c_str(), &duration, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
        {
            move->setDuration(duration);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    text = "speed";
    textSize = ImGui::CalcTextSize(text.c_str());
    float nx = x + alignWidth + getSizes().inputWidth + spacing.x * 4;

    { // Speed
        bb.Min = ImVec2(nx, y);
        bb.Max = ImVec2(nx + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(nx + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = nx + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(getSizes().inputWidth);
        std::string id = "##speed" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
        float speed = move->getSpeed();
        if (ImGui::InputFloat(id.c_str(), &speed, 0, 0, "%0.f", ImGuiInputTextFlags_CharsNoBlank))
        {
            move->setSpeed(speed);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    text = "wp.pos";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = spacing.y + bb.Max.y;

    { // Way point position
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        RigidCoord waypoint = move->getWaypoint();
        for (int i=0; i<3; i++)
        {
            window->DC.CursorPos.y = y;
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);

            ImGui::PushItemWidth(getSizes().inputWidth);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
            ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
            if (ImGui::InputFloat(id.c_str(), &waypoint[i], 0, 0, "%0.f", ImGuiInputTextFlags_CharsNoBlank))
            {
                move->setWaypoint(waypoint);
                track->updateNextMoveInitialPoint(moveID, move->getWaypoint());
            }
            ImGui::PopStyleColor(2);
            ImGui::PopItemWidth();

            ImGui::SameLine();
        }
        ImGui::PopStyleVar();
    }

    text = "wp.rot";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = spacing.y + bb.Max.y;

    { // Way point rotation
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        RigidCoord waypoint = move->getWaypoint();
        for (int i=3; i<7; i++)
        {
            window->DC.CursorPos.y = y;
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);

            ImGui::PushItemWidth(getSizes().inputWidth);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
            ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
            if (ImGui::InputFloat(id.c_str(), &waypoint[i], 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
            {
                move->setWaypoint(waypoint);
                track->updateNextMoveInitialPoint(moveID, move->getWaypoint());
                move->computeSpeed();
            }
            ImGui::PopStyleColor(2);
            ImGui::PopItemWidth();

            ImGui::SameLine();
        }
        ImGui::PopStyleVar();
    }

    text = "type";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = spacing.y + bb.Max.y;

    { // Way point rotation
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, getColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth((getSizes().inputWidth + spacing.x) * 4);
        std::string id = "##speed" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, getColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, getColors().FrameText);
        static const char* items[]{"LINE"};
        int type = move->getType();
        if (ImGui::Combo(id.c_str(), &type, items, IM_ARRAYSIZE(items)))
        {
            move->setType(models::actions::Move::Type(type));
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

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
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 1.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 1.0f, 1.0f, 0.05f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 1.0f, 1.0f, 0.0f));
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
        static bool reverse = false;
        if (reverse)
        {
            groot->setTime(groot->getTime() - 2 * groot->getDt());
            if (groot->getTime() <= 0.)
            {
                reverse = false;
            }
        }
        m_time = groot->getTime();

        for (const auto& track: m_program.getTracks())
        {
            float blockEnd = 0.f;
            float blockStart = 0.f;
            const auto& actions = track->getActions();
            for (const auto& action: actions)
            {
                blockEnd += action->getDuration();
                if ((blockEnd >= m_time && !reverse) || (blockEnd > m_time && reverse))
                {
                    std::shared_ptr<models::actions::Move> move = std::dynamic_pointer_cast<models::actions::Move>(action);
                    if(move)
                    {
                        RigidCoord position = move->getInterpolatedPosition(m_time - blockStart);
                        m_TCPTarget->setPosition(position);
                        break;
                    }

                    std::shared_ptr<models::actions::Wait> wait = std::dynamic_pointer_cast<models::actions::Wait>(action);
                    if(wait)
                    {
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
                } else if (m_reverse)
                {
                    reverse = true;
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

void ProgramWindow::setTCPTarget(std::shared_ptr<models::TCPTarget> TCPTarget)
{
    m_TCPTarget = TCPTarget;
    m_program = models::Program(TCPTarget);
}

void ProgramWindow::setDrivingTCPTarget(const bool &isDrivingSimulation)
{
    if (isDrivingSimulation && !m_isDrivingSimulation)
    {
        const auto& groot = m_TCPTarget->getRootNode().get();
        groot->setTime(0.f);
        m_time = 0.f;
    }
    m_isDrivingSimulation=isDrivingSimulation;
}

void ProgramWindow::update(sofa::simulation::Node* groot)
{
    const auto& tracks = m_program.getTracks();
    for (const auto& track: tracks)
    {
        const auto& actions = track->getActions();
        for (const auto& action: actions)
        {
            std::shared_ptr<models::actions::Move> move = std::dynamic_pointer_cast<models::actions::Move>(action);
            if(move)
                move->addTrajectory(groot);
        }
    }
}

} // namespace


























