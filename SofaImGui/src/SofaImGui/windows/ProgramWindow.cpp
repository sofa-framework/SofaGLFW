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

#include <SofaImGui/models/actions/Move.h>
#include <SofaImGui/models/actions/Pick.h>
#include <SofaImGui/models/actions/Wait.h>
#include <SofaImGui/models/modifiers/Repeat.h>

#include <sofa/helper/system/FileSystem.h>

#include <imgui_internal.h>
#include <GLFW/glfw3.h>

#include <nfd.h>
#include <IconsFontAwesome6.h>

#include <ProgramStyle.h>


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
    if (m_isWindowOpen && m_IPController != nullptr)
    {
        if (baseGUI)
            m_baseGUI = baseGUI;
        else
            return;

        ProgramSizes().TrackMaxHeight = ImGui::GetFrameHeightWithSpacing() * 4.55;
        ProgramSizes().TrackMinHeight = ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.y * 2.;
        static bool firstTime = true;
        if (firstTime)
        {
            firstTime = false;
            ProgramSizes().TrackHeight = ProgramSizes().TrackMaxHeight;
        }
        ProgramSizes().InputWidth = ImGui::CalcTextSize("10000").x;
        ProgramSizes().AlignWidth = ImGui::CalcTextSize("iterations    ").x;

        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen,
                         windowFlags | ImGuiWindowFlags_AlwaysAutoResize
                         ))
        {
            showProgramButtons();

            float width = ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight() - ImGui::GetTextLineHeightWithSpacing() * 3.;
            static float zoomCoef = 6.;
            static float minSize = ImGui::GetFrameHeight() * 1.5;
            ProgramSizes().TimelineOneSecondSize = zoomCoef * minSize;
            ProgramSizes().StartMoveBlockSize = 6. * minSize;
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
            if (ImGui::IsItemHovered() && ImGui::IsKeyDown(ImGuiKey_LeftCtrl))
                zoomCoef += ImGui::GetIO().MouseWheel * 0.4f;
            float coefMax = 20.f;
            zoomCoef = (zoomCoef < 1)? 1 : zoomCoef;
            zoomCoef = (zoomCoef > coefMax)? coefMax : zoomCoef;
        }
        ImGui::End();
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
    ImGui::SetCursorPosX(positionMiddle); // Set position to middle of the header

    if (!isDrivingSimulation())
        ImGui::BeginDisabled();

    if (ImGui::Button("Restart"))
    {
        auto groot = m_baseGUI->getRootNode().get();
        groot->setTime(0.);
        m_time = 0.f;

        for (const auto& track: m_program.getTracks())
        {
            const auto& modifiers = track->getModifiers();
            for (const auto& modifier: modifiers)
                modifier->reset();
        }
    }
    ImGui::SetItemTooltip("Restart the program");

    if (!isDrivingSimulation())
        ImGui::EndDisabled();

            // Right pushed buttons
    ImGui::SameLine();
    ImGui::SetCursorPosX(positionRight); // Set position to right of the header

    ImGui::LocalPushButton(ICON_FA_DRAW_POLYGON"##Draw", &m_drawTrajectory, buttonSize);
    ImGui::SetItemTooltip("Draw trajectory");

    ImGui::SameLine();

    ImGui::LocalPushButton(ICON_FA_REPEAT"##Repeat", &m_repeat, buttonSize);
    ImGui::SetItemTooltip("Repeat program");
    if (m_repeat)
        m_reverse = false;

    ImGui::SameLine();

    ImGui::LocalPushButton(ICON_FA_ARROWS_LEFT_RIGHT"##Reverse", &m_reverse, buttonSize);
    ImGui::SetItemTooltip("Reverse and repeat program");
    if (m_reverse)
        m_repeat = false;
}

void ProgramWindow::showCursorMarker(const int& nbCollaspedTracks)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems)
        return;

    ImVec4 color(0.95f, 0.f, 0.f, 1.0f);

    float thicknessRect = 1.0f;
    float widthTri = ImGui::GetStyle().ItemInnerSpacing.x * 2;

    m_cursor = m_time * ProgramSizes().TimelineOneSecondSize;
    double max = ImGui::GetWindowWidth() + ImGui::GetScrollX();
    ImRect grab_bb(ImVec2(m_trackBeginPos.x + m_cursor - widthTri / 2., m_trackBeginPos.y - widthTri),
                   ImVec2(m_trackBeginPos.x + m_cursor + widthTri / 2., m_trackBeginPos.y));
    const ImRect frame_bb(ImVec2(m_trackBeginPos.x, m_trackBeginPos.y - widthTri),
                          ImVec2(m_trackBeginPos.x + max, m_trackBeginPos.y));

    ImVec2 p0Rect(grab_bb.Min.x + widthTri / 2., grab_bb.Min.y);
    ImVec2 p1Rect(p0Rect.x + thicknessRect,
                  m_trackBeginPos.y + ProgramSizes().TrackHeight * m_program.getNbTracks() + ImGui::GetStyle().ItemSpacing.y * (m_program.getNbTracks() - 1)); // TODO: case multiple tracks

    ImVec2 p0Tri(p0Rect.x + thicknessRect / 2.f, grab_bb.Max.y);
    ImVec2 p1Tri(p0Tri.x - widthTri / 2.f, p0Tri.y - widthTri);
    ImVec2 p2Tri(p0Tri.x + widthTri / 2.f,  p0Tri.y - widthTri);

    ImGui::ItemSize(ImVec2(widthTri, widthTri));
    const ImGuiID id = ImGui::GetID("##cursormarker");
    if (!ImGui::ItemAdd(frame_bb, id))
        return;

    ImGuiContext& g = *GImGui;
    const bool hovered = ImGui::ItemHoverable(frame_bb, id, g.LastItemData.InFlags);
    const bool clicked = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left, ImGuiInputFlags_None, id);
    const bool make_active = (clicked || g.NavActivateId == id);
    if (clicked)
        ImGui::SetKeyOwner(ImGuiKey_MouseLeft, id);

    if (make_active)
    {
        ImGui::SetActiveID(id, window);
        ImGui::SetFocusID(id, window);
        ImGui::FocusWindow(window);
        g.ActiveIdUsingNavDirMask |= (1 << ImGuiDir_Left) | (1 << ImGuiDir_Right);
    }

    double min = 0;
    const bool value_changed = ImGui::SliderBehavior(frame_bb, id, ImGuiDataType_Double,
                                                     &m_cursor, &min, &max, "%0.2f", ImGuiSliderFlags_NoInput, &grab_bb);
    if (value_changed)
    {
        ImGui::MarkItemEdited(id);
        const auto& groot = m_IPController->getRootNode().get();
        m_time = m_cursor / ProgramSizes().TimelineOneSecondSize;
        groot->setTime(m_time);
        stepProgram();
    }

    window->DrawList->AddTriangleFilled(p0Tri, p1Tri, p2Tri, ImGui::GetColorU32(color));
    window->DrawList->AddRectFilled(p0Rect, p1Rect, ImGui::GetColorU32(color), 1.0f);
}

void ProgramWindow::showTimeline()
{
    float width = ImGui::GetWindowWidth() + ImGui::GetScrollX();
    int nbSteps = width / ProgramSizes().TimelineOneSecondSize + 1;

    float indentSize = ImGui::GetFrameHeight();
    ImGui::Indent(indentSize);

    ImGuiWindow* window = ImGui::GetCurrentWindow();
    const ImRect frame_bb(ImVec2(m_trackBeginPos.x, m_trackBeginPos.y - ImGui::GetFrameHeight() * 1.5),
                          ImVec2(m_trackBeginPos.x + width, m_trackBeginPos.y));
    const ImGuiID id = ImGui::GetID("##timeline");
    if (!ImGui::ItemAdd(frame_bb, id))
        return;
    ImGui::SetItemTooltip("Simulation time");

    ImGui::BeginGroup(); // Timeline's number (seconds)
    window->DC.CursorPos.x = m_trackBeginPos.x;
    for (int i=0 ; i<nbSteps; i++)
    {
        std::string text = std::to_string(i) + " s";
        float textSize = ImGui::CalcTextSize(text.c_str()).x;
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(ProgramSizes().TimelineOneSecondSize - textSize, 0.f));
        ImGui::Text("%s", text.c_str());
        ImGui::SameLine();
        ImGui::PopStyleVar();
    }
    ImGui::EndGroup();

    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImGui::NewLine();
    ImGui::BeginGroup(); // Timeline's lines
    ImGui::SameLine();
    window->DC.CursorPos.x = m_trackBeginPos.x;
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(ProgramSizes().TimelineOneSecondSize / 10, 0.f));
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
                m_program.addTrack(std::make_shared<models::Track>(m_IPController));
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
                if (models::actions::Pick::gripperInstalled && ImGui::MenuItem(("Pick##" + std::to_string(trackIndex)).c_str()))
                {
                    track->pushAction(std::make_shared<models::actions::Pick>());
                }
                if (ImGui::MenuItem(("Wait##" + std::to_string(trackIndex)).c_str()))
                {
                    track->pushAction(std::make_shared<models::actions::Wait>());
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu(("Add modifier##" + std::to_string(trackIndex)).c_str()))
            {
                if (track->getActions().empty() || !track->getModifiers().empty()) // TODO: handle showing multiple modifiers
                    ImGui::BeginDisabled();
                if (ImGui::MenuItem(("Repeat##" + std::to_string(trackIndex)).c_str()))
                {
                    track->pushRepeat();
                }
                ImGui::EndMenu();
                if (track->getActions().empty() || !track->getModifiers().empty())
                    ImGui::EndDisabled();
            }
            ImGui::EndPopup();
        }

        bool collapsed = showTrackButtons(trackIndex, menuLabel.c_str());
        if (collapsed)
        {
            nbCollapsedTrack++;
        }

        ImGui::SameLine();
        m_trackBeginPos = ImGui::GetCurrentWindow()->DC.CursorPos;
        m_trackBeginPos.x += ProgramSizes().StartMoveBlockSize;
        showBlocks(track, trackIndex);

        { // Empty track background
            ImGui::SameLine();
            std::string trackLabel = "##Track" + std::to_string(trackIndex) + "Empty";
            ImVec2 size(ImGui::GetWindowWidth() + ImGui::GetScrollX(), ProgramSizes().TrackHeight);

            float x = ImGui::GetCurrentWindow()->DC.CursorPos.x ;
            float y = ImGui::GetCurrentWindow()->DC.CursorPos.y ;
            ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));

            ImGui::ItemSize(size);
            if (ImGui::ItemAdd(bb, ImGui::GetID(trackLabel.c_str())))
            { // Backgroung
                ImGui::GetWindowDrawList()->AddRectFilled(bb.Min, bb.Max,
                                                          ImGui::GetColorU32(ImGuiCol_FrameBg),
                                                          ImGui::GetStyle().FrameRounding,
                                                          ImDrawFlags_None);
            }
        }

        trackIndex++;
    }

    return nbCollapsedTrack;
}

bool ProgramWindow::showTrackButtons(const int &trackIndex, const char* const menuLabel)
{
    static bool collapsed = false;
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 size(ImGui::GetFrameHeight(), ProgramSizes().TrackHeight);

    float x = window->DC.CursorPos.x ;
    float y = window->DC.CursorPos.y ;

    ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));
    ImVec2 topRight = ImVec2(x + size.x, y);

    std::string label = "##TrackButtons" + std::to_string(trackIndex);
    ImGui::ItemSize(size);
    const ImGuiID id = ImGui::GetID(label.c_str());
    if (!ImGui::ItemAdd(bb, id))
        return collapsed;

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

    if (!collapsed || ProgramSizes().TrackHeight > ProgramSizes().TrackMinHeight) // hide the button at the end of the collapsing animation
    {
        std::string optionlabel = ICON_FA_BARS"##TrackOption" + std::to_string(trackIndex);
        if(ImGui::Button(optionlabel.c_str(), ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight())))
        {
            ImGui::OpenPopup(menuLabel);
        }
    }

    window->DC.CursorPos.x = x;
    window->DC.CursorPos.y = y + (collapsed? (ProgramSizes().TrackHeight - ImGui::GetFrameHeight()) / 2.f :
                                      ProgramSizes().TrackHeight - ImGui::GetFrameHeight()) ;
    ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.5, 1)); // Align icon down middle

    std::string collapselabel = "##TrackCollapse" + std::to_string(trackIndex);
    std::vector<std::string> icons{ICON_FA_COMPRESS, ICON_FA_EXPAND};
    static std::string icon = icons[collapsed];
    ImGui::Button((icon + collapselabel).c_str(), ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight()));
    if (ImGui::IsItemClicked())
    {
        collapsed = !collapsed;
    }

    // Animate collapse
    ImGuiContext& g = *GImGui;
    float ANIM_SPEED = 0.08f;
    if (g.LastActiveId == g.CurrentWindow->GetID((icon + collapselabel).c_str()))
    {
        float t_anim = ImSaturate(g.LastActiveIdTimer / ANIM_SPEED);
        ProgramSizes().TrackHeight = collapsed ? ProgramSizes().TrackMinHeight * t_anim + ProgramSizes().TrackMaxHeight * (1 - t_anim) :
                                                 ProgramSizes().TrackMinHeight * (1 - t_anim) + ProgramSizes().TrackMaxHeight * t_anim;
        if (t_anim >= 1) // change the icon at the end of animation
            icon = icons[collapsed];
    }

    ImGui::SetItemTooltip(collapsed? "Expend track": "Collapse track");
    ImGui::PopStyleVar(); // End align icon down middle
    ImGui::PopStyleColor(3); // Color of track button

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;

    return collapsed;
}

void ProgramWindow::showBlocks(std::shared_ptr<models::Track> track,
                               const int& trackIndex)
{
    float x = ImGui::GetCurrentWindow()->DC.CursorPos.x ;
    float y = ImGui::GetCurrentWindow()->DC.CursorPos.y ;
    float blockHeight = ProgramSizes().TrackHeight;

    // Modifiers blocks
    const std::vector<std::shared_ptr<models::modifiers::Modifier>> &modifiers = track->getModifiers();
    sofa::Index modifierIndex = 0;
    while(modifierIndex < modifiers.size())
    {
        std::shared_ptr<models::modifiers::Modifier> modifier = modifiers[modifierIndex];
        float blockWidth = modifier->getDuration() * ProgramSizes().TimelineOneSecondSize - ImGui::GetStyle().ItemSpacing.x;
        std::string blockLabel = "##Modifier" + std::to_string(trackIndex) + std::to_string(modifierIndex);
        std::string menuLabel = std::string("##ModifierOptionsMenu" + blockLabel);
        ImGui::SameLine();

        modifier->getView()->showBlock(blockLabel, ImVec2(blockWidth, blockHeight), m_trackBeginPos);

        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::MenuItem("Delete"))
            {
                track->deleteModifier(modifierIndex);
            }
            else
                modifierIndex++;
            ImGui::EndPopup();
        } else {
            modifierIndex++;
        }

        if (blockWidth > ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 2.0f)
            showBlockOptionButton(menuLabel, blockLabel);
    }

    ImGui::GetCurrentWindow()->DC.CursorPosPrevLine.x = x;
    ImGui::GetCurrentWindow()->DC.CursorPosPrevLine.y = y;

    // StartMove block
    {
        std::shared_ptr<models::actions::StartMove> startmove = track->getStartMove();
        std::string blockLabel = "##StartMove" + std::to_string(trackIndex);
        if (startmove->getView()->showBlock(blockLabel, ImVec2(ProgramSizes().StartMoveBlockSize, blockHeight)))
        {
            track->updateNextMoveInitialPoint(-1, startmove->getWaypoint());
        }
    }

    // Action blocks
    const std::vector<std::shared_ptr<models::actions::Action>> &actions = track->getActions();
    sofa::Index actionIndex = 0;
    while(actionIndex < actions.size())
    {
        std::shared_ptr<models::actions::Action> action = actions[actionIndex];
        float blockWidth = action->getDuration() * ProgramSizes().TimelineOneSecondSize - ImGui::GetStyle().ItemSpacing.x;
        std::string blockLabel = "##Action" + std::to_string(trackIndex) + std::to_string(actionIndex);
        std::string menuLabel = std::string("##ActionOptionsMenu" + blockLabel);
        ImGui::SameLine();

        if (ImGui::BeginPopup(menuLabel.c_str()))
        {
            if (ImGui::BeginMenu("Add before"))
            {
                if (ImGui::MenuItem("Move"))
                    track->insertMove(actionIndex);
                if (models::actions::Pick::gripperInstalled && ImGui::MenuItem("Pick"))
                    track->insertAction(actionIndex, std::make_shared<models::actions::Pick>());
                if (ImGui::MenuItem("Wait"))
                    track->insertAction(actionIndex, std::make_shared<models::actions::Wait>());
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Add after"))
            {
                if (ImGui::MenuItem("Move"))
                    track->insertMove(actionIndex + 1);
                if (models::actions::Pick::gripperInstalled && ImGui::MenuItem("Pick"))
                    track->insertAction(actionIndex + 1, std::make_shared<models::actions::Pick>());
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
            move->setDrawTrajectory(m_drawTrajectory);
            if(move->getView()->showBlock(blockLabel, ImVec2(blockWidth, blockHeight)))
            {
                track->updateNextMoveInitialPoint(actionIndex, move->getWaypoint());
            }
            // Options menu
            if (ImGui::BeginPopup(menuLabel.c_str()))
            {
                if (ImGui::MenuItem("Overwrite waypoint"))
                {
                    move->setWaypoint(m_IPController->getTCPTargetPosition());
                    track->updateNextMoveInitialPoint(actionIndex, move->getWaypoint());
                }
                ImGui::Separator();
                ImGui::EndPopup();
            }
        } else {
            action->getView()->showBlock(blockLabel, ImVec2(blockWidth, blockHeight));
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
            showBlockOptionButton(menuLabel, blockLabel);
    }
}

void ProgramWindow::showBlockOptionButton(const std::string &menulabel,
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

void ProgramWindow::stepProgram(const double &dt, const bool &reverse)
{
    if (m_isDrivingSimulation)
    {
        double eps = 1e-5;
        for (const auto& track: m_program.getTracks())
        {
            double blockEnd = 0;
            double blockStart = 0;
            const auto& actions = track->getActions();
            for (const auto& action: actions)
            {
                blockEnd += action->getDuration();
                if ((!reverse && (blockEnd - m_time) > eps) || (reverse && (blockEnd - m_time - dt) > eps))
                {
                    RigidCoord position = m_IPController->getTCPPosition();
                    if (action->apply(position, m_time + dt - blockStart)) // apply the time corresponding to the end of the time step
                    {
                        m_IPController->setTCPTargetPosition(position);
                    }
                    break;
                }
                blockStart = blockEnd;
            }
        }
    }
}

void ProgramWindow::animateBeginEvent(sofa::simulation::Node *groot)
{
    if (m_isDrivingSimulation)
    {
        if (m_program.isEmpty())
            return;

        double eps = 1e-5;
        static bool reverse = false;
        double dt = reverse? -groot->getDt(): groot->getDt();
        double programDuration = m_program.getDuration();

        for (const auto& track: m_program.getTracks()) // allow the mofifiers to do their jobs first
        {
            const auto& modifiers = track->getModifiers();
            for (const auto& modifier: modifiers)
            {
                modifier->modify(m_time);
                groot->setTime(m_time);
            }
        }

        if (groot->getTime() >= programDuration - eps) // if we've reached the end of the program
        {
            if (m_repeat) // start from beginning
            {
                groot->setTime(0.);

                for (const auto& track: m_program.getTracks())
                {
                    const auto& modifiers = track->getModifiers();
                    for (const auto& modifier: modifiers)
                        modifier->reset();
                }
            }
            else if (m_reverse)
            {
                reverse = true;
                dt = -groot->getDt();
            }
            else // nothing to do, exit
            {
                groot->setTime(programDuration);
                m_time = programDuration;
                return;
            }
        }

        if (reverse)
        {
            if (groot->getTime() <= eps)
            {
                reverse = false;
                dt = groot->getDt();
            }
        }

        m_time = groot->getTime(); // time at the beginning of the time step

        stepProgram(dt, reverse);

        m_time += dt; // for cursor display
    } // isDrivingSimulation
}

void ProgramWindow::animateEndEvent(sofa::simulation::Node *groot)
{
    SOFA_UNUSED(groot);
    if (m_isDrivingSimulation)
        groot->setTime(m_time);
}

void ProgramWindow::setIPController(models::IPController::SPtr IPController)
{
    m_IPController = IPController;
    m_program = models::Program(IPController);
}

void ProgramWindow::setDrivingTCPTarget(const bool &isDrivingSimulation)
{
    m_isDrivingSimulation=isDrivingSimulation;
}

void ProgramWindow::addTrajectoryComponents(sofa::simulation::Node* groot)
{
    const auto& tracks = m_program.getTracks();
    for (const auto& track: tracks)
    {
        const auto& actions = track->getActions();
        for (const auto& action: actions)
        {
            std::shared_ptr<models::actions::Move> move = std::dynamic_pointer_cast<models::actions::Move>(action);
            if(move)
                move->addTrajectoryComponent(groot);
        }
    }
}

} // namespace


























