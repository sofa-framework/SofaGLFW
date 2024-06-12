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

#include <SofaImGui/models/actions/StartMove.h>
#include <imgui_internal.h>
#include <ProgramStyle.h>
#include <SofaImGui/widgets/Buttons.h>
#include <IconsFontAwesome6.h>

namespace sofaimgui::models::actions {

bool StartMove::StartMoveView::showBlock(const std::string &label,
                                         const ImVec2 &size)
{
    bool hasValuesChanged = false;
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    double x = window->DC.CursorPos.x ;
    double y = window->DC.CursorPos.y ;

    ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));
    ImVec2 topRight = ImVec2(x + size.x, y);

    ImGui::ActionBlock(label.c_str(), bb, ProgramColors().StartMoveBlockBg);

    ImVec2 padding(ImGui::GetStyle().FramePadding);
    ImVec2 spacing(ImGui::GetStyle().ItemSpacing);

    auto rectMin = ImGui::GetItemRectMin();
    auto rectMax = ImGui::GetItemRectMax();
    rectMax.x -= padding.x;
    ImGui::PushClipRect(rectMin, rectMax, true);

    ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
    { // Move
        x += padding.y;
        y += padding.y;

        window->DC.CursorPos.x = x;
        window->DC.CursorPos.y = y;

        auto rectMin = ImGui::GetItemRectMin();
        auto rectMax = ImGui::GetItemRectMax();
        rectMax.x -= padding.x * 2 + ImGui::GetFrameHeight(); // leave space for option button
        ImGui::PushClipRect(rectMin, rectMax, true);

        std::string id = "##comment" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0., 0., 0., 0.));
        if (ImGui::InputText(id.c_str(), start.getComment(), models::actions::Action::COMMENTSIZE))
        {
            hasValuesChanged = true;
        }
        ImGui::PopStyleColor();

        ImGui::PopClipRect();
    }
    ImGui::PopStyleColor();

    std::string text = "wp.pos";
    ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
    y += textSize.y + padding.y * 3;

    { // Way point position
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + ProgramSizes().AlignWidth;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        RigidCoord waypoint = start.getWaypoint();
        for (int i=0; i<3; i++)
        {
            window->DC.CursorPos.y = y;
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);

            ImGui::PushItemWidth(ProgramSizes().InputWidth);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
            ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
            if (ImGui::InputDouble(id.c_str(), &waypoint[i], 0, 0, "%0.f", ImGuiInputTextFlags_CharsNoBlank))
            {
                hasValuesChanged = true;
                start.setWaypoint(waypoint);
                start.computeSpeed();
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

        window->DC.CursorPos.x = x;
        window->DC.CursorPos.y = y;

        std::string label = text + " " + (start.isFreeInRotation()? ICON_FA_LOCK_OPEN: ICON_FA_LOCK) + "##wp.rot" + std::to_string(window->DC.CursorPos.x);
        if (ImGui::Button(label.c_str()))
        {
            start.setFreeInRotation(!start.isFreeInRotation());
        }
        ImGui::SetItemTooltip("When unlocked, TCP movement is free in rotation.");

        window->DC.CursorPos.x = x + ProgramSizes().AlignWidth;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        RigidCoord waypoint = start.getWaypoint();
        for (int i=3; i<7; i++)
        {
            window->DC.CursorPos.y = y;
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);

            ImGui::PushItemWidth(ProgramSizes().InputWidth);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
            ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);

            if(start.isFreeInRotation())
                ImGui::BeginDisabled();
            if (ImGui::InputDouble(id.c_str(), &waypoint[i], 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
            {
                hasValuesChanged = true;
                start.setWaypoint(waypoint);
                start.computeSpeed();
            }
            if(start.isFreeInRotation())
                ImGui::EndDisabled();

            ImGui::PopStyleColor(2);
            ImGui::PopItemWidth();

            ImGui::SameLine();
        }
        ImGui::PopStyleVar();
    }

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;

    ImGui::PopClipRect();
    return hasValuesChanged;
}

} // namespace


