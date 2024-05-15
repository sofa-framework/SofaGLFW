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

#include <SofaImGui/models/actions/Pick.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <ProgramStyle.h>
#include <SofaImGui/widgets/Buttons.h>


namespace sofaimgui::models::actions {

bool Pick::PickView::showBlock(const std::string &label,
                               const ImVec2 &size)
{
    bool hasValuesChanged = false;
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float x = window->DC.CursorPos.x ;
    float y = window->DC.CursorPos.y ;

    ImVec2 padding(ImGui::GetStyle().FramePadding);
    ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));
    ImVec2 topRight = ImVec2(x + size.x, y);

    ImGui::ActionBlock(label.c_str(), bb, ProgramColors().PickBlockBg);

    std::string text = "Pick";
    ImVec2 textSize = ImGui::CalcTextSize(text.c_str());


    auto rectMin = ImGui::GetItemRectMin();
    auto rectMax = ImGui::GetItemRectMax();
    rectMax.x -= padding.x;
    ImGui::PushClipRect(rectMin, rectMax, true);

    ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
    { // Pick
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
        if(ImGui::InputText(id.c_str(), pick.getComment(), models::actions::Action::COMMENTSIZE))
        {
            hasValuesChanged = true;
        }
        ImGui::PopStyleColor();

        ImGui::PopClipRect();
    }
    ImGui::PopStyleColor();

    text = "duration";
    textSize = ImGui::CalcTextSize(text.c_str());
    y += textSize.y + padding.y * 3;

    { // Duration
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + ProgramSizes().AlignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(ProgramSizes().InputWidth);
        std::string id = "##duration" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
        double duration = pick.getDuration();
        if (ImGui::InputDouble(id.c_str(), &duration, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
        {
            hasValuesChanged = true;
            pick.setDuration(duration);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    text = "release";
    textSize = ImGui::CalcTextSize(text.c_str());
    y += textSize.y + padding.y * 3;

    { // Release
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + ProgramSizes().AlignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(ProgramSizes().InputWidth);
        std::string id = "##release" + std::to_string(window->DC.CursorPos.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
        ImGui::LocalToggleButton(id.c_str(), &pick.m_release);
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;

    ImGui::PopClipRect();
    return hasValuesChanged;
}

} // namespace


