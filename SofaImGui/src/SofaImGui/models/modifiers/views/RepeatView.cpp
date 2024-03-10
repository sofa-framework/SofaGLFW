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

#include <SofaImGui/models/modifiers/Repeat.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <ProgramStyle.h>
#include <SofaImGui/widgets/Buttons.h>


namespace sofaimgui::models::modifiers {

bool Repeat::RepeatView::showBlock(const std::string &label,
                                   const ImVec2 &size,
                                   const ImVec2& trackBeginPos)
{
    bool hasValuesChanged  = false;
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImVec2 padding(ImGui::GetStyle().FramePadding);
    ImVec2 spacing(ImGui::GetStyle().ItemSpacing);

    window->DC.CursorPos.x = repeat.getStartTime() * ProgramSizes().TimelineOneSecondSize + trackBeginPos.x + spacing.x;
    window->DC.CursorPos.y = window->DC.CursorPos.y + size.y;

    float x = window->DC.CursorPos.x ;
    float y = window->DC.CursorPos.y ;
    ImRect bb(ImVec2(x, y), ImVec2(x + size.x, y + size.y));
    ImVec2 topRight = ImVec2(bb.Max.x, bb.Min.y);

    double dragright, dragleft;
    ImGui::ModifierBlock(label.c_str(), bb, &dragleft, &dragright, ProgramColors().RepeatBlockBg);
    repeat.setEndTime(repeat.getEndTime() + dragright / ProgramSizes().TimelineOneSecondSize);
    repeat.setStartTime(repeat.getStartTime() + dragleft / ProgramSizes().TimelineOneSecondSize);

    std::string text = "Repeat";
    ImVec2 textSize = ImGui::CalcTextSize(text.c_str());

    auto rectMin = ImGui::GetItemRectMin();
    auto rectMax = ImGui::GetItemRectMax();
    rectMax.x -= padding.x;
    ImGui::PushClipRect(rectMin, rectMax, true);

    ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
    { // Repeat
        x += padding.y;
        y += padding.y ;

        window->DC.CursorPos.x = x;
        window->DC.CursorPos.y = y;

        auto rectMin = ImGui::GetItemRectMin();
        auto rectMax = ImGui::GetItemRectMax();
        rectMax.x -= padding.x * 2 + ImGui::GetFrameHeight(); // leave space for option button
        ImGui::PushClipRect(rectMin, rectMax, true);

        std::string id = "##comment" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0., 0., 0., 0.));
        if (ImGui::InputText(id.c_str(), repeat.getComment(), models::modifiers::Modifier::COMMENTSIZE))
        {
            hasValuesChanged = true;
        }
        ImGui::PopStyleColor();

        ImGui::PopClipRect();
    }
    ImGui::PopStyleColor();

    text = "iterations";
    textSize = ImGui::CalcTextSize(text.c_str());
    y += textSize.y + padding.y * 3;

    { // Iterations
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
        ImGui::PushItemWidth(ProgramSizes().InputWidth * 2);
        std::string id = "##iterations" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
        int iterations = repeat.getIterations();
        if (ImGui::InputInt(id.c_str(), &iterations))
        {
            repeat.setIterations(iterations);
            repeat.setCounts(iterations);
            hasValuesChanged = true;
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    text = "counts";
    textSize = ImGui::CalcTextSize(text.c_str());
    double nx = x + ProgramSizes().AlignWidth + ProgramSizes().InputWidth * 2 + spacing.x * 4;

    { // Counts
        bb.Min = ImVec2(nx, y);
        bb.Max = ImVec2(nx + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);

        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
        drawList->AddText(ImVec2(nx + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = nx + ProgramSizes().AlignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(ProgramSizes().InputWidth * 2);
        std::string id = "##counts" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
        int counts = repeat.getCounts();
        if (ImGui::InputInt(id.c_str(), &counts))
        {
            repeat.setCounts(counts);
            hasValuesChanged = true;
        }
        ImGui::PopStyleColor(2);
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

        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + ProgramSizes().AlignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth(ProgramSizes().InputWidth);
        std::string id = "##starttime" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
        double startTime = repeat.getStartTime();
        if (ImGui::InputDouble(id.c_str(), &startTime, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
        {
            hasValuesChanged = true;
            repeat.setStartTime(startTime);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    text = "end time";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = spacing.y + bb.Max.y;

    { // End time
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
        std::string id = "##endtime" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
        double endTime = repeat.getEndTime();
        if (ImGui::InputDouble(id.c_str(), &endTime, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
        {
            hasValuesChanged = true;
            repeat.setEndTime(endTime);
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

        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().Text);
        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
        ImGui::PopStyleColor();

        window->DC.CursorPos.x = x + ProgramSizes().AlignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        ImGui::PushItemWidth((ProgramSizes().InputWidth + spacing.x) * 4);
        std::string id = "##speed" + std::to_string(window->DC.CursorPos.x) + std::to_string(window->DC.CursorPos.y);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ProgramColors().FrameBg);
        ImGui::PushStyleColor(ImGuiCol_Text, ProgramColors().FrameText);
        static const char* items[]{"REPEAT"};
        int type = repeat.getType();
        if (ImGui::Combo(id.c_str(), &type, items, IM_ARRAYSIZE(items)))
        {
            hasValuesChanged = true;
            repeat.setType(models::modifiers::Repeat::Type(type));
        }
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


