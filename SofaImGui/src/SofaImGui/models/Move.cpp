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

#include <SofaImGui/models/Move.h>
#include <imgui_internal.h>
#include <implot.h>

namespace sofaimgui::models {

Move::Move(const RigidCoord& waypoint,
           const float& duration,
           MoveType type):  Action(duration),
                            m_waypoint(waypoint),
                            m_type(type)
{
    m_velocity = computeVelocityFromDuration();
}

void Move::showBlock(const std::string &label, const ImVec2 &size)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float alignWidth = 120;
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

    std::string text = "Move to Way Point";
    ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
    ImVec2 padding(5, 5);

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

        drawList->AddText(ImVec2(x + padding.x * 2,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
    }

    text = "WP.pos";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y * 2 + bb.Max.y;

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
        for (int i=0; i<3; i++)
        {
            ImGui::PushItemWidth(ImGui::CalcTextSize("10000").x);
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);
            float wp = m_waypoint[i];
            if (ImGui::InputFloat(id.c_str(), &wp, 0, 0, "%0.f", ImGuiInputTextFlags_CharsNoBlank))
                m_waypoint[i] = wp;
            ImGui::SameLine();
            ImGui::PopItemWidth();
        }
        ImGui::PopStyleVar();
    }

    text = "WP.quat";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

    { // Way point quaternion
        bb.Min = ImVec2(x, y);
        bb.Max = ImVec2(x + textSize.x + padding.x * 2,
                        y + textSize.y + padding.y * 2);


        drawList->AddText(ImVec2(x + padding.x,
                                 y + padding.y),
                          ImGui::GetColorU32(ImGuiCol_Text), text.c_str());

        window->DC.CursorPos.x = x + alignWidth;
        window->DC.CursorPos.y = y;

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, padding);
        for (int i=3; i<7; i++)
        {
            ImGui::PushItemWidth(ImGui::CalcTextSize("10000").x);
            std::string id = "##wp" + std::to_string(window->DC.CursorPos.x + i);
            float wp = m_waypoint[i];
            if (ImGui::InputFloat(id.c_str(), &wp, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank))
                m_waypoint[i] = wp;
            ImGui::SameLine();
            ImGui::PopItemWidth();
        }
        ImGui::PopStyleVar();
    }

    text = "duration";
    textSize = ImGui::CalcTextSize(text.c_str());
    y = padding.y + bb.Max.y;

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
        ImGui::InputFloat(id.c_str(), &m_duration, 0, 0, "%0.2f", ImGuiInputTextFlags_CharsNoBlank);
        ImGui::SameLine();
        ImGui::PopItemWidth();
        ImGui::PopStyleVar();
    }

    window->DC.CursorPosPrevLine.x = topRight.x;
    window->DC.CursorPosPrevLine.y = topRight.y;
}

void Move::add()
{

}

void Move::remove()
{

}

void Move::insert()
{

}

float Move::computeVelocityFromDuration()
{
    return 0;
}

float Move::computeDurationFromVelocity()
{
    return 0;
}

} // namespace


