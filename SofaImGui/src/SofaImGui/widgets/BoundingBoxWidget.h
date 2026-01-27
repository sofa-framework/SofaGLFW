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
#pragma once
#include <imgui.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/type/BoundingBox.h>

namespace sofaimgui
{

inline void showBoundingBoxWidget(sofa::Data<sofa::type::BoundingBox>& data)
{
    const auto box = data.getValue();

    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;

    if (ImGui::BeginTable("bbox_table", 4, flags))
    {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableHeadersRow();

        // Second row: ["min", min.x, min.y, min.z]
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("min");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", box.minBBox().x());
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.2f", box.minBBox().y());
        ImGui::TableSetColumnIndex(3); ImGui::Text("%.2f", box.minBBox().z());

        // Third row: ["max", max.x, max.y, max.z]
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("max");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", box.maxBBox().x());
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.2f", box.maxBBox().y());
        ImGui::TableSetColumnIndex(3); ImGui::Text("%.2f", box.maxBBox().z());

        ImGui::EndTable();
    }
}

}
