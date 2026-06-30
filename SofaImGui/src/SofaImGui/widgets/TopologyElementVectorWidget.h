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
#include <sofa/core/objectmodel/Data.h>
#include <sofa/topology/Element.h>
#include <imgui.h>
#include <unordered_map>
#include <string>

namespace sofaimgui
{

using namespace sofa;

/***********************************************************************************************************************
 * Vectors of Topology Elements
 **********************************************************************************************************************/

namespace
{
    enum class ElementTableExpansionState
    {
        Collapsed,
        Expanded
    };

    // Helper function to toggle expansion state
    void toggleExpansionState(ElementTableExpansionState& expansionState)
    {
        expansionState = (expansionState == ElementTableExpansionState::Expanded) 
            ? ElementTableExpansionState::Collapsed 
            : ElementTableExpansionState::Expanded;
    }

    // Helper function to get or initialize expanded state for a table
    ElementTableExpansionState& getOrInitializeElementExpandedState(const std::string& tableLabel)
    {
        static std::unordered_map<std::string, ElementTableExpansionState> expandedState;
        if (!expandedState.contains(tableLabel))
        {
            expandedState[tableLabel] = ElementTableExpansionState::Collapsed;
        }
        return expandedState[tableLabel];
    }

    // Helper function to display element count and toggle button
    void showVectorWidgetHeader(std::size_t elementsCount, const std::string& tableLabel, ElementTableExpansionState& expansionState)
    {
        ImGui::Text("%d elements", (int)elementsCount);
        ImGui::SameLine();
        const bool isExpanded = (expansionState == ElementTableExpansionState::Expanded);

        if (ImGui::Button(std::string(isExpanded ? "Collapse##" + tableLabel : "Expand##" + tableLabel).c_str()))
        {
            toggleExpansionState(expansionState);
        }
    }
}

template<typename GeometryElement>
void showTopologyElementVectorWidget(Data<type::vector<topology::Element<GeometryElement> > >& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;

    constexpr auto N = topology::Element<GeometryElement>::static_size;
    const auto nbColumns = N + 1;
    const auto tableLabel = data.getName() + data.getOwner()->getPathName();
    const auto size = data.getValue().size();

    ElementTableExpansionState& expansionState = getOrInitializeElementExpandedState(tableLabel);
    showVectorWidgetHeader(size, tableLabel, expansionState);

    if (expansionState == ElementTableExpansionState::Collapsed)
    {
        flags |= ImGuiTableFlags_ScrollY;
    }
    else
    {
        flags &= ~ImGuiTableFlags_ScrollY;
    }

    static constexpr float NumberOfLinesToShowWhenCollapsed = 8.5f;
    ImVec2 outerSize = ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * NumberOfLinesToShowWhenCollapsed );
    if (ImGui::BeginTable(tableLabel.c_str(), (int)nbColumns, flags, outerSize))
    {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("");
        for (unsigned int i = 0; i < N; ++i)
        {
            ImGui::TableSetupColumn(std::to_string(i).c_str(), ImGuiTableColumnFlags_WidthStretch);
        }
        ImGui::TableHeadersRow();

        auto accessor = helper::getReadAccessor(data);

        for (std::size_t i = 0; i < accessor.size(); ++i)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", (int)i);
            for (const auto& v : accessor[i])
            {
                ImGui::TableNextColumn();
                ImGui::Text("%d", (int)v);
            }
        }

        ImGui::EndTable();
    }
}

}
