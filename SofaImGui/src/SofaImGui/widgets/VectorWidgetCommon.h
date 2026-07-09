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
#include <string>
#include <unordered_map>

namespace sofaimgui
{

enum class TableExpansionState
{
    Collapsed,
    Expanded
};

// Helper function to toggle expansion state
inline void toggleExpansionState(TableExpansionState& expansionState)
{
    expansionState = (expansionState == TableExpansionState::Expanded) 
        ? TableExpansionState::Collapsed 
        : TableExpansionState::Expanded;
}

// Helper function to get or initialize expanded state for a table
inline TableExpansionState& getOrInitializeExpandedState(const std::string& tableLabel)
{
    static std::unordered_map<std::string, TableExpansionState> expandedState;
    if (!expandedState.contains(tableLabel))
    {
        expandedState[tableLabel] = TableExpansionState::Collapsed;
    }
    return expandedState[tableLabel];
}

// Helper function to display element count and toggle button
inline void showVectorWidgetHeader(std::size_t elementsCount, const std::string& tableLabel, TableExpansionState& expansionState)
{
    ImGui::Text("%d elements", (int)elementsCount);
    ImGui::SameLine();
    const bool isExpanded = (expansionState == TableExpansionState::Expanded);

    if (ImGui::Button(std::string(isExpanded ? "Collapse##" + tableLabel : "Expand##" + tableLabel).c_str()))
    {
        toggleExpansionState(expansionState);
    }
}

// Helper function to render all rows when expanded
template<class AccessorType, typename RowRenderer>
inline bool renderExpandedTableRows(AccessorType& accessor, const std::string& tableLabel, RowRenderer&& rowRenderer)
{
    bool anyChange = false;
    for (std::size_t i = 0; i < accessor.size(); ++i)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%d", (int)i);
        anyChange |= rowRenderer(i, tableLabel, accessor[i]);
    }
    return anyChange;
}

}
