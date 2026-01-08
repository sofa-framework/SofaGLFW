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
#include <SofaImGui/widgets/ScalarWidget.h>
#include <imgui.h>
#include <unordered_map>
#include <string>

namespace sofaimgui
{

using namespace sofa;

/***********************************************************************************************************************
 * Vectors of Vec
 **********************************************************************************************************************/

template< Size N, typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<N, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    for (unsigned int i = 0; i < N; ++i)
    {
        ImGui::TableSetupColumn(std::to_string(i).c_str(), ImGuiTableColumnFlags_WidthStretch);
    }
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<ValueType> >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<1, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch);
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<2, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch);
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<3, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthStretch);
}

template<Size N, typename ValueType>
bool showLine(unsigned int lineNumber, const std::string& tableLabel, type::Vec<N, ValueType>& vec)
{
    for (const auto& v : vec)
    {
        ImGui::TableNextColumn();
        ImGui::Text("%f", v);
    }
    return false;
}

template<typename ValueType>
bool showLine(unsigned int lineNumber, const std::string& tableLabel, ValueType& value)
{
    ImGui::TableNextColumn();
    return showScalarWidget("", tableLabel + std::to_string(lineNumber), value);
}

namespace
{
    enum class TableExpansionState
    {
        Collapsed,
        Expanded
    };

    // Helper function to toggle expansion state
    void toggleExpansionState(TableExpansionState& expansionState)
    {
        expansionState = (expansionState == TableExpansionState::Expanded) 
            ? TableExpansionState::Collapsed 
            : TableExpansionState::Expanded;
    }

    // Helper function to get or initialize expanded state for a table
    TableExpansionState& getOrInitializeExpandedState(const std::string& tableLabel)
    {
        static std::unordered_map<std::string, TableExpansionState> expandedState;
        if (!expandedState.contains(tableLabel))
        {
            expandedState[tableLabel] = TableExpansionState::Collapsed;
        }
        return expandedState[tableLabel];
    }

    // Helper function to display element count and toggle button
    void showVectorWidgetHeader(std::size_t size, const std::string& tableLabel, TableExpansionState& expansionState)
    {
        ImGui::Text("%d elements", size);
        ImGui::SameLine();
        const bool isExpanded = (expansionState == TableExpansionState::Expanded);
        if (ImGui::Button(std::string(isExpanded ? "Collapse##" + tableLabel : "Expand##" + tableLabel).c_str()))
        {
            toggleExpansionState(expansionState);
        }
    }

    // Helper function to render a single table row
    template<class AccessorType, class DataType>
    bool renderTableRow(std::size_t index, const std::string& tableLabel, AccessorType& accessor, DataType& data)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%d", index);
        auto& vec = accessor[index];
        if (showLine(index, tableLabel, vec))
        {
            data.setDirtyValue();
            return true;
        }
        return false;
    }

    // Helper function to render all rows when expanded
    template<class AccessorType, class DataType>
    bool renderExpandedTableRows(AccessorType& accessor, const std::string& tableLabel, DataType& data)
    {
        bool anyChange = false;
        for (std::size_t i = 0; i < accessor.size(); ++i)
        {
            if (renderTableRow(i, tableLabel, accessor, data))
            {
                anyChange = true;
            }
        }
        return anyChange;
    }

    // Helper function to render collapsed rows with indicator
    template<class AccessorType, class DataType>
    bool renderCollapsedTableRows(AccessorType& accessor, const std::string& tableLabel, std::size_t nbColumns, DataType& data, TableExpansionState& expansionState)
    {
        constexpr std::size_t collapsedRowCount = 5; // Number of rows to show when collapsed
        bool anyChange = false;

        const std::size_t rowsToShow = std::min(collapsedRowCount, accessor.size());
        for (std::size_t i = 0; i < rowsToShow; ++i)
        {
            if (renderTableRow(i, tableLabel, accessor, data))
            {
                anyChange = true;
            }
        }

        // Show indicator row if there are more rows
        if (accessor.size() > collapsedRowCount)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (ImGui::SmallButton(std::string("...##" + tableLabel + "0").c_str()))
            {
                toggleExpansionState(expansionState);
            }
            for (std::size_t col = 1; col < nbColumns; ++col)
            {
                ImGui::TableNextColumn();
                if (ImGui::SmallButton(std::string("...##" + tableLabel + std::to_string(col)).c_str()))
                {
                    toggleExpansionState(expansionState);
                }
            }
        }

        return anyChange;
    }
}

template<class T>
void showVectorWidget(Data<T>& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;

    const auto nbColumns = data.getValueTypeInfo()->size() + 1;
    const auto tableLabel = data.getName() + data.getOwner()->getPathName();
    const auto size = data.getValue().size();

    TableExpansionState& expansionState = getOrInitializeExpandedState(tableLabel);
    showVectorWidgetHeader(size, tableLabel, expansionState);

    if (ImGui::BeginTable(tableLabel.c_str(), nbColumns, flags))
    {
        showVecTableHeader(data);
        ImGui::TableHeadersRow();

        auto accessor = helper::getWriteAccessor(data);
        bool anyChange = false;

        if (expansionState == TableExpansionState::Expanded)
        {
            anyChange = renderExpandedTableRows(accessor, tableLabel, data);
        }
        else
        {
            anyChange = renderCollapsedTableRows(accessor, tableLabel, nbColumns, data, expansionState);
        }

        if (anyChange)
        {
            data.updateIfDirty();
        }

        ImGui::EndTable();
    }
}

}
