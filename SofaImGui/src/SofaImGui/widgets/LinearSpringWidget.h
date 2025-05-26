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
#include <sofa/component/solidmechanics/spring/LinearSpring.h>
#include <SofaImGui/widgets/ScalarWidget.h>

namespace sofaimgui
{

template<class Real>
void showLinearSpringWidget(sofa::Data<sofa::component::solidmechanics::spring::LinearSpring<Real>>& data)
{
    auto& spring = sofa::helper::getWriteAccessor(data).wref();
    const auto id = data.getName() + data.getOwner()->getPathName();

    int m1 = spring.m1;
    if (ImGui::InputInt(("First point index##" + id).c_str(), &m1))
    {
        spring.m1 = m1;
    }

    int m2 = spring.m2;
    if (ImGui::InputInt(("Second point index##" + id).c_str(), &m2))
    {
        spring.m2 = m2;
    }

    showScalarWidget("Stiffness", id, spring.ks);
    showScalarWidget("Damping", id, spring.kd);
    showScalarWidget("Rest length", id, spring.initpos);
}

template<class Real>
void showLinearSpringWidget(sofa::Data<sofa::type::vector<sofa::component::solidmechanics::spring::LinearSpring<Real>>>& data)
{
    auto& springs = sofa::helper::getWriteAccessor(data).wref();
    const auto id = data.getName() + data.getOwner()->getPathName();

    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d springs", springs.size());

    static constexpr int nCol = 6;
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), nCol, flags))
    {
        ImGui::TableSetupColumn(""); //id
        ImGui::TableSetupColumn("Id 1");
        ImGui::TableSetupColumn("Id 2");
        ImGui::TableSetupColumn("Stiffness");
        ImGui::TableSetupColumn("Damping");
        ImGui::TableSetupColumn("Rest length");

        ImGui::TableHeadersRow();

        int counter = 0;
        for (auto& spring : springs)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();

            ImGui::Text("%d", counter);
            ImGui::TableNextColumn();

            int m1 = spring.m1;
            if (ImGui::InputInt(("##m1" + id + std::to_string(counter)).c_str(), &m1, 0))
            {
                spring.m1 = m1;
            }
            ImGui::TableNextColumn();

            int m2 = spring.m2;
            if (ImGui::InputInt(("##m2" + id + std::to_string(counter)).c_str(), &m2, 0))
            {
                spring.m2 = m2;
            }
            ImGui::TableNextColumn();

            showScalarWidget("", "s" + id + std::to_string(counter), spring.ks);
            ImGui::TableNextColumn();

            showScalarWidget("", "d" + id + std::to_string(counter), spring.kd);
            ImGui::TableNextColumn();

            showScalarWidget("", "l" + id + std::to_string(counter), spring.initpos);


            ++counter;
        }

        ImGui::EndTable();
    }
}

}
