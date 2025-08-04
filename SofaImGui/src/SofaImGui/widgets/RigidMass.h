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
#include <imgui.h>

namespace sofaimgui
{

template<sofa::Size N, typename real>
inline void showRigidMass(const sofa::defaulttype::RigidMass<N,real>& rigidMass)
{
    ImGui::Text("Mass: %f", rigidMass.mass);
    ImGui::Text("Volume: %f", rigidMass.volume);

    std::stringstream ss;
    ss << rigidMass.inertiaMatrix;
    ImGui::Text("Inertia Matrix: %s", ss.str().c_str());
}

template<sofa::Size N, typename real>
inline void showRigidMasses(const sofa::Data<sofa::type::vector<sofa::defaulttype::RigidMass<N, real>>>& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), 4, flags))
    {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Mass");
        ImGui::TableSetupColumn("Volume");
        ImGui::TableSetupColumn("Inertia Matrix");

        ImGui::TableHeadersRow();

        unsigned int counter {};
        for (const auto& rigidMass : *sofa::helper::getReadAccessor(data))
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", counter++);
            ImGui::TableNextColumn();
            ImGui::Text("%f", rigidMass.mass);
            ImGui::TableNextColumn();
            ImGui::Text("%f", rigidMass.volume);

            ImGui::TableNextColumn();
            std::stringstream ss;
            ss << rigidMass.inertiaMatrix;
            ImGui::Text("Inertia Matrix: %s", ss.str().c_str());
        }
        ImGui::EndTable();
    }
}

}
