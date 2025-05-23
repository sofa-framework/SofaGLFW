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

inline bool showScalarWidget(const std::string& label, const std::string& id, float& value)
{
    return ImGui::InputFloat((label + "##" + id).c_str(), &value, 0.0f, 0.0f, "%.8f", ImGuiInputTextFlags_None);
}

inline bool showScalarWidget(const std::string& label, const std::string& id, double& value)
{
    return ImGui::InputDouble((label + "##" + id).c_str(), &value, 0.0f, 0.0f, "%.8f", ImGuiInputTextFlags_None);
}

template<typename Scalar>
void showScalarWidget(sofa::Data<Scalar>& data)
{
    Scalar initialValue = data.getValue();
    const auto& label = data.getName();
    const auto id = data.getName() + data.getOwner()->getPathName();
    if (showScalarWidget(label, id, initialValue))
    {
        data.setValue(initialValue);
    }
}

}
