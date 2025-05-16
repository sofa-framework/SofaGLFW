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
#include <SofaImGui/widgets/DisplayFlagsWidget.h>
#include <imgui.h>

namespace sofaimgui
{

void showDisplayFlagsWidget(sofa::Data<sofa::core::visual::DisplayFlags> & data)
{
    auto& displayFlags = sofa::helper::getWriteAccessor(data).wref();

    {
        const bool initialValue = displayFlags.getShowVisualModels();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Visual Models", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowVisualModels(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowBehaviorModels();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Behavior Models", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowBehaviorModels(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowForceFields();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Force Fields", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowForceFields(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowCollisionModels();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Collision Models", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowCollisionModels(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowDetectionOutputs();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Detection Outputs", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowDetectionOutputs(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowBoundingCollisionModels();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Bounding Collision Models", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowBoundingCollisionModels(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowMappings();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Mappings", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowMappings(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowMechanicalMappings();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Mechanical Mappings", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowMechanicalMappings(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowWireFrame();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Wire Frame", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowWireFrame(changeableValue);
        }
    }

    {
        const bool initialValue = displayFlags.getShowNormals();
        bool changeableValue = initialValue;
        ImGui::Checkbox("Show Normals", &changeableValue);
        if (changeableValue != initialValue)
        {
            displayFlags.setShowNormals(changeableValue);
        }
    }
}

}
