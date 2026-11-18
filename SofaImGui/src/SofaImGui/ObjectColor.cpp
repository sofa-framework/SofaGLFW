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
#include <SofaImGui/ObjectColor.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/simulation/Colors.h>
#include <sofa/core/behavior/BaseInteractionForceField.h>
#include <sofa/core/BaseMapping.h>
#include <Style.h>

namespace sofaimgui
{
    ImVec4 getObjectColor(sofa::core::objectmodel::Base* object)
    {
        sofa::type::RGBAColor color = sofaimgui::getStyleColor(StyleColor::nodeIconColor);
        if(object->toContextObject())
            color = sofaimgui::getStyleColor(StyleColor::contextIconColor);
        else if(object->toBehaviorModel())
            color = sofaimgui::getStyleColor(StyleColor::behaviorModelIconColor);
        else if(object->toCollisionModel())
            color = sofaimgui::getStyleColor(StyleColor::collisionModelIconColor);
        else if(object->toBaseMechanicalState())
            color = sofaimgui::getStyleColor(StyleColor::baseMechanicalStateIconColor);
        else if(object->toBaseProjectiveConstraintSet())
            color = sofaimgui::getStyleColor(StyleColor::baseProjectiveConstraintSetIconColor);
        else if(object->toBaseConstraintSet())
            color = sofaimgui::getStyleColor(StyleColor::baseConstraintSetIconColor);
        else if(object->toBaseInteractionForceField() &&
            object->toBaseInteractionForceField()->getMechModel1()!=object->toBaseInteractionForceField()->getMechModel2())
            color = sofaimgui::getStyleColor(StyleColor::baseInteractionForceFieldIconColor);
        else if(object->toBaseForceField() && !object->toBaseMass())
            color = sofaimgui::getStyleColor(StyleColor::baseForceFieldIconColor);
        else if(object->toBaseAnimationLoop())
            color = sofaimgui::getStyleColor(StyleColor::baseAnimationLoopIconColor);
        else if(object->toBaseIntegrationScheme())
            color = sofaimgui::getStyleColor(StyleColor::baseIntegrationSchemeIconColor);
        else if(object->toPipeline())
            color = sofaimgui::getStyleColor(StyleColor::pipelineIconColor);
        else if(object->toIntersection())
            color = sofaimgui::getStyleColor(StyleColor::intersectionIconColor);
        else if(object->toDetection())
            color = sofaimgui::getStyleColor(StyleColor::detectionIconColor);
        else if(object->toContactManager())
            color = sofaimgui::getStyleColor(StyleColor::contactManagerIconColor);
        else if(object->toCollisionGroupManager())
            color = sofaimgui::getStyleColor(StyleColor::collisionGroupManagerIconColor);
        else if(object->toBaseMapping())
        {
            if(object->toBaseMapping()->isMechanical())
                color = sofaimgui::getStyleColor(StyleColor::baseMechanicalMappingIconColor);
            else
                color = sofaimgui::getStyleColor(StyleColor::baseMappingIconColor);
        }
        else if(object->toBaseMass())
            color = sofaimgui::getStyleColor(StyleColor::baseMassIconColor);
        else if(object->toTopology())
            color = sofaimgui::getStyleColor(StyleColor::topologyIconColor);
        else if(object->toBaseTopologyObject())
            color = sofaimgui::getStyleColor(StyleColor::baseTopologyObjectIconColor);
        else if(object->toBaseLoader())
            color = sofaimgui::getStyleColor(StyleColor::baseLoaderIconColor);
        else if(object->toConfigurationSetting())
            color = sofaimgui::getStyleColor(StyleColor::configurationSettingIconColor);
        else if(object->toVisualModel())
            color = sofaimgui::getStyleColor(StyleColor::visualModelIconColor);

        return ImVec4(color.r(), color.g(), color.b(), color.a());
    }
} //namespace sofaimgui
