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

namespace sofaimgui
{
    ImVec4 getObjectColor(sofa::core::objectmodel::Base* object)
    {
        unsigned int objectType=sofa::simulation::Colors::OBJECT;
        if(object->toContextObject())
            objectType=sofa::simulation::Colors::CONTEXT;
        else if(object->toBehaviorModel())
            objectType=sofa::simulation::Colors::BMODEL;
        else if(object->toCollisionModel())
            objectType=sofa::simulation::Colors::CMODEL;
        else if(object->toBaseMechanicalState())
            objectType=sofa::simulation::Colors::MMODEL;
        else if(object->toBaseProjectiveConstraintSet())
            objectType=sofa::simulation::Colors::PROJECTIVECONSTRAINTSET;
        else if(object->toBaseConstraintSet())
            objectType=sofa::simulation::Colors::CONSTRAINTSET;
        else if(object->toBaseInteractionForceField() &&
            object->toBaseInteractionForceField()->getMechModel1()!=object->toBaseInteractionForceField()->getMechModel2())
            objectType=sofa::simulation::Colors::IFFIELD;
        else if(object->toBaseForceField())
            objectType=sofa::simulation::Colors::FFIELD;
        else if(object->toBaseAnimationLoop())
            objectType=sofa::simulation::Colors::SOLVER;
        else if(object->toOdeSolver())
            objectType=sofa::simulation::Colors::SOLVER;
        else if(object->toPipeline())
            objectType=sofa::simulation::Colors::COLLISION;
        else if(object->toIntersection())
            objectType=sofa::simulation::Colors::COLLISION;
        else if(object->toDetection())
            objectType=sofa::simulation::Colors::COLLISION;
        else if(object->toContactManager())
            objectType=sofa::simulation::Colors::COLLISION;
        else if(object->toCollisionGroupManager())
            objectType=sofa::simulation::Colors::COLLISION;
        else if(object->toBaseMapping())
        {
            if(object->toBaseMapping()->isMechanical())
                objectType=sofa::simulation::Colors::MMAPPING;
            else
                objectType=sofa::simulation::Colors::MAPPING;
        }
        else if(object->toBaseMass())
            objectType=sofa::simulation::Colors::MASS;
        else if(object->toTopology())
            objectType=sofa::simulation::Colors::TOPOLOGY;
        else if(object->toBaseTopologyObject())
            objectType=sofa::simulation::Colors::TOPOLOGY;
        else if(object->toBaseLoader())
            objectType=sofa::simulation::Colors::LOADER;
        else if(object->toConfigurationSetting())
            objectType=sofa::simulation::Colors::CONFIGURATIONSETTING;
        else if(object->toVisualModel())
            objectType=sofa::simulation::Colors::VMODEL;

        const auto iconColor=sofa::simulation::Colors::COLOR[objectType];

        constexpr auto hexval=[](char c)
        {
            if(c >= '0' && c <= '9') return c-'0';
            if(c >= 'a' && c <= 'f') return (c-'a')+10;
            if(c >= 'A' && c <= 'F') return (c-'A')+10;
            return 0;
        };

        const auto r = static_cast<float>(hexval(iconColor[1]) * 16 + hexval(iconColor[2]));
        const auto g = static_cast<float>(hexval(iconColor[3]) * 16 + hexval(iconColor[4]));
        const auto b = static_cast<float>(hexval(iconColor[5]) * 16 + hexval(iconColor[6]));
        return ImVec4(r/255.f, g/255.f, b/255.f, 1.f);
    }
} //namespace sofaimgui
