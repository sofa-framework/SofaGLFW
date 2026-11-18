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

#include <string>
#include <array>
#include <sofa/type/RGBAColor.h>

namespace sofaglfw
{
class SofaGLFWBaseGUI;
}

namespace sofaimgui
{

// SOFA specific style colors
enum class StyleColor : std::size_t
{
    messageFatalIconColor,
    messageWarningIconColor,
    messageInfoIconColor,
    nodeIconColor,
    componentIconColor,
    contextIconColor,
    behaviorModelIconColor,
    collisionModelIconColor,
    baseMechanicalStateIconColor,
    baseProjectiveConstraintSetIconColor,
    baseConstraintSetIconColor,
    baseInteractionForceFieldIconColor,
    baseForceFieldIconColor,
    baseAnimationLoopIconColor,
    baseIntegrationSchemeIconColor,
    pipelineIconColor,
    intersectionIconColor,
    detectionIconColor,
    contactManagerIconColor,
    collisionGroupManagerIconColor,
    baseMappingIconColor,
    baseMechanicalMappingIconColor,
    baseMassIconColor,
    topologyIconColor,
    baseTopologyObjectIconColor,
    baseLoaderIconColor,
    configurationSettingIconColor,
    visualModelIconColor,

    count
};

const sofa::type::RGBAColor& getStyleColor(StyleColor color);

void setStyle(const std::string& style, sofaglfw::SofaGLFWBaseGUI* gui);

inline const std::array<const char*, 8> listStyles = {"sofa_light", "deep_dark", "default_dark", "default_light", "classic", "light_green", "yet_another_dark", "cherry"};
inline const std::string defaultStyle = listStyles[0];

}
