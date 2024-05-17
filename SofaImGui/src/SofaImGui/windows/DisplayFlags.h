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

#include <sofa/simulation/Node.h>



namespace windows
{
        /**
         * @brief Shows the Display Flags window.
         *
         * This function displays a window allowing users to toggle various display flags for visualizing different components in the scene. It provides checkboxes to control the visibility of visual models, behavior models, force fields, collision models, bounding collision models, mappings, mechanical mappings, wireframe, and normals.
         *
         * @param groot The root node of the simulation scene.
         * @param windowNameDisplayFlags The name of the Display Flags window.
         * @param isDisplayFlagsWindowOpen A reference to a boolean flag indicating if the Display Flags window is open.
         */
         void showDisplayFlags(sofa::core::sptr<sofa::simulation::Node> groot,
                               const char* const& windowNameDisplayFlags,
                               bool& isDisplayFlagsWindowOpen);

} // namespace sofaimgui
