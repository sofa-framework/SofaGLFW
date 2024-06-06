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
         * @brief Shows the Scene Graph window.
         *
         * This function displays the hierarchy of nodes and objects in the scene graph, allowing users to interact with and inspect different components and their properties.
         *
         * @param groot The root node of the scene graph.
         * @param windowNameSceneGraph The name of the Scene Graph window.
         * @param isSceneGraphWindowOpen A reference to a boolean flag indicating if the Scene Graph window is open.
         * @param openedComponents A set containing pointers to the components that are currently opened and being inspected.
         * @param focusedComponents A set containing pointers to the components that are currently focused for inspection.
         */
        void showSceneGraph(sofa::core::sptr<sofa::simulation::Node> groot,
                            const char* const& windowNameSceneGraph,
                            bool& isSceneGraphWindowOpen,
                            std::set<sofa::core::objectmodel::BaseObject*>& openedComponents,
                            std::set<sofa::core::objectmodel::BaseObject*>& focusedComponents);


} // namespace sofaimgui
