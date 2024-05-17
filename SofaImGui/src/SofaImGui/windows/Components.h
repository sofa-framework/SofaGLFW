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
        * @brief Shows the Components window.
        *
        * This function displays a window listing all available components along with their categories. It allows users to select a component to view its details, including name, description, templates, aliases, namespaces, parents, targets, and data properties.
        *
        * @param windowNameComponents The name of the Components window.
        * @param isComponentsWindowOpen A reference to a boolean flag indicating if the Components window is open.
        */
         void showComponents(const char* const& windowNameComponents,
                             bool& isComponentsWindowOpen);

} // namespace sofaimgui
