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
         * @brief Shows the Settings window.
         *
         * This function displays settings for configuring the application, such as theme selection, global scale, and viewport settings.
         *
         * @param windowNameSettings The name of the Settings window.
         * @param isSettingsOpen A reference to a boolean flag indicating if the Settings window is open.
         * @param ini The INI file object containing application settings.
         */
        void showSettings(const char* const& windowNameSettings,
                          bool& isSettingsOpen,
                          CSimpleIniA &ini);

} // namespace sofaimgui
