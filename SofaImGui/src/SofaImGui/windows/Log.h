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
#include <SofaImGui/config.h>

#include <memory>
#include <SofaGLFW/BaseGUIEngine.h>
#include <sofa/gl/FrameBufferObject.h>

#include <imgui.h>
#include <sofa/simulation/Node.h>
#include <SimpleIni.h>


namespace windows
{
        /**
         * @brief Shows the Log window.
         *
         * This function displays a window containing log messages. It provides options to filter messages by type and save the log to a file. The displayed log messages include their IDs, types, senders, and the messages themselves.
         *
         * @param windowNameLog The name of the Log window.
         * @param isLogWindowOpen A reference to a boolean flag indicating if the Log window is open.
         */
        void showLog(const char* const& windowNameLog,
                     bool& isLogWindowOpen);


} // namespace sofaimgui
