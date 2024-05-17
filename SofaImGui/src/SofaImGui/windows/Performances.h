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
         * @brief Shows the Performance window.
         *
         * This function displays performance metrics including the average frame time, frames per second (FPS), number of vertices, indices, triangles, visible windows, and active allocations. It also plots the frame times over a certain period.
         *
         * @param windowNamePerformances The name of the Performance window.
         * @param io The ImGuiIO structure containing ImGui's I/O configuration settings.
         * @param isPerformancesWindowOpen A reference to a boolean flag indicating if the Performance window is open.
         */
         void showPerformances(const char* const& windowNamePerformances,
                               const ImGuiIO& io,
                               bool& isPerformancesWindowOpen);


} // namespace sofaimgui
