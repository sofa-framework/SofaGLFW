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
         * @brief Displays the viewport window.
         *
         * This function renders the viewport window, showing the scene rendered into a frame buffer object (FBO).
         * It also provides options to show/hide grid, axis, and frame within the viewport.
         *
         * @param groot The root node of the scene to be rendered.
         * @param windowNameViewport The name of the viewport window.
         * @param isViewportWindowOpen A reference to a boolean flag indicating if the viewport window is open.
         * @param ini The INI file object containing application settings.
         * @param m_fbo The frame buffer object (FBO) used for rendering the scene.
         * @param m_viewportWindowSize A reference to a pair representing the width and height of the viewport window.
         * @param isMouseOnViewport A reference to a boolean flag indicating if the mouse cursor is over the viewport.
         */
         void showViewPort(sofa::core::sptr<sofa::simulation::Node> groot,
                           const char* const& windowNameViewport,
                           bool& isViewportWindowOpen,
                           CSimpleIniA &ini,
                           std::unique_ptr<sofa::gl::FrameBufferObject>& m_fbo,
                           std::pair<float,
                           float>& m_viewportWindowSize,
                           bool & isMouseOnViewport );

} // namespace sofaimgui
