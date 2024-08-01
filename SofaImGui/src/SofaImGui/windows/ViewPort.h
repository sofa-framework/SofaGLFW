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
#include "WindowState.h"

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
                * @param ini The INI file object containing application settings.
                * @param m_fbo The frame buffer object (FBO) used for rendering the scene.
                * @param m_viewportWindowSize A reference to a pair representing the width and height of the viewport window.
                * @param isMouseOnViewport A reference to a boolean flag indicating if the mouse cursor is over the viewport.
                * @param winManagerViewPort The state manager for the viewport window.
                * @param baseGUI A pointer to the base GUI object.
                * @param firstViewport A pointer to a boolean indicating if this is the first time the viewport is being displayed.
                * @param lastViewPortPosX A pointer to the last recorded X position of the viewport.
                * @param lastViewPortPosY A pointer to the last recorded Y position of the viewport.
                */
                void showViewPort(  sofa::core::sptr<sofa::simulation::Node> groot,
                                    const char* const& windowNameViewport,
                                    CSimpleIniA &ini,
                                    std::unique_ptr<sofa::gl::FrameBufferObject>& m_fbo,
                                    std::pair<float,
                                    float>& m_viewportWindowSize,
                                    bool & isMouseOnViewport,
                                    WindowState& winManagerViewPort,
                                    sofaglfw::SofaGLFWBaseGUI* baseGUI,
                                    bool* firstViewport,
                                    float* lastViewPortPosX,
                                    float* lastViewPortPosY);

                /**
                * @brief Checks if the viewport position has moved beyond a specified threshold.
                *
                * This function compares the current viewport position with the last recorded position
                * and determines if the movement exceeds the given precision threshold.
                *
                * @param currentX The current X position of the viewport.
                * @param currentY The current Y position of the viewport.
                * @param lastX The last recorded X position of the viewport.
                * @param lastY The last recorded Y position of the viewport.
                * @param threshold The precision threshold to determine significant movement.
                * @return True if the viewport has moved beyond the threshold, false otherwise.
                */
                bool hasViewportMoved(  float currentX,
                                        float currentY,
                                        float lastX,
                                        float lastY,
                                        float threshold);

                constexpr float precisionThreshold = 1.0f;

} // namespace sofaimgui
