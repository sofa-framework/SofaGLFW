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

#include <SofaImGui/config.h>
#include <SofaImGui/windows/WindowState.h>
#include <string>

namespace sofaimgui::guis
{

/**
 * @brief Base class for additional GUI module that can be injected into the main ImGui loop.
 *
 * Inherit from this to create custom GUI components that are automatically drawn each frame.
 */
class SOFAIMGUI_API BaseAdditionalGUI
{

public:
    virtual ~BaseAdditionalGUI() = default;

    /**
     * @brief Draw the GUI component.
     *
     * You must override this method to implement the GUI logic.
     */
    void draw(windows::WindowState& winManager);

    /**
     * @brief Get the name of the window.
     *
     * This will be used as the title of the ImGui window.
     *
     * @return The name of the window.
     */
    virtual std::string getWindowName() const = 0;

    /**
     * @brief Get the icon for the window.
     *
     * This will be used as the icon of the ImGui window.
     *
     * @return The icon string, default is a cube icon.
     */
    virtual std::string getWindowIcon() const;

private:
    virtual void doDraw() = 0;
};

} // namespace sofaimgui::guis
