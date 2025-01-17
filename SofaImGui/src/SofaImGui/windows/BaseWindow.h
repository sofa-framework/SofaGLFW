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

#include <sofa/simulation/Node.h>
#include <SofaImGui/config.h>
#include "imgui.h"

namespace sofaimgui::windows {

class SOFAIMGUI_API BaseWindow
{
   public:
    BaseWindow() = default;
    ~BaseWindow() = default;

    std::string getName() const {return m_name;}

    /// Set the window as able to drive the robot in simulation.
    virtual void setDrivingTCPTarget(const bool &isDrivingSimulation) {m_isDrivingSimulation=isDrivingSimulation;}

    /// Does the window have tools to drive the robot in simulation.
    bool isDrivingSimulation() {return m_isDrivingSimulation;}

    /// Set the user choice to open the window or not.
    void setOpen(const bool &isOpen) {m_isOpen=isOpen;}

    /// Does the user choose to open the window or not.
    bool& isOpen() {return m_isOpen;}

    /// The window may have nothing to display. It should override this method with the corresponding checks.
    /// For example: the PlottingWindow needs data to plot, if none are given, the window is disabled.
    virtual bool enabled() {return true;}

   protected:

    bool m_isOpen{false}; /// The user choice to open the window or not
    std::string m_name = "Window"; /// The name of the window
    bool m_isDrivingSimulation{false}; /// Does the window have tools to drive the robot in simulation
};

}

