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

#include <SofaImGui/windows/BaseWindow.h>
#include <SofaImGui/models/Program.h>
#include <imgui.h>

struct ImDrawList;
struct ImRect;

namespace sofaimgui::windows {

class ProgramWindow : public BaseWindow
{
    typedef sofa::defaulttype::RigidCoord<3, SReal> RigidCoord;

   public:
   ProgramWindow(const std::string& name, const bool& isWindowOpen);
    ~ProgramWindow() = default;

    using BaseWindow::m_name;
    using BaseWindow::m_isWindowOpen;

    models::Program m_program; // robot program

    void showWindow(sofa::simulation::Node *groot,
                    const ImGuiWindowFlags &windowFlags);

   protected:
    void addTimeline(float sSize);
};

} // namespace


