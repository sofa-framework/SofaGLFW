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
#include <SofaImGui/windows/StateWindow.h>
#include <imgui.h>

namespace sofaimgui::windows {

class SOFAIMGUI_API ViewportWindow : public BaseWindow
{
   public:
    ViewportWindow(const std::string& name, const bool& isWindowOpen, std::shared_ptr<StateWindow> stateWindow);
    ~ViewportWindow() = default;

    void showWindow(sofa::simulation::Node *groot, const ImTextureID& texture,
                    const ImGuiWindowFlags &windowFlags);
    
    bool addStepButton();
    bool addAnimateButton(bool *animate);
    bool addDrivingTabCombo(int *mode, const char *listModes[], const int &sizeListModes, const double &maxItemWidth);

    std::pair<float, float> m_windowSize{0., 0.};
    bool m_isMouseOnViewport{false};

   protected:

    std::shared_ptr<StateWindow> m_stateWindow;

    void addStateWindow();
    void addSimulationTimeAndFPS(sofa::simulation::Node *groot);

};

}


