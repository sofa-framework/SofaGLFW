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
#include <SofaImGui/models/TCPTarget.h>
#include <imgui.h>

namespace sofaimgui::windows {

class MoveWindow : public BaseWindow
{
   public:
    MoveWindow(const std::string& name, const bool& isWindowOpen);
    ~MoveWindow() = default;

    void showWindow(sofa::simulation::Node *groot, const ImGuiWindowFlags &windowFlags);
    void setTCPTarget(std::shared_ptr<models::TCPTarget> TCPTarget) {m_TCPTarget=TCPTarget;}

   protected:

    int m_TCPMaxPosition{500};
    int m_TCPMinPosition{-500};
    double m_TCPMaxOrientation{M_PI};
    double m_TCPMinOrientation{-M_PI};

    int m_maxActuatorPosition{500};
    int m_minActuatorPosition{-500};
    double m_maxActuatorOrientation{M_PI};
    double m_minActuatorOrientation{-M_PI};

    std::shared_ptr<models::TCPTarget> m_TCPTarget;

    void checkLimits(sofa::simulation::Node* groot);
    bool showSliderInt(const char *name, const char* label1, const char *label2, int* v, const int &offset, const ImVec4& color);
    bool showSliderDouble(const char *name, const char* label1, const char *label2, double* v, const ImVec4 &color);
};

}


