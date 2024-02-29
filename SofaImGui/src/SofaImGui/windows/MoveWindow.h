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
    void setTCPTarget(const std::shared_ptr<models::TCPTarget> &TCPTarget) {m_TCPTarget=TCPTarget;}

   protected:

    std::shared_ptr<models::TCPTarget> m_TCPTarget;

    void showSliderInt(const char *name, const char* label1, const char *label2, int* v, const int &offset, const ImVec4& color);
    void showSliderFloat(const char *name, const char* label1, const char *label2, float* v, const ImVec4 &color);
};

}


