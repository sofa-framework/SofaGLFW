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

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/type/Quat.h>

#include <imgui_internal.h>
#include <IconsFontAwesome5.h>

#include <SofaImGui/windows/MoveWindow.h>
#include "SofaImGui/widgets/Buttons.h"

namespace sofaimgui::windows {

MoveWindow::MoveWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
    m_isDrivingSimulation = true;
}

void MoveWindow::showWindow(sofa::simulation::Node* groot, const ImGuiWindowFlags &windowFlags)
{
    if (m_isWindowOpen)
    {
        if(!m_isDrivingSimulation)
            ImGui::BeginDisabled();

        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            ImGui::Spacing();

            static int x=0;
            static int y=0;
            static int z=0;
            static float rx=0;
            static float ry=0;
            static float rz=0;

            getTarget(groot, x, y, z, rx, ry, rz);

            ImGui::Indent();
            ImGui::Text("TCP Target Position (mm):");
            ImGui::Spacing();
            ImGui::Unindent();

            ImGui::Indent();
            ImGui::Indent();

            ImGui::AlignTextToFramePadding();
            ImGui::Text("X"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
            ImGui::DragInt("##Xpos", &x, 1, -500, 500);

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Y"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 1.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
            ImGui::DragInt("##Ypos", &y, 1, -500, 500);

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Z"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 0.f, 1.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
            ImGui::DragInt("##Zpos", &z, 1, -500, 500);

            ImGui::Unindent();
            ImGui::Unindent();

            ImGui::Spacing();

            ImGui::Indent();
            ImGui::Text("TCP Target Rotation (rad):");
            ImGui::Spacing();
            ImGui::Unindent();

            ImGui::Indent();
            ImGui::Indent();
            float pi = 3.1415;

            ImGui::AlignTextToFramePadding();
            ImGui::Text("R"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
            ImGui::DragFloat("##Rrot", &rx, 0.01, -pi, pi, "%0.2f");

            ImGui::AlignTextToFramePadding();
            ImGui::Text("P"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 1.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
            ImGui::DragFloat("##Prot", &ry, 0.01, -pi, pi, "%0.2f");

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Y"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 0.f, 1.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
            ImGui::DragFloat("##Yrot", &rz, 0.01, -pi, pi, "%0.2f");

            ImGui::Unindent();
            ImGui::Unindent();

            if (m_isDrivingSimulation)
                setTarget(groot, x, y, z, rx, ry, rz);

            ImGui::End();
        }

        if(!m_isDrivingSimulation)
            ImGui::EndDisabled();

    }
}

void MoveWindow::getTarget(sofa::simulation::Node* groot,
                           int &x, int &y, int &z, float &rx, float &ry, float &rz)
{
    if (ImGui::IsWindowFocused())
    {
        sofa::simulation::Node *modelling = groot->getChild("Modelling");

        if (modelling != nullptr)
        {
            sofa::simulation::Node *target = modelling->getChild("Target");
            if (target != nullptr)
            {
                sofa::core::behavior::BaseMechanicalState *mechanical = target->getMechanicalState();
                if (mechanical != nullptr)
                {
                    std::stringstream frame;
                    mechanical->writeVec(sofa::core::VecId::position(), frame);

                    frame >> x;
                    frame >> y;
                    frame >> z;

                    sofa::type::Quat<SReal> q;

                    frame >> q[0];
                    frame >> q[1];
                    frame >> q[2];
                    frame >> q[3];

                    sofa::type::Vec3 rotation = q.toEulerVector();
                    rx = rotation[0];
                    ry = rotation[1];
                    rz = rotation[2];
                }
            }
        }
    }
}

void MoveWindow::setTarget(sofa::simulation::Node* groot,
                           const int &x, const int &y, const int &z, const float &rx, const float &ry, const float &rz)
{
    if (ImGui::IsWindowFocused())
    {
        sofa::simulation::Node *modelling = groot->getChild("Modelling");

        if (modelling != nullptr)
        {
            sofa::simulation::Node *target = modelling->getChild("Target");
            if (target != nullptr)
            {
                sofa::core::behavior::BaseMechanicalState *mechanical = target->getMechanicalState();
                if (mechanical != nullptr)
                {
                    sofa::type::Vec3 rotation(rx, ry, rz);
                    sofa::type::Quat<SReal> q = sofa::type::Quat<SReal>::createQuaterFromEuler(rotation);

                    std::stringstream frame;
                    frame << x << " ";
                    frame << y << " ";
                    frame << z << " ";
                    frame << q[0] << " ";
                    frame << q[1] << " ";
                    frame << q[2] << " ";
                    frame << q[3] << " ";
                    mechanical->readVec(sofa::core::VecId::position(), frame);
                }
            }
        }
    }
}

}

