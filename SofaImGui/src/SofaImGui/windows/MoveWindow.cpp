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

namespace sofaimgui::windows {

MoveWindow::MoveWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void MoveWindow::showWindow(sofa::simulation::Node* groot)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, ImGuiWindowFlags_None))
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
            ImGui::Text("TCP target position (mm):");
            ImGui::Unindent();

            ImGui::Indent();
            ImGui::Indent();
            ImGui::DragInt("X##pos", &x, 1, -500, 500); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::DragInt("Y##pos", &y, 1, -500, 500); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 1.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::DragInt("Z##pos", &z, 1, -500, 500); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 0.f, 1.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::Unindent();
            ImGui::Unindent();

            ImGui::Spacing();

            ImGui::Indent();
            ImGui::Text("TCP target rotation (rad):");
            ImGui::Unindent();

            ImGui::Indent();
            ImGui::Indent();
            float pi = 3.1415;
            ImGui::DragFloat("R##rot", &rx, 0.01, -pi, pi, "%0.2f"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::DragFloat("P##rot", &ry, 0.01, -pi, pi, "%0.2f"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 1.f, 0.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::DragFloat("Y##rot", &rz, 0.01, -pi, pi, "%0.2f"); ImGui::SameLine(); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.f, 0.f, 1.f, 1.f)); ImGui::Text(ICON_FA_MINUS); ImGui::PopStyleColor();
            ImGui::Unindent();
            ImGui::Unindent();

            setTarget(groot, x, y, z, rx, ry, rz);

            ImGui::End();
        }
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

