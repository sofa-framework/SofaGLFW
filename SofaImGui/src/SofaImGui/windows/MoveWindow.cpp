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

#include <sofa/type/Quat.h>

#include <imgui_internal.h>
#include <IconsFontAwesome6.h>

#include <SofaImGui/windows/MoveWindow.h>

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
    SOFA_UNUSED(groot);
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            ImGui::Spacing();

            static int x=0;
            static int y=0;
            static int z=0;
            static double rx=0.;
            static double ry=0.;
            static double rz=0.;

            if(m_isDrivingSimulation)
                m_TCPTarget->getPosition(x, y, z, rx, ry, rz);

            ImGui::Indent();
            ImGui::Text("TCP Target Position (mm):");
            ImGui::Spacing();
            ImGui::Unindent();

            ImGui::Indent();
            ImGui::Indent();

            const auto &initPosition = m_TCPTarget->getInitPosition();
            showSliderInt("X", "##Xpos", "##XposInput", &x, initPosition[0], ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            ImGui::Spacing();
            showSliderInt("Y", "##Ypos", "##YposInput", &y, initPosition[1], ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
            ImGui::Spacing();
            showSliderInt("Z", "##Zpos", "##ZposInput", &z, initPosition[2], ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
            ImGui::Spacing();

            ImGui::Unindent();
            ImGui::Unindent();

            ImGui::Spacing();

            ImGui::Indent();
            ImGui::Text("TCP Target Rotation (rad):");
            ImGui::Spacing();
            ImGui::Unindent();

            ImGui::Indent();
            ImGui::Indent();

            showSliderDouble("R", "##Rrot", "##RrotInput", &rx, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            ImGui::Spacing();
            showSliderDouble("P", "##Prot", "##ProtInput", &ry, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
            ImGui::Spacing();
            showSliderDouble("Y", "##Yrot", "##YrotInput", &rz, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
            ImGui::Spacing();

            ImGui::Unindent();

            if (m_isDrivingSimulation)
                m_TCPTarget->setPosition(x, y, z, rx, ry, rz);

            ImGui::Unindent();

            ImGui::End();
        }

    }
}

void MoveWindow::showSliderInt(const char* name, const char* label1, const char* label2, int* v, const int &offset, const ImVec4& color)
{
    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, color); ImGui::Text("l"); ImGui::PopStyleColor(); ImGui::SameLine();
    ImGui::Text("%s", name);

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    ImGui::SliderInt(label1, v, -500 + offset, 500 + offset, "%0.f", ImGuiSliderFlags_NoInput);
    ImGui::PopStyleColor();

    ImGui::SameLine();

    ImGui::PushItemWidth(ImGui::CalcTextSize("-1,000").x + ImGui::GetFrameHeightWithSpacing() * 2);
    ImGui::InputInt(label2, v, 1, 5);
    ImGui::PopItemWidth();
}

void MoveWindow::showSliderDouble(const char* name, const char* label1, const char *label2, double* v, const ImVec4& color)
{
    double pi = 3.1415;
    double mpi = -pi;

    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, color); ImGui::Text("l"); ImGui::PopStyleColor(); ImGui::SameLine();
    ImGui::Text("%s", name);

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    ImGui::SliderScalar(label1, ImGuiDataType_Double, v, &mpi, &pi, "%0.2f", ImGuiSliderFlags_NoInput);
    ImGui::PopStyleColor();

    ImGui::SameLine();

    ImGui::PushItemWidth(ImGui::CalcTextSize("-1,000").x + ImGui::GetFrameHeightWithSpacing() * 2);
    ImGui::InputDouble(label2, v, 0.01, 0.1, "%0.2f");
    ImGui::PopItemWidth();
}

}
