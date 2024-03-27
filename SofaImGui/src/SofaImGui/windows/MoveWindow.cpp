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


void MoveWindow::checkLimits(sofa::simulation::Node* groot)
{
    const auto& node = groot->getChild("UserInterface");
    if(node != nullptr)
    {
        const auto& data = node->getDataFields();
        for(auto d: data)
        {
            const std::string& value = d->getValueString();
            std::string name = d->getName();
            const std::string& group = d->getGroup();

            if(group.find("Move") != std::string::npos)
            {
                if(name.find("TCP") != std::string::npos)
                {
                    if(name.find("MinPosition") != std::string::npos)
                        m_TCPMinPosition = std::stoi(value);
                    if(name.find("MaxPosition") != std::string::npos)
                        m_TCPMaxPosition = std::stoi(value);
                    if(name.find("MinOrientation") != std::string::npos)
                        m_TCPMinOrientation = std::stod(value);
                    if(name.find("MaxOrientation") != std::string::npos)
                        m_TCPMaxOrientation = std::stod(value);
                }
                else if (name.find("Actuators") != std::string::npos)
                {

                }

            }
        }
    }
}


void MoveWindow::showWindow(sofa::simulation::Node* groot, const ImGuiWindowFlags &windowFlags)
{
    if (m_isWindowOpen)
    {
        static bool firstTime = true;
        if (firstTime)
        {
            checkLimits(groot);
            firstTime = false;
        }

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
            ImGui::Unindent();

            ImGui::Spacing();

            if (m_isDrivingSimulation)
                m_TCPTarget->setPosition(x, y, z, rx, ry, rz);

            ImGui::End();
        }

    }
}

bool MoveWindow::showSliderInt(const char* name, const char* label1, const char* label2, int* v, const int &offset, const ImVec4& color)
{
    bool hasValueChanged = false;
    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, color); ImGui::Text("l"); ImGui::PopStyleColor(); ImGui::SameLine();
    ImGui::Text("%s", name);

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    if (ImGui::SliderInt(label1, v, m_TCPMinPosition + offset, m_TCPMaxPosition + offset, "%0.f", ImGuiSliderFlags_NoInput))
        hasValueChanged = true;
    ImGui::PopStyleColor();

    ImGui::SameLine();

    double step = m_TCPMaxPosition - m_TCPMinPosition;
    ImGui::PushItemWidth(ImGui::CalcTextSize("-1,000").x + ImGui::GetFrameHeightWithSpacing() * 2);
    if (ImGui::InputInt(label2, v, powf(10.0f, floorf(log10f(step * 0.01))), step * 0.1))
        hasValueChanged = true;
    ImGui::PopItemWidth();

    return hasValueChanged;
}

bool MoveWindow::showSliderDouble(const char* name, const char* label1, const char *label2, double* v, const ImVec4& color)
{
    bool hasValueChanged = false;

    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, color); ImGui::Text("l"); ImGui::PopStyleColor(); ImGui::SameLine();
    ImGui::Text("%s", name);

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    if (ImGui::SliderScalar(label1, ImGuiDataType_Double, v, &m_TCPMinOrientation, &m_TCPMaxOrientation, "%0.2f", ImGuiSliderFlags_NoInput))
        hasValueChanged=true;
    ImGui::PopStyleColor();

    ImGui::SameLine();

    double step = m_TCPMaxOrientation - m_TCPMinOrientation;

    ImGui::PushItemWidth(ImGui::CalcTextSize("-1,000").x + ImGui::GetFrameHeightWithSpacing() * 2);
    if (ImGui::InputDouble(label2, v, powf(10.0f, floorf(log10f(step * 0.01))), step * 0.1, "%0.2f"))
        hasValueChanged=true;
    ImGui::PopItemWidth();

    return hasValueChanged;
}

}

