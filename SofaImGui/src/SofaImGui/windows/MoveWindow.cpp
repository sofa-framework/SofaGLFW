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
#include <SofaImGui/widgets/Buttons.h>

namespace sofaimgui::windows {

MoveWindow::MoveWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
    m_isDrivingSimulation = true;
}


void MoveWindow::setTCPDescriptions(const std::string &positionDescription, const std::string &rotationDescription)
{
    m_TCPPositionDescription = positionDescription;
    m_TCPRotationDescription = rotationDescription;
}

void MoveWindow::setTCPLimits(int minPosition, int maxPosition, double minOrientation, double maxOrientation)
{
    m_TCPMinPosition = minPosition;
    m_TCPMaxPosition = maxPosition;
    m_TCPMinOrientation = minOrientation;
    m_TCPMaxOrientation = maxOrientation;
}

void MoveWindow::setActuatorsDescriptions(const std::string &description)
{
    m_actuatorsDescription = description;
}

void MoveWindow::setActuatorsLimits(double min, double max)
{
    m_actuatorsMin = min;
    m_actuatorsMax = max;
}

void MoveWindow::showWindow(const ImGuiWindowFlags &windowFlags)
{
    if (m_isWindowOpen && (m_IPController != nullptr || !m_actuators.empty()))
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            if (m_IPController != nullptr)
            {
                ImGui::Spacing();
                auto positionRight = ImGui::GetCursorPosX() + ImGui::GetWindowSize().x; // Get position for right buttons

                static double x=0;
                static double y=0;
                static double z=0;
                static double rx=0.;
                static double ry=0.;
                static double rz=0.;

                if(m_isDrivingSimulation)
                    m_IPController->getTCPTargetPosition(x, y, z, rx, ry, rz);

                ImGui::Indent();
                ImGui::Text("%s", m_TCPPositionDescription.c_str());
                ImGui::Spacing();
                ImGui::Unindent();

                ImGui::Indent();
                ImGui::Indent();

                const auto &initPosition = m_IPController->getTCPTargetInitPosition();
                showSliderDouble("X", "##XSlider", "##XInput", &x, m_TCPMinPosition + initPosition[0], m_TCPMaxPosition + initPosition[0], ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                ImGui::Spacing();
                showSliderDouble("Y", "##YSlider", "##YInput", &y, m_TCPMinPosition + initPosition[1], m_TCPMaxPosition + initPosition[1], ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
                ImGui::Spacing();
                showSliderDouble("Z", "##ZSlider", "##ZInput", &z, m_TCPMinPosition + initPosition[2], m_TCPMaxPosition + initPosition[2], ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
                ImGui::Spacing();

                ImGui::Unindent();
                ImGui::Unindent();

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                ImGui::Indent();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("%s", m_TCPRotationDescription.c_str());

                ImGui::SameLine();

                static bool freeInRotation = true;
                ImGui::SetCursorPosX(positionRight - ImGui::GetFrameHeightWithSpacing() * 1 - ImGuiStyleVar_IndentSpacing * 2); // Set position to right of the header
                if (ImGui::Button(freeInRotation? ICON_FA_LOCK_OPEN: ICON_FA_LOCK, ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight())))
                    freeInRotation = !freeInRotation;
                ImGui::SetItemTooltip("When unlocked, TCP movement is free in rotation.");

                ImGui::Spacing();
                ImGui::Unindent();

                if (freeInRotation)
                    ImGui::BeginDisabled();

                ImGui::Indent();
                ImGui::Indent();

                showSliderDouble("R", "##RSlider", "##RInput", &rx, m_TCPMinOrientation, m_TCPMaxOrientation, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                ImGui::Spacing();
                showSliderDouble("P", "##PSlider", "##PInput", &ry, m_TCPMinOrientation, m_TCPMaxOrientation, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
                ImGui::Spacing();
                showSliderDouble("Y", "##YawSlider", "##YawInput", &rz, m_TCPMinOrientation, m_TCPMaxOrientation, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
                ImGui::Spacing();

                ImGui::Unindent();
                ImGui::Unindent();

                if (freeInRotation)
                    ImGui::EndDisabled();

                if (m_isDrivingSimulation)
                {
                    if (freeInRotation)
                    {
                        auto TCP = m_IPController->getTCPPosition();
                        TCP[0] = x;
                        TCP[1] = y;
                        TCP[2] = z;
                        m_IPController->setTCPTargetPosition(TCP);
                    }
                    else
                    {
                        m_IPController->setTCPTargetPosition(x, y, z, rx, ry, rz);
                    }
                }
            }

            if (!m_actuators.empty())
            {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                ImGui::Indent();
                ImGui::Text("%s", m_actuatorsDescription.c_str());
                ImGui::Spacing();
                ImGui::Unindent();

                ImGui::Indent();
                ImGui::Indent();

                int nbActuators = m_actuators.size();
                bool solveInverseProblem = true;
                for (int i=0; i<nbActuators; i++)
                {
                    std::string name = "M" + std::to_string(i);

                    auto* typeinfo = m_actuators[i].data->getValueTypeInfo();
                    auto* value = m_actuators[i].data->getValueVoidPtr();
                    double buffer = typeinfo->getScalarValue(value, 0);
                    bool hasChanged = showSliderDouble(name.c_str(), ("##Slider" + name).c_str(), ("##Input" + name).c_str(), &buffer, m_actuatorsMin, m_actuatorsMax);
                    if (hasChanged)
                    {
                        std::string value = std::to_string(buffer);
                        std::replace(value.begin(), value.end(), ',', '.');
                        m_actuators[i].data->read(value);
                        solveInverseProblem = false;
                    }
                    m_actuators[i].value=buffer;
                }
                if (m_IPController && !solveInverseProblem)
                {
                    // TODO: don't solve the inverse problem since we'll overwrite the solution
                    m_IPController->setActuators(m_actuators);
                }

                ImGui::Unindent();
                ImGui::Unindent();
            }

            ImGui::End();
        }
    }
}

bool MoveWindow::showSliderDouble(const char* name, const char* label1, const char *label2, double* v, const double& min, const double& max, const ImVec4& color)
{
    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::Text("l");
    ImGui::PopStyleColor();
    ImGui::SameLine();

    return showSliderDouble(name, label1, label2, v, min, max);
}

bool MoveWindow::showSliderDouble(const char* name, const char* label1, const char *label2, double* v, const double& min, const double& max)
{
    bool hasValueChanged = false;

    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s", name);
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    if (ImGui::SliderScalar(label1, ImGuiDataType_Double, v, &min, &max, "%0.2f", ImGuiSliderFlags_NoInput))
        hasValueChanged=true;
    ImGui::PopStyleColor();

    ImGui::SameLine();

    double step = max - min;

    ImGui::PushItemWidth(ImGui::CalcTextSize("-1,000").x + ImGui::GetFrameHeightWithSpacing() * 2);
    const char* format = (log10f(abs(*v))>3)? "%0.2e": "%0.2f";
    if (ImGui::InputDouble(label2, v, powf(10.0f, floorf(log10f(step * 0.01))), step * 0.1, format))
        hasValueChanged=true;
    ImGui::PopItemWidth();

    return hasValueChanged;
}

}

