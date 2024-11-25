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
    m_isOpen = isWindowOpen;
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
    if (enabled() && isOpen())
    {
        if (ImGui::Begin(m_name.c_str(), &m_isOpen, windowFlags))
        {
            if (m_IPController != nullptr)
            {
                ImGui::Spacing();

                static double x=0;
                static double y=0;
                static double z=0;
                static double rx=0.;
                static double ry=0.;
                static double rz=0.;

                if(m_isDrivingSimulation)
                    m_IPController->getTCPTargetPosition(x, y, z, rx, ry, rz);
                
                if (ImGui::LocalBeginCollapsingHeader(m_TCPPositionDescription.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
                {
                    const auto &initPosition = m_IPController->getTCPTargetInitPosition();
                    showSliderDouble("X", "##XSlider", "##XInput", &x, m_TCPMinPosition + initPosition[0], m_TCPMaxPosition + initPosition[0], ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                    ImGui::Spacing();
                    showSliderDouble("Y", "##YSlider", "##YInput", &y, m_TCPMinPosition + initPosition[1], m_TCPMaxPosition + initPosition[1], ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
                    ImGui::Spacing();
                    showSliderDouble("Z", "##ZSlider", "##ZInput", &z, m_TCPMinPosition + initPosition[2], m_TCPMaxPosition + initPosition[2], ImVec4(0.0f, 0.0f, 1.0f, 1.0f));

                    ImGui::LocalEndCollapsingHeader();
                }

                m_IPController->setFreeInRotation(m_freeRoll, m_freePitch, m_freeYaw);

                if (ImGui::LocalBeginCollapsingHeader(m_TCPRotationDescription.c_str(), ImGuiTreeNodeFlags_AllowOverlap))
                {
                    ImGui::SameLine();

                    ImGui::SetCursorPosX(ImGui::GetWindowWidth() - ImGui::GetFrameHeight() - ImGui::GetStyle().FramePadding.x); // Set position to right of the line

                    bool openOptions = false;
                    ImGui::PushStyleColor(ImGuiCol_Button, ImGuiCol_Header);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGuiCol_Header);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGuiCol_Header);
                    if (ImGui::Button(ICON_FA_BARS, ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight())))
                        openOptions = true;
                    ImGui::PopStyleColor(3);

                    if (openOptions)
                    {
                        ImGui::OpenPopup("##RotationOptions");
                    }

                    if (ImGui::BeginPopup("##RotationOptions"))
                    {
                        showOptions();
                        ImGui::EndPopup();
                    }

                    if (m_freeRoll)
                        ImGui::BeginDisabled();
                    showSliderDouble("R", "##RSlider", "##RInput", &rx, m_TCPMinOrientation, m_TCPMaxOrientation, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                    if (m_freeRoll)
                        ImGui::EndDisabled();

                    ImGui::Spacing();

                    if (m_freePitch)
                        ImGui::BeginDisabled();
                    showSliderDouble("P", "##PSlider", "##PInput", &ry, m_TCPMinOrientation, m_TCPMaxOrientation, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
                    if (m_freePitch)
                        ImGui::EndDisabled();

                    ImGui::Spacing();

                    if (m_freeYaw)
                        ImGui::BeginDisabled();
                    showSliderDouble("Y", "##YawSlider", "##YawInput", &rz, m_TCPMinOrientation, m_TCPMaxOrientation, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
                    if (m_freeYaw)
                        ImGui::EndDisabled();

                    ImGui::LocalEndCollapsingHeader();
                }

                if (m_isDrivingSimulation)
                {
                    sofa::type::Quat<SReal> q = m_IPController->getTCPPosition().getOrientation();
                    sofa::type::Vec3 rotation = q.toEulerVector();
                    m_IPController->setTCPTargetPosition(x, y, z,
                                                         m_freeRoll? rotation[0]: rx,
                                                         m_freePitch? rotation[1]: ry,
                                                         m_freeYaw? rotation[2]: rz);
                }
            }

            if (!m_actuators.empty())
            {
                if (ImGui::LocalBeginCollapsingHeader(m_actuatorsDescription.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
                {
                    int nbActuators = m_actuators.size();
                    bool solveInverseProblem = true;
                    for (int i=0; i<nbActuators; i++)
                    {
                        std::string name = "M" + std::to_string(i);

                        auto* typeinfo = m_actuators[i].data->getValueTypeInfo();
                        auto* value = m_actuators[i].data->getValueVoidPtr();
                        double buffer = typeinfo->getScalarValue(value, 0);
                        bool hasChanged = showSliderDouble(name.c_str(), ("##Slider" + name).c_str(), ("##Input" + name).c_str(), &buffer, m_actuatorsMin, m_actuatorsMax,
                                                           ImVec4(0, 0, 0, 0));
                        if (hasChanged)
                        {
                            std::string value = std::to_string(buffer);
                            std::replace(value.begin(), value.end(), ',', '.');
                            m_actuators[i].data->read(value);
                            solveInverseProblem = false;
                        }
                        m_actuators[i].value=buffer;
                    }
                    if (m_IPController && !solveInverseProblem && m_isDrivingSimulation)
                    {
                        // TODO: don't solve the inverse problem since we'll overwrite the solution
                        m_IPController->applyActuatorsForce(m_actuators);
                    }

                    ImGui::LocalEndCollapsingHeader();
                }
            }

            if (!m_accessories.empty())
            {
                if (ImGui::LocalBeginCollapsingHeader("Accessories", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    for (auto& accessory: m_accessories)
                    {
                        std::string name = accessory.description;

                        auto* typeinfo = accessory.data->getValueTypeInfo();
                        auto* value = accessory.data->getValueVoidPtr();
                        double buffer = typeinfo->getScalarValue(value, 0);
                        bool hasChanged = showSliderDouble(name.c_str(),
                                                           ("##Slider" + name).c_str(),
                                                           ("##Input" + name).c_str(),
                                                           &buffer, accessory.min, accessory.max,
                                                           ImVec4(0, 0, 0, 0));
                        if (hasChanged && m_isDrivingSimulation)
                        {
                            std::string value = std::to_string(buffer);
                            std::replace(value.begin(), value.end(), ',', '.');
                            accessory.data->read(value);
                        }
                    }
                    ImGui::LocalEndCollapsingHeader();
                }
            }
        }
        ImGui::End();
    }
}

bool MoveWindow::showSliderDouble(const char* name, const char* label1, const char *label2, double* v, const double& min, const double& max, const ImVec4& color)
{
    ImGui::AlignTextToFramePadding();
    ImVec2 pos = ImGui::GetCurrentWindow()->DC.CursorPos;
    pos.y += ImGui::GetFrameHeight() / 4.;
    ImVec2 size(1.0f, ImGui::GetFrameHeight() / 2.);
    ImGui::GetWindowDrawList()->AddRectFilled(pos,
                                              ImVec2(pos.x + size.x, pos.y + size.y),
                                              ImGui::GetColorU32(color), ImGuiStyleVar_FrameRounding);
    ImGui::Spacing();
    ImGui::SameLine();

    return showSliderDouble(name, label1, label2, v, min, max);
}

bool MoveWindow::showSliderDouble(const char* name, const char* label1, const char *label2, double* v, const double& min, const double& max)
{
    bool hasValueChanged = false;

    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s", name);
    ImGui::SameLine();
    float inputWidth = ImGui::CalcTextSize("-100000,00").x + ImGui::GetFrameHeight() / 2 + ImGui::GetStyle().ItemSpacing.x * 2;
    float sliderWidth = ImGui::GetWindowWidth() - inputWidth - ImGui::CalcTextSize(name).x - ImGui::GetStyle().FramePadding.x * 3 - ImGui::GetStyle().IndentSpacing - ImGui::GetStyle().ScrollbarSize;

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    ImGui::PushItemWidth(sliderWidth);
    if (ImGui::SliderScalar(label1, ImGuiDataType_Double, v, &min, &max, "%0.2f", ImGuiSliderFlags_NoInput))
        hasValueChanged=true;
    ImGui::PopItemWidth();
    ImGui::PopStyleColor();

    ImGui::SameLine();

    double step = max - min;

    if (ImGui::LocalInputDouble(label2, v, powf(10.0f, floorf(log10f(step * 0.01))), step * 0.1))
        hasValueChanged=true;

    return hasValueChanged;
}

void MoveWindow::showOptions()
{
    if (ImGui::BeginTable("##option", 2, ImGuiTableFlags_None))
    {
        ImGui::TableNextColumn();
        ImGui::LocalCheckBox("Free roll", &m_freeRoll);
        if (m_freeRoll)
            ImGui::BeginDisabled();
        ImGui::TableNextColumn();
        showWeightOption(0);
        if (m_freeRoll)
            ImGui::EndDisabled();

        ImGui::TableNextColumn();
        ImGui::LocalCheckBox("Free pitch", &m_freePitch);
        if (m_freePitch)
            ImGui::BeginDisabled();
        ImGui::TableNextColumn();
        showWeightOption(1);
        if (m_freePitch)
            ImGui::EndDisabled();

        ImGui::TableNextColumn();
        ImGui::LocalCheckBox("Free yaw", &m_freeYaw);
        if (m_freeYaw)
            ImGui::BeginDisabled();
        ImGui::TableNextColumn();
        showWeightOption(2);
        if (m_freeYaw)
            ImGui::EndDisabled();

        ImGui::EndTable();
    }
}

void MoveWindow::showWeightOption(const int &i)
{
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();

    auto* weight = m_IPController->getRotationWeight();
    double w = weight[i];
    ImGui::AlignTextToFramePadding();
    ImGui::Text("weight");
    ImGui::SameLine();
    ImGui::PushID(i);
    ImGui::LocalInputDouble("##Input ", &w, 0, 0);
    ImGui::PopID();
    weight[i] = w;
}

}

