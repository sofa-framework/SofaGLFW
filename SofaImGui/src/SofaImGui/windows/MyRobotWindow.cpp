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

#include <string>
#include <SofaImGui/widgets/Buttons.h>
#include <SofaImGui/windows/MyRobotWindow.h>

namespace sofaimgui::windows {

std::string MyRobotWindow::DEFAULTGROUP = "empty";

MyRobotWindow::MyRobotWindow(const std::string& name,
                         const bool& isWindowOpen)
{
    m_name = name;
    m_isOpen = isWindowOpen;
    m_isDrivingSimulation = true;
}

void MyRobotWindow::clearData()
{
    m_informationGroups.clear();
    m_settingGroups.clear();
}

bool MyRobotWindow::isInEmptyGroup(const std::string &group)
{
    return DEFAULTGROUP.find(group) != std::string::npos;
}

void MyRobotWindow::addInformation(const Information &info, const std::string &group)
{
    bool found=false;
    for (auto &g: m_informationGroups)
    {
        if (g.description.find(group) != std::string::npos)
        {
            found=true;
            g.information.push_back(info);
            break;
        }
    }

    if (!found)
    {
        InformationGroup g;
        g.description = group;
        g.information.push_back(info);

        if (isInEmptyGroup(group))
            m_informationGroups.insert(m_informationGroups.begin(), g);
        else
            m_informationGroups.push_back(g);
    }
}

void MyRobotWindow::addSetting(const Setting &setting, const std::string &group)
{
    bool found=false;
    for (auto &s: m_settingGroups)
    {
        if (s.description.find(group) != std::string::npos)
        {
            found=true;
            s.settings.push_back(setting);
            break;
        }
    }

    if (!found)
    {
        SettingGroup s;
        s.description = group;
        s.settings.push_back(setting);

        if (isInEmptyGroup(group))
            m_settingGroups.insert(m_settingGroups.begin(), s);
        else
            m_settingGroups.push_back(s);
    }
}

void MyRobotWindow::showWindow(const ImGuiWindowFlags &windowFlags)
{
    if (enabled() && isOpen())
    {
        if (ImGui::Begin(m_name.c_str(), &m_isOpen, windowFlags))
        {
            ImGui::Spacing();

            if (!m_informationGroups.empty())
            {
                if (ImGui::LocalBeginCollapsingHeader("Information", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    std::string groups;
                    int k=0;
                    for (auto &group: m_informationGroups)
                    {
                        ImGui::PushID(k++);
                        if (!isInEmptyGroup(group.description))
                        {
                            ImGui::TextDisabled("%s", group.description.c_str());
                            ImGui::Indent();
                        }

                        int i=0;
                        for (auto &information: group.information)
                        {
                            ImGui::PushID(i++);
                            ImGui::AlignTextToFramePadding();
                            ImGui::Text("%s", information.description.c_str());
                            ImGui::SameLine();

                            auto* typeinfo = information.data->getValueTypeInfo();
                            auto* values = information.data->getValueVoidPtr();

                            ImGui::BeginDisabled();
                            for (size_t i=0; i<typeinfo->size(); i++)
                            {
                                double buffer = typeinfo->getScalarValue(values, i);
                                ImGui::LocalInputDouble(("##information" + information.description).c_str(), &buffer, 0, 0);
                            }
                            ImGui::EndDisabled();
                            ImGui::PopID();
                        }

                        if (!isInEmptyGroup(group.description))
                            ImGui::Unindent();

                        ImGui::PopID();
                    }

                    ImGui::LocalEndCollapsingHeader();
                }
            }

            if (!m_settingGroups.empty())
            {
                if (ImGui::LocalBeginCollapsingHeader("Settings", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    std::string groups;
                    int k=0;
                    for (auto &group: m_settingGroups)
                    {
                        ImGui::PushID(k++);
                        if (!isInEmptyGroup(group.description))
                        {
                            ImGui::TextDisabled("%s", group.description.c_str());
                            ImGui::Indent();
                        }

                        for (auto &setting: group.settings)
                        {
                            ImGui::AlignTextToFramePadding();
                            ImGui::Text("%s", setting.description.c_str());
                            ImGui::SameLine();

                            auto* typeinfo = setting.data->getValueTypeInfo();
                            auto* values = setting.data->getValueVoidPtr();

                            std::string uiValue;
                            for (size_t i=0; i<typeinfo->size(); i++)
                            {
                                setting.buffer = typeinfo->getScalarValue(values, i);
                                showSliderDouble(setting.description, &setting.buffer, setting.min, setting.max, (isInEmptyGroup(group.description))? 1: 2);
                                setting.buffer = std::clamp(setting.buffer, setting.min, setting.max);
                                uiValue += std::to_string(setting.buffer) + " ";
                            }
                            setting.data->read(uiValue);
                        }

                        if (!isInEmptyGroup(group.description))
                            ImGui::Unindent();

                        ImGui::PopID();
                    }
                    ImGui::LocalEndCollapsingHeader();
                }
            }
        }
        ImGui::End();
    }
}

bool MyRobotWindow::showSliderDouble(const std::string& name, double* v, const double& min, const double& max, const int nbIndents)
{
    bool hasValueChanged = false;
    float inputWidth = ImGui::CalcTextSize("-100000,00").x + ImGui::GetFrameHeight() / 2 + ImGui::GetStyle().ItemSpacing.x * 2;
    float sliderWidth = ImGui::GetWindowWidth() - inputWidth - ImGui::CalcTextSize(name.c_str()).x - ImGui::GetStyle().FramePadding.x - ImGui::GetStyle().IndentSpacing * nbIndents - ImGui::GetStyle().ScrollbarSize;

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    ImGui::PushItemWidth(sliderWidth);
    if (ImGui::SliderScalar(("##SettingSlider" + name).c_str() , ImGuiDataType_Double, v, &min, &max, "%0.2f", ImGuiSliderFlags_NoInput))
        hasValueChanged=true;
    ImGui::PopItemWidth();
    ImGui::PopStyleColor();

    ImGui::SameLine();

    double step = max - min;

    if (ImGui::LocalInputDouble(("##SettingInput" + name).c_str(), v, powf(10.0f, floorf(log10f(step * 0.01))), step * 0.1))
        hasValueChanged=true;

    return hasValueChanged;
}

}

