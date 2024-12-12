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
#include <imgui.h>

namespace sofaimgui::windows {

class SOFAIMGUI_API MyRobotWindow : public BaseWindow
{
   public:
    MyRobotWindow(const std::string& name, const bool& isWindowOpen);
    ~MyRobotWindow() = default;

    void showWindow(const ImGuiWindowFlags &windowFlags);
    bool enabled() override {return (!m_informationGroups.empty() || !m_settingGroups.empty());}

    struct Information{
        std::string description;
        sofa::core::BaseData* data;
    };

    struct Setting{
        double buffer;
        std::string description;
        sofa::core::BaseData* data;
        double min;
        double max;
    };

    static std::string DEFAULTGROUP;

    struct InformationGroup{
        std::string description;
        std::vector<Information> information;
    };

    struct SettingGroup{
        std::string description;
        std::vector<Setting> settings;
    };

    void clearData();
    void addInformation(const Information &info, const std::string &group);
    void addSetting(const Setting &setting, const std::string &group);

   protected:

    std::vector<InformationGroup> m_informationGroups;
    std::vector<SettingGroup> m_settingGroups;

    bool isInEmptyGroup(const std::string &group);
    bool showSliderDouble(const std::string &name, double* v, const double& min, const double& max, const int nbIndents);
};

}


