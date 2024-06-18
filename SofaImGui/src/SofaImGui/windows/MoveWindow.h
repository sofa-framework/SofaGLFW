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
#include <SofaImGui/models/IPController.h>
#include <imgui.h>

namespace sofaimgui::windows {

class SOFAIMGUI_API MoveWindow : public BaseWindow
{
   public:
    MoveWindow(const std::string& name, const bool& isWindowOpen);
    ~MoveWindow() = default;

    void showWindow(const ImGuiWindowFlags &windowFlags);

    void setTCPDescriptions(const std::string &positionDescription, const std::string &rotationDescription);
    void setIPController(models::IPController::SPtr IPController) {m_IPController=IPController;}
    void setTCPLimits(int minPosition, int maxPosition, double minOrientation, double maxOrientation);

    void setActuatorsDescriptions(const std::string &description);
    void setActuatorsLimits(double min, double max);
    void setActuators(std::vector<models::IPController::Actuator> actuators) {m_actuators = actuators;}

    struct Accessory {
        double buffer;
        std::string description;
        sofa::core::BaseData* data;
        float min;
        float max;
    };

    void clearData() {m_accessories.clear();}
    void addAccessory(const Accessory &accessory) {m_accessories.push_back(accessory);}

   protected:
    
    models::IPController::SPtr m_IPController;
    std::string m_TCPPositionDescription{"TCP Target Position (mm)"};
    std::string m_TCPRotationDescription{"TCP Target Rotation (rad)"};
    double m_TCPMinPosition{-500.};
    double m_TCPMaxPosition{500.};
    double m_TCPMinOrientation{-M_PI};
    double m_TCPMaxOrientation{M_PI};
    
    std::vector<models::IPController::Actuator> m_actuators;
    std::string m_actuatorsDescription{"Motors Position (rad)"};
    double m_actuatorsMin{-500.};
    double m_actuatorsMax{500.};

    bool m_freeRoll{true};
    bool m_freePitch{true};
    bool m_freeYaw{true};

    std::vector<Accessory> m_accessories;

    bool showSliderDouble(const char *name, const char* label1, const char *label2, double* v, const double& min, const double& max, const ImVec4 &color);
    bool showSliderDouble(const char *name, const char* label1, const char *label2, double* v, const double& min, const double& max);
    void showOptions();
    void showWeightOption(const int &index);
};

}


