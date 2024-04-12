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

class PlottingWindow : public BaseWindow
{
   public:

    struct RollingBuffer {
        float Span;
        ImVector<ImVec2> Data;
        RollingBuffer()
        {
            Span = 20.0f;
            Data.reserve(2000);
        }
        void addPoint(float x, float y)
        {
            float xmod = fmodf(x, Span);
            if (!Data.empty() && xmod < Data.back().x)
                Data.shrink(0);
            Data.push_back(ImVec2(xmod, y));
        }
    };

    struct PlottingData{
        sofa::core::objectmodel::BaseData* value;
        std::string description;
        int idSubplot{0};
    };

    PlottingWindow(const std::string& name, const bool& isWindowOpen);
    ~PlottingWindow() = default;

    void showWindow(sofa::simulation::Node::SPtr groot, const ImGuiWindowFlags &windowFlags);
    void addData(const PlottingData data) {m_data.push_back(data);}
    void clearData();

   protected:
    std::vector<PlottingData> m_data;
    std::vector<RollingBuffer> m_buffers;

    void exportData();
};

}


