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
#include <implot.h>
#include <implot_internal.h>

namespace sofaimgui::windows {

#define MAX_NB_PLOT 4

class SOFAIMGUI_API PlottingWindow : public BaseWindow
{
   public:

    struct RollingBuffer
    {
        float span;
        float ratio = 1;
        ImVector<ImVec2> data;
        RollingBuffer()
        {
            span = 20.0f;
            data.reserve(2000);
        }
        void addPoint(float x, float y)
        {
            float xmod = fmodf(x, span);
            if (!data.empty() && xmod < data.back().x)
                data.erase(data.begin());
            data.push_back(ImVec2(x, y * ratio));
        }
    };

    struct PlottingData
    {
        sofa::core::objectmodel::BaseData* value;
        std::string description;
        size_t idSubplot{0};
    };

    PlottingWindow(const std::string& name, const bool& isWindowOpen);
    ~PlottingWindow() = default;

    void showWindow(sofa::simulation::Node::SPtr groot, const ImGuiWindowFlags &windowFlags);
    void addData(const PlottingData data) {m_data.push_back(data);}
    void clearData();

   protected:
    std::vector<PlottingData> m_data;
    std::vector<RollingBuffer> m_buffers;
    float m_ratio[MAX_NB_PLOT] = {1, 1, 1, 1};

    size_t m_nbRows{1};
    size_t m_nbCols{1};

    void exportData();
    void showMenu();
    void showMenu(ImPlotPlot &plot, const size_t &idSubplot);
};

}


