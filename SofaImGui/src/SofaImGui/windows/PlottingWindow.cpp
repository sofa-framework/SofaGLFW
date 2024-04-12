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
#include <SofaImGui/windows/PlottingWindow.h>
#include <implot.h>
#include <implot_demo.cpp>
#include <IconsFontAwesome6.h>

namespace sofaimgui::windows {

PlottingWindow::PlottingWindow(const std::string& name,
                               const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void PlottingWindow::clearData()
{
    m_data.clear();
    m_buffers.clear();
}

void PlottingWindow::showWindow(sofa::simulation::Node::SPtr groot, const ImGuiWindowFlags &windowFlags)
{
    static bool firstTime = true;

    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, ImGuiWindowFlags_NoScrollbar))
        {
            static int nbRows = 1;
            static bool lockedAxis = true;
            static ImPlotAxisFlags axesFlags = ImPlotAxisFlags_AutoFit;
            static PlottingData* dragedData;
            ImVec2 buttonSize(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
            auto positionRight = ImGui::GetCursorPosX() + ImGui::GetWindowSize().x - buttonSize.x * 3 - ImGui::GetStyle().ItemSpacing.y * 4; // Get position for right buttons

            size_t nbData = m_data.size();
            if (m_buffers.empty())
                m_buffers.resize(nbData);

            if (ImGui::Button("Clear"))
            {
                for(auto& buffer: m_buffers)
                    buffer.Data.clear();
            }

            ImGui::SameLine();
            ImGui::SetCursorPosX(positionRight); // Set position to right of the header

            if(ImGui::Button((lockedAxis)? ICON_FA_LOCK: ICON_FA_LOCK_OPEN, buttonSize))
            {
                lockedAxis = !lockedAxis;
                axesFlags = (lockedAxis)? ImPlotAxisFlags_AutoFit: ImPlotAxisFlags_None;
            }
            ImGui::SetItemTooltip("Lock x and y axes to fit content.");

            ImGui::SameLine();

            if(ImGui::Button("+##plotting", buttonSize))
            {
                if (nbRows<4)
                    nbRows+=1;
            }
            ImGui::SetItemTooltip("Show an additional subplot.");

            ImGui::SameLine();

            if (ImGui::Button("-##plotting", buttonSize))
            {
                if (nbRows>1)
                    nbRows-=1;
            }
            ImGui::SetItemTooltip("Hide last subplot.");

            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));

            if (ImPlot::BeginSubplots("##myplots", nbRows, 1, ImVec2(-1, -1), ImPlotSubplotFlags_ShareItems))
            {
                for (int i=0; i<nbRows; i++)
                {
                    if (ImPlot::BeginPlot(("##" +std::to_string(i)).c_str(), ImVec2(-1, 150),
                                           ImPlotFlags_NoMouseText))
                    {
                        ImPlot::SetupLegend(ImPlotLocation_East, ImPlotLegendFlags_Sort | ImPlotLegendFlags_Outside);
                        ImPlot::SetupAxes("Time (s)", nullptr,
                                          axesFlags,
                                          axesFlags);
                        for (size_t k=0; k<nbData; k++)
                        {
                            auto& data = m_data[k];
                            if (data.idSubplot == i)
                            {
                                const sofa::defaulttype::AbstractTypeInfo* typeInfo = data.value->getValueTypeInfo();
                                float value = typeInfo->getScalarValue(data.value->getValueVoidPtr(), 0);
                                float time = groot->getTime();
                                RollingBuffer& buffer = m_buffers[k];                                

                                if (groot->getAnimate())
                                    buffer.addPoint(time, value);

                                ImPlot::PlotLine(data.description.c_str(),
                                                 &buffer.Data[0].x,
                                                 &buffer.Data[0].y,
                                                 buffer.Data.size(),
                                                 0, 0, 2 * sizeof(float));

                                if (ImPlot::BeginDragDropSourceItem(data.description.c_str())) {
                                    dragedData = &data;
                                    ImGui::SetDragDropPayload("dragndrop", nullptr, 0);
                                    ImPlot::ItemIcon(ImPlot::GetLastItemColor());
                                    ImGui::SameLine();
                                    ImGui::TextUnformatted(data.description.c_str());
                                    ImGui::Spacing();
                                    ImPlot::EndDragDropSource();
                                }
                            }
                        }

                        if (ImPlot::BeginDragDropTargetPlot())
                        {
                            if (ImGui::AcceptDragDropPayload("dragndrop"))
                            {
                                if (dragedData)
                                    dragedData->idSubplot = i;
                            }
                            ImPlot::EndDragDropTarget();
                        }

                        if (firstTime)
                        {
                            for (size_t k=0; k<nbData; k++)
                            {
                                auto item = ImPlot::GetItem(m_data[k].description.c_str());
                                if (item && k!=0)
                                    item->Show = false;
                            }
                        }

                        ImPlot::EndPlot();
                    }
                    ImGui::Spacing();
                }

                ImPlot::EndSubplots();
            }

            ImGui::PopStyleColor(1);
        }
        firstTime = false;
    }
}

}

