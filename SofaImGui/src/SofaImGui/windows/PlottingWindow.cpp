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

#include <SofaImGui/windows/PlottingWindow.h>

#include <imgui_internal.h>
#include <IconsFontAwesome6.h>

#include <iostream>
#include <fstream>
#include <SofaImGui/widgets/Buttons.h>
#include <nfd.h>

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

void PlottingWindow::exportData()
{
    nfdchar_t *outPath;
    size_t nbData = m_data.size();

    const nfdresult_t result = NFD_SaveDialog(&outPath, nullptr, 0, nullptr, "plotting.csv");
    if (result == NFD_OKAY)
    {
        if (nbData)
        {
            std::ofstream outputFile;
            outputFile.open(outPath, std::ios::out);

            if (outputFile.is_open())
            {
                outputFile << "time,";
                for (const auto& d : m_buffers[0].data)
                    outputFile << d.x << ",";
                outputFile << "\n";

                for (size_t i=0; i<nbData; i++)
                {
                    outputFile << m_data[i].description << ",";
                    auto buffer = m_buffers[i];
                    for (const auto& d : buffer.data)
                        outputFile << d.y << ",";
                    outputFile << "\n";
                }
                outputFile.close();
            }
        }
    }
}

void PlottingWindow::showWindow(sofa::simulation::Node::SPtr groot, const ImGuiWindowFlags &windowFlags)
{
    SOFA_UNUSED(windowFlags);

    if (m_isWindowOpen && !m_data.empty())
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, ImGuiWindowFlags_NoScrollbar))
        {
            static int nbRows = 1;
            static int nbCols = 1;
            static PlottingData* dragedData;
            ImVec2 buttonSize(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
            auto positionRight = ImGui::GetCursorPosX() + ImGui::GetWindowSize().x - buttonSize.x * 2 - ImGui::GetStyle().ItemSpacing.y * 3; // Get position for right buttons
            auto positionMiddle = ImGui::GetCursorPosX() + ImGui::GetWindowSize().x / 2.f; // Get position for middle button

            size_t nbData = m_data.size();
            if (m_buffers.empty())
                m_buffers.resize(nbData);

            if (ImGui::Button("Clear"))
            {
                for(auto& buffer: m_buffers)
                    buffer.data.clear();
            }

            ImGui::SameLine();

            if (ImGui::Button(ICON_FA_ARROW_UP_FROM_BRACKET, ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight())))
            {
                exportData();
            }

            ImGui::SameLine();

            ImGui::SetCursorPosX(positionMiddle - ImGui::CalcTextSize("Clears every 20 s").x / 2.); // Set position to the middle of the header

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Clears every 20 s");

            ImGui::SameLine();
            ImGui::SetCursorPosX(positionRight); // Set position to right of the header

            if(ImGui::Button("+##plotting", buttonSize))
            {
                if (nbRows<MAX_NB_PLOT)
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

            bool portraitLayout = (ImGui::GetWindowWidth() * 0.75 < ImGui::GetWindowHeight());
            if (ImPlot::BeginSubplots("##myplots",
                                      portraitLayout? nbRows: nbCols,
                                      portraitLayout? nbCols: nbRows,
                                      ImVec2(-1, -1),
                                      ImPlotSubplotFlags_ShareItems
                                      ))
            {
                for (int i=0; i<nbRows * nbCols; i++)
                {
                    if (ImPlot::BeginPlot(("##" +std::to_string(i)).c_str(), ImVec2(-1, 0),
                                           ImPlotFlags_NoMouseText | ImPlotFlags_NoMenus))
                    {
                        ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Sort | ImPlotLegendFlags_Outside);
                        ImPlot::SetupAxes("Time (s)", nullptr,
                                          ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoSideSwitch | ImPlotAxisFlags_NoHighlight,
                                          ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoSideSwitch | ImPlotAxisFlags_NoHighlight);
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
                                                 &buffer.data[0].x,
                                                 &buffer.data[0].y,
                                                 buffer.data.size(),
                                                 0, 0, 2 * sizeof(float));

                                if (ImPlot::BeginDragDropSourceItem(data.description.c_str())) {
                                    dragedData = &data;
                                    ImGui::SetDragDropPayload("dragndrop", nullptr, 0);
                                    ImPlot::ItemIcon(ImPlot::GetLastItemColor());
                                    ImGui::SameLine();
                                    ImGui::TextUnformatted(data.description.c_str());
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

                        ImPlotContext& gp = *GImPlot;
                        ImPlotPlot &plot  = *gp.CurrentPlot;

                        ImPlot::EndPlot();

                        ImGui::PushOverrideID(plot.ID);

                        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
                            !plot.Items.Legend.Hovered &&
                            plot.Hovered)
                        {
                            ImGui::OpenPopup("##MyPlotContext");
                        }

                        if (ImGui::BeginPopup("##MyPlotContext"))
                        {
                            ImGui::PopStyleColor();
                            showMenu(plot, i);
                            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
                            ImGui::EndPopup();
                        }

                        ImGui::PopID();
                    }
                }
                ImPlot::EndSubplots();
            }
            ImGui::PopStyleColor();
        }
    }
}

void PlottingWindow::showMenu(ImPlotPlot &plot, const int &idSubplot)
{
    ImGui::PushID(plot.ID);
    if (ImGui::BeginTable("Columns", 2, ImGuiTableFlags_None))
    {
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Ratio");
        ImGui::TableNextColumn();
        ImGui::SameLine();

        float ratio = m_ratio[idSubplot];
        ImGui::PushItemWidth(ImGui::CalcTextSize("-100000,00").x);
        if (ImGui::InputFloat(("##Ratio" + std::to_string(idSubplot)).c_str(), &ratio, 0, 0, "%0.2e"))
        {
            size_t nbData = m_data.size();
            for (size_t i=0; i<nbData; i++)
            {
                auto& data = m_data[i];
                auto& buffer = m_buffers[i];
                if (data.idSubplot == idSubplot)
                {
                    for (auto& point: buffer.data)
                    {
                        point.y /= buffer.ratio;
                        point.y *= ratio;
                    }
                    buffer.ratio = ratio;
                }
            }
            m_ratio[idSubplot] = ratio;
        }
        ImGui::PopItemWidth();
        ImGui::EndTable();
    }

    ImGui::Separator();

    bool showMousePosition = !ImHasFlag(plot.Flags, ImPlotFlags_NoMouseText);
    if (ImGui::LocalCheckBox("Show mouse position", &showMousePosition))
        ImFlipFlag(plot.Flags, ImPlotFlags_NoMouseText);

    bool showGrid = !ImHasFlag(plot.XAxis(0).Flags, ImPlotAxisFlags_NoGridLines);
    if (ImGui::LocalCheckBox("Show grid", &showGrid))
    {
        ImFlipFlag(plot.XAxis(0).Flags, ImPlotAxisFlags_NoGridLines);
        ImFlipFlag(plot.YAxis(0).Flags, ImPlotAxisFlags_NoGridLines);
    }

    bool autofit = ImHasFlag(plot.XAxis(0).Flags, ImPlotAxisFlags_AutoFit);
    if (ImGui::LocalCheckBox("Auto fit content", &autofit))
    {
        ImFlipFlag(plot.XAxis(0).Flags, ImPlotAxisFlags_AutoFit);
        ImFlipFlag(plot.YAxis(0).Flags, ImPlotAxisFlags_AutoFit);
    }

    ImGui::PopID();
}

}

