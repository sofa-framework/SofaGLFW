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
#include <imgui.h>
#include <imgui_internal.h> //imgui_internal.h is included in order to use the DockspaceBuilder API (which is still in development)
#include <sofa/type/vector.h>
#include <SofaImGui/ImGuiGUIEngine.h>

#include <unordered_set>

#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>


#include <sofa/simulation/Simulation.h>

#include <sofa/helper/AdvancedTimer.h>

#include <implot.h>

#include <IconsFontAwesome6.h>

#include <sofa/helper/Utils.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/component/visual/LineAxis.h>

#include <sofa/gui/common/BaseGUI.h>

#include "Profiler.h"


namespace windows {

    void showProfiler(sofa::core::sptr<sofa::simulation::Node> groot
            , const char* const& windowNameProfiler
            , WindowState& winManagerProfiler)
    {
        if (*winManagerProfiler.getStatePtr())
        {
            static int selectedFrame = 0;

            if (ImGui::Begin(windowNameProfiler, winManagerProfiler.getStatePtr()))
            {
                const auto convertInMs = [](sofa::helper::system::thread::ctime_t t)
                {
                    static auto timer_freqd = static_cast<SReal>(sofa::helper::system::thread::CTime::getTicksPerSec());
                    return 1000.0 * static_cast<SReal>(t) / static_cast<SReal>(timer_freqd);
                };

                static std::deque< sofa::type::vector<sofa::helper::Record> > allRecords;
                static int bufferSize = 500;
                ImGui::SliderInt("Buffer size", &bufferSize, 10, 5000);
                selectedFrame = std::min(selectedFrame, bufferSize - 1);

                static bool showChart = true;
                ImGui::Checkbox("Show Chart", &showChart);

                if (groot->animate_.getValue())
                {
                    sofa::type::vector<sofa::helper::Record> _records = sofa::helper::AdvancedTimer::getRecords("Animate");
                    allRecords.emplace_back(std::move(_records));

                    while (allRecords.size() >= bufferSize)
                    {
                        allRecords.pop_front();
                    }
                }

                static std::unordered_set<int> selectedTimers;

                struct Chart
                {
                    std::string label;
                    sofa::type::vector<float> values;
                };

                sofa::type::vector<float> frameChart;
                frameChart.reserve(allRecords.size());

                sofa::type::vector<Chart> charts;
                charts.reserve(selectedTimers.size());

                if (showChart)
                {
                    for (const auto& records : allRecords)
                    {
                        if (records.size() >= 2)
                        {
                            const auto tMin = records.front().time;
                            const auto tMax = records.back().time;
                            const auto frameDuration = convertInMs(tMax - tMin);
                            frameChart.push_back(frameDuration);
                        }
                        else
                        {
                            frameChart.push_back(0.);
                        }
                    }

                    for (const auto timerId : selectedTimers)
                    {
                        Chart chart;
                        for (const auto& records : allRecords)
                        {
                            float value = 0.f;
                            sofa::helper::system::thread::ctime_t t0;
                            for (const auto& rec : records)
                            {
                                if (timerId == rec.id)
                                {
                                    chart.label = rec.label;
                                    if (rec.type == sofa::helper::Record::RBEGIN || rec.type == sofa::helper::Record::RSTEP_BEGIN || rec.type == sofa::helper::Record::RSTEP)
                                    {
                                        t0 = rec.time;
                                    }
                                    if (rec.type == sofa::helper::Record::REND || rec.type == sofa::helper::Record::RSTEP_END)
                                    {
                                        value += convertInMs(rec.time - t0);
                                    }
                                }
                            }
                            chart.values.push_back(value);
                        }
                        charts.push_back(chart);
                    }

                    static double selectedFrameInChart = selectedFrame;
                    selectedFrameInChart = selectedFrame;
                    if (ImPlot::BeginPlot("##ProfilerChart"))
                    {
                        static ImPlotAxisFlags xflags = ImPlotAxisFlags_None;
                        static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit;
                        ImPlot::SetupAxes("Time Step","Duration (ms)", xflags, yflags);
                        ImPlot::SetupAxesLimits(0, bufferSize, 0, 10);
                        if (ImPlot::DragLineX(0, &selectedFrameInChart, IMPLOT_AUTO_COL))
                        {
                            selectedFrame = std::round(selectedFrameInChart);
                        }
                        ImPlot::PlotLine("Total", frameChart.data(), frameChart.size());
                        for (const auto& chart : charts)
                        {
                            ImPlot::PlotLine(chart.label.c_str(), chart.values.data(), chart.values.size());
                        }
                        ImPlot::EndPlot();
                    }
                }

                ImGui::SliderInt("Frame", &selectedFrame, 0, allRecords.size());


                if (selectedFrame >= 0 && selectedFrame < allRecords.size())
                {
                    const auto records = allRecords[selectedFrame];
                    if (!records.empty())
                    {
                        auto tStart = records.front().time;
                        auto tEnd = tStart;
                        std::unordered_map<unsigned int, SReal > duration;
                        std::stack<sofa::helper::system::thread::ctime_t> durationStack;
                        std::stack<unsigned int> timerIdStack;
                        unsigned int timerIdCounter {};
                        for (const auto& rec : allRecords[selectedFrame])
                        {
                            tStart = std::min(tStart, rec.time);
                            tEnd = std::max(tEnd, rec.time);

                            if (rec.type == sofa::helper::Record::RBEGIN || rec.type == sofa::helper::Record::RSTEP_BEGIN || rec.type == sofa::helper::Record::RSTEP)
                            {
                                durationStack.push(rec.time);
                                timerIdStack.push(timerIdCounter++);
                            }
                            if (rec.type == sofa::helper::Record::REND || rec.type == sofa::helper::Record::RSTEP_END)
                            {
                                const auto t = durationStack.top();
                                durationStack.pop();
                                duration[timerIdStack.top()] = convertInMs(rec.time - t);
                                timerIdStack.pop();
                            }
                        }

                        const auto frameDuration = convertInMs(tEnd - tStart);
                        ImGui::Text("Frame duration (ms): %f", frameDuration);

                        const bool expand = ImGui::Button(ICON_FA_EXPAND);
                        ImGui::SameLine();
                        const bool collapse = ImGui::Button(ICON_FA_COMPRESS);

                        static ImGuiTableFlags flags = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody;
                        if (ImGui::BeginTable("profilerTable", 3, flags))
                        {
                            std::stack<bool> openStack;

                            ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_NoHide);
                            ImGui::TableSetupColumn("Percent (%)", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("A").x * 12.0f);
                            ImGui::TableSetupColumn("Duration (ms)", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("A").x * 12.0f);
                            ImGui::TableHeadersRow();

                            int node_clicked = -1;
                            static ImGuiTreeNodeFlags base_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
                            timerIdCounter = 0;
                            for (auto it = records.begin(); it != records.end(); ++it)
                            {
                                const auto& rec = *it;
                                if (rec.type == sofa::helper::Record::RBEGIN || rec.type == sofa::helper::Record::RSTEP_BEGIN || rec.type == sofa::helper::Record::RSTEP)
                                {
                                    if (openStack.empty() || openStack.top())
                                    {
                                        ImGuiTreeNodeFlags node_flags = base_flags;
                                        if (selectedTimers.find(rec.id) != selectedTimers.end())
                                        {
                                            node_flags |= ImGuiTreeNodeFlags_Selected;
                                        }

                                        if (it + 1 != records.end())
                                        {
                                            const auto& nextRec = *(it + 1);
                                            if (it->label == nextRec.label && (nextRec.type == sofa::helper::Record::REND || nextRec.type == sofa::helper::Record::RSTEP_END))
                                            {
                                                node_flags |= ImGuiTreeNodeFlags_Leaf;
                                            }
                                        }

                                        ImGui::TableNextRow();

                                        ImGui::TableNextColumn();
                                        if (expand) ImGui::SetNextItemOpen(true);
                                        if (collapse) ImGui::SetNextItemOpen(false);
                                        const bool isOpen = ImGui::TreeNodeEx(rec.label.c_str(), node_flags);
                                        if (ImGui::IsItemHovered())
                                        {
                                            ImGui::BeginTooltip();
                                            ImGui::TextDisabled(rec.label.c_str());
                                            ImGui::TextDisabled("ID: %s", std::to_string(rec.id).c_str());
                                            ImGui::EndTooltip();
                                        }
                                        openStack.push(isOpen);

                                        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
                                            node_clicked = rec.id;

                                        const auto& d = (rec.label == "Animate") ? frameDuration : duration[timerIdCounter];

                                        ImVec4 color;
                                        color.w = 1.f;
                                        const auto ratio = (rec.label == "Animate") ? 1. : d/frameDuration;
                                        constexpr auto clamp = [](double d){ return std::max(0., std::min(1., d));};
                                        ImGui::ColorConvertHSVtoRGB(120./360. * clamp(1.-ratio*10.), 0.72f, 1.f, color.x,color.y, color.z);
                                        ImGui::TableNextColumn();
                                        ImGui::TextColored(color, "%.2f", 100 * ratio);

                                        ImGui::TableNextColumn();
                                        ImGui::TextColored(color, "%f", d);
                                    }
                                    else
                                    {
                                        openStack.push(false);
                                    }
                                    ++timerIdCounter;
                                }
                                if (rec.type == sofa::helper::Record::REND || rec.type == sofa::helper::Record::RSTEP_END)
                                {
                                    if (openStack.top())
                                    {
                                        ImGui::TreePop();
                                    }
                                    openStack.pop();
                                }
                            }
                            while(!openStack.empty())
                            {
                                if (openStack.top())
                                {
                                    ImGui::TreePop();
                                }
                                openStack.pop();
                            }

                            ImGui::EndTable();

                            if (node_clicked != -1)
                            {
                                auto it = selectedTimers.find(node_clicked);
                                if (it == selectedTimers.end())
                                {
                                    selectedTimers.insert(node_clicked);
                                }
                                else
                                {
                                    selectedTimers.erase(it);
                                }
                            }
                        }
                    }
                }
            }
            ImGui::End();
        }
    }

}
