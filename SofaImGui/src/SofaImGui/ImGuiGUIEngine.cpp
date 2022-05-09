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
#include <SofaImGui/ImGuiGUIEngine.h>

#include <iomanip>
#include <ostream>
#include <unordered_set>
#include <SofaGLFW/SofaGLFWBaseGUI.h>

#include <sofa/core/CategoryLibrary.h>
#include <sofa/helper/logging/LoggingMessageHandler.h>

#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>

#include <sofa/helper/system/FileSystem.h>
#include <sofa/simulation/Simulation.h>

#include <sofa/helper/AdvancedTimer.h>

#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_internal.h> //imgui_internal.h is included in order to use the DockspaceBuilder API (which is still in development)
#include <implot.h>
#include <nfd.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_opengl2.h>
#include <IconsFontAwesome5.h>
#include <fa-regular-400.h>
#include <fa-solid-900.h>
#include <Roboto-Medium.h>
#include <Style.h>
#include <SofaImGui/ImGuiDataWidget.h>

#include <sofa/helper/Utils.h>
#include <sofa/type/vector.h>
#include <sofa/simulation/Node.h>
#include <SofaBaseVisual/VisualStyle.h>
#include <sofa/core/ComponentLibrary.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>
#include <SofaImGui/ObjectColor.h>
#include <sofa/core/visual/VisualParams.h>

using namespace sofa;

namespace sofaimgui
{

void ImGuiGUIEngine::init()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    NFD_Init();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    static const std::string iniFile(sofa::helper::Utils::getExecutableDirectory() + "/imgui.ini");
    io.IniFilename = iniFile.c_str();

    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    // Setup Dear ImGui style
    setDeepDarkStyle();
}

void ImGuiGUIEngine::initBackend(GLFWwindow* glfwWindow)
{
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(glfwWindow, true);

#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_Init();
#else
    ImGui_ImplOpenGL3_Init(nullptr);
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    GLFWmonitor* monitor = glfwGetWindowMonitor(glfwWindow);
    if (!monitor)
    {
        monitor = glfwGetPrimaryMonitor();
    }
    if (monitor)
    {
        float xscale, yscale;
        glfwGetMonitorContentScale(monitor, &xscale, &yscale);

        ImGuiIO& io = ImGui::GetIO();

        io.Fonts->AddFontFromMemoryCompressedTTF(ROBOTO_MEDIUM_compressed_data, ROBOTO_MEDIUM_compressed_size, 16 * yscale);

        ImFontConfig config;
        config.MergeMode = true;
        config.GlyphMinAdvanceX = 16.0f; // Use if you want to make the icon monospaced
        static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
        io.Fonts->AddFontFromMemoryCompressedTTF(FA_REGULAR_400_compressed_data, FA_REGULAR_400_compressed_size, 16 * yscale, &config, icon_ranges);
        io.Fonts->AddFontFromMemoryCompressedTTF(FA_SOLID_900_compressed_data, FA_SOLID_900_compressed_size, 16 * yscale, &config, icon_ranges);
    }
}

void loadFile(sofaglfw::SofaGLFWBaseGUI* baseGUI, sofa::core::sptr<sofa::simulation::Node>& groot, const std::string filePathName)
{
    sofa::simulation::getSimulation()->unload(groot);

    groot = sofa::simulation::getSimulation()->load(filePathName.c_str());
    if( !groot )
        groot = sofa::simulation::getSimulation()->createNewGraph("");
    baseGUI->setSimulation(groot, filePathName);

    sofa::simulation::getSimulation()->init(groot.get());
    auto camera = baseGUI->findCamera(groot);
    if (camera)
    {
        camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
        baseGUI->changeCamera(camera);
    }
    baseGUI->initVisual();
}

void ImGuiGUIEngine::showViewport(const char* const& windowNameViewport, bool& isViewportWindowOpen)
{
    if (isViewportWindowOpen)
    {
        if (ImGui::Begin(windowNameViewport, &isViewportWindowOpen))
        {
            ImGui::BeginChild("Render");
            ImVec2 wsize = ImGui::GetWindowSize();
            m_viewportWindowSize = { wsize.x, wsize.y};

            ImGui::Image((ImTextureID)m_fbo->getColorTexture(), wsize, ImVec2(0, 1), ImVec2(1, 0));

            isMouseOnViewport = ImGui::IsItemHovered();
            ImGui::EndChild();
        }
        ImGui::End();
    }
}

void ImGuiGUIEngine::showPerformances(const char* const& windowNamePerformances, const ImGuiIO& io, bool& isPerformancesWindowOpen)
{
    if (isPerformancesWindowOpen)
    {
        static sofa::type::vector<float> msArray;
        if (ImGui::Begin(windowNamePerformances, &isPerformancesWindowOpen))
        {
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Text("%d vertices, %d indices (%d triangles)", io.MetricsRenderVertices, io.MetricsRenderIndices, io.MetricsRenderIndices / 3);
            ImGui::Text("%d visible windows, %d active allocations", io.MetricsRenderWindows, io.MetricsActiveAllocations);

            msArray.push_back(1000.0f / io.Framerate);
            if (msArray.size() >= 2000)
            {
                msArray.erase(msArray.begin());
            }
            ImGui::PlotLines("Frame Times", msArray.data(), msArray.size(), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 100));
        }
        ImGui::End();
    }
}

void ImGuiGUIEngine::showProfiler(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameProfiler, bool& isProfilerOpen)
{
    if (isProfilerOpen)
    {
        static int selectedFrame = 0;

        if (ImGui::Begin(windowNameProfiler, &isProfilerOpen))
        {
            const auto convertInMs = [](helper::system::thread::ctime_t t)
            {
                static auto timer_freqd = static_cast<SReal>(helper::system::thread::CTime::getTicksPerSec());
                return 1000.0 * static_cast<SReal>(t) / static_cast<SReal>(timer_freqd);
            };

            static std::deque< type::vector<helper::Record> > allRecords;
            static int bufferSize = 500;
            ImGui::SliderInt("Buffer size", &bufferSize, 10, 5000);
            selectedFrame = std::min(selectedFrame, bufferSize - 1);

            static bool showChart = true;
            ImGui::Checkbox("Show Chart", &showChart);

            if (groot->animate_.getValue())
            {
                type::vector<helper::Record> _records = sofa::helper::AdvancedTimer::getRecords("Animate");
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
                        helper::system::thread::ctime_t t0;
                        for (const auto& rec : records)
                        {
                            if (timerId == rec.id)
                            {
                                chart.label = rec.label;
                                if (rec.type == helper::Record::RBEGIN || rec.type == helper::Record::RSTEP_BEGIN || rec.type == helper::Record::RSTEP)
                                {
                                    t0 = rec.time;
                                }
                                if (rec.type == helper::Record::REND || rec.type == helper::Record::RSTEP_END)
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

                        if (rec.type == helper::Record::RBEGIN || rec.type == helper::Record::RSTEP_BEGIN || rec.type == helper::Record::RSTEP)
                        {
                            durationStack.push(rec.time);
                            timerIdStack.push(timerIdCounter++);
                        }
                        if (rec.type == helper::Record::REND || rec.type == helper::Record::RSTEP_END)
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
                            if (rec.type == helper::Record::RBEGIN || rec.type == helper::Record::RSTEP_BEGIN || rec.type == helper::Record::RSTEP)
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
                                        if (it->label == nextRec.label && (nextRec.type == helper::Record::REND || nextRec.type == helper::Record::RSTEP_END))
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
                            if (rec.type == helper::Record::REND || rec.type == helper::Record::RSTEP_END)
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

void ImGuiGUIEngine::showSceneGraph(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameSceneGraph, bool& isSceneGraphWindowOpen, std::set<core::objectmodel::BaseObject*>& openedComponents, std::set<core::objectmodel::BaseObject*>& focusedComponents)
{
    if (isSceneGraphWindowOpen)
    {
        if (ImGui::Begin(windowNameSceneGraph, &isSceneGraphWindowOpen))
        {
            const bool expand = ImGui::Button(ICON_FA_EXPAND);
            ImGui::SameLine();
            const bool collapse = ImGui::Button(ICON_FA_COMPRESS);
            ImGui::SameLine();
            static bool showSearch = false;
            if (ImGui::Button(ICON_FA_SEARCH))
            {
                showSearch = !showSearch;
            }

            static ImGuiTextFilter filter;
            if (showSearch)
            {
                filter.Draw("Search");
            }

            unsigned int treeDepth {};
            static core::objectmodel::Base* clickedObject { nullptr };

            std::function<void(simulation::Node*)> showNode;
            showNode = [&showNode, &treeDepth, expand, collapse, &openedComponents](simulation::Node* node)
            {
                if (node == nullptr) return;
                if (treeDepth == 0)
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (expand)
                    ImGui::SetNextItemOpen(true);
                if (collapse)
                    ImGui::SetNextItemOpen(false);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();

                const auto& nodeName = node->getName();
                const bool isNodeHighlighted = !filter.Filters.empty() && filter.PassFilter(nodeName.c_str());
                if (isNodeHighlighted)
                {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1,1,0,1));
                }

                const bool open = ImGui::TreeNode(std::string(ICON_FA_CUBES "  " + nodeName).c_str());
                ImGui::TableNextColumn();
                ImGui::TextDisabled("Node");
                if (isNodeHighlighted)
                {
                    ImGui::PopStyleColor();
                }
                if (ImGui::IsItemClicked())
                    clickedObject = node;
                if (open)
                {
                    for (const auto object : node->getNodeObjects())
                    {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();

                        ImGuiTreeNodeFlags objectFlags = ImGuiTreeNodeFlags_SpanFullWidth;

                        const auto& slaves = object->getSlaves();
                        if (slaves.empty())
                        {
                            objectFlags |= ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_Leaf;
                        }
                        else
                        {
                            if (expand)
                                ImGui::SetNextItemOpen(true);
                            if (collapse)
                                ImGui::SetNextItemOpen(false);
                        }

                        const auto& objectName = object->getName();
                        const auto objectClassName = object->getClassName();
                        const bool isObjectHighlighted = !filter.Filters.empty() && (filter.PassFilter(objectName.c_str()) || filter.PassFilter(objectClassName.c_str()));

                        ImVec4 objectColor;

                        auto icon = ICON_FA_CUBE;
                        if (object->countLoggedMessages({helper::logging::Message::Error, helper::logging::Message::Fatal})!=0)
                        {
                            icon = ICON_FA_BUG;
                            objectColor = ImVec4(1.f, 0.f, 0.f, 1.f); //red
                        }
                        else if (object->countLoggedMessages({helper::logging::Message::Warning})!=0)
                        {
                            icon = ICON_FA_EXCLAMATION_TRIANGLE;
                            objectColor = ImVec4(1.f, 0.5f, 0.f, 1.f); //orange
                        }
                        else if (object->countLoggedMessages({helper::logging::Message::Info, helper::logging::Message::Deprecated, helper::logging::Message::Advice})!=0)
                        {
                            objectColor = getObjectColor(object);
                            icon = ICON_FA_COMMENT;
                        }
                        else
                        {
                            objectColor = getObjectColor(object);
                        }

                        ImGui::PushStyleColor(ImGuiCol_Text, objectColor);
                        const auto objectOpen = ImGui::TreeNodeEx(icon, objectFlags);
                        ImGui::PopStyleColor();

                        if (ImGui::IsItemClicked())
                        {
                            if (ImGui::IsMouseDoubleClicked(0))
                            {
                                openedComponents.insert(object);
                                clickedObject = nullptr;
                            }
                            else
                            {
                                clickedObject = object;
                            }
                        }

                        ImGui::SameLine();

                        if (isObjectHighlighted)
                        {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1,1,0,1));
                        }
                        ImGui::Text(object->getName().c_str());

                        ImGui::TableNextColumn();
                        ImGui::TextDisabled(objectClassName.c_str());

                        if (isObjectHighlighted)
                        {
                            ImGui::PopStyleColor();
                        }

                        if (objectOpen && !slaves.empty())
                        {
                            for (const auto slave : slaves)
                            {
                                ImGui::TableNextRow();
                                ImGui::TableNextColumn();

                                const auto& slaveName = slave->getName();
                                const auto slaveClassName = slave->getClassName();
                                const bool isSlaveHighlighted = !filter.Filters.empty() && (filter.PassFilter(slaveName.c_str()) || filter.PassFilter(slaveClassName.c_str()));
                                if (isSlaveHighlighted)
                                {
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1,1,0,1));
                                }

                                ImGui::TreeNodeEx(std::string(ICON_FA_CUBE "  " + slave->getName()).c_str(), ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanFullWidth);
                                if (ImGui::IsItemClicked())
                                {
                                    if (ImGui::IsMouseDoubleClicked(0))
                                    {
                                        openedComponents.insert(slave.get());
                                        clickedObject = nullptr;
                                    }
                                    else
                                    {
                                        clickedObject = slave.get();
                                    }
                                }
                                ImGui::TableNextColumn();
                                ImGui::TextDisabled(slave->getClassName().c_str());

                                if (isSlaveHighlighted)
                                {
                                    ImGui::PopStyleColor();
                                }
                            }
                            ImGui::TreePop();
                        }
                    }
                    ++treeDepth;
                    for (const auto child : node->getChildren())
                    {
                        showNode(dynamic_cast<simulation::Node*>(child));
                    }

                    --treeDepth;
                    ImGui::TreePop();
                }
            };

            static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody;

            ImVec2 outer_size = ImVec2(0.0f, static_cast<bool>(clickedObject) * ImGui::GetTextLineHeightWithSpacing() * 20);
            if (ImGui::BeginTable("sceneGraphTable", 2, flags, outer_size))
            {
                ImGui::TableSetupScrollFreeze(0, 1); // Make top row always visible
                ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_NoHide);
                ImGui::TableSetupColumn("Class Name", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("A").x * 12.0f);
                ImGui::TableHeadersRow();

                showNode(groot.get());

                ImGui::EndTable();
            }

            static bool areDataDisplayed;
            areDataDisplayed = clickedObject != nullptr;
            if (clickedObject != nullptr)
            {
                ImGui::Separator();
                ImGui::SetNextItemOpen(true, ImGuiCond_Appearing);
                if (ImGui::CollapsingHeader((ICON_FA_CUBE "  " + clickedObject->getName()).c_str(), &areDataDisplayed))
                {
                    ImGui::Indent();
                    std::map<std::string, std::vector<core::BaseData*> > groupMap;
                    for (auto* data : clickedObject->getDataFields())
                    {
                        groupMap[data->getGroup()].push_back(data);
                    }
                    for (auto& [group, datas] : groupMap)
                    {
                        const auto groupName = group.empty() ? "Property" : group;
                        ImGui::SetNextItemOpen(true, ImGuiCond_Appearing);
                        if (ImGui::CollapsingHeader(groupName.c_str()))
                        {
                            ImGui::Indent();
                            for (auto& data : datas)
                            {
                                const bool isOpen = ImGui::CollapsingHeader(data->m_name.c_str());
                                if (ImGui::IsItemHovered())
                                {
                                    ImGui::BeginTooltip();
                                    ImGui::TextDisabled(data->getHelp().c_str());
                                    ImGui::TextDisabled("Type: %s", data->getValueTypeString().c_str());
                                    ImGui::EndTooltip();
                                }
                                if (isOpen)
                                {
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                                    ImGui::TextWrapped(data->getHelp().c_str());

                                    if (data->getParent())
                                    {
                                        const auto linkPath = data->getLinkPath();
                                        if (!linkPath.empty())
                                        {
                                            ImGui::TextWrapped(linkPath.c_str());
                                            if (ImGui::IsItemHovered())
                                            {
                                                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                            }
                                            if (ImGui::IsItemClicked())
                                            {
                                                auto* owner = dynamic_cast<core::objectmodel::BaseObject*>(data->getParent()->getOwner());
                                                focusedComponents.insert(owner);
                                            }
                                        }
                                    }

                                    ImGui::PopStyleColor();
                                    showWidget(*data);
                                }
                            }
                            ImGui::Unindent();
                        }
                    }
                    ImGui::SetNextItemOpen(true, ImGuiCond_Appearing);
                    if (ImGui::CollapsingHeader("Links"))
                    {
                        ImGui::Indent();
                        for (const auto* link : clickedObject->getLinks())
                        {
                            const auto linkValue = link->getValueString();
                            const auto linkTitle = link->getName();

                            const bool isOpen = ImGui::CollapsingHeader(linkTitle.c_str());
                            if (ImGui::IsItemHovered())
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextDisabled(link->getHelp().c_str());
                                ImGui::EndTooltip();
                            }
                            if (isOpen)
                            {
                                ImGui::TextDisabled(link->getHelp().c_str());
                                ImGui::TextWrapped(linkValue.c_str());
                            }
                        }
                        ImGui::Unindent();
                    }
                    ImGui::Unindent();
                }
                if (!areDataDisplayed)
                {
                    clickedObject = nullptr;
                }
            }
        }
        ImGui::End();
    }

    openedComponents.insert(focusedComponents.begin(), focusedComponents.end());
    focusedComponents.clear();

    sofa::type::vector<core::objectmodel::BaseObject*> toRemove;
    for (auto* component : openedComponents)
    {
        bool isOpen = true;
        ImGui::PushStyleColor(ImGuiCol_Text, getObjectColor(component));
        if (ImGui::Begin((ICON_FA_CUBE "  " + component->getName() + " (" + component->getPathName() + ")").c_str(), &isOpen))
        {
            ImGui::PopStyleColor();
            std::map<std::string, std::vector<core::BaseData*> > groupMap;
            for (auto* data : component->getDataFields())
            {
                groupMap[data->getGroup()].push_back(data);
            }
            if (ImGui::BeginTabBar(("##tabs"+component->getName()).c_str(), ImGuiTabBarFlags_None))
            {
                for (auto& [group, datas] : groupMap)
                {
                    const auto groupName = group.empty() ? "Property" : group;
                    // ImGui::SetNextItemOpen(true, ImGuiCond_Appearing);
                    if (ImGui::BeginTabItem(groupName.c_str()))
                    {
                        for (auto& data : datas)
                        {
                            const bool isOpenData = ImGui::CollapsingHeader(data->m_name.c_str());
                            if (ImGui::IsItemHovered())
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextDisabled(data->getHelp().c_str());
                                ImGui::TextDisabled("Type: %s", data->getValueTypeString().c_str());
                                ImGui::EndTooltip();
                            }
                            if (isOpenData)
                            {
                                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                                ImGui::TextWrapped(data->getHelp().c_str());

                                if (data->getParent())
                                {
                                    const auto linkPath = data->getLinkPath();
                                    if (!linkPath.empty())
                                    {
                                        ImGui::TextWrapped(linkPath.c_str());

                                        if (ImGui::IsItemHovered())
                                        {
                                            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                        }
                                        if (ImGui::IsItemClicked())
                                        {
                                            auto* owner = dynamic_cast<core::objectmodel::BaseObject*>(data->getParent()->getOwner());
                                            focusedComponents.insert(owner);
                                        }
                                    }
                                }

                                ImGui::PopStyleColor();
                                showWidget(*data);
                            }
                        }
                        ImGui::EndTabItem();
                    }
                }
                // ImGui::SetNextItemOpen(true, ImGuiCond_Appearing);
                if (ImGui::BeginTabItem("Links"))
                {
                    for (const auto* link : component->getLinks())
                    {
                        const auto linkValue = link->getValueString();
                        const auto linkTitle = link->getName();

                        const bool isOpenData = ImGui::CollapsingHeader(linkTitle.c_str());
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::BeginTooltip();
                            ImGui::TextDisabled(link->getHelp().c_str());
                            ImGui::EndTooltip();
                        }
                        if (isOpenData)
                        {
                            ImGui::TextDisabled(link->getHelp().c_str());
                            ImGui::TextWrapped(linkValue.c_str());
                        }
                    }
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Infos"))
                {
                    ImGui::Text("Name: %s", component->getClassName().c_str());
                    ImGui::Spacing();
                    ImGui::TextDisabled("Template:");
                    ImGui::TextWrapped(component->getClass()->templateName.c_str());
                    ImGui::Spacing();
                    ImGui::TextDisabled("Namespace:");
                    ImGui::TextWrapped(component->getClass()->namespaceName.c_str());

                    core::ObjectFactory::ClassEntry entry = core::ObjectFactory::getInstance()->getEntry(component->getClassName());
                    if (! entry.creatorMap.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Description:");
                        ImGui::TextWrapped(entry.description.c_str());
                    }

                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Messages"))
                {
                    const auto& messages = component->getLoggedMessages();
                    if (ImGui::BeginTable(std::string("logTableComponent"+component->getName()).c_str(), 2, ImGuiTableFlags_RowBg))
                    {
                        ImGui::TableSetupColumn("message type", ImGuiTableColumnFlags_WidthFixed);
                        ImGui::TableSetupColumn("message", ImGuiTableColumnFlags_WidthStretch);
                        for (const auto& message : messages)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableNextColumn();

                            constexpr auto writeMessageType = [](const helper::logging::Message::Type t)
                            {
                                switch (t)
                                {
                                case helper::logging::Message::Advice     : return ImGui::TextColored(ImVec4(0.f, 0.5686f, 0.9176f, 1.f), "[SUGGESTION]");
                                case helper::logging::Message::Deprecated : return ImGui::TextColored(ImVec4(0.5529f, 0.4314f, 0.3882f, 1.f), "[DEPRECATED]");
                                case helper::logging::Message::Warning    : return ImGui::TextColored(ImVec4(1.f, 0.4275f, 0.f, 1.f), "[WARNING]");
                                case helper::logging::Message::Info       : return ImGui::Text("[INFO]");
                                case helper::logging::Message::Error      : return ImGui::TextColored(ImVec4(0.8667f, 0.1725f, 0.f, 1.f), "[ERROR]");
                                case helper::logging::Message::Fatal      : return ImGui::TextColored(ImVec4(0.8353, 0.f, 0.f, 1.f), "[FATAL]");
                                case helper::logging::Message::TEmpty     : return ImGui::Text("[EMPTY]");
                                default: return;
                                }
                            };
                            writeMessageType(message.type());

                            ImGui::TableNextColumn();
                            ImGui::TextWrapped(message.message().str().c_str());
                        }
                        ImGui::EndTable();
                    }

                }
                ImGui::EndTabItem();

                ImGui::EndTabBar();
            }
        }
        else
        {
            ImGui::PopStyleColor();
        }
        ImGui::End();
        if (!isOpen)
        {
            toRemove.push_back(component);
        }
    }
    while(!toRemove.empty())
    {
        auto it = openedComponents.find(toRemove.back());
        if (it != openedComponents.end())
        {
            openedComponents.erase(it);
        }
        toRemove.pop_back();
    }
}

void ImGuiGUIEngine::showDisplayFlags(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameDisplayFlags, bool& isDisplayFlagsWindowOpen)
{
    if (isDisplayFlagsWindowOpen)
    {
        if (ImGui::Begin(windowNameDisplayFlags, &isDisplayFlagsWindowOpen))
        {
            component::visualmodel::VisualStyle::SPtr visualStyle = nullptr;
            groot->get(visualStyle);
            if (visualStyle)
            {
                auto& displayFlags = sofa::helper::getWriteAccessor(visualStyle->displayFlags).wref();

                {
                    const bool initialValue = displayFlags.getShowVisualModels();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Visual Models", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowVisualModels(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowBehaviorModels();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Behavior Models", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowBehaviorModels(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowForceFields();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Force Fields", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowForceFields(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowCollisionModels();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Collision Models", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowCollisionModels(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowBoundingCollisionModels();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Bounding Collision Models", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowBoundingCollisionModels(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowMappings();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Mappings", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowMappings(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowMechanicalMappings();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Mechanical Mappings", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowMechanicalMappings(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowWireFrame();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Wire Frame", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowWireFrame(changeableValue);
                    }
                }

                {
                    const bool initialValue = displayFlags.getShowNormals();
                    bool changeableValue = initialValue;
                    ImGui::Checkbox("Show Normals", &changeableValue);
                    if (changeableValue != initialValue)
                    {
                        displayFlags.setShowNormals(changeableValue);
                    }
                }

            }
        }
        ImGui::End();
    }
}

void ImGuiGUIEngine::showPlugins(const char* const& windowNamePlugins, bool& isPluginsWindowOpen)
{
    if (isPluginsWindowOpen)
    {
        if (ImGui::Begin(windowNamePlugins, &isPluginsWindowOpen))
        {
            ImGui::Columns(2);

            const auto& pluginMap = helper::system::PluginManager::getInstance().getPluginMap();

            static std::map<std::string, bool> isSelected;
            static std::string selectedPlugin;
            for (const auto& [path, plugin] : pluginMap)
            {
                if (ImGui::Selectable(plugin.getModuleName(), selectedPlugin == path))
                {
                    selectedPlugin = path;
                }
            }

            ImGui::NextColumn();

            const auto pluginIt = pluginMap.find(selectedPlugin);
            if (pluginIt != pluginMap.end())
            {
                ImGui::Text("Plugin: %s", pluginIt->second.getModuleName());
                ImGui::Text("Version: %s", pluginIt->second.getModuleVersion());
                ImGui::Text("License: %s", pluginIt->second.getModuleLicense());
                ImGui::Spacing();
                ImGui::TextDisabled("Description:");
                ImGui::TextWrapped("%s", pluginIt->second.getModuleDescription());
                ImGui::Spacing();
                ImGui::TextDisabled("Components:");
                ImGui::TextWrapped("%s", pluginIt->second.getModuleComponentList());
                ImGui::Spacing();
                ImGui::TextDisabled("Path:");
                ImGui::TextWrapped(selectedPlugin.c_str());
            }
        }
        ImGui::End();
    }
}

void ImGuiGUIEngine::showComponents(const char* const& windowNameComponents, bool& isComponentsWindowOpen)
{
    if (isComponentsWindowOpen)
    {
        if (ImGui::Begin(windowNameComponents, &isComponentsWindowOpen))
        {
            unsigned int nbLoadedComponents = 0;
            if (ImGui::BeginTable("split", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_Resizable))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();

                static core::ClassEntry::SPtr selectedEntry;

                static std::vector<core::ClassEntry::SPtr> entries;
                entries.clear();
                core::ObjectFactory::getInstance()->getAllEntries(entries);
                nbLoadedComponents = entries.size();

                static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody | ImGuiTableFlags_Sortable;
                if (ImGui::BeginTable("componentTable", 2, flags, ImVec2(0.f, 400.f)))
                {
                    ImGui::TableSetupColumn("Name");
                    ImGui::TableSetupColumn("Category");
                    ImGui::TableHeadersRow();

                    struct ComponentEntry
                    {
                        std::string name;
                        std::string category;
                        core::ClassEntry::SPtr classEntry;
                    };
                    static std::vector<ComponentEntry> componentEntries;
                    if (componentEntries.empty())
                    {
                        for (const auto& entry : entries)
                        {
                            std::set<std::string> categoriesSet;
                            std::stringstream templateSs;
                            for (const auto& [templateInstance, creator] : entry->creatorMap)
                            {
                                std::vector<std::string> categories;
                                core::CategoryLibrary::getCategories(entry->creatorMap.begin()->second->getClass(), categories);
                                categoriesSet.insert(categories.begin(), categories.end());
                            }
                            std::stringstream categorySs;
                            for (const auto& c : categoriesSet)
                                categorySs << c << ", ";

                            const std::string categoriesText = categorySs.str().substr(0, categorySs.str().size()-2);

                            componentEntries.push_back({entry->className, categoriesText, entry});
                        }
                    }

                    if (ImGuiTableSortSpecs* sorts_specs = ImGui::TableGetSortSpecs())
                    {
                        if (sorts_specs->SpecsDirty)
                        {
                            std::sort(componentEntries.begin(), componentEntries.end(), [sorts_specs](const ComponentEntry& lhs, const ComponentEntry& rhs)
                            {
                                for (int n = 0; n < sorts_specs->SpecsCount; n++)
                                {
                                    const ImGuiTableColumnSortSpecs* sort_spec = &sorts_specs->Specs[n];
                                    const bool ascending = sort_spec->SortDirection == ImGuiSortDirection_Ascending;
                                    switch (sort_spec->ColumnIndex)
                                    {
                                    case 0:
                                        {
                                            if (lhs.name < rhs.name) return ascending;
                                            if (lhs.name > rhs.name) return !ascending;
                                            break;
                                        }
                                    case 1:
                                        {
                                            if (lhs.category < rhs.category) return ascending;
                                            if (lhs.category > rhs.category) return !ascending;
                                            break;
                                            return lhs.name < rhs.name;
                                        }
                                    default:
                                        IM_ASSERT(0); break;
                                    }
                                }
                                return false;
                            });
                        }
                    }

                    static const std::map<std::string, ImVec4> colorMap = []()
                    {
                        std::map<std::string, ImVec4> m;
                        int i {};
                        auto categories = core::CategoryLibrary::getCategories();
                        std::sort(categories.begin(), categories.end(), std::less<std::string>());
                        for (const auto& cat : categories)
                        {
                            ImVec4 color;
                            color.w = 1.f;
                            ImGui::ColorConvertHSVtoRGB(i++ / (static_cast<float>(categories.size())-1.f), 0.72f, 1.f, color.x, color.y, color.z);
                            m[cat] = color;
                        }
                        return m;
                    }();

                    for (const auto& entry : componentEntries)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        if (ImGui::Selectable(entry.name.c_str(), entry.classEntry == selectedEntry))
                            selectedEntry = entry.classEntry;
                        ImGui::TableNextColumn();

                        const auto colorIt = colorMap.find(entry.category);
                        if (colorIt != colorMap.end())
                            ImGui::TextColored(colorIt->second, colorIt->first.c_str());
                        else
                            ImGui::Text(entry.category.c_str());
                    }
                    ImGui::EndTable();
                }

                ImGui::TableNextColumn();

                if (selectedEntry)
                {
                    ImGui::Text("Name: %s", selectedEntry->className.c_str());
                    ImGui::Spacing();
                    ImGui::TextDisabled("Description:");
                    ImGui::TextWrapped(selectedEntry->description.c_str());
                    ImGui::Spacing();

                    bool hasTemplate = false;
                    for (const auto& [templateInstance, creator] : selectedEntry->creatorMap)
                    {
                        if (hasTemplate |= !templateInstance.empty())
                            break;
                    }

                    if (hasTemplate)
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Templates:");
                        for (const auto& [templateInstance, creator] : selectedEntry->creatorMap)
                        {
                            ImGui::BulletText(templateInstance.c_str());
                        }
                    }

                    if (!selectedEntry->aliases.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Aliases:");
                        for (const auto& alias : selectedEntry->aliases)
                        {
                            ImGui::BulletText(alias.c_str());
                        }
                    }

                    std::set<std::string> namespaces;
                    for (const auto& [templateInstance, creator] : selectedEntry->creatorMap)
                    {
                        namespaces.insert(creator->getClass()->namespaceName);
                    }
                    if (!namespaces.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Namespaces:");
                        for (const auto& nm : namespaces)
                        {
                            ImGui::BulletText(nm.c_str());
                        }
                    }

                    std::set<std::string> parents;
                    for (const auto& [templateInstance, creator] : selectedEntry->creatorMap)
                    {
                        for (const auto& p : creator->getClass()->parents)
                        {
                            parents.insert(p->className);
                        }
                    }
                    if (!parents.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Parents:");
                        for (const auto& p : parents)
                        {
                            ImGui::BulletText(p.c_str());
                        }
                    }

                    std::set<std::string> targets;
                    for (const auto& [templateInstance, creator] : selectedEntry->creatorMap)
                    {
                        targets.insert(creator->getTarget());
                    }
                    if (!targets.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Targets:");
                        for (const auto& t : targets)
                        {
                            ImGui::BulletText(t.c_str());
                        }
                    }
                }
                else
                {
                    ImGui::Text("Select a component");
                }

                ImGui::EndTable();
            }
            ImGui::Text("%d loaded components", nbLoadedComponents);
        }
        ImGui::End();
    }
}

void ImGuiGUIEngine::showLog(const char* const& windowNameLog, bool& isLogWindowOpen)
{
    if (isLogWindowOpen)
    {
        if (ImGui::Begin(windowNameLog, &isLogWindowOpen))
        {
            unsigned int i {};
            const auto& messages = sofa::helper::logging::MainLoggingMessageHandler::getInstance().getMessages();
            const int digits = [&messages]()
            {
                int d = 0;
                auto s = messages.size();
                while (s != 0) { s /= 10; d++; }
                return d;
            }();

            if (ImGui::BeginTable("logTable", 4, ImGuiTableFlags_RowBg))
            {
                ImGui::TableSetupColumn("logId", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("message type", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("sender", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("message", ImGuiTableColumnFlags_WidthStretch);
                for (const auto& message : messages)
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();

                    std::stringstream ss;
                    ss << std::setfill('0') << std::setw(digits) << i++;
                    ImGui::Text(ss.str().c_str());

                    ImGui::TableNextColumn();

                    constexpr auto writeMessageType = [](const helper::logging::Message::Type t)
                    {
                        switch (t)
                        {
                        case helper::logging::Message::Advice     : return ImGui::TextColored(ImVec4(0.f, 0.5686f, 0.9176f, 1.f), "[SUGGESTION]");
                        case helper::logging::Message::Deprecated : return ImGui::TextColored(ImVec4(0.5529f, 0.4314f, 0.3882f, 1.f), "[DEPRECATED]");
                        case helper::logging::Message::Warning    : return ImGui::TextColored(ImVec4(1.f, 0.4275f, 0.f, 1.f), "[WARNING]");
                        case helper::logging::Message::Info       : return ImGui::Text("[INFO]");
                        case helper::logging::Message::Error      : return ImGui::TextColored(ImVec4(0.8667f, 0.1725f, 0.f, 1.f), "[ERROR]");
                        case helper::logging::Message::Fatal      : return ImGui::TextColored(ImVec4(0.8353, 0.f, 0.f, 1.f), "[FATAL]");
                        case helper::logging::Message::TEmpty     : return ImGui::Text("[EMPTY]");
                        default: return;
                        }
                    };
                    writeMessageType(message.type());

                    ImGui::TableNextColumn();
                    ImGui::Text(message.sender().c_str());

                    ImGui::TableNextColumn();
                    ImGui::TextWrapped(message.message().str().c_str());
                }
                ImGui::EndTable();
            }
        }
        ImGui::End();
    }
}

void ImGuiGUIEngine::showSettings(const char* const& windowNameSettings, bool& isSettingsOpen)
{
    if (isSettingsOpen)
    {
        if (ImGui::Begin(windowNameSettings, &isSettingsOpen))
        {
            ImGuiIO& io = ImGui::GetIO();
            const float MIN_SCALE = 0.3f;
            const float MAX_SCALE = 2.0f;
            ImGui::DragFloat("global scale", &io.FontGlobalScale, 0.005f, MIN_SCALE, MAX_SCALE, "%.2f", ImGuiSliderFlags_AlwaysClamp); // Scale everything
        }
        ImGui::End();
    }
}

void ImGuiGUIEngine::startFrame(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    auto groot = baseGUI->getRootNode();

    // Start the Dear ImGui frame
#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_NewFrame();
#else
    ImGui_ImplOpenGL3_NewFrame();
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;

    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    window_flags |= ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpace", nullptr, window_flags);
    ImGui::PopStyleVar();
    ImGui::PopStyleVar(2);

    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode);

    static constexpr auto windowNameViewport = ICON_FA_DICE_D6 "  Viewport";
    static constexpr auto windowNamePerformances = ICON_FA_CHART_LINE "  Performances";
    static constexpr auto windowNameProfiler = ICON_FA_HOURGLASS "  Profiler";
    static constexpr auto windowNameSceneGraph = ICON_FA_SITEMAP "  Scene Graph";
    static constexpr auto windowNameDisplayFlags = ICON_FA_EYE "  Display Flags";
    static constexpr auto windowNamePlugins = ICON_FA_PLUS_CIRCLE "  Plugins";
    static constexpr auto windowNameComponents = ICON_FA_LIST "  Components";
    static constexpr auto windowNameLog = ICON_FA_TERMINAL "  Log";
    static constexpr auto windowNameSettings = ICON_FA_SLIDERS_H "  Settings";

    static auto first_time = true;
    if (first_time)
    {
        first_time = false;

        ImGui::DockBuilderRemoveNode(dockspace_id); // clear any previous layout
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode | ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

        auto dock_id_right = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, 0.4f, nullptr, &dockspace_id);
        ImGui::DockBuilderDockWindow(windowNameSceneGraph, dock_id_right);
        auto dock_id_down = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.3f, nullptr, &dockspace_id);
        ImGui::DockBuilderDockWindow(windowNameLog, dock_id_down);
        ImGui::DockBuilderDockWindow(windowNameViewport, dockspace_id);
        ImGui::DockBuilderFinish(dockspace_id);
    }
    ImGui::End();


    const ImGuiIO& io = ImGui::GetIO();

    static bool isViewportWindowOpen = true;
    static bool isPerformancesWindowOpen = false;
    static bool isSceneGraphWindowOpen = true;
    static bool isDisplayFlagsWindowOpen = false;
    static bool isPluginsWindowOpen = false;
    static bool isComponentsWindowOpen = false;
    static bool isLogWindowOpen = true;
    static bool isProfilerOpen = false;
    static bool isSettingsOpen = false;

    static bool showFPSInMenuBar = true;
    static bool showTime = true;

    ImVec2 mainMenuBarSize;

    static bool animate;
    animate = groot->animate_.getValue();

    /***************************************
     * Main menu bar
     **************************************/
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "  Open Simulation"))
            {
                simulation::SceneLoaderFactory::SceneLoaderList* loaders =simulation::SceneLoaderFactory::getInstance()->getEntries();
                std::vector<std::pair<std::string, std::string> > filterList;
                filterList.reserve(loaders->size());
                std::pair<std::string, std::string> allFilters {"SOFA files", {} };
                for (auto it=loaders->begin(); it!=loaders->end(); ++it)
                {
                    const auto filterName = (*it)->getFileTypeDesc();

                    sofa::simulation::SceneLoader::ExtensionList extensions;
                    (*it)->getExtensionList(&extensions);
                    std::string extensionsString;
                    for (auto itExt=extensions.begin(); itExt!=extensions.end(); ++itExt)
                    {
                        extensionsString += *itExt;
                        std::cout << *itExt << std::endl;
                        if (itExt != extensions.end() - 1)
                        {
                            extensionsString += ",";
                        }
                    }

                    filterList.emplace_back(filterName, extensionsString);

                    allFilters.second += extensionsString;
                    if (it != loaders->end()-1)
                    {
                        allFilters.second += ",";
                    }
                }
                std::vector<nfdfilteritem_t> nfd_filters;
                nfd_filters.reserve(filterList.size() + 1);
                for (auto& f : filterList)
                {
                    nfd_filters.push_back({f.first.c_str(), f.second.c_str()});
                }
                nfd_filters.insert(nfd_filters.begin(), {allFilters.first.c_str(), allFilters.second.c_str()});

                nfdchar_t *outPath;
                nfdresult_t result = NFD_OpenDialog(&outPath, nfd_filters.data(), nfd_filters.size(), NULL);
                if (result == NFD_OKAY)
                {
                    if (helper::system::FileSystem::exists(outPath))
                    {
                        sofa::simulation::getSimulation()->unload(groot);

                        groot = sofa::simulation::getSimulation()->load(outPath);
                        if( !groot )
                            groot = sofa::simulation::getSimulation()->createNewGraph("");
                        baseGUI->setSimulation(groot, outPath);

                        sofa::simulation::getSimulation()->init(groot.get());
                        auto camera = baseGUI->findCamera(groot);
                        if (camera)
                        {
                            camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
                            baseGUI->changeCamera(camera);
                        }
                        baseGUI->initVisual();
                    }
                    NFD_FreePath(outPath);
                }
            }

            const auto filename = baseGUI->getFilename();
            if (ImGui::MenuItem(ICON_FA_REDO "  Reload File"))
            {
                if (!filename.empty() && helper::system::FileSystem::exists(filename))
                {
                    msg_info("GUI") << "Reloading file " << filename;
                    loadFile(baseGUI, groot, filename);
                }
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::TextDisabled(filename.c_str());
                ImGui::EndTooltip();
            }

            if (ImGui::MenuItem(ICON_FA_TIMES_CIRCLE "  Close Simulation"))
            {
                sofa::simulation::getSimulation()->unload(groot);
                baseGUI->setSimulationIsRunning(false);
                sofa::simulation::getSimulation()->init(baseGUI->getRootNode().get());
                return;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit"))
            {
                //TODO: brutal exit, need to clean up everything (simulation, window, opengl, imgui etc)
                exit(EXIT_SUCCESS);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            ImGui::Checkbox("Show FPS", &showFPSInMenuBar);
            bool isFullScreen = baseGUI->isFullScreen();
            if (ImGui::Checkbox(ICON_FA_EXPAND "  Fullscreen", &isFullScreen))
            {
                baseGUI->switchFullScreen();
            }
            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_CAMERA "  Center Camera"))
            {
                sofa::component::visualmodel::BaseCamera::SPtr camera;
                groot->get(camera);
                if (camera)
                {
                    if( groot->f_bbox.getValue().isValid())
                    {
                        camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
                    }
                    else
                    {
                        msg_error_when(!groot->f_bbox.getValue().isValid(), "GUI") << "Global bounding box is invalid: " << groot->f_bbox.getValue();
                    }
                }
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Windows"))
        {
            ImGui::Checkbox(windowNameViewport, &isViewportWindowOpen);
            ImGui::Checkbox(windowNamePerformances, &isPerformancesWindowOpen);
            ImGui::Checkbox(windowNameProfiler, &isProfilerOpen);
            ImGui::Checkbox(windowNameSceneGraph, &isSceneGraphWindowOpen);
            ImGui::Checkbox(windowNameDisplayFlags, &isDisplayFlagsWindowOpen);
            ImGui::Checkbox(windowNamePlugins, &isPluginsWindowOpen);
            ImGui::Checkbox(windowNameComponents, &isComponentsWindowOpen);
            ImGui::Checkbox(windowNameLog, &isLogWindowOpen);
            ImGui::Separator();
            ImGui::Checkbox(windowNameSettings, &isSettingsOpen);
            ImGui::EndMenu();
        }

        ImGui::SetCursorPosX(ImGui::GetColumnWidth() / 2); //approximatively the center of the menu bar
        if (ImGui::Button(animate ? ICON_FA_PAUSE : ICON_FA_PLAY))
        {
            sofa::helper::getWriteOnlyAccessor(groot->animate_).wref() = !animate;
        }
        ImGui::SameLine();
        if (animate)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        if (ImGui::Button(ICON_FA_STEP_FORWARD))
        {
            if (!animate)
            {
                sofa::helper::AdvancedTimer::begin("Animate");

                simulation::getSimulation()->animate(groot.get(), groot->getDt());
                simulation::getSimulation()->updateVisual(groot.get());

                sofa::helper::AdvancedTimer::end("Animate");
            }
        }
        if (animate)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        ImGui::SameLine();
        if (ImGui::Button(ICON_FA_REDO_ALT))
        {
            groot->setTime(0.);
            simulation::getSimulation()->reset ( groot.get() );
        }

        const auto posX = ImGui::GetCursorPosX();
        if (showFPSInMenuBar)
        {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("1000.0 FPS ").x
                 - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%.1f FPS", io.Framerate);
            ImGui::SetCursorPosX(posX);
        }
        if (showTime)
        {
            auto position = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("Time: 000.000  ").x
                - 2 * ImGui::GetStyle().ItemSpacing.x;
            if (showFPSInMenuBar)
                position -= ImGui::CalcTextSize("1000.0 FPS ").x;
            ImGui::SetCursorPosX(position);
            ImGui::TextDisabled("Time: %.3f", groot->getTime());
            ImGui::SetCursorPosX(posX);
        }
        mainMenuBarSize = ImGui::GetWindowSize();
        ImGui::EndMainMenuBar();
    }

    /***************************************
     * Viewport window
     **************************************/
    showViewport(windowNameViewport, isViewportWindowOpen);

    /***************************************
     * Performances window
     **************************************/
    showPerformances(windowNamePerformances, io, isPerformancesWindowOpen);


    /***************************************
     * Profiler window
     **************************************/
    sofa::helper::AdvancedTimer::setEnabled("Animate", isProfilerOpen);
    sofa::helper::AdvancedTimer::setInterval("Animate", 1);
    sofa::helper::AdvancedTimer::setOutputType("Animate", "gui");

    showProfiler(groot, windowNameProfiler, isProfilerOpen);

    /***************************************
     * Scene graph window
     **************************************/
    static std::set<core::objectmodel::BaseObject*> openedComponents;
    static std::set<core::objectmodel::BaseObject*> focusedComponents;
    showSceneGraph(groot, windowNameSceneGraph, isSceneGraphWindowOpen, openedComponents, focusedComponents);


    /***************************************
     * Display flags window
     **************************************/
    showDisplayFlags(groot, windowNameDisplayFlags, isDisplayFlagsWindowOpen);

    /***************************************
     * Plugins window
     **************************************/
    showPlugins(windowNamePlugins, isPluginsWindowOpen);

    /***************************************
     * Components window
     **************************************/
    showComponents(windowNameComponents, isComponentsWindowOpen);

    /***************************************
     * Log window
     **************************************/
    showLog(windowNameLog, isLogWindowOpen);

    /***************************************
     * Settings window
     **************************************/
    showSettings(windowNameSettings, isSettingsOpen);

    ImGui::Render();
#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
#else
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    // Update and Render additional Platform Windows
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
    }
}

void ImGuiGUIEngine::beforeDraw(GLFWwindow*)
{
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT);

    if (!m_fbo)
    {
        m_fbo = std::make_unique<sofa::gl::FrameBufferObject>();
        m_currentFBOSize = {500, 500};
        m_fbo->init(m_currentFBOSize.first, m_currentFBOSize.second);
    }
    else
    {
        if (m_currentFBOSize.first != static_cast<unsigned int>(m_viewportWindowSize.first)
            || m_currentFBOSize.second != static_cast<unsigned int>(m_viewportWindowSize.second))
        {
            m_fbo->setSize(static_cast<unsigned int>(m_viewportWindowSize.first), static_cast<unsigned int>(m_viewportWindowSize.second));
            m_currentFBOSize = {static_cast<unsigned int>(m_viewportWindowSize.first), static_cast<unsigned int>(m_viewportWindowSize.second)};
        }
    }
    sofa::core::visual::VisualParams::defaultInstance()->viewport() = {0,0,m_currentFBOSize.first, m_currentFBOSize.second};

    m_fbo->start();
}

void ImGuiGUIEngine::afterDraw()
{
    m_fbo->stop();
}

void ImGuiGUIEngine::terminate()
{
    NFD_Quit();

#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_Shutdown();
#else
    ImGui_ImplOpenGL3_Shutdown();
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

bool ImGuiGUIEngine::dispatchMouseEvents()
{
    return !ImGui::GetIO().WantCaptureMouse || isMouseOnViewport;
}

} //namespace sofaimgui
