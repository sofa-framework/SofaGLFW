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

#include <ostream>
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
#include <filesystem>
#include <Roboto-Medium.h>
#include <Style.h>
#include <SofaImGui/ImGuiDataWidget.h>

#include <sofa/helper/Utils.h>
#include <sofa/type/vector.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/visual/VisualStyle.h>
#include <sofa/core/ComponentLibrary.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>
#include <SofaImGui/ObjectColor.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/io/File.h>
#include <sofa/component/visual/VisualGrid.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/gl/component/rendering3d/OglSceneFrame.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/helper/io/STBImage.h>

#include <sofa/component/playback/ReadState.h>

using namespace sofa;

namespace sofaimgui
{

constexpr const char* VIEW_FILE_EXTENSION = ".view";

void ImGuiGUIEngine::init()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    NFD_Init();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    static const std::string imguiIniFile(sofa::helper::Utils::getExecutableDirectory() + "/imgui.ini");
    io.IniFilename = imguiIniFile.c_str();

    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;


    ini.SetUnicode();
    if (sofa::helper::system::FileSystem::exists(getAppIniFile()))
    {
        SI_Error rc = ini.LoadFile(getAppIniFile().c_str());
        assert(rc == SI_OK);
    }

    const char* pv;
    pv = ini.GetValue("Style", "theme");
    if (!pv)
    {
        ini.SetValue("Style", "theme", sofaimgui::defaultStyle.c_str(), "# Preset of colors and properties to change the theme of the application");
        SI_Error rc = ini.SaveFile(getAppIniFile().c_str());
        pv = sofaimgui::defaultStyle.c_str();
    }

    // Setup Dear ImGui style
    sofaimgui::setStyle(pv);

    sofa::helper::system::PluginManager::getInstance().readFromIniFile(
        sofa::gui::common::BaseGUI::getConfigDirectoryPath() + "/loadedPlugins.ini");
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

void ImGuiGUIEngine::loadFile(sofaglfw::SofaGLFWBaseGUI* baseGUI, sofa::core::sptr<sofa::simulation::Node>& groot, const std::string filePathName)
{
    sofa::simulation::node::unload(groot);

    groot = sofa::simulation::node::load(filePathName.c_str());
    if( !groot )
        groot = sofa::simulation::getSimulation()->createNewGraph("");
    baseGUI->setSimulation(groot, filePathName);

    sofa::simulation::node::initRoot(groot.get());
    auto camera = baseGUI->findCamera(groot);
    if (camera)
    {
        camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
        baseGUI->changeCamera(camera);
    }
    baseGUI->initVisual();
}

void ImGuiGUIEngine::showViewport(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameViewport, bool& isViewportWindowOpen)
{
    if (isViewportWindowOpen)
    {
        ImVec2 pos;
        if (ImGui::Begin(windowNameViewport, &isViewportWindowOpen))
        {
            pos = ImGui::GetWindowPos();

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

                    const std::string instantiationSourceFilename = component->getInstanciationSourceFileName();
                    if (!instantiationSourceFilename.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Definition:");
                        ImGui::TextWrapped(component->getInstanciationSourceFileName().c_str());
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

                    ImGui::EndTabItem();
                }

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

void ImGuiGUIEngine::showROS(sofa::core::sptr<sofa::simulation::Node> groot, const char* const& windowNameROS, bool& isROSWindowOpen)
{
    if (isROSWindowOpen)
    {
        if (ImGui::Begin(windowNameROS, &isROSWindowOpen))
        {
            ImGui::BeginChild("Sub", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y), false, ImGuiWindowFlags_HorizontalScrollbar);

            ImGui::Text("Topic: ");

            ImGui::EndChild();

            ImGui::BeginChild("Pub", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y), false, ImGuiWindowFlags_HorizontalScrollbar);

            ImGui::Text("Topic: ");

            ImGui::EndChild();
        }
        ImGui::End();
    }
}


const std::string& ImGuiGUIEngine::getAppIniFile()
{
    static const std::string appIniFile(sofa::helper::Utils::getExecutableDirectory() + "/settings.ini");
    return appIniFile;
}

void ImGuiGUIEngine::startFrame(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    auto groot = baseGUI->getRootNode();

    bool alwaysShowFrame = ini.GetBoolValue("Visualization", "alwaysShowFrame", true);
    if (alwaysShowFrame)
    {
        auto sceneFrame = groot->get<sofa::gl::component::rendering3d::OglSceneFrame>();
        if (!sceneFrame)
        {
            auto newSceneFrame = sofa::core::objectmodel::New<sofa::gl::component::rendering3d::OglSceneFrame>();
            sofa::helper::OptionsGroup styleOptions{"Arrows", "Cylinders", "CubeCones"};
            styleOptions.setSelectedItem(2);
            newSceneFrame->d_style.setValue(styleOptions);

            sofa::helper::OptionsGroup alignmentOptions{"BottomLeft", "BottomRight", "TopRight", "TopLeft"};
            alignmentOptions.setSelectedItem(2);
            newSceneFrame->d_alignment.setValue(alignmentOptions);

            groot->addObject(newSceneFrame);
            newSceneFrame->setName("viewportFrame");
            newSceneFrame->addTag(core::objectmodel::Tag("createdByGUI"));
            newSceneFrame->d_drawFrame.setValue(true);
            newSceneFrame->init();
        }
    }

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

    static constexpr auto windowNameViewport = "Viewport";
    static constexpr auto windowNameSceneGraph = "Scene Graph";
    static constexpr auto windowNameROS = "ROS";

    static auto first_time = true;
    if (first_time)
    {
        first_time = false;

        ImGui::DockBuilderRemoveNode(dockspace_id); // clear any previous layout
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode | ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

        ImGui::DockBuilderDockWindow(windowNameViewport, dockspace_id);
        ImGui::DockBuilderGetNode(dockspace_id)->WantHiddenTabBarToggle = true;
        ImGui::DockBuilderFinish(dockspace_id);
    }
    ImGui::End();


    const ImGuiIO& io = ImGui::GetIO();

    static bool isViewportWindowOpen = true;
    static bool isSceneGraphWindowOpen = false;
    static bool isROSWindowOpen = false;

    static bool showFPSInMenuBar = true;
    static bool showTime = true;

    ImVec2 mainMenuBarSize;

    static bool animate;
    animate = groot->animate_.getValue();
    static bool record;
    static bool connect;

    /***************************************
     * Main menu bar
     **************************************/
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open Simulation"))
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
                        loadFile(baseGUI, groot, outPath);
                    }
                    NFD_FreePath(outPath);
                }
            }

            const auto filename = baseGUI->getFilename();
            if (ImGui::MenuItem("Reload Simulation"))
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

            if (ImGui::MenuItem("Close Simulation"))
            {
                sofa::simulation::node::unload(groot);
                baseGUI->setSimulationIsRunning(false);
                sofa::simulation::node::initRoot(baseGUI->getRootNode().get());
                return;
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Import Trajectory"))
            {
                nfdchar_t *outPath;
                std::vector<nfdfilteritem_t> nfd_filters;
                nfd_filters.push_back({"trajectory file", "txt"});
                nfdresult_t result = NFD_OpenDialog(&outPath, nfd_filters.data(), nfd_filters.size(), NULL);
                if (result == NFD_OKAY)
                {
                    if (helper::system::FileSystem::exists(outPath))
                    {
                        const auto& node = groot->getChild("Modelling")->getChild("Target");

                        auto oldrs = node->getObject("Trajectory");
                        if (oldrs)
                            node->removeObject(oldrs);

                        auto rs = sofa::core::objectmodel::New<component::playback::ReadState>();
                        rs->setName("Trajectory");
                        rs->d_filename.setValue(outPath);
                        rs->d_draw.setValue(true);
                        node->addObject(rs);
                        rs->init();
                    }
                    NFD_FreePath(outPath);
                }
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
            if (ImGui::Checkbox("Fullscreen", &isFullScreen))
            {
                baseGUI->switchFullScreen();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Center Camera"))
            {
                sofa::component::visual::BaseCamera::SPtr camera;
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

            const std::string viewFileName = baseGUI->getFilename() + VIEW_FILE_EXTENSION;
            if (ImGui::MenuItem("Save Camera"))
            {
                sofa::component::visual::BaseCamera::SPtr camera;
                groot->get(camera);
                if (camera)
                {
                    if (camera->exportParametersInFile(viewFileName))
                    {
                        msg_info("GUI") << "Current camera parameters have been exported to "<< viewFileName << " .";
                    }
                    else
                    {
                        msg_error("GUI") << "Could not export camera parameters to " << viewFileName << " .";
                    }
                }
            }
            bool fileExists = sofa::helper::system::FileSystem::exists(viewFileName);
            ImGui::BeginDisabled(!fileExists);
            if (ImGui::MenuItem("Restore Camera"))
            {
                sofa::component::visual::BaseCamera::SPtr camera;
                groot->get(camera);
                if (camera)
                {
                    if (camera->importParametersFromFile(viewFileName))
                    {
                        msg_info("GUI") << "Current camera parameters have been imported from " << viewFileName << " .";
                    }
                    else
                    {
                        msg_error("GUI") << "Could not import camera parameters from " << viewFileName << " .";
                    }
                }
            }

            ImGui::EndDisabled();

            ImGui::Separator();
            if (ImGui::MenuItem("Save Screenshot"))
            {
                nfdchar_t *outPath;
                std::array<nfdfilteritem_t, 1> filterItem{ {"Image", "jpg,png"} };
                auto sceneFilename = baseGUI->getFilename();
                if (!sceneFilename.empty())
                {
                    std::filesystem::path path(sceneFilename);
                    path = path.replace_extension(".png");
                    sceneFilename = path.filename().string();
                }

                nfdresult_t result = NFD_SaveDialog(&outPath,
                    filterItem.data(), filterItem.size(), nullptr, sceneFilename.c_str());
                if (result == NFD_OKAY)
                {
                    helper::io::STBImage image;
                    image.init(m_currentFBOSize.first, m_currentFBOSize.second, 1, 1, sofa::helper::io::Image::DataType::UINT32, sofa::helper::io::Image::ChannelFormat::RGBA);

                    glBindTexture(GL_TEXTURE_2D, m_fbo->getColorTexture());

                    // Read the pixel data from the OpenGL texture
                    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.getPixels());

                    glBindTexture(GL_TEXTURE_2D, 0);

                    image.save(outPath, 90);
                }
            }

            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Windows"))
        {
            ImGui::Checkbox(windowNameViewport, &isViewportWindowOpen);
            ImGui::Checkbox(windowNameSceneGraph, &isSceneGraphWindowOpen);
            ImGui::Checkbox(windowNameROS, &isROSWindowOpen);
            ImGui::Separator();
            ImGui::EndMenu();
        }

        ImGui::SetCursorPosX(ImGui::GetColumnWidth() / 2); //approximatively the center of the menu bar
        if (ImGui::Checkbox("Animate", &animate))
        {
            sofa::helper::getWriteOnlyAccessor(groot->animate_).wref() = animate;
        }
        ImGui::SameLine();
        if (animate)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        if (animate)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        ImGui::SameLine();
        if (ImGui::Button("      Reload"))
        {
            groot->setTime(0.);
            sofa::simulation::node::reset ( groot.get() );
        }

        if (ImGui::Checkbox("Record", &record))
        {
        }

        if (ImGui::Checkbox("Connect", &connect))
        {
        }

        const auto posX = ImGui::GetCursorPosX();
        if (showTime)
        {
            auto position = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("Time: 000.000").x
                            - 2 * ImGui::GetStyle().ItemSpacing.x;
            ImGui::SetCursorPosX(position);
            ImGui::TextDisabled("Time: %.3f", groot->getTime());
            ImGui::SetCursorPosX(posX);
        }
        if (showFPSInMenuBar && animate)
        {
            auto position = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("1000.0 FPS ").x
                            - 2 * ImGui::GetStyle().ItemSpacing.x;
            position -= ImGui::CalcTextSize("Time: 000.000  ").x;
            ImGui::SetCursorPosX(position);
            ImGui::Text("%.1f FPS", io.Framerate);
            ImGui::SetCursorPosX(posX);
        }
        mainMenuBarSize = ImGui::GetWindowSize();
        ImGui::EndMainMenuBar();
    }

    /***************************************
     * Viewport window
     **************************************/
    showViewport(groot, windowNameViewport, isViewportWindowOpen);

    /***************************************
     * Profiler window
     **************************************/
    sofa::helper::AdvancedTimer::setInterval("Animate", 1);
    sofa::helper::AdvancedTimer::setOutputType("Animate", "gui");

    /***************************************
     * Scene graph window
     **************************************/
    static std::set<core::objectmodel::BaseObject*> openedComponents;
    static std::set<core::objectmodel::BaseObject*> focusedComponents;
    showSceneGraph(groot, windowNameSceneGraph, isSceneGraphWindowOpen, openedComponents, focusedComponents);

    /***************************************
     * ROS window
     **************************************/
    showROS(groot, windowNameROS, isROSWindowOpen);

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
