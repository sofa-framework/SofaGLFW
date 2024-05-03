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
#include <filesystem>
#include <fstream>
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
#include <sofa/simulation/graph/DAGNode.h>
#include "Plugins.h"

namespace sofaimgui
{
    void Plugins::showPlugins(const char* const& windowNamePlugins, bool& isPluginsWindowOpen)
    {
        if (isPluginsWindowOpen)
        {
            if (ImGui::Begin(windowNamePlugins, &isPluginsWindowOpen))
            {
                if (ImGui::Button("Load"))
                {
                    std::vector<nfdfilteritem_t> nfd_filters {
                            {"SOFA plugin", sofa::helper::system::DynamicLibrary::extension.c_str() } };

                    nfdchar_t *outPath;
                    nfdresult_t result = NFD_OpenDialog(&outPath, nfd_filters.data(), nfd_filters.size(), NULL);
                    if (result == NFD_OKAY)
                    {
                        if (sofa::helper::system::FileSystem::exists(outPath))
                        {
                            sofa::helper::system::PluginManager::getInstance().loadPluginByPath(outPath);
                            sofa::helper::system::PluginManager::getInstance().writeToIniFile(
                                    sofa::gui::common::BaseGUI::getConfigDirectoryPath() + "/loadedPlugins.ini");
                        }
                    }
                }

                ImGui::BeginChild("Plugins", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f, ImGui::GetContentRegionAvail().y), false, ImGuiWindowFlags_HorizontalScrollbar);

                const auto& pluginMap = sofa::helper::system::PluginManager::getInstance().getPluginMap();

                static std::map<std::string, bool> isSelected;
                static std::string selectedPlugin;
                for (const auto& [path, plugin] : pluginMap)
                {
                    if (ImGui::Selectable(plugin.getModuleName(), selectedPlugin == path))
                    {
                        selectedPlugin = path;
                    }
                }

                ImGui::EndChild();
                ImGui::SameLine();

                if (!selectedPlugin.empty())
                {
                    ImGui::BeginChild("selectedPlugin", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y), false, ImGuiWindowFlags_HorizontalScrollbar);

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

                ImGui::EndChild();
            }
            ImGui::End();
        }
    }


}