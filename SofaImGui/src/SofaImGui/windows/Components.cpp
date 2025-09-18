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
#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/core/CategoryLibrary.h>
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>
#include <sofa/simulation/Simulation.h>
#include <imgui.h>
#include <nfd.h>
#include <IconsFontAwesome6.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <sofa/type/vector.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/simulation/Node.h>
#include <fstream>

#include "Components.h"

namespace windows
{

    void showComponents(const char* const& windowNameComponents,
                        WindowState& winManagerComponents)
    {
        if (*winManagerComponents.getStatePtr())
        {
            if (ImGui::Begin(windowNameComponents, winManagerComponents.getStatePtr()))
            {
                unsigned int nbLoadedComponents = 0;
                if (ImGui::BeginTable("split", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_Resizable))
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();

                    static sofa::core::ClassEntry::SPtr selectedEntry;

                    static std::vector<sofa::core::ClassEntry::SPtr> entries;
                    entries.clear();
                    sofa::core::ObjectFactory::getInstance()->getAllEntries(entries);
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
                            sofa::core::ClassEntry::SPtr classEntry;
                        };
                        static std::vector<ComponentEntry> componentEntries;
                        if (componentEntries.empty())
                        {
                            for (const auto& entry : entries)
                            {
                                std::set<std::string> categoriesSet;
                                for (const auto& [templateInstance, creator] : entry->creatorMap)
                                {
                                    std::vector<std::string> categories;
                                    sofa::core::CategoryLibrary::getCategories(entry->creatorMap.begin()->second->getClass(), categories);
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
                            auto categories = sofa::core::CategoryLibrary::getCategories();
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

                        ImGui::Separator();

                        struct DataInfo
                        {
                            sofa::type::vector<std::string> templateType;
                            std::string description;
                            std::string defaultValue;
                            std::string type;
                        };

                        std::map<std::string, std::map<std::string, DataInfo>> allData;
                        {
                            const auto tmpNode = sofa::core::objectmodel::New<sofa::simulation::Node>("tmp");
                            for (const auto& [templateInstance, creator] : selectedEntry->creatorMap)
                            {
                                sofa::core::objectmodel::BaseObjectDescription desc;
                                const auto object = creator->createInstance(tmpNode.get(), &desc);
                                if (object)
                                {
                                    for (const auto& data : object->getDataFields())
                                    {
                                        allData[data->getGroup()][data->getName()].templateType.push_back(templateInstance);
                                        allData[data->getGroup()][data->getName()].description = data->getHelp();
                                        allData[data->getGroup()][data->getName()].defaultValue = data->getDefaultValueString();
                                        allData[data->getGroup()][data->getName()].type = data->getValueTypeString();
                                    }
                                }
                            }
                        }

                        if (!allData.empty())
                        {
                            ImGui::Spacing();
                            ImGui::TextDisabled("Data:");

                            for (const auto& [group, templateData] : allData)
                            {
                                const auto groupName = group.empty() ? "Property" : group;
                                if (ImGui::CollapsingHeader(groupName.c_str()))
                                {
                                    ImGui::Indent();
                                    for (auto& data : templateData)
                                    {
                                        if (ImGui::CollapsingHeader(data.first.c_str()))
                                        {
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                                            ImGui::TextWrapped(data.second.description.c_str());
                                            const std::string defaultValue = "default value: " + data.second.defaultValue;
                                            ImGui::TextWrapped(defaultValue.c_str());
                                            const std::string type = "type: " + data.second.type;
                                            ImGui::TextWrapped(type.c_str());

                                            ImGui::PopStyleColor();
                                        }
                                    }
                                    ImGui::Unindent();
                                }
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

                if (ImGui::Button(ICON_FA_FLOPPY_DISK" "))
                {
                    nfdchar_t *outPath;
                    const nfdresult_t result = NFD_SaveDialog(&outPath, nullptr, 0, nullptr, "log.txt");
                    if (result == NFD_OKAY)
                    {
                        static std::vector<sofa::core::ClassEntry::SPtr> entries;
                        entries.clear();
                        sofa::core::ObjectFactory::getInstance()->getAllEntries(entries);

                        if (!entries.empty())
                        {
                            std::ofstream outputFile;
                            outputFile.open(outPath, std::ios::out);



                            if (outputFile.is_open())
                            {
                                for (const auto& entry : entries)
                                {
                                    struct EntryProperty
                                    {
                                        std::set<std::string> categories;
                                        std::string target;
                                        bool operator<(const EntryProperty& other) const { return target < other.target && categories < other.categories; }
                                    };
                                    std::set<EntryProperty> entryProperties;

                                    for (const auto& [templateInstance, creator] : entry->creatorMap)
                                    {
                                        EntryProperty property;

                                        std::vector<std::string> categories;
                                        sofa::core::CategoryLibrary::getCategories(entry->creatorMap.begin()->second->getClass(), categories);
                                        property.categories.insert(categories.begin(), categories.end());
                                        property.target = creator->getTarget();

                                        entryProperties.insert(property);
                                    }

                                    for (const auto& [categories, target] : entryProperties)
                                    {
                                        outputFile
                                                << entry->className << ','
                                                << sofa::helper::join(categories.begin(), categories.end(), ';') << ','
                                                << target << ','
                                                << '\n';
                                    }
                                }

                                outputFile.close();
                            }
                        }


                    }

                }
            }
            ImGui::End();
        }
    }


}
