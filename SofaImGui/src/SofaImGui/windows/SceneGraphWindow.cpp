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

#include <SofaImGui/windows/SceneGraphWindow.h>
#include <IconsFontAwesome6.h>
#include <SofaImGui/ObjectColor.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <sofa/core/ObjectFactory.h>

namespace sofaimgui::windows {

SceneGraphWindow::SceneGraphWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;
}

void SceneGraphWindow::showWindow(sofa::simulation::Node *groot, const ImGuiWindowFlags& windowFlags)
{
    static std::set<sofa::core::objectmodel::BaseObject*> openedComponents;
    static std::set<sofa::core::objectmodel::BaseObject*> focusedComponents;

    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            ImVec2 buttonSize(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
            const bool expand = ImGui::Button(ICON_FA_EXPAND, buttonSize);
            ImGui::SameLine();
            const bool collapse = ImGui::Button(ICON_FA_COMPRESS, buttonSize);
            ImGui::SameLine();
            static bool showSearch = false;
            if (ImGui::Button(ICON_FA_MAGNIFYING_GLASS, buttonSize))
            {
                showSearch = !showSearch;
            }
            static ImGuiTextFilter filter;
            if (showSearch)
            {
                filter.Draw("Search");
            }

            unsigned int treeDepth {};
            static sofa::core::objectmodel::Base* clickedObject { nullptr };

            std::function<void(sofa::simulation::Node*)> showNode;
            showNode = [&showNode, &treeDepth, expand, collapse](sofa::simulation::Node* node)
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
                    int i = 0;
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
                        if (object->countLoggedMessages({sofa::helper::logging::Message::Error,
                                                         sofa::helper::logging::Message::Fatal})!=0)
                        {
                            icon = ICON_FA_BUG;
                            objectColor = ImVec4(1.f, 0.f, 0.f, 1.f); //red
                        }
                        else if (object->countLoggedMessages({sofa::helper::logging::Message::Warning})!=0)
                        {
                            icon = ICON_FA_TRIANGLE_EXCLAMATION;
                            objectColor = ImVec4(1.f, 0.5f, 0.f, 1.f); //orange
                        }
                        else if (object->countLoggedMessages({sofa::helper::logging::Message::Info,
                                                              sofa::helper::logging::Message::Deprecated,
                                                              sofa::helper::logging::Message::Advice})!=0)
                        {
                            objectColor = getObjectColor(object);
                            icon = ICON_FA_COMMENT;
                        }
                        else
                        {
                            objectColor = getObjectColor(object);
                        }

                        ImGui::PushStyleColor(ImGuiCol_Text, objectColor);
                        ImGui::PushID(i++);
                        const auto objectOpen = ImGui::TreeNodeEx(icon, objectFlags);
                        ImGui::PopID();
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
                        ImGui::Text("%s", object->getName().c_str());

                        ImGui::TableNextColumn();
                        ImGui::TextDisabled("%s", objectClassName.c_str());

                        if (isObjectHighlighted)
                        {
                            ImGui::PopStyleColor();
                        }

                        if (objectOpen && !slaves.empty())
                        {
                            for (const auto &slave : slaves)
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
                                ImGui::TextDisabled("%s", slave->getClassName().c_str());

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
                        showNode(dynamic_cast<sofa::simulation::Node*>(child));
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

                showNode(groot);

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
                    std::map<std::string, std::vector<sofa::core::BaseData*> > groupMap;
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
                                    ImGui::TextDisabled("%s", data->getHelp().c_str());
                                    ImGui::TextDisabled("Type: %s", data->getValueTypeString().c_str());
                                    ImGui::EndTooltip();
                                }
                                if (isOpen)
                                {
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                                    ImGui::TextWrapped("%s", data->getHelp().c_str());

                                    if (data->getParent())
                                    {
                                        const auto linkPath = data->getLinkPath();
                                        if (!linkPath.empty())
                                        {
                                            ImGui::TextWrapped("%s", linkPath.c_str());
                                            if (ImGui::IsItemHovered())
                                            {
                                                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                            }
                                            if (ImGui::IsItemClicked())
                                            {
                                                auto* owner = dynamic_cast<sofa::core::objectmodel::BaseObject*>(data->getParent()->getOwner());
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
                                ImGui::TextDisabled("%s", link->getHelp().c_str());
                                ImGui::EndTooltip();
                            }
                            if (isOpen)
                            {
                                ImGui::TextDisabled("%s", link->getHelp().c_str());
                                ImGui::TextWrapped("%s", linkValue.c_str());
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

    sofa::type::vector<sofa::core::objectmodel::BaseObject*> toRemove;
    for (auto* component : openedComponents)
    {
        bool isOpen = true;
        ImGui::PushStyleColor(ImGuiCol_Text, getObjectColor(component));
        if (ImGui::Begin((ICON_FA_CUBE "  " + component->getName() + " (" + component->getPathName() + ")").c_str(), &isOpen))
        {
            ImGui::PopStyleColor();
            std::map<std::string, std::vector<sofa::core::BaseData*> > groupMap;
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
                                ImGui::TextDisabled("%s", data->getHelp().c_str());
                                ImGui::TextDisabled("Type: %s", data->getValueTypeString().c_str());
                                ImGui::EndTooltip();
                            }
                            if (isOpenData)
                            {
                                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                                ImGui::TextWrapped("%s", data->getHelp().c_str());

                                if (data->getParent())
                                {
                                    const auto linkPath = data->getLinkPath();
                                    if (!linkPath.empty())
                                    {
                                        ImGui::TextWrapped("%s", linkPath.c_str());

                                        if (ImGui::IsItemHovered())
                                        {
                                            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                        }
                                        if (ImGui::IsItemClicked())
                                        {
                                            auto* owner = dynamic_cast<sofa::core::objectmodel::BaseObject*>(data->getParent()->getOwner());
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
                            ImGui::TextDisabled("%s", link->getHelp().c_str());
                            ImGui::EndTooltip();
                        }
                        if (isOpenData)
                        {
                            ImGui::TextDisabled("%s", link->getHelp().c_str());
                            ImGui::TextWrapped("%s", linkValue.c_str());
                        }
                    }
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Infos"))
                {
                    ImGui::Text("Name: %s", component->getClassName().c_str());
                    ImGui::Spacing();
                    ImGui::TextDisabled("Template:");
                    ImGui::TextWrapped("%s", component->getClass()->templateName.c_str());
                    ImGui::Spacing();
                    ImGui::TextDisabled("Namespace:");
                    ImGui::TextWrapped("%s", component->getClass()->namespaceName.c_str());

                    sofa::core::ObjectFactory::ClassEntry entry = sofa::core::ObjectFactory::getInstance()->getEntry(component->getClassName());
                    if (! entry.creatorMap.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Description:");
                        ImGui::TextWrapped("%s", entry.description.c_str());
                    }

                    const std::string instantiationSourceFilename = component->getInstanciationSourceFileName();
                    if (!instantiationSourceFilename.empty())
                    {
                        ImGui::Spacing();
                        ImGui::TextDisabled("Definition:");
                        ImGui::TextWrapped("%s", component->getInstanciationSourceFileName().c_str());
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

                            constexpr auto writeMessageType = [](const sofa::helper::logging::Message::Type t)
                            {
                                switch (t)
                                {
                                case sofa::helper::logging::Message::Advice     : return ImGui::TextColored(ImVec4(0.f, 0.5686f, 0.9176f, 1.f), "[SUGGESTION]");
                                case sofa::helper::logging::Message::Deprecated : return ImGui::TextColored(ImVec4(0.5529f, 0.4314f, 0.3882f, 1.f), "[DEPRECATED]");
                                case sofa::helper::logging::Message::Warning    : return ImGui::TextColored(ImVec4(1.f, 0.4275f, 0.f, 1.f), "[WARNING]");
                                case sofa::helper::logging::Message::Info       : return ImGui::Text("[INFO]");
                                case sofa::helper::logging::Message::Error      : return ImGui::TextColored(ImVec4(0.8667f, 0.1725f, 0.f, 1.f), "[ERROR]");
                                case sofa::helper::logging::Message::Fatal      : return ImGui::TextColored(ImVec4(0.8353, 0.f, 0.f, 1.f), "[FATAL]");
                                case sofa::helper::logging::Message::TEmpty     : return ImGui::Text("[EMPTY]");
                                default: return;
                                }
                            };
                            writeMessageType(message.type());

                            ImGui::TableNextColumn();
                            ImGui::TextWrapped("%s", message.message().str().c_str());
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
}

