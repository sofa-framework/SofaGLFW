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

#include <SofaImGui/windows/ConnectionWindow.h>
#include <IconsFontAwesome5.h>

#if SOFAIMGUI_WITH_ROS == 1
#include <rclcpp/node.hpp>
#endif

namespace sofaimgui::windows {

ConnectionWindow::ConnectionWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;

#if SOFAIMGUI_WITH_ROS == 1
    rclcpp::init(0, nullptr);
    m_rosnode = std::make_shared<ROSPublisher>("SofaComplianceRobotics");
#endif
}

ConnectionWindow::~ConnectionWindow()
{
#if SOFAIMGUI_WITH_ROS == 1
    rclcpp::shutdown();
#endif
}

void ConnectionWindow::init()
{
}

void ConnectionWindow::showWindow(const sofa::core::sptr<sofa::simulation::Node>& groot,
                                  const ImGuiWindowFlags &windowFlags)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            if (m_locked)
                ImGui::BeginDisabled();

            static int method = -1;
            static const char* items[]{"ROS"};

            ImGui::Combo("Method", &method, items, IM_ARRAYSIZE(items));

            ImGui::Spacing();

            const std::vector<std::string> simulationDataList = getSimulationDataList(groot);

            if (method == 0) // ROS
            {
                static char nodeBuf[30];
                static char topicBuf[30];

                { // Send section
                    ImGui::Text("Send " ICON_FA_ARROW_CIRCLE_UP);
                    ImGui::Indent();

                    bool hasNodeNameChanged = ImGui::InputTextWithHint("##NodeSend", "Enter a node name", nodeBuf, 30, ImGuiInputTextFlags_CharsNoBlank);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Default is SofaComplianceRobotics");
                    if (hasNodeNameChanged)
                    {
                        msg_warning("Connection") << "changed for = " << nodeBuf;
                        // m_rosnode->set_parameter(rclcpp::Parameter(nodeBuf));
                    }

                    bool hasTopicNameChanged = ImGui::InputTextWithHint("##TopicSend", "Enter a topic name", topicBuf, 30, ImGuiInputTextFlags_CharsNoBlank);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Default is Actuators");
                    if (hasTopicNameChanged)
                    {
                        msg_warning("Connection") << "changed for = " << nodeBuf;
                        // m_rosnode->set_parameter(rclcpp::Parameter(nodeBuf));
                    }

                    { // Data list box
                        static bool sendFirstTime = true;
                        static std::map<std::string, bool> sendListboxItems;
                        ImGui::Text("Select simulation data to send:");
                        ImGui::ListBoxHeader("##DataSend");
                        for (const std::string& d : simulationDataList)
                        {
                            if (sendFirstTime)
                                sendListboxItems[d] = false;
                            if (ImGui::Checkbox(d.c_str(), &sendListboxItems[d]))
                            {
                               // handle selection
                            }
                        }
                        ImGui::ListBoxFooter();
                        sendFirstTime = false;
                    }

                    ImGui::Unindent();
                }

                ImGui::Spacing();
                ImGui::Spacing();

                { // Receive section
                    ImGui::Text("Receive " ICON_FA_ARROW_CIRCLE_DOWN);
                    ImGui::Indent();

                    // Nodes
                    static int nodeID = -1;

                    // List of found nodes
                    const std::vector<std::string>& nodelist = m_rosnode->get_node_names();
                    int nbNodes = nodelist.size();
                    const char* nodes[nbNodes];
                    for (int i=0; i<nbNodes; i++)
                        nodes[i] = nodelist[i].c_str();

                    ImGui::Text("Select a node:");
                    ImGui::Combo("##NodeReceive", &nodeID, nodes, IM_ARRAYSIZE(nodes));

                    // Topics
                    static int topicID = -1;

                    // List of found topics
                    auto topiclist = m_rosnode->get_topic_names_and_types();
                    if (nodeID >= 0)
                    {
                        for (auto it = topiclist.cbegin(); it != topiclist.cend();)
                        {
                            bool isTopicOnlyPublishedByChosenNode = true;
                            const std::vector<rclcpp::TopicEndpointInfo>& publishers = m_rosnode->get_publishers_info_by_topic(it->first.c_str());
                            for (const auto& p : publishers)
                            {
                                if (nodes[nodeID] != "/" + p.node_name())
                                    isTopicOnlyPublishedByChosenNode = false;
                            }
                            if (!isTopicOnlyPublishedByChosenNode)
                                topiclist.erase(it++);
                            else
                                ++it;
                        }
                    }

                    int nbTopics = topiclist.size();
                    const char* topics[nbTopics];
                    int i=0;
                    for (const auto& [key, value]: topiclist)
                        topics[i++]=key.c_str();

                    ImGui::Text("Select a topic:");
                    ImGui::Combo("##TopicReceive", &topicID, topics, IM_ARRAYSIZE(topics));

                    { // Data list box
                        static bool receiveFirstTime = true;
                        static std::map<std::string, bool> receiveListboxItems;
                        ImGui::Text("Select simulation data to overwrite:");
                        ImGui::ListBoxHeader("##DataReceive");
                        for (const std::string& d : simulationDataList)
                        {
                            if (receiveFirstTime)
                                receiveListboxItems[d] = false;
                            if (ImGui::Checkbox(d.c_str(), &receiveListboxItems[d]))
                            {
                               // handle selection
                            }
                        }
                        ImGui::ListBoxFooter();
                        receiveFirstTime = false;
                    }

                    ImGui::Unindent();
                }

                { // Check entries
                }
                // Test if everything is okay and then set isConnectable to true
                m_isConnectable = true;
            }
            else
            {
                m_isConnectable = false;
            }

            if (m_locked)
                ImGui::EndDisabled();
        }
    }
}


std::vector<std::string> ConnectionWindow::getSimulationDataList(const sofa::core::sptr<sofa::simulation::Node>& groot)
{
    std::vector<std::string> list;

    const auto& node = groot->getChild("UserInterface");
    if(node != nullptr)
    {
        const auto& data = node->getDataFields();
        for(auto d: data)
        {
            if(d->getGroup().find("state") != std::string::npos)
            {
                list.push_back(d->getName());
            }
        }
    }

    return list;
}

}

