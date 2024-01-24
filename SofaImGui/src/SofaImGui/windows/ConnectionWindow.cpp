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

void ConnectionWindow::showWindow(const ImGuiWindowFlags &windowFlags)
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

            ImGui::Separator();
            ImGui::Spacing();

            if (method == 0) // ROS
            {
                static char nodeBuf[30];
                static char topicBuf[30];

                {
                    ImGui::Text("Send");

                    bool hasNodeNameChanged = ImGui::InputTextWithHint("Node##Send", "Enter a node name", nodeBuf, 30, ImGuiInputTextFlags_CharsNoBlank);
                    bool hasTopicNameChanged = ImGui::InputTextWithHint("Topic##Send", "Enter a topic name", topicBuf, 30, ImGuiInputTextFlags_CharsNoBlank);

                    if (hasNodeNameChanged)
                    {
                        msg_warning("Connection") << "changed for = " << nodeBuf;
                        // m_rosnode->set_parameter(rclcpp::Parameter(nodeBuf));
                    }
                }

                ImGui::Spacing();
                ImGui::Spacing();

                {
                    ImGui::Text("Receive");

                    { // Nodes
                        static int nodeID = -1;

                        // List of found nodes
                        const std::vector<std::string>& nodelist = m_rosnode->get_node_names();
                        int nbNodes = nodelist.size();
                        const char* nodes[nbNodes];
                        for (int i=0; i<nbNodes; i++)
                            nodes[i] = nodelist[i].c_str();

                        ImGui::Combo("Node##Receive", &nodeID, nodes, IM_ARRAYSIZE(nodes));
                    }

                    { // Topics
                        static int topicID = -1;

                        // List of found topics
                        const auto& topiclist = m_rosnode->get_topic_names_and_types();
                        int nbTopics = topiclist.size();
                        const char* topics[nbTopics];
                        int index = 0;
                        for (const auto& [key, value]: topiclist)
                            topics[index++] = key.c_str();

                        ImGui::Combo("Topic##Receive", &topicID, topics, IM_ARRAYSIZE(topics));
                    }
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

}

