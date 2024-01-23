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


namespace sofaimgui::windows {

ConnectionWindow::ConnectionWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;

#if SOFAIMGUI_WITH_ROS == 1
    // rclcpp::init(0, nullptr);
    // rclcpp::spin(std::make_shared<ROSPublisher>());
#endif
}

ConnectionWindow::~ConnectionWindow()
{
#if SOFAIMGUI_WITH_ROS == 1
    // rclcpp::shutdown();
#endif
}

void ConnectionWindow::init()
{
}

void ConnectionWindow::showWindow()
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen))
        {
            static int method = -1;
            static const char* items[]{"ROS", "Method1", "Method2"};
            ImGui::Combo("Connection method", &method, items, IM_ARRAYSIZE(items));

            ImGui::Separator();
            ImGui::Spacing();

            if (method == 0) // ROS
            {
                static char nodeBuf[30];
                static char topicBuf[30];

                {
                    ImGui::Text("Send");

                    ImGui::InputTextWithHint("Node##Send", "Enter a node name", nodeBuf, 30, ImGuiInputTextFlags_CharsNoBlank);
                    ImGui::InputTextWithHint("Topic##Send", "Enter a topic name", topicBuf, 30, ImGuiInputTextFlags_CharsNoBlank);
                }

                ImGui::Spacing();
                ImGui::Spacing();

                {
                    ImGui::Text("Receive");

                    static int node = -1;
                    static const char* nodes[]{"Node1", "Node2"};
                    ImGui::Combo("Node##Receive", &node, nodes, IM_ARRAYSIZE(nodes));

                    static int topic = -1;
                    static const char* topics[]{"Topic1", "Topic2"};
                    ImGui::Combo("Topic##Receive", &topic, topics, IM_ARRAYSIZE(topics));
                }

                // Test if everything is okay and then set isConnectable to true
                m_isConnectable = true;
            }
            else
            {
                m_isConnectable = false;
            }
        }
    }
}

}

