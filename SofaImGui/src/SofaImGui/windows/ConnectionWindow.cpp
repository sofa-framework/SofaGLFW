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
#include "sofa/core/behavior/BaseMechanicalState.h"

#if SOFAIMGUI_WITH_ROS == 1
#include <rclcpp/node.hpp>
#endif

namespace sofaimgui::windows {

int ConnectionWindow::m_method = -1;

ConnectionWindow::ConnectionWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;

#if SOFAIMGUI_WITH_ROS == 1
    rclcpp::init(0, nullptr);
    m_rosnode = std::make_shared<ROSNode>("SofaComplianceRobotics");
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

void ConnectionWindow::showWindow(sofa::simulation::Node *groot,
                                  const ImGuiWindowFlags &windowFlags)
{
    if (m_isWindowOpen)
    {
        if (ImGui::Begin(m_name.c_str(), &m_isWindowOpen, windowFlags))
        {
            if (m_isLocked)
                ImGui::BeginDisabled();

            static const char* items[]{
#if SOFAIMGUI_WITH_ROS == 1
                                       "ROS",
#endif
                                       "None"
            };

            if (ImGui::CollapsingHeader("Method", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Indent();
                ImGui::Combo("##ComboMethod", &m_method, items, IM_ARRAYSIZE(items));
                ImGui::Unindent();
            }

            ImGui::Spacing();

            const std::map<std::string, std::string>& simulationDataList = getSimulationDataList(groot);
            m_isConnectable = false;

#if SOFAIMGUI_WITH_ROS == 1
            if (m_method == 0) // ROS
                showROSWindow(simulationDataList);
#endif

            if (m_isLocked)
                ImGui::EndDisabled();
        }
    }
}

std::map<std::string, std::string> ConnectionWindow::getSimulationDataList(const sofa::core::sptr<sofa::simulation::Node>& groot)
{
    std::map<std::string, std::string> list;

    const auto& node = groot->getChild("UserInterface");
    if(node != nullptr)
    {
        const auto& data = node->getDataFields();
        for(auto d: data)
        {
            if(d->getGroup().find("state") != std::string::npos)
            {
                list[d->getName()] = d->getValueString();
            }
        }
    }

    return list;
}

void ConnectionWindow::animateBeginEvent(sofa::simulation::Node *groot)
{
#if SOFAIMGUI_WITH_ROS == 1
    if (m_method == 0) // ROS
        animateBeginEventROS(groot);
#endif
}

void ConnectionWindow::animateEndEvent(sofa::simulation::Node *groot)
{
#if SOFAIMGUI_WITH_ROS == 1
    if (m_method == 0) // ROS
        animateEndEventROS(groot);
#endif
}

void ConnectionWindow::connect()
{
#if SOFAIMGUI_WITH_ROS == 1
    if (m_method == 0) // ROS
    {
        createTopics(); // to send selected data
        createSubscriptions(); // to get selected data
    }
#endif

    m_isConnected = true;
    m_isLocked = true;
}

void ConnectionWindow::disconnect()
{
#if SOFAIMGUI_WITH_ROS == 1
    if (m_method == 0) // ROS
    {
        m_rosnode->m_publishers.clear();
        m_rosnode->m_subscriptions.clear();
    }
#endif

    m_isConnected = false;
    m_isLocked = false;
}

#if SOFAIMGUI_WITH_ROS == 1

void ConnectionWindow::showROSWindow(const std::map<std::string, std::string> &simulationDataList)
{
    static char nodeBuf[30];

    if (ImGui::CollapsingHeader("Send "))
    {
        ImGui::Indent();

        bool hasNodeNameChanged = ImGui::InputTextWithHint("##NodeSend", "Enter a node name", nodeBuf, 30, ImGuiInputTextFlags_CharsNoBlank);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Default is /SofaComplianceRobotics");
        if (hasNodeNameChanged)
        {
            msg_warning("Connection") << "changed for = " << nodeBuf;
            // m_rosnode->set_parameter(rclcpp::Parameter(nodeBuf));
        }

        { // Data list box
            static bool sendFirstTime = true;
            static std::map<std::string, bool> sendListboxItems;
            ImGui::Text("Select simulation data to send:");
            ImGui::ListBoxHeader("##DataSend");
            m_rosnode->m_selectedDataToSend.clear();
            for (const auto& [key, value] : simulationDataList)
            {
                if (sendFirstTime)
                    sendListboxItems[key] = false;
                ImGui::Checkbox(key.c_str(), &sendListboxItems[key]);
                if(sendListboxItems[key])
                    m_rosnode->m_selectedDataToSend["/" + key] = value;
            }
            ImGui::ListBoxFooter();
            sendFirstTime = false;
        }

        ImGui::Unindent();
    }

    if (ImGui::CollapsingHeader("Receive "))
    {
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

        { // Data list box receive
            static bool receiveFirstTime = true;
            static std::map<std::string, bool> receiveListboxItems;
            ImGui::Text("Select simulation data to overwrite from available topics:");
            ImGui::ListBoxHeader("##DataReceive");
            m_rosnode->m_selectedDataToOverwrite.clear();
            for (const auto& [dataName, dataValue] : simulationDataList)
            {
                if (receiveFirstTime)
                    receiveListboxItems[dataName] = false;

                bool hasMatchingTopic = topiclist.find("/" + dataName) != topiclist.end();
                if (!hasMatchingTopic)
                    ImGui::BeginDisabled();
                ImGui::Checkbox(dataName.c_str(), &receiveListboxItems[dataName]);
                if (!hasMatchingTopic)
                    ImGui::EndDisabled();

                if(receiveListboxItems[dataName])
                {
                    m_rosnode->m_selectedDataToOverwrite["/" + dataName] = dataValue;  // default temp value, will be overwritten by chosen topic's callback
                }
            }
            ImGui::ListBoxFooter();
            receiveFirstTime = false;
        }

        ImGui::Unindent();
    }

    m_isConnectable = true;
}

void ConnectionWindow::createTopics()
{
    if (!m_rosnode->m_selectedDataToSend.empty())
    {
        m_rosnode->m_publishers.reserve(m_rosnode->m_selectedDataToSend.size());
        for (const auto& [key, value] : m_rosnode->m_selectedDataToSend)
        {
            const auto& publisher = m_rosnode->create_publisher<std_msgs::msg::String>(key, 10);
            m_rosnode->m_publishers.push_back(publisher);
        }
    }
}

void ConnectionWindow::createSubscriptions()
{
    if (!m_rosnode->m_selectedDataToOverwrite.empty())
    {
        m_rosnode->m_subscriptions.reserve(m_rosnode->m_selectedDataToOverwrite.size());
        for (const auto& [key, value] : m_rosnode->m_selectedDataToOverwrite)
        {
            m_rosnode->createSubscription(key);
        }
    }
}

void ConnectionWindow::animateBeginEventROS(sofa::simulation::Node *groot)
{
    if (m_isConnected)
    {
        rclcpp::spin_some(m_rosnode);  // Create a default single-threaded executor and execute any immediately available work.
        for (const auto& [dataName, dataValue]: m_rosnode->m_selectedDataToOverwrite)
        {
            if (dataName.find("effector") != std::string::npos)
            {
                sofa::simulation::Node *modelling = groot->getChild("Modelling");

                if (modelling != nullptr)
                {
                    sofa::simulation::Node *target = modelling->getChild("Target");
                    if (target != nullptr)
                    {
                        sofa::core::behavior::BaseMechanicalState *mechanical = target->getMechanicalState();
                        if (mechanical != nullptr)
                        {
                            std::istringstream str(dataValue);
                            mechanical->readVec(sofa::core::VecId::position(), str);
                        }
                    }
                }
            }
            // for actuator, is it interesting to send actuation to the simulation ? => direct mode
            // from the user interface, switch between direct and inverse mode ?
        }
    }
}

void ConnectionWindow::animateEndEventROS(sofa::simulation::Node *groot)
{
    SOFA_UNUSED(groot);
    if (m_isConnected)
    {
        for (const auto& publisher : m_rosnode->m_publishers)
        {
            auto message = std_msgs::msg::String();
            message.data = m_rosnode->m_selectedDataToSend[publisher->get_topic_name()];
            publisher->publish(message);
        }
    }
}

#endif

}

