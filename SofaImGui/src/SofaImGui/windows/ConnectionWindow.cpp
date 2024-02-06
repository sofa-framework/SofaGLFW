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
#include <rmw/validate_node_name.h>
#endif


namespace sofaimgui::windows {

int ConnectionWindow::m_method = -1;

ConnectionWindow::ConnectionWindow(const std::string& name, const bool& isWindowOpen)
{
    m_name = name;
    m_isWindowOpen = isWindowOpen;

#if SOFAIMGUI_WITH_ROS == 1
    rclcpp::init(0, nullptr);
    m_rosnode = std::make_shared<ROSNode>(m_defaultNodeName);
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

            const std::map<std::string, std::vector<float>>& simulationStateList = getSimulationStateList(groot);
            m_isConnectable = false;

#if SOFAIMGUI_WITH_ROS == 1
            if (m_method == 0) // ROS
                showROSWindow(simulationStateList);
#endif

            if (m_isLocked)
                ImGui::EndDisabled();
        }
    }
}

std::map<std::string, std::vector<float>> ConnectionWindow::getSimulationStateList(const sofa::core::sptr<sofa::simulation::Node>& groot)
{
    std::map<std::string, std::vector<float>> list;

    const auto& node = groot->getChild("UserInterface");
    if(node != nullptr)
    {
        const auto& data = node->getDataFields();
        for(auto d: data)
        {
            if(d->getGroup().find("state") != std::string::npos)
            {
                std::vector<float> vector;
                std::stringstream str(d->getValueString());
                std::string value;
                while (str >> value)
                {
                    std::replace(value.begin(), value.end(), '.', ',');
                    vector.push_back(std::stof(value));
                }
                list[d->getName()] = vector;
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
        m_rosnode->createTopics(); // to publish selected state
        m_rosnode->createSubscriptions(); // to get selected state
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

void ConnectionWindow::showROSWindow(const std::map<std::string, std::vector<float>> &simulationStateList)
{
    static char nodeBuf[30];
    static bool validNodeName = true;

    static float pulseDuration = 0;
    pulseDuration += ImGui::GetIO().DeltaTime;
    float pulse = pulseDuration / 2.0f;
    if (pulse > 2)
        pulseDuration = 0;

    ImGui::PushStyleColor(ImGuiCol_Text, (m_isConnected && !m_rosnode->m_selectedStateToPublish.empty())? ImVec4(0.50f, 1.00f, 0.50f, 0.75f + 0.25f * sin(pulse * 2 * 3.1415)): ImGui::GetStyle().Colors[ImGuiCol_Text]);
    if (ImGui::CollapsingHeader("Publishers"))
    {
        ImGui::PopStyleColor();
        ImGui::Indent();

        bool hasNodeNameChanged = ImGui::InputTextWithHint("##NodePublishers", "Enter a node name", nodeBuf, 30, ImGuiInputTextFlags_CharsNoBlank);

        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Default is %s", m_defaultNodeName.c_str());

        static int nameCheckResult;
        if (hasNodeNameChanged)
        {
            size_t index;
            [[maybe_unused]] auto result = rmw_validate_node_name(nodeBuf, &nameCheckResult, &index);
            if(nameCheckResult == 0)
            {
                m_rosnode = std::make_shared<ROSNode>(nodeBuf);
                validNodeName = true;
            }
            else
            {
                if (nameCheckResult == 1) // empty buf
                {
                    m_rosnode = std::make_shared<ROSNode>(m_defaultNodeName);
                    validNodeName = true;
                }
                else
                    validNodeName = false;
            }
        }

        if (!validNodeName)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0, 0., 0., 1.0));
            ImGui::Text("%s", rmw_node_name_validation_result_string(nameCheckResult));
            ImGui::PopStyleColor();
        }

        { // State list box
            static bool publishFirstTime = true;
            static std::map<std::string, bool> publishListboxItems;
            ImGui::Text("Select simulation states to publish:");
            ImGui::ListBoxHeader("##StatePublish");
            m_rosnode->m_selectedStateToPublish.clear();
            for (const auto& [key, value] : simulationStateList)
            {
                if (publishFirstTime)
                    publishListboxItems[key] = false;

                ImGui::Checkbox(key.c_str(), &publishListboxItems[key]);

                if(publishListboxItems[key])
                    m_rosnode->m_selectedStateToPublish["/" + key] = value;
            }
            ImGui::ListBoxFooter();
            publishFirstTime = false;
        }

        ImGui::Unindent();
    }
    else
        ImGui::PopStyleColor();

    ImGui::PushStyleColor(ImGuiCol_Text, (m_isConnected && !m_rosnode->m_selectedStateToOverwrite.empty())? ImVec4(0.50f, 1.00f, 0.50f, 0.75f + 0.25f * sin(pulse * 2 * 3.1415)): ImGui::GetStyle().Colors[ImGuiCol_Text]);
    if (ImGui::CollapsingHeader("Subscriptions"))
    {
        ImGui::PopStyleColor();
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
        ImGui::Combo("##NodeSubscription", &nodeID, nodes, IM_ARRAYSIZE(nodes));

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

        { // State list box subcriptions
            static bool subscribeFirstTime = true;
            static std::map<std::string, bool> subcriptionListboxItems;
            ImGui::Text("Select simulation states to overwrite:");
            ImGui::ListBoxHeader("##StateSubscription");
            if (!m_isConnected)
                m_rosnode->m_selectedStateToOverwrite.clear();
            for (const auto& [simStateName, stateValue] : simulationStateList)
            {
                std::string stateName = simStateName;
                if (stateName.find("effector")!=std::string::npos)
                    stateName = "target/position";

                if (subscribeFirstTime)
                    subcriptionListboxItems[stateName] = false;

                bool hasMatchingTopic = topiclist.find("/" + stateName) != topiclist.end();

                if (!hasMatchingTopic)
                    ImGui::BeginDisabled();
                ImGui::Checkbox(stateName.c_str(), &subcriptionListboxItems[stateName]);
                if (!hasMatchingTopic)
                    ImGui::EndDisabled();

                if(subcriptionListboxItems[stateName] & !m_isConnected)
                    m_rosnode->m_selectedStateToOverwrite["/" + stateName] = stateValue;  // default temp value, will be overwritten by chosen topic's callback

            }
            ImGui::ListBoxFooter();
            subscribeFirstTime = false;
        }

        ImGui::Unindent();
    }
    else
        ImGui::PopStyleColor();

    m_isConnectable = validNodeName;
}

void ConnectionWindow::animateBeginEventROS(sofa::simulation::Node *groot)
{
    if (m_isConnected)
    {
        rclcpp::spin_some(m_rosnode);  // Create a default single-threaded executor and execute any immediately available work.
        for (const auto& [stateName, stateValue]: m_rosnode->m_selectedStateToOverwrite)
        {
            if (stateName.find("target") != std::string::npos)
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
                            std::stringstream str;
                            for (const float& value: stateValue)
                                str << value << " ";
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
            auto message = std_msgs::msg::Float32MultiArray();
            const auto& stateVector = m_rosnode->m_selectedStateToPublish[publisher->get_topic_name()];
            message.data.insert(message.data.end(), stateVector.begin(), stateVector.end());
            publisher->publish(message);
        }
    }
}

#endif

}

