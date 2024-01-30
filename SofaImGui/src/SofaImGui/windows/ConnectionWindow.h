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
#pragma once

#include <string>

#include <SofaImGui/config.h>
#include <SofaImGui/windows/BaseWindow.h>
#include <imgui.h>

#if SOFAIMGUI_WITH_ROS == 1
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

using namespace std::chrono_literals;

namespace sofaimgui::windows {

#if SOFAIMGUI_WITH_ROS == 1
class ROSNode: public rclcpp::Node
{
   public:
    ROSNode(const std::string& name): Node(name){}

    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> m_publishers;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> m_subscriptions;

    std::map<std::string, std::string> m_selectedDataToSend;
    std::map<std::string, std::string> m_selectedDataToOverwrite;

    void createSubscription(const std::string& topicName)
    {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription = this->create_subscription<std_msgs::msg::String>(
           topicName, 10, std::bind(&ROSNode::topicCallback, this, std::placeholders::_1));
        this->m_subscriptions.push_back(subscription);
    }

    void topicCallback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
};
#endif


class ConnectionWindow : public BaseWindow
{
   public:
    ConnectionWindow(const std::string& name, const bool& isWindowOpen);
    ~ConnectionWindow();

    using BaseWindow::m_name;
    using BaseWindow::m_isWindowOpen;

    void showWindow(sofa::simulation::Node *groot, const ImGuiWindowFlags &windowFlags);

    void animateBeginEvent(sofa::simulation::Node *groot);
    void animateEndEvent(sofa::simulation::Node *groot);

    bool isConnected() {return m_isConnected;}
    bool isConnectable() {return m_isConnectable;}

    void connect();
    void disconnect();

   protected:

    bool m_isConnectable = false;
    bool m_isConnected = false;
    bool m_isLocked = false;
    static int m_method;

    void init();

    std::map<std::string, std::string> getSimulationDataList(const sofa::core::sptr<sofa::simulation::Node>& groot);

#if SOFAIMGUI_WITH_ROS == 1
    std::shared_ptr<ROSNode> m_rosnode;

    void showROSWindow(const std::map<std::string, std::string>& simulationDataList);
    void createTopics();
    void createSubscriptions();
    void animateBeginEventROS(sofa::simulation::Node *groot);
    void animateEndEventROS(sofa::simulation::Node *groot);
#endif
};

}


