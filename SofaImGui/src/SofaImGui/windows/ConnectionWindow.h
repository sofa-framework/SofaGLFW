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
class ROSPublisher: public rclcpp::Node
{

   public:
    ROSPublisher(const std::string& name): Node(name), m_count(0)
    {
        m_publisher = this->create_publisher<std_msgs::msg::String>("Actuators", 10);
        // todo : call the callback at each time step of the simulatin
    }

   protected:
    void callback()
    {
        // todo : implement "get and send" simulation data
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(m_count++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        m_publisher->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    size_t m_count;
    std::string m_name;

};
#endif


class ConnectionWindow : public BaseWindow
{
   public:
    ConnectionWindow(const std::string& name, const bool& isWindowOpen);
    ~ConnectionWindow();

    using BaseWindow::m_name;
    using BaseWindow::m_isWindowOpen;

    bool m_isConnectable = false;
    bool m_locked = false;

    void showWindow(const sofa::core::sptr<sofa::simulation::Node> &groot, const ImGuiWindowFlags &windowFlags);
    void lock() {m_locked=true;}
    void unlock() {m_locked=false;}

   protected:

    void init();
    std::vector<std::string> getSimulationDataList(const sofa::core::sptr<sofa::simulation::Node>& groot);

#if SOFAIMGUI_WITH_ROS == 1
    std::shared_ptr<ROSPublisher> m_rosnode;
#endif
};

}


