/*
hardware_interface.hpp
Copyright (C) 2020 Niryo
All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <ros/ros.h>
#include <memory>

#include "joints_interface/joints_interface_core.hpp"
#include "tools_interface/tools_interface_core.hpp"
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "cpu_interface/cpu_interface_core.hpp"
#include "ttl_driver/ttl_driver_core.hpp"
#include "can_driver/can_driver_core.hpp"
#include "fake_interface/fake_interface_core.hpp"

#include "niryo_robot_msgs/Trigger.h"
#include "niryo_robot_msgs/SetBool.h"

#include "niryo_robot_msgs/HardwareStatus.h"
#include "niryo_robot_msgs/SoftwareVersion.h"
#include "niryo_robot_msgs/CommandStatus.h"

namespace niryo_robot_hardware_interface
{
/**
 * @brief The HardwareInterface class
 */
class HardwareInterface
{
public:
    HardwareInterface(ros::NodeHandle &nh);
    virtual ~HardwareInterface();

private:
    void initNodes(ros::NodeHandle &nh);
    void initParameters(ros::NodeHandle &nh);
    void initPublishers(ros::NodeHandle &nh);

    bool _callbackLaunchMotorsReport(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
    bool _callbackStopMotorsReport(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
    bool _callbackRebootMotors(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);

    void _publishHardwareStatus();
    void _publishSoftwareVersion();

private:
    ros::NodeHandle &_nh;

    ros::Publisher _hardware_status_publisher;
    ros::Publisher _software_version_publisher;

    std::thread _publish_software_version_thread;
    std::thread _publish_hw_status_thread;

    ros::ServiceServer _motors_report_service;
    ros::ServiceServer _stop_motors_report_service;
    ros::ServiceServer _reboot_motors_service;

    std::shared_ptr<ttl_driver::TtlDriverCore> _ttl_driver;
    std::shared_ptr<can_driver::CanDriverCore> _can_driver;
    std::shared_ptr<cpu_interface::CpuInterfaceCore> _cpu_interface;
    std::shared_ptr<conveyor_interface::ConveyorInterfaceCore> _conveyor_interface;
    std::shared_ptr<tools_interface::ToolsInterfaceCore> _tools_interface;
    std::shared_ptr<joints_interface::JointsInterfaceCore> _joints_interface;
    std::shared_ptr<fake_interface::FakeInterfaceCore> _fake_interface;

    double _publish_hw_status_frequency{0.0};
    double _publish_software_version_frequency{0.0};

    bool _simulation_mode{true};
    bool _gazebo{false};

    bool _can_enabled{false};
    bool _ttl_enabled{false};

    std::string _rpi_image_version;
    std::string _ros_niryo_robot_version;
};
} // namespace niryo_robot_hardware_interface
#endif
