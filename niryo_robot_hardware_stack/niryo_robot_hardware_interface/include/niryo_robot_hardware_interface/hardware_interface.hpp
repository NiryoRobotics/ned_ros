/*
    hardware_interface.cpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include "joints_interface/joints_interface_core.hpp"
#include "tools_interface/tools_interface_core.hpp"
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "cpu_interface/cpu_interface_core.hpp"
#include "dynamixel_driver/dxl_driver_core.hpp"
#include "stepper_driver/stepper_driver_core.hpp"
#include "fake_interface/fake_interface_core.hpp"
#include "dynamixel_driver/dxl_enum.hpp"

#include "niryo_robot_msgs/Trigger.h"
#include "niryo_robot_msgs/SetBool.h"

#include "niryo_robot_msgs/HardwareStatus.h"
#include "niryo_robot_msgs/SoftwareVersion.h"
#include "niryo_robot_msgs/CommandStatus.h"

namespace NiryoRobotHardwareInterface
{
    class HardwareInterface
    {
    public:
        HardwareInterface(ros::NodeHandle &nh);

    private:
        ros::NodeHandle &_nh;

        bool _simulation_mode;
        bool _gazebo;
        bool _can_enabled;
        bool _dxl_enabled;

        ros::Publisher _hardware_status_publisher;
        ros::Publisher _software_version_publisher;

        double _publish_hw_status_frequency;
        double _publish_software_version_frequency;

        std::string _rpi_image_version;
        std::string _ros_niryo_robot_version;

        void initNodes();
        void initPublishers();
        void initParams();

        void _publishHardwareStatus();
        void _publishSoftwareVersion();

        bool _callbackLaunchMotorsReport(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        ros::ServiceServer _motors_report_service;

        bool _callbackStopMotorsReport(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        ros::ServiceServer _stop_motors_report_service;

        bool _callbackRebootMotors(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        ros::ServiceServer _reboot_motors_service;

        boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> _dynamixel_driver;
        boost::shared_ptr<StepperDriver::StepperDriverCore> _stepper_driver;
        boost::shared_ptr<CpuInterfaceCore> _cpu_interface;
        boost::shared_ptr<ConveyorInterfaceCore> _conveyor_interface;
        boost::shared_ptr<ToolsInterfaceCore> _tools_interface;
        boost::shared_ptr<JointsInterfaceCore> _joints_interface;
        boost::shared_ptr<FakeInterfaceCore> _fake_interface;

        boost::shared_ptr<std::thread> _publish_hardware_status_thread;
        boost::shared_ptr<std::thread> _publish_software_version_thread;
    };
} // namespace NiryoRobotHardwareInterface
#endif
