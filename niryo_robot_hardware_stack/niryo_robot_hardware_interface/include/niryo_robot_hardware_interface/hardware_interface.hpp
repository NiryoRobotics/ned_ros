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

#include "common/util/i_interface_core.hpp"

#include "joints_interface/joints_interface_core.hpp"
#include "tools_interface/tools_interface_core.hpp"
#include "end_effector_interface/end_effector_interface_core.hpp"
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "cpu_interface/cpu_interface_core.hpp"
#include "ttl_driver/ttl_interface_core.hpp"
#include "can_driver/can_interface_core.hpp"

#include "niryo_robot_msgs/Trigger.h"
#include "niryo_robot_msgs/SetBool.h"

#include "niryo_robot_msgs/HardwareStatus.h"
#include "niryo_robot_msgs/SoftwareVersion.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include "niryo_robot_status/RobotStatus.h"

namespace niryo_robot_hardware_interface
{
/**
 * @brief The HardwareInterface class
 */
class HardwareInterface : common::util::IInterfaceCore
{
    public:
        HardwareInterface(ros::NodeHandle &nh);
        ~HardwareInterface() override = default;

        // non copyable class
        HardwareInterface( const HardwareInterface& ) = delete;
        HardwareInterface( HardwareInterface&& ) = delete;

        HardwareInterface& operator= ( HardwareInterface && ) = delete;
        HardwareInterface& operator= ( const HardwareInterface& ) = delete;

        bool init(ros::NodeHandle &nh) override;

    private:
        void initParameters(ros::NodeHandle &nh) override;
        void startServices(ros::NodeHandle &nh) override;
        void startPublishers(ros::NodeHandle &nh) override;
        void startSubscribers(ros::NodeHandle &nh) override;

        void initNodes(ros::NodeHandle &nh);

        int rebootMotors(int32_t &status, std::string &message);

        bool _callbackLaunchMotorsReport(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        bool _callbackStopMotorsReport(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        bool _callbackRebootMotors(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);

        void _callbackRobotStatus(const niryo_robot_status::RobotStatus::ConstPtr& msg);

        void _publishHardwareStatus(const ros::TimerEvent&);
        void _publishSoftwareVersion(const ros::TimerEvent&);

    private:
        ros::NodeHandle _nh;

        ros::Publisher _hw_status_publisher;
        ros::Timer _hw_status_publisher_timer;
        ros::Duration _hw_status_publisher_duration{1.0};

        ros::Publisher _sw_version_publisher;
        ros::Timer _sw_version_publisher_timer;
        ros::Duration _sw_version_publisher_duration{1.0};

        std::vector<ros::Publisher> _cancel_goal_publishers;

        ros::ServiceServer _motors_report_service;
        ros::ServiceServer _stop_motors_report_service;
        ros::ServiceServer _reboot_motors_service;

        ros::Subscriber _robot_status_subscriber;

        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;
        std::shared_ptr<can_driver::CanInterfaceCore> _can_interface;
        std::shared_ptr<cpu_interface::CpuInterfaceCore> _cpu_interface;
        std::shared_ptr<conveyor_interface::ConveyorInterfaceCore> _conveyor_interface;
        std::shared_ptr<tools_interface::ToolsInterfaceCore> _tools_interface;
        std::shared_ptr<end_effector_interface::EndEffectorInterfaceCore> _end_effector_interface;
        std::shared_ptr<joints_interface::JointsInterfaceCore> _joints_interface;

        bool _gazebo{false};

        bool _can_enabled{false};
        bool _ttl_enabled{false};
        bool _end_effector_enabled{false};
        int8_t _hardware_state{niryo_robot_msgs::HardwareStatus::NORMAL};

        common::model::EBusProtocol _conveyor_bus{common::model::EBusProtocol::CAN};

        std::string _rpi_image_version;
        std::string _ros_niryo_robot_version;
        std::string _hardware_version;

        int32_t _previous_robot_status{niryo_robot_status::RobotStatus::UNKNOWN};
};

} // namespace niryo_robot_hardware_interface
#endif
