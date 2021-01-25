/*
    fake_interface_core.hpp
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

#ifndef FAKE_INTERFACE_CORE_HPP
#define FAKE_INTERFACE_CORE_HPP

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <thread>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/SetBool.h"
#include "niryo_robot_msgs/HardwareStatus.h"
#include "niryo_robot_msgs/SoftwareVersion.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include "niryo_robot_msgs/Trigger.h"

#include "tools_interface/PingDxlTool.h"
#include "tools_interface/OpenGripper.h"
#include "tools_interface/CloseGripper.h"
#include "tools_interface/PullAirVacuumPump.h"
#include "tools_interface/PushAirVacuumPump.h"

#include "conveyor_interface/SetConveyor.h"
#include "conveyor_interface/ControlConveyor.h"
#include "conveyor_interface/ConveyorFeedback.h"

#include "dynamixel_driver/dxl_driver_core.hpp"
#include "stepper_driver/stepper_driver_core.hpp"
#include "joints_interface/joints_interface_core.hpp"
#include "tools_interface/tools_interface_core.hpp"
#include "conveyor_interface/conveyor_interface_core.hpp"

#include "fake_interface/FakeJointHardwareInterface.hpp"

class FakeInterfaceCore
{
public:
    FakeInterfaceCore();

    void initServices();
    void startPublishersSubscribers();
    void initParams();

    void rosControlLoop();

    dynamixel_driver::DxlArrayMotorHardwareStatus getDxlHwStatus();
    niryo_robot_msgs::BusState getDxlBusState();
    stepper_driver::StepperArrayMotorHardwareStatus getStepperHwStatus();
    niryo_robot_msgs::BusState getCanBusState();
    void getCalibrationState(boost::shared_ptr<bool> &need_calibration, boost::shared_ptr<bool> &calibration_in_progress);

    int getCpuTemperature();

    void pubToolId(int id);

    std::string jointIdToJointName(int id, uint8_t motor_type);

    std::vector<uint8_t> _dxl_motors_id{2, 3, 6};
    std::vector<uint8_t> _dxl_motors_type{niryo_robot_msgs::MotorHeader::MOTOR_TYPE_XL430, niryo_robot_msgs::MotorHeader::MOTOR_TYPE_XL430, niryo_robot_msgs::MotorHeader::MOTOR_TYPE_XL320};

    std::vector<uint8_t> _stepper_motors_id{1, 2, 3};

private:
    ros::NodeHandle _nh;

    double _publish_hw_status_frequency;
    double _publish_software_version_frequency;
    double _publish_learning_mode_frequency;
    double _ros_control_frequency;

    bool _gazebo;
    bool _simu_gripper;

    bool _learning_mode;
    std::string _ros_niryo_robot_version;

    boost::shared_ptr<FakeJointHardwareInterface> _robot;
    boost::shared_ptr<controller_manager::ControllerManager> _cm;

    boost::shared_ptr<std::thread> _ros_control_thread;
    boost::shared_ptr<std::thread> _publish_hardware_status_thread;
    boost::shared_ptr<std::thread> _publish_software_version_thread;
    boost::shared_ptr<std::thread> _publish_learning_mode_thread;

    ros::ServiceServer _reset_controller_server; // workaround to compensate missed steps
    ros::Subscriber _trajectory_result_subscriber;

    ros::ServiceServer _calibrate_motors_server;
    ros::ServiceServer _request_new_calibration_server;

    ros::ServiceServer _activate_learning_mode_server;

    ros::ServiceServer _ping_and_set_dxl_tool_server;
    ros::ServiceServer _open_gripper_server;
    ros::ServiceServer _close_gripper_server;
    ros::ServiceServer _pull_air_vacuum_pump_server;
    ros::ServiceServer _push_air_vacuum_pump_server;

    ros::ServiceServer _ping_and_set_stepper_server;
    ros::ServiceServer _control_conveyor_server;

    ros::Publisher _hardware_status_publisher;
    ros::Publisher _software_version_publisher;
    ros::Publisher _learning_mode_publisher;
    ros::Publisher _current_tools_id_publisher;

    bool _callbackResetController(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
    void _callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult &msg);

    bool _callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
    bool _callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
    bool _callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req, niryo_robot_msgs::SetBool::Response &res);

    bool _callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &req, tools_interface::PingDxlTool::Response &res);
    bool _callbackOpenGripper(tools_interface::OpenGripper::Request &req, tools_interface::OpenGripper::Response &res);
    bool _callbackCloseGripper(tools_interface::CloseGripper::Request &req, tools_interface::CloseGripper::Response &res);
    bool _callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req, tools_interface::PullAirVacuumPump::Response &res);
    bool _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req, tools_interface::PushAirVacuumPump::Response &res);

    bool _callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req, conveyor_interface::SetConveyor::Response &res);
    bool _callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req, conveyor_interface::ControlConveyor::Response &res);

    void _publishLearningMode();
};

#endif
