/*
    fake_interface_core.cpp
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

#include "fake_interface/fake_interface_core.hpp"

// std
#include <functional>
#include <string>
#include <std_msgs/Int32.h>

// niryo
#include "common/model/tool_state.hpp"
#include "common/util/util_defs.hpp"

// CC replace with fake driver ??
namespace fake_interface
{
FakeInterfaceCore::FakeInterfaceCore(ros::NodeHandle& nh)
{
    ROS_DEBUG("Fake Interface Core - ctor");

    init(nh);

    ROS_INFO("Fake Hardware Interface - Started ");
    _robot = std::make_unique<FakeJointHardwareInterface>(nh);

    _learning_mode = true;

    if (!_gazebo && _robot)
    {
        ROS_DEBUG("Fake Hardware Interface - Create controller manager");
        _cm.reset(new controller_manager::ControllerManager(_robot.get(), _nh));
        ros::Duration(0.1).sleep();

        ROS_DEBUG("Fake Hardware Interface - Starting ROS control thread...");
        _control_loop_thread = std::thread(&FakeInterfaceCore::rosControlLoop, this);
    }

    pubToolId(0);

    ros::Duration(0.1).sleep();
}

/**
 * @brief FakeInterfaceCore::~FakeInterfaceCore
 */
FakeInterfaceCore::~FakeInterfaceCore()
{
    if (_publish_learning_mode_thread.joinable())
        _publish_learning_mode_thread.join();

    if (_control_loop_thread.joinable())
        _control_loop_thread.join();
}

/**
 * @brief FakeInterfaceCore::init
 * @param nh
 * @return
 */
bool FakeInterfaceCore::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("FakeInterfaceCore::init - Init parameters...");
    initParameters(nh);

    ROS_DEBUG("FakeInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("FakeInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("FakeInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief FakeInterfaceCore::initParams
 */
void FakeInterfaceCore::initParameters(ros::NodeHandle &nh)
{
    ros::param::get("~gazebo", _gazebo);
    ros::param::get("~simu_gripper", _simu_gripper);
    ros::param::get("~ros_control_loop_frequency", _ros_control_frequency);
    
    ROS_DEBUG("FakeInterfaceCore::initParameters - gazebo ? %s", _gazebo ? "yes" : "no");
    ROS_DEBUG("FakeInterfaceCore::initParameters - simu_gripper ? %s", _simu_gripper ? "yes" : "no");
    ROS_DEBUG("FakeInterfaceCore::initParameters - ros control loop freqeuncy %f", _ros_control_frequency);

    ros::param::get("~publish_hw_status_frequency", _publish_hw_status_frequency);
    ros::param::get("~publish_software_version_frequency", _publish_software_version_frequency);
    ros::param::get("~publish_learning_mode_frequency", _publish_learning_mode_frequency);
    ros::param::get("/niryo_robot/info/ros_version", _ros_niryo_robot_version);

    _ros_niryo_robot_version.erase(_ros_niryo_robot_version.find_last_not_of(" \n\r\t") + 1);

    ROS_DEBUG("FakeInterfaceCore::initParameters - Publish_hw_status_frequency : %f", _publish_hw_status_frequency);
    ROS_DEBUG("FakeInterfaceCore::initParameters - Publish_software_version_frequency : %f", _publish_software_version_frequency);
    ROS_DEBUG("FakeInterfaceCore::initParameters - Publish_learning_mode_frequency : %f", _publish_learning_mode_frequency);
    ROS_DEBUG("FakeInterfaceCore::initParameters - ROS version : %s", _ros_niryo_robot_version.c_str());
}

/**
 * @brief FakeInterfaceCore::startServices
 */
void FakeInterfaceCore::startServices(ros::NodeHandle& nh)
{
    _calibrate_motors_server = _nh.advertiseService("/niryo_robot/joints_interface/calibrate_motors",
                                                    &FakeInterfaceCore::_callbackCalibrateMotors, this);

    _request_new_calibration_server = _nh.advertiseService("/niryo_robot/joints_interface/request_new_calibration",
                                                           &FakeInterfaceCore::_callbackRequestNewCalibration, this);

    _activate_learning_mode_server = _nh.advertiseService("/niryo_robot/learning_mode/activate",
                                                          &FakeInterfaceCore::_callbackActivateLearningMode, this);

    _ping_and_set_dxl_tool_server = _nh.advertiseService("/niryo_robot/tools/ping_and_set_dxl_tool",
                                                         &FakeInterfaceCore::_callbackPingAndSetDxlTool, this);

    _open_gripper_server = _nh.advertiseService("/niryo_robot/tools/open_gripper",
                                                &FakeInterfaceCore::_callbackOpenGripper, this);

    _close_gripper_server = _nh.advertiseService("/niryo_robot/tools/close_gripper",
                                                 &FakeInterfaceCore::_callbackCloseGripper, this);

    _pull_air_vacuum_pump_server = _nh.advertiseService("/niryo_robot/tools/pull_air_vacuum_pump",
                                                        &FakeInterfaceCore::_callbackPullAirVacuumPump, this);

    _push_air_vacuum_pump_server = _nh.advertiseService("/niryo_robot/tools/push_air_vacuum_pump",
                                                        &FakeInterfaceCore::_callbackPushAirVacuumPump, this);

    _ping_and_set_stepper_server = _nh.advertiseService("/niryo_robot/conveyor/ping_and_set_conveyor",
                                                        &FakeInterfaceCore::_callbackPingAndSetConveyor, this);

    _control_conveyor_server = _nh.advertiseService("/niryo_robot/conveyor/control_conveyor",
                                                    &FakeInterfaceCore::_callbackControlConveyor, this);

    _reset_controller_server = _nh.advertiseService("/niryo_robot/joints_interface/steppers_reset_controller",
                                                    &FakeInterfaceCore::_callbackResetController, this);
}

/**
 * @brief FakeInterfaceCore::startPublishers
 * @param nh
 */
void FakeInterfaceCore::startPublishers(ros::NodeHandle &nh)
{
    _current_tools_id_publisher = _nh.advertise<std_msgs::Int32>("/niryo_robot_hardware/tools/current_id", 1, true);

    _learning_mode_publisher = _nh.advertise<std_msgs::Bool>("/niryo_robot/learning_mode/state", 10);
    _publish_learning_mode_thread = std::thread(&FakeInterfaceCore::_publishLearningMode, this);
}

/**
 * @brief FakeInterfaceCore::startSubscribers
 */
void FakeInterfaceCore::startSubscribers(ros::NodeHandle& nh)
{
    _trajectory_result_subscriber = _nh.subscribe(
                "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory/result",
                10, &FakeInterfaceCore::_callbackTrajectoryResult, this);
}

/**
 * @brief FakeInterfaceCore::rosControlLoop
 */
void FakeInterfaceCore::rosControlLoop()
{
    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time;
    ros::Rate control_loop_rate = ros::Rate(_ros_control_frequency);
    while (ros::ok())
    {
        current_time = ros::Time::now();
        elapsed_time = ros::Duration(current_time - last_time);
        _robot->read(current_time, elapsed_time);

        last_time = current_time;
        _cm->update(ros::Time::now(), elapsed_time, false);
        _robot->write(current_time, elapsed_time);
        control_loop_rate.sleep();
    }
}

/**
 * @brief FakeInterfaceCore::pubToolId
 * @param id
 */
void FakeInterfaceCore::pubToolId(int id)
{
    std_msgs::Int32 msg;
    msg.data = id;
    _current_tools_id_publisher.publish(msg);
}

/**
 * @brief FakeInterfaceCore::_callbackResetController
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackResetController(niryo_robot_msgs::Trigger::Request &req,
                                                 niryo_robot_msgs::Trigger::Response &res)
{
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Reset done";
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackTrajectoryResult
 * @param msg
 */
void FakeInterfaceCore::_callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult& msg)
{
    // Nothing To Do
}

/**
 * @brief FakeInterfaceCore::_callbackCalibrateMotors
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req,
                                                 niryo_robot_msgs::SetInt::Response &res)
{
    ROS_INFO("Fake Hardware Interface - Calibrate motors");
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Calibration done";
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackRequestNewCalibration
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &req,
                                                       niryo_robot_msgs::Trigger::Response &res)
{
    ROS_INFO("Fake Hardware Interface - Request New Calibration");
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "New calibration request has been made, you will be requested to confirm it.";
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackActivateLearningMode
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req,
                                                      niryo_robot_msgs::SetBool::Response &res)
{
    bool learning_mode_on;
    std_msgs::Bool msg;

    learning_mode_on = req.value;
    if (learning_mode_on == _learning_mode)
    {
        res.message = (learning_mode_on) ? "Learning mode already activated" : "Learning mode already deactivating";
    }
    else
    {
        if (!learning_mode_on)
        {
            ROS_INFO("Fake Hardware Interface - Deactivate Learning Mode");
            res.message = "Deactivating learning mode";
        }
        else
        {
            ROS_INFO("Fake Hardware Interface - Activate Learning Mode");
            res.message =  "Activating learning mode";
        }
        _learning_mode = learning_mode_on;
        msg.data = _learning_mode;
        _learning_mode_publisher.publish(msg);
    }

    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackPingAndSetDxlTool
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &req,
                                                   tools_interface::PingDxlTool::Response &res)
{
    ROS_DEBUG("Fake Hardware Interface - Ping Dxl Tool");
    if (_simu_gripper)
    {
        res.id = 11;
    }
    else
    {
        res.id = 0;
    }
    pubToolId(res.id);
    res.state = common::model::ToolState::TOOL_STATE_PING_OK;
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackOpenGripper
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackOpenGripper(tools_interface::OpenGripper::Request &req,
                                             tools_interface::OpenGripper::Response &res)
{
    ROS_INFO("Fake Hardware Interface - Open gripper with id : %03d", req.id);
    res.state = common::model::ToolState::GRIPPER_STATE_OPEN;
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackCloseGripper
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackCloseGripper(tools_interface::CloseGripper::Request &req,
                                              tools_interface::CloseGripper::Response &res)
{
    ROS_INFO("Fake Hardware Interface - Close gripper with id : %03d", req.id);
    res.state = common::model::ToolState::GRIPPER_STATE_CLOSE;
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackPullAirVacuumPump
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req,
                                                   tools_interface::PullAirVacuumPump::Response &res)
{
    ROS_INFO("Fake Hardware Interface - Pull air on vacuum pump with id : %03d", req.id);
    res.state = common::model::ToolState::VACUUM_PUMP_STATE_PULLED;
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackPushAirVacuumPump
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore:: _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req,
                                                    tools_interface::PushAirVacuumPump::Response &res)
{
    ROS_INFO("Fake Hardware Interface - Push air on vacuum pump with id : %03d", req.id);
    res.state = common::model::ToolState::VACUUM_PUMP_STATE_PUSHED;
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackPingAndSetConveyor
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req,
                                                    conveyor_interface::SetConveyor::Response &res)
{
    uint8_t new_id = 12;
    ROS_INFO("Fake Hardware Interface - Set conveyor with id : %d", new_id);
    std::string message = "";
    message = "Set Conveyor id ";
    message += std::to_string(new_id);
    message += " OK";
    res.message = message;
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.id = new_id;
    return true;
}

/**
 * @brief FakeInterfaceCore::_callbackControlConveyor
 * @param req
 * @param res
 * @return
 */
bool FakeInterfaceCore::_callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req,
                                                 conveyor_interface::ControlConveyor::Response &res)
{
    ROS_INFO("Fake Hardware Interface - Set command conveyor with id : %03d", req.id);
    std::string message = "";
    message = "Set command on conveyor id ";
    message += std::to_string(req.id);
    message += " is OK";
    res.message = message;
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    return true;
}

/**
 * @brief FakeInterfaceCore::getTtlHwStatus
 * @return
 */
ttl_driver::DxlArrayMotorHardwareStatus FakeInterfaceCore::getTtlHwStatus() const
{
    ttl_driver::DxlMotorHardwareStatus data;
    ttl_driver::DxlArrayMotorHardwareStatus hw_state;
    for (size_t i = 0; i < 3; i++)
    {
        data.motor_identity.motor_id = _dxl_motors_id.at(i);
        data.motor_identity.motor_type = _dxl_motors_type.at(i);
        data.temperature = 0;
        data.voltage = 0;
        data.error = 0;
        hw_state.motors_hw_status.push_back(data);
    }
    return hw_state;
}

/**
 * @brief FakeInterfaceCore::getTtlBusState
 * @return
 */
niryo_robot_msgs::BusState FakeInterfaceCore::getTtlBusState() const
{
    niryo_robot_msgs::BusState dxl_bus_state;

    dxl_bus_state.connection_status = true;
    dxl_bus_state.motor_id_connected = _dxl_motors_id;
    dxl_bus_state.error = "";
    return dxl_bus_state;
}

/**
 * @brief FakeInterfaceCore::getCanHwStatus
 * @return
 */
can_driver::StepperArrayMotorHardwareStatus FakeInterfaceCore::getCanHwStatus() const
{
    can_driver::StepperMotorHardwareStatus data;
    can_driver::StepperArrayMotorHardwareStatus hw_state;

    for (size_t i = 0; i < 3; i++)
    {
        data.motor_identity.motor_id = _stepper_motors_id.at(i);
        data.motor_identity.motor_type = niryo_robot_msgs::MotorHeader::MOTOR_TYPE_STEPPER;
        data.temperature = 0;
        data.error = 0;
        data.firmware_version = "";
        hw_state.motors_hw_status.push_back(data);
    }
    return hw_state;
}

/**
 * @brief FakeInterfaceCore::getCanBusState
 * @return
 */
niryo_robot_msgs::BusState FakeInterfaceCore::getCanBusState()
{
    niryo_robot_msgs::BusState can_bus_state;

    can_bus_state.connection_status = true;
    can_bus_state.motor_id_connected = _stepper_motors_id;
    can_bus_state.error = "";
    return can_bus_state;
}

/**
 * @brief FakeInterfaceCore::getCalibrationState
 * @param need_calibration
 * @param calibration_in_progress
 */
void FakeInterfaceCore::getCalibrationState(bool &need_calibration, bool &calibration_in_progress) const
{
    need_calibration = false;
    calibration_in_progress = false;
}

/**
 * @brief FakeInterfaceCore::getCpuTemperature
 * @return
 */
int FakeInterfaceCore::getCpuTemperature() const
{
    return 0;
}

/**
 * @brief FakeInterfaceCore::_publishLearningMode
 */
void FakeInterfaceCore::_publishLearningMode()
{
    ros::Rate publish_learning_mode_rate = ros::Rate(_publish_learning_mode_frequency);
    while (ros::ok())
    {
        std_msgs::Bool msg;
        msg.data = _learning_mode;
        _learning_mode_publisher.publish(msg);
        publish_learning_mode_rate.sleep();
    }
}

/**
 * @brief FakeInterfaceCore::jointIdToJointName
 * @param id
 * @param motor_type
 * @return
 */
std::string FakeInterfaceCore::jointIdToJointName(uint8_t id, uint8_t motor_type) const
{
    return _robot->jointIdToJointName(id, static_cast<common::model::EMotorType>(motor_type));
}

}  // namespace fake_interface
