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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "niryo_robot_hardware_interface/hardware_interface.hpp"

#include "common/model/hardware_type_enum.hpp"

#include "common/util/util_defs.hpp"

using ::common::model::EBusProtocol;

namespace niryo_robot_hardware_interface
{

// List of topics to cancel goals, when the motors are stopped
static const char* ACTION_CANCEL_TOPICS[] = {"/niryo_robot_arm_commander/robot_action/cancel",
                                             "/niryo_robot_programs_manager_v2/execute_program/cancel",
                                             "/niryo_robot_tools_commander/action_server/cancel"};


/**
 * @brief HardwareInterface::HardwareInterface
 * @param nh
 */
HardwareInterface::HardwareInterface(ros::NodeHandle &nh) : _nh(nh)
{
    /*for (int i = 0; i < 5; ++i)
    {
      ros::Duration(1).sleep();
      ROS_WARN("sleeping for %d", i);
    }*/
    init(nh);
}

/**
 * @brief HardwareInterface::init
 * @param nh
 * @return
 */
bool HardwareInterface::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("HardwareInterface::init - Initializing parameters...");
    initParameters(nh);

    ROS_DEBUG("HardwareInterface::init - Init Nodes...");
    initNodes(nh);

    ROS_DEBUG("HardwareInterface::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("HardwareInterface::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("HardwareInterface::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief HardwareInterface::initParameters
 * @param nh
 */
void HardwareInterface::initParameters(ros::NodeHandle &nh)
{
    double hw_status_frequency{1.0};
    double sw_version_frequency{1.0};
    nh.getParam("publish_hw_status_frequency", hw_status_frequency);
    nh.getParam("publish_software_version_frequency", sw_version_frequency);

    ROS_DEBUG("HardwareInterface::initParameters - publish_hw_status_frequency : %f", hw_status_frequency);
    ROS_DEBUG("HardwareInterface::initParameters - publish_software_version_frequency : %f", sw_version_frequency);

    assert(hw_status_frequency);
    assert(sw_version_frequency);

    _hw_status_publisher_duration = ros::Duration(1.0 / hw_status_frequency);
    _sw_version_publisher_duration = ros::Duration(1.0 / sw_version_frequency);

    nh.getParam("/niryo_robot/info/image_version", _rpi_image_version);
    nh.getParam("/niryo_robot/info/ros_version", _ros_niryo_robot_version);
    nh.getParam("hardware_version", _hardware_version);

    nh.getParam("gazebo", _gazebo);

    nh.getParam("can_enabled", _can_enabled);
    nh.getParam("ttl_enabled", _ttl_enabled);

    _rpi_image_version.erase(_rpi_image_version.find_last_not_of(" \n\r\t") + 1);
    _ros_niryo_robot_version.erase(_ros_niryo_robot_version.find_last_not_of(" \n\r\t") + 1);

    std::string conveyor_bus_str = "can";
    nh.getParam("conveyor/bus", conveyor_bus_str);
    if ("can" == conveyor_bus_str)
        _conveyor_bus = EBusProtocol::CAN;
    else if ("ttl" == conveyor_bus_str)
        _conveyor_bus = EBusProtocol::TTL;
    else
        _conveyor_bus = EBusProtocol::UNKNOWN;

    // end effector is enabled if an id is defined
    _end_effector_enabled = nh.hasParam("end_effector_interface/end_effector_id");

    ROS_DEBUG("HardwareInterface::initParameters - image_version : %s", _rpi_image_version.c_str());
    ROS_DEBUG("HardwareInterface::initParameters - ros_version : %s", _ros_niryo_robot_version.c_str());

    ROS_DEBUG("HardwareInterface::initParameters - gazebo : %s", _gazebo ? "true" : "false");

    ROS_DEBUG("HardwareInterface::initParameters - can_enabled : %s", _can_enabled ? "true" : "false");
    ROS_DEBUG("HardwareInterface::initParameters - ttl_enabled : %s", _ttl_enabled ? "true" : "false");
    ROS_DEBUG("HardwareInterface::initParameters - end_effector_enabled : %s", _end_effector_enabled ? "true" : "false");
}

/**
 * @brief HardwareInterface::initNodes
 * @param nh
 */
void HardwareInterface::initNodes(ros::NodeHandle &nh)
{
    ROS_DEBUG("HardwareInterface::initNodes - Init Nodes");

    ROS_DEBUG("HardwareInterface::initNodes - Start CPU Interface Node");
    ros::NodeHandle nh_cpu(nh, "cpu_interface");
    _cpu_interface = std::make_shared<cpu_interface::CpuInterfaceCore>(nh_cpu);
    ros::Duration(0.25).sleep();

    if (_ttl_enabled)
    {
        ROS_DEBUG("HardwareInterface::initNodes - Start Dynamixel Driver Node");
        ros::NodeHandle nh_ttl(nh, "ttl_driver");
        _ttl_interface = std::make_shared<ttl_driver::TtlInterfaceCore>(nh_ttl);
        ros::Duration(0.25).sleep();

        ROS_DEBUG("HardwareInterface::initNodes - Start Tools Interface Node");
        ros::NodeHandle nh_tool(nh, "tools_interface");
        _tools_interface = std::make_shared<tools_interface::ToolsInterfaceCore>(nh_tool, _ttl_interface);
        ros::Duration(0.25).sleep();

        if (_end_effector_enabled)
        {
            ROS_DEBUG("HardwareInterface::initNodes - Start End Effector Interface Node");
            ros::NodeHandle nh_ee(nh, "end_effector_interface");
            _end_effector_interface = std::make_shared<end_effector_interface::EndEffectorInterfaceCore>(nh_ee, _ttl_interface);
        }
        ros::Duration(0.25).sleep();
    }
    else
    {
        ROS_WARN("HardwareInterface::initNodes - DXL communication is disabled for debug purposes");
    }

    if (_can_enabled)
    {
        ROS_DEBUG("HardwareInterface::initNodes - Start CAN Driver Node");
        ros::NodeHandle nh_can(nh, "can_driver");
        _can_interface = std::make_shared<can_driver::CanInterfaceCore>(nh_can);
        ros::Duration(0.25).sleep();
    }
    else
    {
        ROS_DEBUG("HardwareInterface::initNodes - CAN communication is disabled for debug purposes");
    }

    ROS_DEBUG("HardwareInterface::initNodes - Start Joints Interface Node");
    ros::NodeHandle nh_joints(nh, "joints_interface");
    _joints_interface = std::make_shared<joints_interface::JointsInterfaceCore>(nh, nh_joints, _ttl_interface, _can_interface);
    ros::Duration(0.25).sleep();

    ROS_DEBUG("HardwareInterface::initNodes - Start Conveyor Interface Node");
    ros::NodeHandle nh_conveyor(nh, "conveyor");
    _conveyor_interface = std::make_shared<conveyor_interface::ConveyorInterfaceCore>(nh_conveyor, _ttl_interface, _can_interface);
    ros::Duration(0.25).sleep();
}

/**
 * @brief HardwareInterface::startServices
 * @param nh
 */
void HardwareInterface::startServices(ros::NodeHandle &nh)
{
    _motors_report_service = nh.advertiseService("/niryo_robot_hardware_interface/launch_motors_report", &HardwareInterface::_callbackLaunchMotorsReport, this);

    _stop_motors_report_service = nh.advertiseService("/niryo_robot_hardware_interface/stop_motors_report", &HardwareInterface::_callbackStopMotorsReport, this);

    _reboot_motors_service = nh.advertiseService("/niryo_robot_hardware_interface/reboot_motors", &HardwareInterface::_callbackRebootMotors, this);
}

/**
 * @brief HardwareInterface::startPublishers
 */
void HardwareInterface::startPublishers(ros::NodeHandle &nh)
{
    _hw_status_publisher = nh.advertise<niryo_robot_msgs::HardwareStatus>("/niryo_robot_hardware_interface/hardware_status", 10);

    _hw_status_publisher_timer = nh.createTimer(_hw_status_publisher_duration, &HardwareInterface::_publishHardwareStatus, this);

    _sw_version_publisher = nh.advertise<niryo_robot_msgs::SoftwareVersion>("/niryo_robot_hardware_interface/software_version", 10);

    _sw_version_publisher_timer = nh.createTimer(_sw_version_publisher_duration, &HardwareInterface::_publishSoftwareVersion, this);

    // Cancel goals publisher used if an emergency stop is detected
    _cancel_goal_publishers.clear();
    for (auto& cancel_topic_name : ACTION_CANCEL_TOPICS)
    {
      _cancel_goal_publishers.push_back(nh.advertise<actionlib_msgs::GoalID>(cancel_topic_name, 10));
    }
}

/**
 * @brief HardwareInterface::startSubscribers
 * @param nh
 */
void HardwareInterface::startSubscribers(ros::NodeHandle & nh) {
    _robot_status_subscriber = nh.subscribe("/niryo_robot_status/robot_status", 10, &HardwareInterface::_callbackRobotStatus, this);
}

// ********************
//  Callbacks
// ********************

void HardwareInterface::_callbackRobotStatus(const niryo_robot_status::RobotStatus::ConstPtr& msg) {
    if (msg->robot_status == niryo_robot_status::RobotStatus::ESTOP) {
        // Cancel all action goals
        for (auto &pub : _cancel_goal_publishers) {
            pub.publish(actionlib_msgs::GoalID());
        }

        // Pause hardware status reading and control loop
        _joints_interface->setEstopFlag(true);
        _ttl_interface->setEstopFlag(true);
    }

    // Handle emergency stop
    if ((msg->robot_status != niryo_robot_status::RobotStatus::ESTOP)
        && (_previous_robot_status == niryo_robot_status::RobotStatus::ESTOP)
        && _hardware_state != niryo_robot_msgs::HardwareStatus::REBOOT) {
        _ttl_interface->setEstopFlag(false);
        _joints_interface->setEstopFlag(false);

        int32_t status;
        std::string message;
        int ret = rebootMotors(status, message);

        if (!ret)
        {
            ROS_INFO("Hardware Interface - Reboot motors done");
        }
        else
        {
            ROS_ERROR("Hardware Interface - Reboot motors Problems: %s", message.c_str());
        }
    }

    _previous_robot_status = msg->robot_status;
}

/**
 * @brief HardwareInterface::_callbackStopMotorsReport
 * @param req
 * @param res
 * @return
 */
bool HardwareInterface::_callbackStopMotorsReport(niryo_robot_msgs::Trigger::Request & /*req*/, niryo_robot_msgs::Trigger::Response &res)
{
    res.status = niryo_robot_msgs::CommandStatus::FAILURE;
    _hardware_state = niryo_robot_msgs::HardwareStatus::DEBUG;

    ROS_INFO("Hardware Interface - Stop Motor Report");

    if (_can_interface)
        _can_interface->activeDebugMode(false);

    if (_ttl_interface)
        _ttl_interface->activeDebugMode(false);

    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "";

    _hardware_state = niryo_robot_msgs::HardwareStatus::NORMAL;
    return true;
}

/**
 * @brief HardwareInterface::_callbackLaunchMotorsReport
 * @param req
 * @param res
 * @return
 */
bool HardwareInterface::_callbackLaunchMotorsReport(niryo_robot_msgs::Trigger::Request & /*req*/, niryo_robot_msgs::Trigger::Response &res)
{
    res.status = niryo_robot_msgs::CommandStatus::FAILURE;
    _hardware_state = niryo_robot_msgs::HardwareStatus::DEBUG;

    ROS_INFO("Hardware Interface - Start Motors Report");

    int can_status = niryo_robot_msgs::CommandStatus::FAILURE;
    int ttl_status = niryo_robot_msgs::CommandStatus::FAILURE;

    if (_ttl_interface)
    {
        _ttl_interface->activeDebugMode(true);
        ttl_status = _ttl_interface->launchMotorsReport();
        _ttl_interface->activeDebugMode(false);
    }

    if (_can_interface)
    {
        _can_interface->activeDebugMode(true);
        can_status = _can_interface->launchMotorsReport();
        _can_interface->activeDebugMode(false);
        ROS_WARN("Hardware Interface - Motors report ended");

        if ((niryo_robot_msgs::CommandStatus::SUCCESS == can_status) && (niryo_robot_msgs::CommandStatus::SUCCESS == ttl_status))
        {
            res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
            res.message = "Hardware interface seems working properly";
            ROS_INFO("Hardware Interface - Motors report ended");
            return true;
        }
    }
    else
    {
        if (niryo_robot_msgs::CommandStatus::SUCCESS == ttl_status)
        {
            res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
            res.message = "Hardware interface seems working properly";
            ROS_INFO("Hardware Interface - Motors report ended");
            return true;
        }
    }

    res.status = niryo_robot_msgs::CommandStatus::FAILURE;
    res.message = "Steppers status: ";
    res.message += (can_status == niryo_robot_msgs::CommandStatus::SUCCESS) ? "Ok" : "Error";
    res.message += ", Dxl status: ";
    res.message += (ttl_status == niryo_robot_msgs::CommandStatus::SUCCESS) ? "Ok" : "Error";

    ROS_INFO("Hardware Interface - Motors report ended");
    _hardware_state = niryo_robot_msgs::HardwareStatus::NORMAL;

    return true;
}

/**
 * @brief HardwareInterface::_callbackRebootMotors
 * @param req
 * @param res
 * @return
 */
bool HardwareInterface::_callbackRebootMotors(niryo_robot_msgs::Trigger::Request & /*req*/, niryo_robot_msgs::Trigger::Response &res)
{
    rebootMotors(res.status, res.message);
    // no reboot for can interface for now

    return true;
}


/**
 * @brief HardwareInterface::rebootMotors
 * @param status
 * @param message
 * @return Number of hardware interfaces that failed to reboot
 */
int HardwareInterface::rebootMotors(int32_t &status, std::string &message)
{
    status = niryo_robot_msgs::CommandStatus::FAILURE;
    _hardware_state = niryo_robot_msgs::HardwareStatus::REBOOT;

    // for each interface, call for reboot
    int ret = 0;
    bool torque_on = ("ned2" == _hardware_version || "ned3pro" == _hardware_version);

    if ("ned2" == _hardware_version || "ned3pro" == _hardware_version)
        ros::Duration(3).sleep();

    if (_joints_interface && !_joints_interface->rebootAll(torque_on))
    {
        ret++;
        message += "Joints Interface, ";
    }
    if (_tools_interface && !_tools_interface->rebootHardware(torque_on))
    {
        ret++;
        message += "Tools Interface, ";
    }

    if (_end_effector_interface && !_end_effector_interface->rebootHardware())
    {
        ret++;
        message += "End Effector Panel Interface, ";
    }

    if (!ret)
    {
        status = niryo_robot_msgs::CommandStatus::SUCCESS;
        message = "Reboot motors done";
    }
    else
    {
        message = "Reboot motors Problems: " + message;
    }

    _hardware_state = niryo_robot_msgs::HardwareStatus::NORMAL;

    return ret;
}

/**
 * @brief HardwareInterface::_publishHardwareStatus
 * Called every _hw_status_publisher_duration via the _hw_status_publisher_timer
 */
void HardwareInterface::_publishHardwareStatus(const ros::TimerEvent &)
{
    niryo_robot_msgs::BusState ttl_bus_state;
    niryo_robot_msgs::BusState can_bus_state;

    bool need_calibration = false;
    bool calibration_in_progress = false;

    int cpu_temperature = 0;

    niryo_robot_msgs::HardwareStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.connection_up = true;

    std::vector<std::string> motor_names;
    std::vector<int32_t> temperatures;
    std::vector<double> voltages;
    std::vector<std::string> motor_types;

    std::vector<int32_t> hw_errors;
    std::vector<std::string> hw_errors_msg;

    if (_can_interface)
    {
        can_bus_state = _can_interface->getBusState();
        msg.connection_up = msg.connection_up && can_bus_state.connection_status;
    }

    if (_ttl_interface)
    {
        ttl_bus_state = _ttl_interface->getBusState();
        msg.connection_up = msg.connection_up && ttl_bus_state.connection_status;
    }

    if (_joints_interface)
    {
        need_calibration = _joints_interface->needCalibration();
        calibration_in_progress = _joints_interface->isCalibrationInProgress();

        auto joints_states = _joints_interface->getJointsState();
        for (const auto &jState : joints_states)
        {
            motor_names.emplace_back(jState->getName());
            voltages.emplace_back(jState->getVoltage());
            temperatures.emplace_back(jState->getTemperature());
            hw_errors.emplace_back(jState->getHardwareError());
            hw_errors_msg.emplace_back(jState->getHardwareErrorMessage());
            motor_types.emplace_back(common::model::HardwareTypeEnum(jState->getHardwareType()).toString());
        }
    }

    if (_end_effector_interface)
    {
        auto ee_state = _end_effector_interface->getEndEffectorState();
        if (ee_state && ee_state->isValid())
        {
            motor_names.emplace_back("End Effector");
            voltages.emplace_back(ee_state->getVoltage());
            temperatures.emplace_back(ee_state->getTemperature());
            hw_errors.emplace_back(ee_state->getHardwareError());
            hw_errors_msg.emplace_back(ee_state->getHardwareErrorMessage());
            motor_types.emplace_back(common::model::HardwareTypeEnum(ee_state->getHardwareType()).toString());
        }
    }

    if (_tools_interface)
    {
        auto tool_state = _tools_interface->getToolState();
        if (tool_state && tool_state->isValid())
        {
            motor_names.emplace_back("Tool");
            voltages.emplace_back(tool_state->getVoltage());
            temperatures.emplace_back(tool_state->getTemperature());
            hw_errors.emplace_back(tool_state->getHardwareError());
            hw_errors_msg.emplace_back(tool_state->getHardwareErrorMessage());
            motor_types.emplace_back(common::model::HardwareTypeEnum(tool_state->getHardwareType()).toString());
        }
    }

    if (_conveyor_interface)
    {
        auto conveyor_states = _conveyor_interface->getConveyorStates();
        for (const auto &cState : conveyor_states)
        {
            if (cState && cState->isValid())
            {
                motor_names.emplace_back("Conveyor_" + std::to_string(cState->getId()));
                voltages.emplace_back(cState->getVoltage());
                temperatures.emplace_back(cState->getTemperature());
                hw_errors.emplace_back(cState->getHardwareError());
                hw_errors_msg.emplace_back(cState->getHardwareErrorMessage());
                motor_types.emplace_back(common::model::HardwareTypeEnum(cState->getHardwareType()).toString());
            }
        }
    }

    cpu_temperature = _cpu_interface->getCpuTemperature();

    msg.rpi_temperature = cpu_temperature;
    msg.hardware_version = _hardware_version;

    std::string error_message = can_bus_state.error;
    if (!ttl_bus_state.error.empty())
    {
        error_message += "\n";
        error_message += ttl_bus_state.error;
    }

    msg.error_message = error_message;
    msg.hardware_state = _hardware_state;

    msg.calibration_needed = need_calibration;
    msg.calibration_in_progress = calibration_in_progress;

    msg.motor_names = motor_names;
    msg.motor_types = motor_types;

    msg.temperatures = temperatures;
    msg.voltages = voltages;
    msg.hardware_errors = hw_errors;
    msg.hardware_errors_message = hw_errors_msg;

    _hw_status_publisher.publish(msg);
}

/**
 * @brief HardwareInterface::_publishSoftwareVersion
 * Called every _sw_version_publisher_duration via the _sw_version_publisher_timer
 */
void HardwareInterface::_publishSoftwareVersion(const ros::TimerEvent &)
{
    std::vector<std::string> motor_names;
    std::vector<std::string> firmware_versions;

    if (_joints_interface)
    {
        auto joints_states = _joints_interface->getJointsState();
        for (const auto &jState : joints_states)
        {
            motor_names.emplace_back(jState->getName());
            firmware_versions.emplace_back(jState->getFirmwareVersion());
        }
    }

    if (_end_effector_interface)
    {
        auto ee_state = _end_effector_interface->getEndEffectorState();
        if (ee_state && ee_state->isValid())
        {
            motor_names.emplace_back("End Effector");
            firmware_versions.emplace_back(ee_state->getFirmwareVersion());
        }
    }

    if (_tools_interface)
    {
        auto tool_state = _tools_interface->getToolState();
        if (tool_state && tool_state->isValid())
        {
            motor_names.emplace_back("Tool");
            firmware_versions.emplace_back(tool_state->getFirmwareVersion());
        }
    }

    if (_conveyor_interface)
    {
        auto conveyor_states = _conveyor_interface->getConveyorStates();
        for (const auto &cState : conveyor_states)
        {
            if (cState && cState->isValid())
            {
                motor_names.emplace_back("Conveyor");
                firmware_versions.emplace_back(cState->getFirmwareVersion());
            }
        }
    }

    niryo_robot_msgs::SoftwareVersion msg;
    msg.motor_names = motor_names;
    msg.stepper_firmware_versions = firmware_versions;
    msg.rpi_image_version = _rpi_image_version;
    msg.ros_niryo_robot_version = _ros_niryo_robot_version;
    msg.robot_version = _hardware_version;

    _sw_version_publisher.publish(msg);
}

}  // namespace niryo_robot_hardware_interface
