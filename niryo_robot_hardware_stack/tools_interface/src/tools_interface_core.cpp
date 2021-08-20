/*
    tools_interface_core.cpp
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

// c++
#include <functional>
#include <string>
#include <vector>

// ros

// niryo
#include "tools_interface/tools_interface_core.hpp"
#include "ttl_driver/ttl_manager.hpp"

#include "common/model/tool_state.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/util/util_defs.hpp"

using ::std::string;
using ::std::ostringstream;
using ::std::lock_guard;
using ::std::mutex;

using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;
using ::common::model::ToolState;
using ::common::model::EDxlCommandType;
using ::common::model::DxlCommandTypeEnum;
using ::common::model::DxlSingleCmd;

namespace tools_interface
{

/**
 * @brief ToolsInterfaceCore::ToolsInterfaceCore
 * @param nh
 * @param ttl_interface
 */
ToolsInterfaceCore::ToolsInterfaceCore(ros::NodeHandle& nh,
                                       std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface):
    _ttl_interface(ttl_interface)
{
    ROS_DEBUG("ToolsInterfaceCore::ctor");

    init(nh);

    pubToolId(0);
}

/**
 * @brief ToolsInterfaceCore::~ToolsInterfaceCore
 */
ToolsInterfaceCore::~ToolsInterfaceCore()
{
    if (_publish_tool_connection_thread.joinable())
        _publish_tool_connection_thread.join();
}

/**
 * @brief ToolsInterfaceCore::init
 * @param nh
 * @return
 */
bool ToolsInterfaceCore::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("ToolsInterfaceCore::init - Initializing parameters...");
    initParameters(nh);

    ROS_DEBUG("ToolsInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("ToolsInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("ToolsInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief ToolsInterfaceCore::initParameters
 * @param nh
 */
void ToolsInterfaceCore::initParameters(ros::NodeHandle& nh)
{
    std::vector<int> idList;
    std::vector<string> typeList;

    _available_tools_map.clear();
    nh.getParam("tools_params/id_list", idList);
    nh.getParam("tools_params/type_list", typeList);

    nh.getParam("check_tool_connection_frequency",
                 _check_tool_connection_frequency);

    // debug - display info
    ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < idList.size() && i < typeList.size() ; ++i)
    {
        ss << " id " << idList.at(i) << ": " << typeList.at(i) << ",";
    }

    string available_tools_list = ss.str();
    available_tools_list.pop_back();  // remove last ","
    available_tools_list += "]";

    ROS_INFO("ToolsInterfaceCore::initParameters - List of tool ids : %s", available_tools_list.c_str());
    ROS_DEBUG("EndEffectorInterfaceCore::initParameters - check tool connection frequency : %f", _check_tool_connection_frequency);

    // check that the two lists have the same size
    if (idList.size() != typeList.size())
        ROS_ERROR("ToolsInterfaceCore::initParameters - wrong dynamixel configuration. "
                  "Please check your configuration file (tools_interface/config/default.yaml)");

    // put everything in maps
    for (size_t i = 0; i < idList.size(); ++i)
    {
        uint8_t id = static_cast<uint8_t>(idList.at(i));
        EHardwareType type = HardwareTypeEnum(typeList.at(i).c_str());

        if (!_available_tools_map.count(id))
        {
            if (EHardwareType::UNKNOWN != type)
                _available_tools_map.insert(std::make_pair(id, type));
            else
                ROS_ERROR("ToolsInterfaceCore::initParameters - unknown type %s. "
                          "Please check your configuration file (tools_interface/config/default.yaml)",
                          typeList.at(id).c_str());
        }
        else
            ROS_ERROR("ToolsInterfaceCore::initParameters - duplicate id %d. "
                      "Please check your configuration file (tools_interface/config/default.yaml)", id);
    }

    for (auto const &tool : _available_tools_map)
    {
        ROS_DEBUG("ToolsInterfaceCore::initParameters - Available tools map: %d => %s",
                                        static_cast<int>(tool.first),
                                        HardwareTypeEnum(tool.second).toString().c_str());
    }
}

/**
 * @brief ToolsInterfaceCore::startServices
 * @param nh
 */
void ToolsInterfaceCore::startServices(ros::NodeHandle& nh)
{
    _ping_and_set_dxl_tool_server = nh.advertiseService("/niryo_robot/tools/ping_and_set_dxl_tool",
                                                         &ToolsInterfaceCore::_callbackPingAndSetDxlTool, this);

    _open_gripper_server = nh.advertiseService("/niryo_robot/tools/open_gripper",
                                                &ToolsInterfaceCore::_callbackOpenGripper, this);

    _close_gripper_server = nh.advertiseService("/niryo_robot/tools/close_gripper",
                                                 &ToolsInterfaceCore::_callbackCloseGripper, this);

    _pull_air_vacuum_pump_server = nh.advertiseService("/niryo_robot/tools/pull_air_vacuum_pump",
                                                        &ToolsInterfaceCore::_callbackPullAirVacuumPump, this);

    _push_air_vacuum_pump_server = nh.advertiseService("/niryo_robot/tools/push_air_vacuum_pump",
                                                        &ToolsInterfaceCore::_callbackPushAirVacuumPump, this);

    _tool_reboot_server = nh.advertiseService("/niryo_robot/tools/reboot",
                                               &ToolsInterfaceCore::_callbackToolReboot, this);
}

/**
 * @brief ToolsInterfaceCore::startPublishers
 * @param nh
 */
void ToolsInterfaceCore::startPublishers(ros::NodeHandle& nh)
{
    _tool_connection_publisher = nh.advertise<std_msgs::Int32>("/niryo_robot_hardware/tools/current_id", 1, true);
    _publish_tool_connection_thread = std::thread(&ToolsInterfaceCore::_publishToolConnection, this);
}

/**
 * @brief ToolsInterfaceCore::startSubscribers
 * @param nh
 */
void ToolsInterfaceCore::startSubscribers(ros::NodeHandle& /*nh*/)
{
    ROS_DEBUG("No subscribers to start");
}

/**
 * @brief ToolsInterfaceCore::isInitialized
 * @return
 */
bool ToolsInterfaceCore::isInitialized()
{
    return !_available_tools_map.empty();
}

/**
 * @brief ToolsInterfaceCore::pubToolId
 * @param id
 */
void ToolsInterfaceCore::pubToolId(int id)
{
    std_msgs::Int32 msg;
    msg.data = id;
    _tool_connection_publisher.publish(msg);
}

/**
 * @brief ToolsInterfaceCore::_callbackPingAndSetDxlTool
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &/*req*/,
                                                    tools_interface::PingDxlTool::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_PING_ERROR;

    // Unequip tool
    if (_toolState.isValid())
    {
        _ttl_interface->unsetTool(_toolState.getId());
        res.id = 0;

        // reset tool as default = no tool
        _toolState.reset();
    }

    // Search new tool
    // CC add retries ?
    std::vector<uint8_t> motor_list = _ttl_interface->scanTools();

    for (auto const& m_id : motor_list)
    {
        if (_available_tools_map.count(m_id))
        {
            _toolState = ToolState("auto", _available_tools_map.at(m_id), m_id);
            break;
        }
    }

    // if new tool has been found
    if (_toolState.isValid())
    {
        // Try 3 times
        for (int tries = 0; tries < 3; tries++)
        {
            ros::Duration(0.05).sleep();
            int result = _ttl_interface->setTool(_toolState.getType(), _toolState.getId());

            // on success, tool is set, we go out of loop
            if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
            {
                pubToolId(_toolState.getId());
                res.state = ToolState::TOOL_STATE_PING_OK;
                res.id = _toolState.getId();

                ros::Duration(0.05).sleep();

                // cc put it in setTool ? only used here
                _ttl_interface->update_leds();

                ROS_INFO("ToolsInterfaceCore::_callbackPingAndSetDxlTool - Set end effector success");

                break;
            }
            else
            {
                ROS_WARN("ToolsInterfaceCore::_callbackPingAndSetDxlTool - "
                         "Set end effector failure, return : %d. Retrying (%d)...",
                         result, tries);
            }
        }

        // on failure after three tries
        if (ToolState::TOOL_STATE_PING_OK != res.state)
        {
            ROS_ERROR("ToolsInterfaceCore::_callbackPingAndSetDxlTool - Fail to set end effector, return : %d",
                      res.state);

            pubToolId(0);

            ros::Duration(0.05).sleep();
            res.id = 0;
        }
    }
    else  // no tool found, no tool set (it is not an error, the tool does not exists)
    {
        pubToolId(0);

        ros::Duration(0.05).sleep();
        res.state = ToolState::TOOL_STATE_PING_OK;
        res.id = 0;
    }

    return (ToolState::TOOL_STATE_PING_OK == res.state);
}

/**
 * @brief ToolsInterfaceCore::_callbackToolReboot
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackToolReboot(std_srvs::Trigger::Request &/*req*/, std_srvs::Trigger::Response &res)
{
    res.success = false;

    if (_toolState.isValid())
    {
        std::lock_guard<std::mutex> lck(_tool_mutex);
        res.success = _ttl_interface->rebootMotor(_toolState.getId());
        res.message = (res.success) ? "Tool reboot succeeded" : "Tool reboot failed";
    }
    else
    {
        res.success = true;
        res.message = "No Tool";
    }

    return res.success;
}

/**
 * @brief ToolsInterfaceCore::_callbackOpenGripper
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackOpenGripper(tools_interface::OpenGripper::Request &req,
                                              tools_interface::OpenGripper::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;
    uint8_t tool_id = _toolState.getId();

    if (_toolState.isValid() && req.id == tool_id)
    {
        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_VELOCITY,
                                                                                   tool_id, std::initializer_list<uint32_t>{req.open_speed}));

        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION,
                                                                                    tool_id,  std::initializer_list<uint32_t>{req.open_position}));

        // cmd.setParam(req.open_max_torque);  // cc adapt niryo studio and srv for that
        // need to set max torque. It makes cmd changing speed take effect
        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                    tool_id,  std::initializer_list<uint32_t>{req.open_max_torque}));

        double dxl_speed = static_cast<double>(req.open_speed * _toolState.getStepsForOneSpeed());  // position . sec-1
        assert(dxl_speed != 0.00);

        double dxl_steps_to_do = std::abs(static_cast<double>(req.open_position) - _ttl_interface->getPosition(tool_id));

        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed + 0.25;  // sec
        ROS_DEBUG("Waiting for %d seconds", static_cast<int>(seconds_to_wait));
        ros::Duration(seconds_to_wait).sleep();

        // set hold torque
        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                    tool_id,  std::initializer_list<uint32_t>{req.open_hold_torque}));

        res.state = ToolState::GRIPPER_STATE_OPEN;
        ROS_DEBUG("Opened !");
    }

    return (ToolState::GRIPPER_STATE_OPEN == res.state);
}

/**
 * @brief ToolsInterfaceCore::_callbackCloseGripper
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackCloseGripper(tools_interface::CloseGripper::Request &req,
                                               tools_interface::CloseGripper::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;
    uint8_t tool_id = _toolState.getId();

    if (_toolState.isValid() && req.id == tool_id)
    {
        uint32_t position_command = (req.close_position < 50) ? 0 : req.close_position - 50;

        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_VELOCITY,
                                                                                   tool_id, std::initializer_list<uint32_t>{req.close_speed}));

        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION,
                                                                                    tool_id, std::initializer_list<uint32_t>{position_command}));

        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                    tool_id, std::initializer_list<uint32_t>{req.close_max_torque}));

        // calculate close duration
        // cc to be removed => acknoledge instead
        double dxl_speed = static_cast<double>(req.close_speed * _toolState.getStepsForOneSpeed());  // position . sec-1
        assert(dxl_speed != 0.0);

        // position
        double dxl_steps_to_do = std::abs(static_cast<double>(req.close_position) -
                                          _ttl_interface->getPosition(tool_id));
        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed + 0.25;  // sec
        ROS_DEBUG("Waiting for %d seconds", static_cast<int>(seconds_to_wait));

        ros::Duration(seconds_to_wait).sleep();

        // set hold torque
        _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                    tool_id, std::initializer_list<uint32_t>{req.close_hold_torque}));

        res.state = ToolState::GRIPPER_STATE_CLOSE;
        ROS_DEBUG("Closed !");
    }

    return (ToolState::GRIPPER_STATE_CLOSE == res.state);
}

/**
 * @brief ToolsInterfaceCore::_callbackPullAirVacuumPump
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req,
                                                    tools_interface::PullAirVacuumPump::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;
    uint8_t tool_id = _toolState.getId();

    // check gripper id, in case no ping has been done before, or wrong id given
    if (_toolState.isValid() && req.id == _toolState.getId())
    {
        // to be put in tool state
        uint32_t pull_air_velocity = static_cast<uint32_t>(req.pull_air_velocity);
        uint32_t pull_air_position = static_cast<uint32_t>(req.pull_air_position);
        uint32_t pull_air_hold_torque = static_cast<uint32_t>(req.pull_air_hold_torque);
        uint32_t pull_air_max_torque = static_cast<uint32_t>(req.pull_air_max_torque);
        // set vacuum pump pos, vel and torque
        if (_ttl_interface)
        {
            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_VELOCITY,
                                                                                        tool_id, std::initializer_list<uint32_t>{pull_air_velocity}));

            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION,
                                                                                        tool_id, std::initializer_list<uint32_t>{pull_air_position}));

            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                        tool_id, std::initializer_list<uint32_t>{pull_air_max_torque}));

            // calculate close duration
            // cc to be removed => acknoledge instead
            double dxl_speed = static_cast<double>(req.pull_air_velocity * _toolState.getStepsForOneSpeed());  // position . sec-1
            assert(dxl_speed != 0.0);

            // position
            double dxl_steps_to_do = std::abs(static_cast<double>(req.pull_air_position) -
                                            _ttl_interface->getPosition(tool_id));
            double seconds_to_wait =  dxl_steps_to_do /  dxl_speed + 0.5;  // sec
            ROS_DEBUG("Waiting for %d seconds", static_cast<int>(seconds_to_wait));

            ros::Duration(seconds_to_wait).sleep();

            // set hold torque
            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                        tool_id, std::initializer_list<uint32_t>{pull_air_hold_torque}));
        }

        res.state = ToolState::VACUUM_PUMP_STATE_PULLED;
    }

    return (ToolState::VACUUM_PUMP_STATE_PULLED == res.state);
}

/**
 * @brief ToolsInterfaceCore::_callbackPushAirVacuumPump
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore:: _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req,
                                                     tools_interface::PushAirVacuumPump::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;
    uint8_t tool_id = _toolState.getId();

    // check gripper id, in case no ping has been done before, or wrong id given
    if (_toolState.isValid() && req.id == _toolState.getId())
    {
        // to be defined in the toolstate
        uint32_t push_air_velocity = static_cast<uint32_t>(req.push_air_velocity);
        uint32_t push_air_position = static_cast<uint32_t>(req.push_air_position);
        uint32_t push_air_max_torque = static_cast<uint32_t>(req.push_air_max_torque);

        // set vacuum pump pos, vel and torque
        if (_ttl_interface)
        {
            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_VELOCITY,
                                                                                        tool_id, std::initializer_list<uint32_t>{push_air_velocity}));

            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION,
                                                                                        tool_id, std::initializer_list<uint32_t>{push_air_position}));

            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                        tool_id, std::initializer_list<uint32_t>{push_air_max_torque}));

            // cc to be removed => acknoledge instead
            double dxl_speed = static_cast<double>(req.push_air_velocity * _toolState.getStepsForOneSpeed());  // position . sec-1
            assert(dxl_speed != 0.0);

            // position
            double dxl_steps_to_do = std::abs(static_cast<double>(req.push_air_position) -
                                            _ttl_interface->getPosition(tool_id));
            double seconds_to_wait =  dxl_steps_to_do /  dxl_speed + 0.25;  // sec
            ROS_DEBUG("Waiting for %d seconds", static_cast<int>(seconds_to_wait));

            ros::Duration(seconds_to_wait).sleep();

            // set torque to 0
            _ttl_interface->addSingleCommandToQueue(std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT,
                                                                                        tool_id, std::initializer_list<uint32_t>{0}));
        }
        res.state = ToolState::VACUUM_PUMP_STATE_PUSHED;
    }

    return (ToolState::VACUUM_PUMP_STATE_PUSHED == res.state);
}

/**
 * @brief ToolsInterfaceCore::_publishToolConnection
 */
void ToolsInterfaceCore::_publishToolConnection()
{
    ros::Rate check_connection_rate = ros::Rate(_check_tool_connection_frequency);
    std_msgs::Int32 msg;

    while (ros::ok())
    {
        {
            lock_guard<mutex> lck(_tool_mutex);
            std::vector<uint8_t> motor_list = _ttl_interface->getRemovedMotorList();

            for (auto const& motor : motor_list)
            {
                if (_toolState.getId() == motor)
                {
                    ROS_INFO("Tools Interface - Unset Current Tools");
                    _ttl_interface->unsetTool(_toolState.getId());
                    _toolState.reset();
                    msg.data = 0;
                    _tool_connection_publisher.publish(msg);
                }
            }
        }
        check_connection_rate.sleep();
    }
}
}  // namespace tools_interface
