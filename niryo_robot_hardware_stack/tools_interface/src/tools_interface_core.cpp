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
#include <cinttypes>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <map>

// ros

// niryo
#include "tools_interface/tools_interface_core.hpp"
#include "ttl_driver/ttl_manager.hpp"

#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/tool_state.hpp"
#include "common/util/util_defs.hpp"

using ::std::lock_guard;
using ::std::mutex;
using ::std::ostringstream;
using ::std::string;

using ::common::model::DxlSingleCmd;
using ::common::model::EDxlCommandType;
using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;
using ::common::model::ToolState;

namespace tools_interface
{

/**
 * @brief ToolsInterfaceCore::ToolsInterfaceCore
 * @param nh
 * @param ttl_interface
 */
ToolsInterfaceCore::ToolsInterfaceCore(ros::NodeHandle &nh, std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface) : _ttl_interface(std::move(ttl_interface))
{
    ROS_DEBUG("ToolsInterfaceCore::ctor");

    // init tool state with unconnected state
    _toolState = std::make_shared<ToolState>();

    init(nh);
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
 * @brief ToolsInterfaceCore::rebootHardware
 * @param torque_on
 * @return
 */
bool ToolsInterfaceCore::rebootHardware(bool torque_on)
{
    bool res = true;
    if (_toolState && _toolState->isValid())
    {
        // reboot
        bool res = _ttl_interface->rebootHardware(_toolState);

        // re init
        if (res)
            initHardware(torque_on, _temperature_limit, _shutdown_configuration, _available_tools_map.at(_toolState->getId()));
    }
    // else no tool, return ok

    return res;
}

/**
 * @brief ToolsInterfaceCore::initHardware
 */
int ToolsInterfaceCore::initHardware(bool torque_on, uint8_t temperature_limit, uint8_t shutdown_configuration, const ToolConfig &config)
{
    uint8_t motor_id;
    if (_toolState)
    {
        motor_id = _toolState->getId();
        if (EHardwareType::XL330 == _toolState->getHardwareType())
        {
            _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_TORQUE, motor_id, std::initializer_list<uint32_t>{false}));

            _ttl_interface->addSingleCommandToQueue(
                std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_TEMPERATURE_LIMIT, motor_id, std::initializer_list<uint32_t>{temperature_limit}));
            _ttl_interface->addSingleCommandToQueue(
                std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_SHUTDOWN, motor_id, std::initializer_list<uint32_t>{shutdown_configuration}));

            uint8_t control_mode = 5;  // torque + position
            _ttl_interface->addSingleCommandToQueue(
                std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_CONTROL_MODE, motor_id, std::initializer_list<uint32_t>{control_mode}));

            _toolState->setPositionPGain(static_cast<uint32_t>(config.params.pid.at("p")));
            _toolState->setPositionIGain(static_cast<uint32_t>(config.params.pid.at("i")));
            _toolState->setPositionDGain(static_cast<uint32_t>(config.params.pid.at("d")));
            _ttl_interface->addSingleCommandToQueue(
                std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_PID,
                motor_id,
                std::initializer_list<uint32_t>{_toolState->getPositionPGain(),
                _toolState->getPositionIGain(),
                _toolState->getPositionDGain(),
                _toolState->getVelocityPGain(),
                _toolState->getVelocityIGain(),
                _toolState->getFF1Gain(),
                _toolState->getFF2Gain()}));

            _toolState->setVelProfile(config.params.velocity_profile);
            _toolState->setAccProfile(config.params.acceleration_profile);
            _ttl_interface->addSingleCommandToQueue(
                    std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_PROFILE,
                    motor_id,
                    std::initializer_list<uint32_t>{_toolState->getVelProfile(),
                    _toolState->getAccProfile()}));
        }
        // TORQUE cmd on if ned2, off otherwise
        _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_TORQUE, motor_id, std::initializer_list<uint32_t>{torque_on}));
    }
    return niryo_robot_msgs::CommandStatus::SUCCESS;
}

/**
 * @brief ToolsInterfaceCore::initParameters
 * @param nh
 */
void ToolsInterfaceCore::initParameters(ros::NodeHandle &nh)
{
    double tool_connection_frequency{1.0};
    nh.getParam("check_tool_connection_frequency", tool_connection_frequency);
    nh.getParam("vacuum_pump_timeout", _vacuum_pump_timeout);
    nh.getParam("gripper_timeout", _gripper_timeout);


    ROS_DEBUG("ToolsInterfaceCore::initParameters - check tool connection frequency : %f", tool_connection_frequency);

    assert(tool_connection_frequency);
    _tool_connection_publisher_duration = ros::Duration(1.0 / tool_connection_frequency);

    std::vector<int> idList;
    std::vector<string> typeList;
    std::vector<string> nameList;
    std::vector<ToolParams> tool_params;
    XmlRpc::XmlRpcValue params_list;

    _available_tools_map.clear();
    nh.getParam("tools_params/id_list", idList);
    nh.getParam("tools_params/type_list", typeList);
    nh.getParam("tools_params/name_list", nameList);
    nh.getParam("tools_params/temperature_limit", _temperature_limit);
    nh.getParam("tools_params/shutdown_configuration", _shutdown_configuration);

    if (nh.getParam("tools_params/params_list", params_list)) {
        for (int i = 0; i < params_list.size(); ++i) {
            ToolParams params;
            params.velocity_profile = static_cast<int>(params_list[i]["velocity_profile"]);
            params.acceleration_profile = static_cast<int>(params_list[i]["acceleration_profile"]);
            params.pid["p"] = static_cast<int>(params_list[i]["pid"]["p"]);
            params.pid["i"] = static_cast<int>(params_list[i]["pid"]["i"]);
            params.pid["d"] = static_cast<int>(params_list[i]["pid"]["d"]);
            tool_params.push_back(params);
        }
    }

    // check that the three lists have the same size
    if (idList.size() == typeList.size() && idList.size() == nameList.size())
    {
        // put everything in maps
        for (size_t i = 0; i < idList.size(); ++i)
        {
            auto id = static_cast<uint8_t>(idList.at(i));
            EHardwareType type = HardwareTypeEnum(typeList.at(i).c_str());
            std::string name = nameList.at(i);
            int velocity_profile = tool_params.at(i).velocity_profile;
            int acceleration_profile = tool_params.at(i).acceleration_profile;
            std::map<std::string, uint32_t> pid = tool_params.at(i).pid;

            if (!_available_tools_map.count(id))
            {
                if (EHardwareType::UNKNOWN != type)
                {
                    auto tool_config = ToolConfig{
                        name,
                        type,
                        ToolParams{
                            velocity_profile,
                            acceleration_profile,
                            pid
                        }
                    };

                    _available_tools_map.insert(std::make_pair(id, tool_config));
                }
                else
                    ROS_ERROR("ToolsInterfaceCore::initParameters - unknown type %s. "
                              "Please check your configuration file (tools_interface/config/default.yaml)",
                              typeList.at(id).c_str());
            }
            else
                ROS_ERROR("ToolsInterfaceCore::initParameters - duplicate id %d. "
                          "Please check your configuration file (tools_interface/config/default.yaml)",
                          id);
        }
    }
    else
    {
        ROS_ERROR("ToolsInterfaceCore::initParameters - wrong dynamixel configuration. "
                  "Please check your configuration file (tools_interface/config/default.yaml)");
    }

    for (auto const &tool : _available_tools_map)
    {
        ROS_DEBUG("ToolsInterfaceCore::initParameters - Available tools map: %d => %s (%s)", static_cast<int>(tool.first), tool.second.name.c_str(),
                  HardwareTypeEnum(tool.second.type).toString().c_str());
    }
}

/**
 * @brief ToolsInterfaceCore::startServices
 * @param nh
 */
void ToolsInterfaceCore::startServices(ros::NodeHandle &nh)
{
    _ping_and_set_dxl_tool_server = nh.advertiseService("/niryo_robot/tools/ping_and_set_dxl_tool", &ToolsInterfaceCore::_callbackPingAndSetTool, this);

    _open_gripper_server = nh.advertiseService("/niryo_robot/tools/open_gripper", &ToolsInterfaceCore::_callbackOpenGripper, this);

    _close_gripper_server = nh.advertiseService("/niryo_robot/tools/close_gripper", &ToolsInterfaceCore::_callbackCloseGripper, this);

    _pull_air_vacuum_pump_server = nh.advertiseService("/niryo_robot/tools/pull_air_vacuum_pump", &ToolsInterfaceCore::_callbackPullAirVacuumPump, this);

    _push_air_vacuum_pump_server = nh.advertiseService("/niryo_robot/tools/push_air_vacuum_pump", &ToolsInterfaceCore::_callbackPushAirVacuumPump, this);

    _tool_reboot_server = nh.advertiseService("/niryo_robot/tools/reboot", &ToolsInterfaceCore::_callbackToolReboot, this);
}

/**
 * @brief ToolsInterfaceCore::startPublishers
 * @param nh
 */
void ToolsInterfaceCore::startPublishers(ros::NodeHandle &nh)
{
    _tool_connection_publisher = nh.advertise<tools_interface::Tool>("/niryo_robot_hardware/tools/motor", 1, true);

    _tool_connection_publisher_timer = nh.createTimer(_tool_connection_publisher_duration, &ToolsInterfaceCore::_publishToolConnection, this);
}

/**
 * @brief ToolsInterfaceCore::startSubscribers
 * @param nh
 */
void ToolsInterfaceCore::startSubscribers(ros::NodeHandle & /*nh*/) { ROS_DEBUG("No subscribers to start"); }

/**
 * @brief ToolsInterfaceCore::isInitialized
 * @return
 */
bool ToolsInterfaceCore::isInitialized() { return !_available_tools_map.empty(); }

/**
 * @brief ToolsInterfaceCore::_callbackPingAndSetDxlTool
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackPingAndSetTool(tools_interface::PingDxlTool::Request & /*req*/, tools_interface::PingDxlTool::Response &res)
{
    res.tool.id = -1;
    res.tool.motor_type = Tool::NO_MOTOR;

    res.tool.position = 0;
    res.tool.state = ToolState::TOOL_STATE_PING_ERROR;

    res.state = res.tool.state;

    std::lock_guard<mutex> lck(_tool_mutex);
    // Unequip tool
    if (_toolState && _toolState->isValid())
    {
        // unmanage tool
        _ttl_interface->unsetTool(_toolState->getId());

        // reset tool state as default = no tool
        _toolState->reset();
    }

    // Search new tool
    std::vector<uint8_t> motor_list = _ttl_interface->scanTools();

    for (auto const &m_id : motor_list)
    {
        if (_available_tools_map.count(m_id))
        {
            // reset tool state
            _toolState = std::make_shared<ToolState>(_available_tools_map.at(m_id).name, _available_tools_map.at(m_id).type, m_id);
            break;
        }
    }

    // if new tool has been found
    if (_toolState)
    {
        if (_toolState->isValid())
        {
            // Try 3 times
            for (int tries = 0; tries < 3; tries++)
            {
                int result = _ttl_interface->setTool(_toolState);

                // on success, tool is set, we initialize it and go out of loop
                if (niryo_robot_msgs::CommandStatus::SUCCESS == result &&
                    niryo_robot_msgs::CommandStatus::SUCCESS == initHardware(true, _temperature_limit, _shutdown_configuration, _available_tools_map.at(_toolState->getId())))
                {
                    _toolState->setState(ToolState::TOOL_STATE_PING_OK);

                    ros::Duration(0.05).sleep();
                    ROS_INFO("ToolsInterfaceCore::_callbackPingAndSetDxlTool - Set tool success !");

                    break;
                }

                ROS_WARN("ToolsInterfaceCore::_callbackPingAndSetDxlTool - "
                         "Set tool failure, return : %d. Retrying (%d)...",
                         result, tries);
            }

            // on failure after three tries
            if (ToolState::TOOL_STATE_PING_OK != _toolState->getState())
            {
                ROS_ERROR("ToolsInterfaceCore::_callbackPingAndSetDxlTool - Fail to set tool, return : %d", _toolState->getState());

                // reset toolstate to indicate that it is not usable
                _toolState->reset();

                ros::Duration(0.05).sleep();
            }
        }
        else  // no tool found, no tool set (it is not an error, the tool does not exists)
        {
            _toolState->setState(ToolState::TOOL_STATE_PING_OK);

            ros::Duration(0.05).sleep();
        }

        res.state = _toolState->getState();
    }

    return true;
}

/**
 * @brief ToolsInterfaceCore::_callbackToolReboot
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackToolReboot(niryo_robot_msgs::Trigger::Request & /*req*/, niryo_robot_msgs::Trigger::Response &res)
{
    bool success = false;

    if (_toolState && _toolState->isValid())
    {
        success = rebootHardware(true);
        res.status = (success) ? niryo_robot_msgs::CommandStatus::SUCCESS : niryo_robot_msgs::CommandStatus::TOOL_FAILURE;
        res.message = (success) ? "Tool reboot succeeded" : "Tool reboot failed";
    }
    else
    {
        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
        res.message = "No Tool";
    }

    return true;
}

/**
 * @brief ToolsInterfaceCore::_callbackOpenGripper
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackOpenGripper(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;

    if (_toolState && _toolState->isValid() && req.id == _toolState->getId())
    {
        _toolCommand(req.position, req.max_torque, req.speed);

        _waitForToolStop(req.id, _gripper_timeout);

        // set hold torque
        _toolCommand(_toolState->getPosition(), req.hold_torque, req.speed);

        _toolState->setState(ToolState::GRIPPER_STATE_OPEN);

        ROS_DEBUG("ToolsInterfaceCore::_callbackOpenGripper : Opened !");
        res.state = _toolState->getState();
    }

    return true;
}

/**
 * @brief ToolsInterfaceCore::_callbackCloseGripper
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackCloseGripper(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;

    if (_toolState && _toolState->isValid() && req.id == _toolState->getId())
    {
        uint32_t position_command = (req.position < 50) ? 0 : req.position - 50;

        _toolCommand(position_command, req.max_torque, req.speed);

        _waitForToolStop(req.id, _gripper_timeout);

        // set hold torque
        _toolCommand(_toolState->getPosition(), req.hold_torque, req.speed);

        _toolState->setState(ToolState::GRIPPER_STATE_CLOSE);
        ROS_DEBUG("Closed !");

        res.state = _toolState->getState();
    }

    return true;
}

/**
 * @brief ToolsInterfaceCore::_callbackPullAirVacuumPump
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackPullAirVacuumPump(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;

    // check gripper id, in case no ping has been done before, or wrong id given
    if (_toolState)
    {
        if (_toolState->isValid() && req.id == _toolState->getId())
        {
            // to be put in tool state
            auto pull_air_velocity = static_cast<uint32_t>(req.speed);
            auto pull_air_position = static_cast<uint32_t>(req.position);
            int pull_air_hold_torque = req.hold_torque;
            int pull_air_max_torque = req.max_torque;
            // set vacuum pump pos, vel and torque
            if (_ttl_interface)
            {
                _toolCommand(pull_air_position, pull_air_max_torque, pull_air_velocity);

                _waitForToolStop(req.id, _vacuum_pump_timeout);

                // set hold torque
                _toolCommand(_toolState->getPosition(), pull_air_hold_torque, pull_air_velocity);

                _toolState->setState(ToolState::VACUUM_PUMP_STATE_PULLED);
            }
        }
        res.state = _toolState->getState();
    }

    return true;
}

/**
 * @brief ToolsInterfaceCore::_callbackPushAirVacuumPump
 * @param req
 * @param res
 * @return
 */
bool ToolsInterfaceCore::_callbackPushAirVacuumPump(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res)
{
    lock_guard<mutex> lck(_tool_mutex);
    res.state = ToolState::TOOL_STATE_WRONG_ID;

    // check gripper id, in case no ping has been done before, or wrong id given
    if (_toolState)
    {
        if (_toolState->isValid() && req.id == _toolState->getId())
        {
            // to be defined in the toolstate
            auto push_air_velocity = static_cast<uint32_t>(req.speed);
            auto push_air_position = static_cast<uint32_t>(req.position);

            // set vacuum pump pos, vel and torque
            if (_ttl_interface)
            {
                _toolCommand(push_air_position, req.max_torque, push_air_velocity);

                _waitForToolStop(req.id, _vacuum_pump_timeout);

                // set torque to 0
                _toolCommand(_toolState->getPosition(), 0, push_air_velocity);

                _toolState->setState(ToolState::VACUUM_PUMP_STATE_PUSHED);
            }
        }
        res.state = _toolState->getState();
    }

    return true;
}

/**
 * @brief ToolsInterfaceCore::_waitForToolStop
 * @param id
 * @param timeout
*/
void ToolsInterfaceCore::_waitForToolStop(int id, int timeout)
{
    // Wait for moving status to acknowledge that the motion has stopped or return if timeout
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    int moving_counter{10};

    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < timeout)
    {
        uint8_t moving_status;
        _ttl_interface->readMoving(id, moving_status);

        // Count the number of times the tool has been detected as not moving consecutively
        if (moving_status == 0)
        {
            moving_counter--;
        }
        else
        {
            moving_counter = 10;
        }

        // If the tool has been detected as not moving 10 times in a row, we consider that the vacuum pump cannot move anymore
        if (moving_counter <= 0)
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

/**
 * @brief ToolsInterfaceCore::_toolCommand
 * @param position
 * @param torque
 * @param velocity
 * @return
 */
void ToolsInterfaceCore::_toolCommand(uint32_t position, int torque, uint32_t velocity)
{
    uint8_t tool_id = _toolState->getId();
    EHardwareType tool_motor_type = _toolState->getHardwareType();

    if (EHardwareType::XL320 == tool_motor_type)
        _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_VELOCITY, tool_id, std::initializer_list<uint32_t>{velocity}));

    // need to set max torque. It makes cmd changing speed take effect
    _ttl_interface->addSingleCommandToQueue(
        std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_EFFORT, tool_id, std::initializer_list<uint32_t>{static_cast<uint16_t>(torque)}));

    _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION, tool_id, std::initializer_list<uint32_t>{position}));
}

/**
 * @brief ToolsInterfaceCore::_publishToolConnection
 */
void ToolsInterfaceCore::_publishToolConnection(const ros::TimerEvent &)
{
    lock_guard<mutex> lck(_tool_mutex);

    ROS_DEBUG("ToolsInterfaceCore::_publishToolConnection : publishing");

    tools_interface::Tool msg;

    if (_toolState)
    {
        // check that current managed tool is still connected
        if (_toolState->isValid())
        {
            if (_ttl_interface && !_ttl_interface->scanMotorId(_toolState->getId()))
            {
                // try 3 times to ping tool, ttl failed to ping tool sometimes
                if (3 == _tool_ping_failed_cnt)
                {
                    ROS_INFO("Tools Interface - Unset Current Tools");
                    _ttl_interface->unsetTool(_toolState->getId());

                    // reset state
                    _toolState->reset();
                    _tool_ping_failed_cnt = 0;
                }
                else
                    _tool_ping_failed_cnt++;
            }
            else
            {
                _tool_ping_failed_cnt = 0;
            }
        }

        // publish message

        int id = _toolState->getId();
        EHardwareType motor_type = _toolState->getHardwareType();

        msg.id = static_cast<int8_t>(id);
        msg.position = _toolState->getPosition();
        msg.state = _toolState->getState();

        switch (motor_type)
        {
        case EHardwareType::STEPPER:
            msg.motor_type = tools_interface::Tool::STEPPER;
            break;
        case EHardwareType::XL430:
            msg.motor_type = tools_interface::Tool::XL430;
            break;
        case EHardwareType::XC430:
            msg.motor_type = tools_interface::Tool::XC430;
            break;
        case EHardwareType::XM430:
            msg.motor_type = tools_interface::Tool::XM430;
            break;
        case EHardwareType::XL320:
            msg.motor_type = tools_interface::Tool::XL320;
            break;
        case EHardwareType::XL330:
            msg.motor_type = tools_interface::Tool::XL330;
            break;
        case EHardwareType::FAKE_DXL_MOTOR:
            msg.motor_type = tools_interface::Tool::FAKE_DXL_MOTOR;
            break;
        case EHardwareType::FAKE_STEPPER_MOTOR:
            msg.motor_type = tools_interface::Tool::STEPPER;
            break;
        default:
            msg.motor_type = tools_interface::Tool::NO_MOTOR;
            break;
        }
    }

    _tool_connection_publisher.publish(msg);
}
}  // namespace tools_interface

