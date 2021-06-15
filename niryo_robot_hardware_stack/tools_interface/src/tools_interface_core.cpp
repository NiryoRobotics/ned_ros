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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// c++
#include <functional>
#include <future>

//ros

//niryo
#include "tools_interface/tools_interface_core.hpp"
#include "ttl_driver/ttl_driver.hpp"

#include "common/model/tool_state.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/util/util_defs.hpp"

using namespace ttl_driver;
using namespace common::model;
using namespace std;

namespace tools_interface {

    /**
     * @brief ToolsInterfaceCore::ToolsInterfaceCore
     * @param ttl_driver
     */
    ToolsInterfaceCore::ToolsInterfaceCore(shared_ptr<TtlDriverCore> ttl_driver):
        _ttl_driver_core(ttl_driver)
    {
        initParams();
        initServices();
        initPublishers();


        pubToolId(0);
    }

    /**
     * @brief ToolsInterfaceCore::~ToolsInterfaceCore
     */
    ToolsInterfaceCore::~ToolsInterfaceCore()
    {
        if(_publish_tool_connection_thread.joinable())
            _publish_tool_connection_thread.join();
    }

    /**
     * @brief ToolsInterfaceCore::initParams
     */
    void ToolsInterfaceCore::initParams()
    {
        vector<int> idList;
        vector<string> typeList;

        _available_tools_map.clear();
        _nh.getParam("/niryo_robot_hardware_interface/tools_interface/tools_params/id_list", idList);
        _nh.getParam("/niryo_robot_hardware_interface/tools_interface/tools_params/type_list", typeList);

        _nh.getParam("/niryo_robot_hardware_interface/tools_interface/check_tool_connection_frequency", _check_tool_connection_frequency);

        //debug - display info
        ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < idList.size() && i < typeList.size() ; ++i) {
            ss << " id " << idList.at(i) << ": " << typeList.at(i) << ",";
        }

        string available_tools_list = ss.str();
        available_tools_list.pop_back(); //remove last ","
        available_tools_list += "]";

        ROS_INFO("ToolsInterfaceCore::initParams - List of tool ids : %s", available_tools_list.c_str());

        //check that the two lists have the same size
        if(idList.size() != typeList.size())
            ROS_ERROR("ToolsInterfaceCore::initParams - wrong dynamixel configuration. Please check your configuration file (tools_interface/config/default.yaml)");

        //put everything in maps
        for(size_t i = 0; i < idList.size(); ++i) {

            uint8_t id = static_cast<uint8_t>(idList.at(i));
            EMotorType type = MotorTypeEnum(typeList.at(i).c_str());

            if(!_available_tools_map.count(id))
            {
                if(EMotorType::UNKNOWN != type)
                    _available_tools_map.insert(make_pair(id, type));
                else
                    ROS_ERROR("ToolsInterfaceCore::initParams - unknown type %s. Please check your configuration file (tools_interface/config/default.yaml)", typeList.at(id).c_str());
            }
            else
                ROS_ERROR("ToolsInterfaceCore::initParams - duplicate id %d. Please check your configuration file (tools_interface/config/default.yaml)", id);

        }

        for(auto const &tool : _available_tools_map)
        {
            ROS_DEBUG("ToolsInterfaceCore::initParams - Available tools map: %d => %s",
                                            static_cast<int>(tool.first),
                                            MotorTypeEnum(tool.second).toString().c_str());
        }
    }

    /**
     * @brief ToolsInterfaceCore::initServices
     */
    void ToolsInterfaceCore::initServices()
    {
        _ping_and_set_dxl_tool_server = _nh.advertiseService("niryo_robot/tools/ping_and_set_dxl_tool", &ToolsInterfaceCore::_callbackPingAndSetDxlTool, this);
        _open_gripper_server = _nh.advertiseService("niryo_robot/tools/open_gripper", &ToolsInterfaceCore::_callbackOpenGripper, this);
        _close_gripper_server = _nh.advertiseService("niryo_robot/tools/close_gripper", &ToolsInterfaceCore::_callbackCloseGripper, this);
        _pull_air_vacuum_pump_server = _nh.advertiseService("niryo_robot/tools/pull_air_vacuum_pump", &ToolsInterfaceCore::_callbackPullAirVacuumPump, this);
        _push_air_vacuum_pump_server = _nh.advertiseService("niryo_robot/tools/push_air_vacuum_pump", &ToolsInterfaceCore::_callbackPushAirVacuumPump, this);

    }

    /**
     * @brief ToolsInterfaceCore::initPublishers
     */
    void ToolsInterfaceCore::initPublishers()
    {
        _tool_connection_publisher = _nh.advertise<std_msgs::Int32>("/niryo_robot_hardware/tools/current_id", 1, true);
        _publish_tool_connection_thread = std::thread(&ToolsInterfaceCore::_publishToolConnection, this);
    }

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
        bool ret = false;

        // Unequip tool
        if(_toolState.isValid())
        {
            _ttl_driver_core->unsetEndEffector(_toolState.getId());
            res.state = ToolState::TOOL_STATE_PING_OK;
            res.id = 0;

            //reset tool as default = no tool
            _toolState.reset();
        }

        // Search new tool
        vector<uint8_t> motor_list = _ttl_driver_core->scanTools();

        for(auto const& m_id : motor_list) {
            if(_available_tools_map.count(m_id))
            {
                _toolState = ToolState("auto", _available_tools_map.at(m_id), m_id);
                break;
            }
        }

        //if new tool has been found
        if(_toolState.isValid())
        {
            // Try 3 times
            for(int tries = 0; tries < 3; tries++)
            {
                ros::Duration(0.05).sleep();
                res.state = _ttl_driver_core->setEndEffector(_toolState.getType(), _toolState.getId());

                //on success, tool is set, we go out of loop
                if (niryo_robot_msgs::CommandStatus::SUCCESS == res.state)
                {
                    pubToolId(_toolState.getId());
                    res.id = _toolState.getId();

                    ros::Duration(0.05).sleep();

                    _ttl_driver_core->update_leds();

                    ROS_INFO("ToolsInterfaceCore::_callbackPingAndSetDxlTool - Set end effector success, return : %d", res.state);

                    ret = true;
                    break;
                }
            }

            //on failure after three tries
            if (niryo_robot_msgs::CommandStatus::SUCCESS != res.state)
            {
                ROS_ERROR("ToolsInterfaceCore::_callbackPingAndSetDxlTool - Fail to set end effector, return : %d", res.state);

                pubToolId(0);

                ros::Duration(0.05).sleep();
                res.state = ToolState::TOOL_STATE_PING_ERROR;
                res.id = 0;
            }
        }
        else //no tool found, no tool set (it is not an error, the tool does not exists)
        {
            ret = true;
            pubToolId(0);

            ros::Duration(0.05).sleep();
            res.state = ToolState::TOOL_STATE_PING_OK;
            res.id = 0;
        }

        return ret;
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
        bool ret = false;

        if( req.id == _toolState.getId() )
        {
            SingleMotorCmd cmd;
            vector<SingleMotorCmd> list_cmd;
            cmd.setId(_toolState.getId());

            // cc for new motors, use profile velocity instead
            //new dxl motors cannot use this command
            cmd.setType(EDxlCommandType::CMD_TYPE_VELOCITY);
            cmd.setParam(req.open_speed);
            list_cmd.emplace_back(cmd);

            cmd.setType(EDxlCommandType::CMD_TYPE_POSITION);
            cmd.setParam(req.open_position);
            list_cmd.emplace_back(cmd);

            cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
            //cmd.setParam(req.open_max_torque); // cc adapt niryo studio and srv for that
            cmd.setParam(1023);
            list_cmd.emplace_back(cmd);
            _ttl_driver_core->addEndEffectorCommandToQueue(list_cmd);

            double dxl_speed = static_cast<double>(req.open_speed * _toolState.getStepsForOneSpeed()); // position . sec-1
            assert(dxl_speed != 0.00);

            double dxl_steps_to_do = abs(static_cast<double>(req.open_position) - _ttl_driver_core->getEndEffectorState(_toolState.getId())); // position
            double seconds_to_wait =  dxl_steps_to_do /  dxl_speed + 0.25; // sec
            ROS_DEBUG("Waiting for %d seconds", static_cast<int>(seconds_to_wait));
            ros::Duration(seconds_to_wait).sleep();

            // set hold torque
            cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
            cmd.setParam(req.open_hold_torque);
            _ttl_driver_core->addEndEffectorCommandToQueue(cmd);

            res.state = ToolState::GRIPPER_STATE_OPEN;
            ROS_DEBUG("Opened !");
            ret = true;
        }

        return ret;
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
        bool ret = false;

        if( req.id == _toolState.getId() )
        {
            SingleMotorCmd cmd;
            vector<SingleMotorCmd> list_cmd;
            cmd.setId(_toolState.getId());

            uint32_t position_command = ( req.close_position < 50) ? 0 : req.close_position - 50;

            // cc for new motors, use profile velocity instead
            //new dxl motors cannot use this command
            cmd.setType(EDxlCommandType::CMD_TYPE_VELOCITY);
            cmd.setParam(req.close_speed);
            list_cmd.emplace_back(cmd);

            cmd.setType(EDxlCommandType::CMD_TYPE_POSITION);
            cmd.setParam(position_command);
            list_cmd.emplace_back(cmd);

            cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
            cmd.setParam(req.close_max_torque); // two's complement of 1536
            list_cmd.emplace_back(cmd);
            _ttl_driver_core->addEndEffectorCommandToQueue(list_cmd);

            // calculate close duration
            // cc to be removed => acknoledge instead
            double dxl_speed = static_cast<double>(req.close_speed * _toolState.getStepsForOneSpeed()); // position . sec-1
            assert(dxl_speed != 0.0);

            double dxl_steps_to_do = abs(static_cast<double>(req.close_position - _ttl_driver_core->getEndEffectorState(_toolState.getId()))); // position
            double seconds_to_wait =  dxl_steps_to_do /  dxl_speed + 0.25; // sec
            ROS_DEBUG("Waiting for %d seconds", static_cast<int>(seconds_to_wait));

            ros::Duration(seconds_to_wait).sleep();

            // set hold torque
            cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
            cmd.setParam(req.close_hold_torque);
            _ttl_driver_core->addEndEffectorCommandToQueue(cmd);

            res.state = ToolState::GRIPPER_STATE_CLOSE;
            ROS_DEBUG("Closed !");

            ret = true;
        }

        return ret;
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
        bool ret = false;

        // check gripper id, in case no ping has been done before, or wrong id given
        if( req.id == _toolState.getId() )
        {
            //to be put in tool state
            uint32_t pull_air_velocity = 1023;
            uint32_t pull_air_position = static_cast<uint32_t>(req.pull_air_position);
            uint32_t pull_air_hold_torque = static_cast<uint32_t>(req.pull_air_hold_torque);

            // set vacuum pump pos, vel and torque
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_VELOCITY, _toolState.getId(), pull_air_velocity));
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION, _toolState.getId(), pull_air_position));
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), 500));

            // set hold torque
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), pull_air_hold_torque));

            res.state = ToolState::VACUUM_PUMP_STATE_PULLED;
            ret = true;
        }

        return ret;
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
        bool ret = false;

        // check gripper id, in case no ping has been done before, or wrong id given
        if( req.id == _toolState.getId() ) {

            //to be defined in the toolstate
            uint32_t push_air_velocity = 1023;
            uint32_t push_air_position = static_cast<uint32_t>(req.push_air_position);

            // set vacuum pump pos, vel and torque
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_VELOCITY, _toolState.getId(), push_air_velocity));
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION, _toolState.getId(), push_air_position));
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), 64000));// two's complement of 1536

            // set torque to 0
            _ttl_driver_core->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), 0));

            res.state = ToolState::VACUUM_PUMP_STATE_PUSHED;

            ret = true;
        }

        return ret;
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
                vector<uint8_t> motor_list = _ttl_driver_core->getRemovedMotorList();

                for(auto const& motor : motor_list)
                {
                    if(_toolState.getId() == motor)
                    {
                        ROS_INFO("Tools Interface - Unset Current Tools");
                        _ttl_driver_core->unsetEndEffector(_toolState.getId());
                        _toolState.reset();
                        msg.data = 0;
                        _tool_connection_publisher.publish(msg);
                    }
                }
            }
            check_connection_rate.sleep();
        }
    }
} //ToolsInterface
