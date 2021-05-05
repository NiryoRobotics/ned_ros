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

#include <functional>
#include <future>

#include "tools_interface/tools_interface_core.hpp"
#include "dynamixel_driver/dxl_driver.hpp"
#include "model/tool_state.hpp"
#include "model/dxl_command_type_enum.hpp"

#include "util/util_defs.hpp"


using namespace DynamixelDriver;
using namespace common::model;
using namespace std;

namespace ToolsInterface {

    ToolsInterfaceCore::ToolsInterfaceCore(shared_ptr<DxlDriverCore> dynamixel):
        _dynamixel(dynamixel)
    {
        initParams();
        initServices();

        _check_tool_connection_thread = std::thread(&ToolsInterfaceCore::_checkToolConnection, this);

        pubToolId(0);
    }

    ToolsInterfaceCore::~ToolsInterfaceCore()
    {
        if(_check_tool_connection_thread.joinable())
            _check_tool_connection_thread.join();
    }

    void ToolsInterfaceCore::initServices()
    {
        _ping_and_set_dxl_tool_server = _nh.advertiseService("niryo_robot/tools/ping_and_set_dxl_tool", &ToolsInterfaceCore::_callbackPingAndSetDxlTool, this);
        _open_gripper_server = _nh.advertiseService("niryo_robot/tools/open_gripper", &ToolsInterfaceCore::_callbackOpenGripper, this);
        _close_gripper_server = _nh.advertiseService("niryo_robot/tools/close_gripper", &ToolsInterfaceCore::_callbackCloseGripper, this);
        _pull_air_vacuum_pump_server = _nh.advertiseService("niryo_robot/tools/pull_air_vacuum_pump", &ToolsInterfaceCore::_callbackPullAirVacuumPump, this);
        _push_air_vacuum_pump_server = _nh.advertiseService("niryo_robot/tools/push_air_vacuum_pump", &ToolsInterfaceCore::_callbackPushAirVacuumPump, this);

        _current_tools_id_publisher = _nh.advertise<std_msgs::Int32>("/niryo_robot_hardware/tools/current_id", 1, true);
    }

    void ToolsInterfaceCore::initParams()
    {
        vector<int> idList;
        vector<string> typeList;

        _available_tools_map.clear();
        _nh.getParam("/niryo_robot_hardware_interface/tools_interface/tools_params/id_list", idList);
        _nh.getParam("/niryo_robot_hardware_interface/tools_interface/tools_params/type_list", typeList);

        _nh.getParam("/niryo_robot_hardware_interface/tools_interface/check_tool_connection_frequency",_check_tool_connection_frequency);

        //debug - display info
        ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < idList.size() && i < typeList.size() ; ++i) {
            ss << " id " << idList.at(i) << ": " << typeList.at(i) << ",";
        }

        string available_tools_list = ss.str();
        available_tools_list.pop_back(); //remove last ","
        available_tools_list += "]";

        ROS_INFO("Tools Interface - List of tool ids : %s", available_tools_list.c_str());

        //check that the two lists have the same size
        if(idList.size() != typeList.size())
            ROS_ERROR("Tools Interface - wrong dynamixel configuration. Please check your configuration file (tools_interface/config/default.yaml)");

        //put everything in maps
        for(size_t i = 0; i < idList.size(); ++i) {

            int id = idList.at(i);
            EMotorType type = MotorTypeEnum(typeList.at(i).c_str());

            if(0 == _available_tools_map.count(id))
            {
                if(EMotorType::MOTOR_TYPE_UNKNOWN != type)
                    _available_tools_map.insert(make_pair(id, type));
                else
                    ROS_ERROR("Tools Interface - unknown type %s. Please check your configuration file (tools_interface/config/default.yaml)", typeList.at(id).c_str());
            }
            else
                ROS_ERROR("Tools Interface - duplicate id %d. Please check your configuration file (tools_interface/config/default.yaml)", id);

        }

        for(auto const &tool : _available_tools_map) {
            ROS_DEBUG("ToolsInterfaceCore::initParams - Available tools map: %d => %s",
                      static_cast<int>(tool.first),
                      MotorTypeEnum(tool.second).toString().c_str());
        }

    }

    void ToolsInterfaceCore::pubToolId(int id)
    {
        std_msgs::Int32 msg;
        msg.data = id;
        _current_tools_id_publisher.publish(msg);
    }

    bool ToolsInterfaceCore::_callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &/*req*/,
                                                        tools_interface::PingDxlTool::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        // Unequipped tool
        if(_toolState.getId() != 0)
        {
            _dynamixel->unsetEndEffector(_toolState.getId());
            res.state = ToolState::TOOL_STATE_PING_OK;
            res.id = 0;
        }

        // Search new tool
        vector<uint8_t> motor_list;
        motor_list = _dynamixel->scanTools();

        for(auto m_id : motor_list) {
            if(_available_tools_map.count(m_id))
            {
                _toolState = ToolState("auto", _available_tools_map.at(m_id), m_id);
                break;
            }
        }

        if(_toolState.isValid())
        {
            // Try 3 times
            int tries = 0;
            bool tool_set = false;
            while (!tool_set && tries < 3)
            {
                tries++;
                ros::Duration(0.05).sleep();
                res.state = _dynamixel->setEndEffector(_toolState.getType(), _toolState.getId());

                if (res.state != niryo_robot_msgs::CommandStatus::SUCCESS) continue;

                pubToolId(_toolState.getId());

                tool_set = true;
                res.id = _toolState.getId();
                ros::Duration(0.05).sleep();
                _dynamixel->update_leds();
                ROS_INFO("Tools Interface - Set End Effector return : %d", res.state);
            }
        }
        else
        {
            //reset tool as default = no tool
            _toolState.reset();

            pubToolId(0);

            ros::Duration(0.05).sleep();
            res.state = ToolState::TOOL_STATE_PING_OK;
            res.id = 0;
        }

        return true;
    }

    bool ToolsInterfaceCore::_callbackOpenGripper(tools_interface::OpenGripper::Request &req,
                                                  tools_interface::OpenGripper::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        if( req.id != _toolState.getId() )
        {
            //return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
            return false;
        }
        SingleMotorCmd cmd;
        vector<SingleMotorCmd> list_cmd;
        cmd.setId(_toolState.getId());
    /*
    // use profile velocity instead

        cmd.setType(DxlCommandType_t::CMD_TYPE_VELOCITY);
        cmd.setParam(req.open_speed);
        list_cmd.push_back(cmd);
    */
        cmd.setType(EDxlCommandType::CMD_TYPE_POSITION);
        cmd.setParam(req.open_position);
        list_cmd.push_back(cmd);

        cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.open_hold_torque);
        list_cmd.push_back(cmd);

        _dynamixel->addEndEffectorCommandToQueue(list_cmd);
        list_cmd.clear();

        double dxl_speed = (double)req.open_speed * DynamixelDriver::XL320Driver::XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.open_position - (double)_dynamixel->getEndEffectorState(_toolState.getId())); // position
        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec
        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set hold torque
        cmd.setId(_toolState.getId());
        cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.open_hold_torque);
        list_cmd.push_back(cmd);
        _dynamixel->addEndEffectorCommandToQueue(list_cmd);

        res.state = ToolState::GRIPPER_STATE_OPEN;

        return niryo_robot_msgs::CommandStatus::SUCCESS;
    }

    bool ToolsInterfaceCore::_callbackCloseGripper(tools_interface::CloseGripper::Request &req, tools_interface::CloseGripper::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        if( req.id != _toolState.getId() )
        {
           // return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
            return false;
        }

        SingleMotorCmd cmd;
        vector<SingleMotorCmd> list_cmd;
        cmd.setId(_toolState.getId());

        int position_command = ( req.close_position < 50) ? 0 : req.close_position - 50;
    /*
    // use profile velocity instead
        cmd.setType(DxlCommandType_t::CMD_TYPE_VELOCITY);
        cmd.setParam(req.close_speed);
        list_cmd.push_back(cmd);
    */
        cmd.setType(EDxlCommandType::CMD_TYPE_POSITION);
        cmd.setParam(position_command);
        list_cmd.push_back(cmd);

        cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.close_max_torque); // two's complement of 1536
        list_cmd.push_back(cmd);

        _dynamixel->addEndEffectorCommandToQueue(list_cmd);
        list_cmd.clear();

        // calculate close duration
        double dxl_speed = (double)req.close_speed * DynamixelDriver::XL320Driver::XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.close_position - (double)_dynamixel->getEndEffectorState(_toolState.getId())); // position
        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec
        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set hold torque
        cmd.setType(EDxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.close_hold_torque);
        list_cmd.push_back(cmd);
        _dynamixel->addEndEffectorCommandToQueue(list_cmd);

        res.state = ToolState::GRIPPER_STATE_CLOSE;

        return true;
    }

    bool ToolsInterfaceCore::_callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req, tools_interface::PullAirVacuumPump::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        // check gripper id, in case no ping has been done before, or wrong id given
        if( req.id != _toolState.getId() )
        {
            //return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
            return false;
        }

        int pull_air_velocity = 1023;

        // set vacuum pump pos, vel and torque
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_VELOCITY, _toolState.getId(), pull_air_velocity));
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION, _toolState.getId(), req.pull_air_position));
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), 500));

        // calculate pull air duration
        double dxl_speed = (double)pull_air_velocity * DynamixelDriver::XL320Driver::XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.pull_air_position - (double)_dynamixel->getEndEffectorState(_toolState.getId())); // position
        double seconds_to_wait = dxl_steps_to_do / dxl_speed; // sec

        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set hold torque
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), req.pull_air_hold_torque));

        res.state = ToolState::VACUUM_PUMP_STATE_PULLED;

        return true;
    }

    bool ToolsInterfaceCore:: _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req, tools_interface::PushAirVacuumPump::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        // check gripper id, in case no ping has been done before, or wrong id given
        if( req.id != _toolState.getId() )
        {
            //return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
            return false;
        }

        int push_air_velocity = 1023;

        // set vacuum pump pos, vel and torque
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_VELOCITY, _toolState.getId(), push_air_velocity));
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION, _toolState.getId(), req.push_air_position));
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), 64000));// two's complement of 1536

        // calculate push air duration
        double dxl_speed = (double)push_air_velocity * (double)DynamixelDriver::XL320Driver::XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.push_air_position - (double)_dynamixel->getEndEffectorState(_toolState.getId())); // position
        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec

        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set torque to 0
        _dynamixel->addEndEffectorCommandToQueue(SingleMotorCmd(EDxlCommandType::CMD_TYPE_EFFORT, _toolState.getId(), 0));

        res.state = ToolState::VACUUM_PUMP_STATE_PUSHED;

        return true;
    }

    void ToolsInterfaceCore::_checkToolConnection()
    {
        ros::Rate check_connection = ros::Rate(_check_tool_connection_frequency);
        std_msgs::Int32 msg;
        while (ros::ok())
        {
            {
                lock_guard<mutex> lck(_tool_mutex);
                vector<uint8_t> motor_list;
                motor_list = _dynamixel->getRemovedMotorList();
                for(auto const& motor : motor_list)
                {
                    if(_toolState.getId() == motor)
                    {
                        ROS_INFO("Tools Interface - Unset Current Tools");
                        _dynamixel->unsetEndEffector(_toolState.getId());
                        _toolState.reset();
                        msg.data = 0;
                        _current_tools_id_publisher.publish(msg);
                    }
                }
            }
            check_connection.sleep();
        }
    }
} //ToolsInterface
