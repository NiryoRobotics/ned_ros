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

#include "tools_interface/tools_interface_core.hpp"
#include "dynamixel_driver/dxl_driver.hpp"
#include <functional>


using namespace DynamixelDriver;
using namespace std;

namespace ToolsInterface {

    ToolsInterfaceCore::ToolsInterfaceCore(shared_ptr<DynamixelDriverCore> dynamixel):
        _dynamixel(dynamixel)
    {
        initParams();
        initServices();

        _check_tool_connection_thread.reset(new thread(&ToolsInterfaceCore::_checkToolConnection, this));

        pubToolId(0);
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
        for (auto i = 0; i < idList.size() && i < typeList.size() ; ++i) {
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
        for(auto i = 0; i < idList.size(); ++i) {

            int id = idList.at(i);
            DxlMotorType_t type = DxlMotorType::fromString(typeList.at(i));

            if(0 == _available_tools_map.count(id))
            {
                if(DxlMotorType_t::MOTOR_TYPE_UNKNOWN != type)
                    _available_tools_map.insert(make_pair(id, type));
                else
                    ROS_ERROR("Tools Interface - unknown type %s. Please check your configuration file (tools_interface/config/default.yaml)", typeList.at(id).c_str());
            }
            else
                ROS_ERROR("Tools Interface - duplicate id %d. Please check your configuration file (tools_interface/config/default.yaml)", id);

        }
    }

    void ToolsInterfaceCore::pubToolId(int id)
    {
        std_msgs::Int32 msg;
        msg.data = id;
        _current_tools_id_publisher.publish(msg);
    }

    bool ToolsInterfaceCore::_callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &req, tools_interface::PingDxlTool::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        // Unequipped tool
        if(_toolState.getId() != 0)
        {
            _dynamixel->unsetEndEffector(_toolState.getId(), _toolState.getType());
            res.state = TOOL_STATE_PING_OK;
            res.id = 0;
        }

        // Search new tool
        vector<uint8_t> motor_list;
        motor_list = _dynamixel->scanTools();
        bool tool_found = false;
        uint8_t tool_id;

        for(auto i = 0; i < motor_list.size(); ++i) {
            if(_available_tools_map.count(motor_list.at(i)))
            {
                tool_id = motor_list.at(i);
                tool_found = true;
                break;
            }
        }

        if(tool_found)
        {
            _toolState.reset();

            // Try 3 times
            int tries = 0;
            bool tool_set = false;
            while (!tool_set && tries<3)
            {
                tries++;
                ros::Duration(0.05).sleep();
                res.state = _dynamixel->setEndEffector(_toolState.getId(), _toolState.getType());

                if (res.state != niryo_robot_msgs::CommandStatus::SUCCESS) continue;

                pubToolId(tool_id);

                tool_set = true;
                res.id = tool_id;
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
            res.state = TOOL_STATE_PING_OK;
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
            return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
        }
        SingleMotorCmd cmd;
        vector<SingleMotorCmd> list_cmd;
        cmd.setId(_toolState.getId());
    /*
    // use profile velocity instead

        cmd.setType(DxlCommandType::CMD_TYPE_VELOCITY);
        cmd.setParam(req.open_speed);
        list_cmd.push_back(cmd);
    */
        cmd.setType(DxlCommandType::CMD_TYPE_POSITION);
        cmd.setParam(req.open_position);
        list_cmd.push_back(cmd);

        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.open_hold_torque);
        list_cmd.push_back(cmd);

        _dynamixel->setEndEffectorCommands(list_cmd);
        list_cmd.clear();

        double dxl_speed = (double)req.open_speed * (double)XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.open_position - (double)_dynamixel->getEndEffectorState(_toolState.getId(), _toolState.getType())); // position
        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec
        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set hold torque
        cmd.setId(_toolState.getId());
        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.open_hold_torque);
        list_cmd.push_back(cmd);
        _dynamixel->setEndEffectorCommands(list_cmd);

        res.state = GRIPPER_STATE_OPEN;

        return true;
    }

    bool ToolsInterfaceCore::_callbackCloseGripper(tools_interface::CloseGripper::Request &req, tools_interface::CloseGripper::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        if( req.id != _toolState.getId() )
        {
            return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
        }

        SingleMotorCmd cmd;
        vector<SingleMotorCmd> list_cmd;
        cmd.setId(_toolState.getId());

        int position_command = ( req.close_position < 50) ? 0 : req.close_position - 50;
    /*
    // use profile velocity instead
        cmd.setType(DxlCommandType::CMD_TYPE_VELOCITY);
        cmd.setParam(req.close_speed);
        list_cmd.push_back(cmd);
    */
        cmd.setType(DxlCommandType::CMD_TYPE_POSITION);
        cmd.setParam(position_command);
        list_cmd.push_back(cmd);

        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.close_max_torque); // two's complement of 1536
        list_cmd.push_back(cmd);

        _dynamixel->setEndEffectorCommands(list_cmd);
        list_cmd.clear();

        // calculate close duration
        double dxl_speed = (double)req.close_speed *(double) XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.close_position - (double)_dynamixel->getEndEffectorState(_toolState.getId(), _toolState.getType())); // position
        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec
        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set hold torque
        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.close_hold_torque);
        list_cmd.push_back(cmd);
        _dynamixel->setEndEffectorCommands(list_cmd);

        res.state = GRIPPER_STATE_CLOSE;

        return true;
    }

    bool ToolsInterfaceCore::_callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req, tools_interface::PullAirVacuumPump::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        // check gripper id, in case no ping has been done before, or wrong id given
        if( req.id != _toolState.getId() )
        {
            return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
        }

        int pull_air_velocity = 1023;

        // set vacuum pump pos, vel and torque
        SingleMotorCmd cmd;
        vector<SingleMotorCmd> list_cmd;
        cmd.setId(_toolState.getId());

        cmd.setType(DxlCommandType::CMD_TYPE_VELOCITY);
        cmd.setParam(pull_air_velocity);
        list_cmd.push_back(cmd);

        cmd.setType(DxlCommandType::CMD_TYPE_POSITION);
        cmd.setParam(req.pull_air_position);
        list_cmd.push_back(cmd);

        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(500);
        list_cmd.push_back(cmd);

        _dynamixel->setEndEffectorCommands(list_cmd);
        list_cmd.clear();

        // calculate pull air duration
        double dxl_speed = (double)pull_air_velocity * (double)XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.pull_air_position - (double)_dynamixel->getEndEffectorState(_toolState.getId(), _toolState.getType())); // position
        double seconds_to_wait = dxl_steps_to_do / dxl_speed; // sec

        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set hold torque
        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(req.pull_air_hold_torque);
        list_cmd.push_back(cmd);
        _dynamixel->setEndEffectorCommands(list_cmd);

        res.state = VACUUM_PUMP_STATE_PULLED;

        return true;
    }

    bool ToolsInterfaceCore:: _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req, tools_interface::PushAirVacuumPump::Response &res)
    {
        lock_guard<mutex> lck(_tool_mutex);
        // check gripper id, in case no ping has been done before, or wrong id given
        if( req.id != _toolState.getId() )
        {
            return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
        }

        int push_air_velocity = 1023;

        // set vacuum pump pos, vel and torque
        SingleMotorCmd cmd;
        vector<SingleMotorCmd> list_cmd;
        cmd.setId(_toolState.getId());

        cmd.setType(DxlCommandType::CMD_TYPE_VELOCITY);
        cmd.setParam(push_air_velocity);
        list_cmd.push_back(cmd);

        cmd.setType(DxlCommandType::CMD_TYPE_POSITION);
        cmd.setParam(req.push_air_position);
        list_cmd.push_back(cmd);

        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(64000); // two's complement of 1536
        list_cmd.push_back(cmd);

        _dynamixel->setEndEffectorCommands(list_cmd);
        list_cmd.clear();

        // calculate push air duration
        double dxl_speed = (double)push_air_velocity * (double)XL320_STEPS_FOR_1_SPEED; // position . sec-1
        double dxl_steps_to_do = abs((double)req.push_air_position - (double)_dynamixel->getEndEffectorState(_toolState.getId(), _toolState.getType())); // position
        double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec

        ros::Duration(seconds_to_wait + 0.25).sleep();

        // set torque to 0
        cmd.setType(DxlCommandType::CMD_TYPE_EFFORT);
        cmd.setParam(0);
        list_cmd.push_back(cmd);
        _dynamixel->setEndEffectorCommands(list_cmd);

        res.state = VACUUM_PUMP_STATE_PUSHED;

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
                for(int i = 0 ; i < motor_list.size(); i++)
                {
                    if(_toolState.getId() == motor_list.at(i))
                    {
                        ROS_INFO("Tools Interface - Unset Current Tools");
                        _dynamixel->unsetEndEffector(_toolState.getId(), _toolState.getType());
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
