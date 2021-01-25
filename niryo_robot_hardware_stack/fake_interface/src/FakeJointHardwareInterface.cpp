/*
    FakeJointHardwareInterface.cpp
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

#include "fake_interface/FakeJointHardwareInterface.hpp"

FakeJointHardwareInterface::FakeJointHardwareInterface()
{
    ROS_DEBUG("Starting Fake Joint Hardware Interface...");

    for (int i = 0; i < 6; i++)
    {
        _nh.getParam("/niryo_robot_hardware_interface/joint_" + std::to_string(i + 1) + "_id", _joints_id[i]);
        _nh.getParam("/niryo_robot_hardware_interface/joint_" + std::to_string(i + 1) + "_name", _joints_name[i]);
    }

    ROS_INFO("Fake Joint Interface - Joints' Name : (1 : %s, 2 : %s, 3 : %s)", _joints_name[0].c_str(), _joints_name[1].c_str(), _joints_name[2].c_str());
    ROS_INFO("Fake Joint Interface - Joints' Name : (4 : %s, 5 : %s, 6 : %s)", _joints_name[3].c_str(), _joints_name[4].c_str(), _joints_name[5].c_str());
    ROS_INFO("Fake Joint Interface - Joints' ID : (1 : %d, 2 : %d, 3 : %d)", _joints_id[0], _joints_id[1], _joints_id[2]);
    ROS_INFO("Fake Joint Interface - Joints' ID : (4 : %d, 5 : %d, 6 : %d)", _joints_id[3], _joints_id[4], _joints_id[5]);

    // connect and register joint state interface
    std::vector<hardware_interface::JointStateHandle> state_handle;
    for (int i = 0; i < 6; i++)
    {
        state_handle.push_back(hardware_interface::JointStateHandle(_joints_name[i], &_pos[i], &_vel[i], &_eff[i]));
        _joint_state_interface.registerHandle(state_handle[i]);
    }
    registerInterface(&_joint_state_interface);

    std::vector<hardware_interface::JointHandle> position_handle;
    for (int i = 0; i < 6; i++)
    {
        position_handle.push_back(hardware_interface::JointHandle(_joint_state_interface.getHandle(_joints_name[i]), &_cmd[i]));
        _joint_position_interface.registerHandle(position_handle[i]);
    }
    registerInterface(&_joint_position_interface);


    // Create motors with previous params
    _list_stepper_id.clear();
    _map_stepper_name.clear();
    _list_dxl_id.clear();
    _map_dxl_name.clear();

    for (int i = 0; i < 3; i++)
    {
        _list_stepper_id.push_back(_joints_id[i]);
        _map_stepper_name[_joints_id[i]] = _joints_name[i];
    }
    for (int i = 3; i < 5; i++)
    {
        _list_dxl_id.push_back(_joints_id[i]);
        _map_dxl_name[_joints_id[i]] = _joints_name[i];
    }
    for (int i = 5; i < 6; i++)
    {
        _list_dxl_id.push_back(_joints_id[i]);
        _map_dxl_name[_joints_id[i]] = _joints_name[i];
    }
}

void FakeJointHardwareInterface::read()
{
    _pos[0] = _cmd[0];
    _pos[1] = _cmd[1];
    _pos[2] = _cmd[2];
    _pos[3] = _cmd[3];
    _pos[4] = _cmd[4];
    _pos[5] = _cmd[5];
}

void FakeJointHardwareInterface::write()
{
    _pos[0] = _cmd[0];
    _pos[1] = _cmd[1];
    _pos[2] = _cmd[2];
    _pos[3] = _cmd[3];
    _pos[4] = _cmd[4];
    _pos[5] = _cmd[5];
}

std::string FakeJointHardwareInterface::jointIdToJointName(int id, uint8_t motor_type)
{
    if (motor_type == (uint8_t)StepperDriver::StepperMotorType::MOTOR_TYPE_STEPPER)
    {
        std::map<uint8_t, std::string>::iterator  it= _map_stepper_name.find(id);
        if (it !=  _map_stepper_name.end())
            return it->second;
    }
    else if (motor_type == (uint8_t)DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL430 or motor_type ==(uint8_t)DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL320)
    {
        std::map<uint8_t, std::string>::iterator  it= _map_dxl_name.find(id);
        if (it !=  _map_dxl_name.end())
            return it->second;
    }
    return "";
}
