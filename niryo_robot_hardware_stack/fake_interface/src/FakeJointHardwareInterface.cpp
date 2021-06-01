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

#include "model/motor_type_enum.hpp"
#include "model/motor_type_enum.hpp"

namespace FakeInterface {

    /**
     * @brief FakeJointHardwareInterface::FakeJointHardwareInterface
     */
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
        for (size_t i = 0; i < 6; i++)
        {
            state_handle.emplace_back(hardware_interface::JointStateHandle(_joints_name[i], &_pos[i], &_vel[i], &_eff[i]));
            _joint_state_interface.registerHandle(state_handle[i]);
        }
        registerInterface(&_joint_state_interface);

        std::vector<hardware_interface::JointHandle> position_handle;
        for (size_t i = 0; i < 6; i++)
        {
            position_handle.emplace_back(hardware_interface::JointHandle(_joint_state_interface.getHandle(_joints_name[i]), &_cmd[i]));
            _joint_position_interface.registerHandle(position_handle[i]);
        }
        registerInterface(&_joint_position_interface);


        // Create motors with previous params
        _list_stepper_id.clear();
        _map_stepper_name.clear();
        _list_dxl_id.clear();
        _map_dxl_name.clear();

        for (size_t i = 0; i < 3; i++)
        {
            _list_stepper_id.emplace_back(static_cast<uint8_t>(_joints_id[i]));
            _map_stepper_name[static_cast<uint8_t>(_joints_id[i])] = _joints_name[i];
        }
        for (size_t i = 3; i < 5; i++)
        {
            _list_dxl_id.emplace_back(static_cast<uint8_t>(_joints_id[i]));
            _map_dxl_name[static_cast<uint8_t>(_joints_id[i])] = _joints_name[i];
        }
        for (size_t i = 5; i < 6; i++)
        {
            _list_dxl_id.emplace_back(static_cast<uint8_t>(_joints_id[i]));
            _map_dxl_name[static_cast<uint8_t>(_joints_id[i])] = _joints_name[i];
        }
    }

    /**
     * @brief FakeJointHardwareInterface::read
     */
    void FakeJointHardwareInterface::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
    {
        _pos[0] = _cmd[0];
        _pos[1] = _cmd[1];
        _pos[2] = _cmd[2];
        _pos[3] = _cmd[3];
        _pos[4] = _cmd[4];
        _pos[5] = _cmd[5];
    }

    /**
     * @brief FakeJointHardwareInterface::write
     */
    void FakeJointHardwareInterface::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
    {
        _pos[0] = _cmd[0];
        _pos[1] = _cmd[1];
        _pos[2] = _cmd[2];
        _pos[3] = _cmd[3];
        _pos[4] = _cmd[4];
        _pos[5] = _cmd[5];
    }

    /**
     * @brief FakeJointHardwareInterface::jointIdToJointName
     * @param id
     * @return
     */
    std::string FakeJointHardwareInterface::jointIdToJointName(uint8_t id, common::model::EMotorType motor_type) const
    {

        if(common::model::EMotorType::STEPPER == motor_type && _map_stepper_name.count(id))
            return _map_stepper_name.at(id);
        else if(_map_dxl_name.count(id))
            return _map_dxl_name.at(id);

        return "";
    }
} //FakeInterface
