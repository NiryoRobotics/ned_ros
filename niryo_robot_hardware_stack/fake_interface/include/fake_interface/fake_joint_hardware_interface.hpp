/*
fake_joint_hardware_interface.hpp
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

#ifndef FAKE_JOINT_HARDWARE_INTERFACE_HPP
#define FAKE_JOINT_HARDWARE_INTERFACE_HPP

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include "common/model/motor_type_enum.hpp"

namespace fake_interface
{

/**
 * @brief The FakeJointHardwareInterface class
 */
class FakeJointHardwareInterface : public hardware_interface::RobotHW
{
    public:
        FakeJointHardwareInterface(ros::NodeHandle& nh);

        std::string jointIdToJointName(uint8_t id, common::model::EMotorType motor_type) const;

        // RobotHW interface
    public:
        virtual void read(const ros::Time &/*time*/, const ros::Duration &/*period*/) override;
        virtual void write(const ros::Time &/*time*/, const ros::Duration &/*period*/) override;

    private:
        std::vector<uint8_t> _list_stepper_id;
        std::map<uint8_t, std::string> _map_stepper_name;

        std::vector<uint8_t> _list_dxl_id;
        std::map<uint8_t, std::string> _map_dxl_name;

        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::PositionJointInterface _joint_position_interface;

        std::string _joints_name[6] = {""};
        int _joints_id[6] = {0};

        double _cmd[6] = {0, 0.6, -1.3, 0, 0, 0};
        double _pos[6] = {0, 0.6, -1.3, 0, 0, 0};
        double _vel[6] = {0};
        double _eff[6] = {0};

};
} // FakeInterface

#endif