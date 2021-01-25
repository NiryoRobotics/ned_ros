/*
    JointHardwareInterface.hpp
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

#ifndef JOINT_HARDWARE_INTERFACE_HPP
#define JOINT_HARDWARE_INTERFACE_HPP

#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joints_interface/joint_state.hpp"
#include "joints_interface/calibration_interface.hpp"

#include "stepper_driver/stepper_driver_core.hpp"
#include "dynamixel_driver/dxl_driver_core.hpp"

#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>

class JointHardwareInterface : public hardware_interface::RobotHW
{

public:
    JointHardwareInterface(
        boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> &dynamixel,
        boost::shared_ptr<StepperDriver::StepperDriverCore> &stepper);

    void initPublisherSubscribers();
    void initServices();
    void initMotors();

    void sendInitMotorsParams();

    void read();
    void write();

    bool needCalibration();

    int calibrateJoints(int mode, std::string &result_message);

    void newCalibration();

    bool isCalibrationInProgress();

    void activateLearningMode();

    void deactivateLearningMode();

    void setCommandToCurrentPosition();

    void synchronizeMotors(bool synchronise);

    std::vector<JointState> &getJointsState();

    std::string jointIdToJointName(int id, uint8_t motor_type);

private:
    ros::NodeHandle _nh;

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::PositionJointInterface _joint_position_interface;

    boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> &_dynamixel;
    boost::shared_ptr<StepperDriver::StepperDriverCore> &_stepper;

    std::vector<uint8_t> _list_stepper_id;
    std::map<uint8_t, std::string> _map_stepper_name;

    std::vector<uint8_t> _list_dxl_id;
    std::map<uint8_t, std::string> _map_dxl_name;

    std::vector<JointState> _joint_list;

    boost::shared_ptr<CalibrationInterface> _calibration_interface;

    std::string _joints_name[6] = {""};
    int _joints_id[6] = {0};

    double _cmd[6] = {0, 0.64, -1.39, 0, 0, 0};
    double _pos[6] = {0, 0.64, -1.39, 0, 0, 0};
    double _vel[6] = {0};
    double _eff[6] = {0};

    double _gear_ratio_1, _gear_ratio_2, _gear_ratio_3;
    double _home_position_1, _home_position_2, _home_position_3;
    double _offset_position_stepper_1, _offset_position_stepper_2, _offset_position_stepper_3;
    double _offset_position_dxl_1, _offset_position_dxl_2, _offset_position_dxl_3;
    double _direction_1, _direction_2, _direction_3;
    int _max_effort_1, _max_effort_2, _max_effort_3;

    int _p_gain_1, _p_gain_2, _p_gain_3;
    int _i_gain_1, _i_gain_2, _i_gain_3;
    int _d_gain_3;
    bool _learning_mode;
};

#endif