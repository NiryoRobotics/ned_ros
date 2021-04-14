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

//std
#include <memory>
#include <algorithm>

//ros
#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>

//niryo
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joints_interface/calibration_interface.hpp"
#include "stepper_driver/stepper_driver_core.hpp"
#include "dynamixel_driver/dxl_driver_core.hpp"

#include "model/joint_state.hpp"

namespace JointsInterface {

    class JointHardwareInterface : public hardware_interface::RobotHW
    {

    public:
        JointHardwareInterface(
            std::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel,
            std::shared_ptr<StepperDriver::StepperDriverCore> stepper);


        //careful, this does not override base class methods
        void read();
        void write();

        void sendInitMotorsParams();
        int calibrateJoints(int mode, std::string &result_message);
        void deactivateLearningMode();
        void newCalibration();
        void activateLearningMode();
        void synchronizeMotors(bool synchronise);

        void setCommandToCurrentPosition();

        bool needCalibration() const;
        bool isCalibrationInProgress() const;

        std::string jointIdToJointName(uint8_t id);
        const std::vector<common::model::JointState> &getJointsState() const;

    private:
        void initParameters();
        void initJoints();

        void initPublisherSubscribers();
        void initServices();
        void initMotors();
        bool setMotorPID(int motor_id, common::model::EMotorType motor_type, int p_gain, int i_gain, int d_gain);

    private:
        ros::NodeHandle _nh;

        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::PositionJointInterface _joint_position_interface;

        std::shared_ptr<DynamixelDriver::DynamixelDriverCore> _dynamixel;
        std::shared_ptr<StepperDriver::StepperDriverCore> _stepper;
        std::unique_ptr<CalibrationInterface> _calibration_interface;

        std::vector<uint8_t> _list_stepper_id;
        std::map<uint8_t, std::string> _map_stepper_name;

        std::vector<uint8_t> _list_dxl_id;
        std::map<uint8_t, std::string> _map_dxl_name;

        std::vector<common::model::JointState> _joint_list;

        int _nb_joints;

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
        int _d_gain_1, _d_gain_2, _d_gain_3;
        bool _learning_mode;

    };
} // JointsInterface

#endif
