/*
    calibration_interface.hpp
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

#ifndef CALIBRATION_INTERFACE_HPP
#define CALIBRATION_INTERFACE_HPP

#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <thread>

#include "model/joint_state.hpp"

#include "stepper_driver/stepper_driver_core.hpp"
#include "dynamixel_driver/dxl_driver_core.hpp"

#include "niryo_robot_msgs/CommandStatus.h"

namespace JointsInterface {

    class CalibrationInterface
    {

    public:
        CalibrationInterface(std::vector<common::model::JointState> joint_list,
                             std::shared_ptr<StepperDriver::StepperDriverCore> stepper,
                             std::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel);

        int startCalibration(int mode, std::string &result_message);

        bool CalibrationInprogress();

    private:
        void _setCalibrationCommand(common::model::JointState motor, int offset, int delay,
                                    int motor_direction, int calibration_direction, int timeout,
                                    int32_t &calibration_result);

        int _getCalibrationResult(common::model::JointState &motor);

        bool _check_steppers_connected();

        void _auto_calibration();
        void _send_calibration_offset(uint8_t id, int offset_to_send, int absolute_steps_at_offset_position);
        bool _can_process_manual_calibration(std::string &result_message);
        int _manual_calibration();

        void _motorTorque(common::model::JointState &motor, bool status);
        void _moveMotor(common::model::JointState &motor, int steps, float delay);
        int _relativeMoveMotor(common::model::JointState &motor, int steps, int delay, bool wait);

        bool set_motors_calibration_offsets(std::vector<int> &motor_id_list, std::vector<int> &steps_list);
        bool get_motors_calibration_offsets(std::vector<int> &motor_id_list, std::vector<int> &steps_list);

    private:
        std::vector<common::model::JointState> _joint_list;
        ros::NodeHandle _nh;
        std::shared_ptr<StepperDriver::StepperDriverCore> _stepper;
        std::shared_ptr<DynamixelDriver::DynamixelDriverCore> _dynamixel;

        int _xl430_middle_position, _xl320_middle_position;
        int _xc430_middle_position, _xl330_middle_position;

        double _home_position_1, _home_position_2, _home_position_3;
        double _gear_ratio_1, _gear_ratio_2, _gear_ratio_3;
        double _direction_1, _direction_2, _direction_3;
        double _offset_position_stepper_1, _offset_position_stepper_2, _offset_position_stepper_3;
        double _offset_position_dxl_1, _offset_position_dxl_2, _offset_position_dxl_3;

        bool _calibration_in_progress;
        int _calibration_timeout;

        std::vector<int32_t> _motor_calibration_list;
    };

} //JointsInterface
#endif
