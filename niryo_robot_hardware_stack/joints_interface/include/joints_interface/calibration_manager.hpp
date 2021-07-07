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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef CALIBRATION_INTERFACE_HPP
#define CALIBRATION_INTERFACE_HPP

#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <thread>

#include "common/model/joint_state.hpp"

#include "can_driver/can_driver_core.hpp"
#include "ttl_driver/ttl_driver_core.hpp"

#include "niryo_robot_msgs/CommandStatus.h"

namespace joints_interface
{

class CalibrationManager
{

public:
    CalibrationManager(ros::NodeHandle& nh,
                       std::vector<std::shared_ptr<common::model::JointState> > joint_list,
                       std::shared_ptr<can_driver::CanDriverCore> can_driver,
                       std::shared_ptr<ttl_driver::TtlDriverCore> ttl_driver);

    virtual ~CalibrationManager();

    int startCalibration(int mode, std::string &result_message);

    bool CalibrationInprogress() const;

private:
    void initParameters(ros::NodeHandle& nh);

    void setStepperCalibrationCommand(const std::shared_ptr<common::model::StepperMotorState>& pState,
                                      int32_t delay, int32_t calibration_direction, int32_t timeout);

    bool _check_steppers_connected();

    common::model::EStepperCalibrationStatus _auto_calibration();
    void _send_calibration_offset(uint8_t id, int offset_to_send, int absolute_steps_at_offset_position);
    bool _can_process_manual_calibration(std::string &result_message);
    common::model::EStepperCalibrationStatus _manual_calibration();

    void _motorTorque(const std::shared_ptr<common::model::JointState>& motor, bool status);
    void _moveMotor(const std::shared_ptr<common::model::JointState>& motor, int steps, double delay);
    int _relativeMoveMotor(const std::shared_ptr<common::model::JointState>& motor, int steps, int delay, bool wait);

    bool set_motors_calibration_offsets(const std::vector<int>& motor_id_list, const std::vector<int> &steps_list);
    bool get_motors_calibration_offsets(std::vector<int> &motor_id_list, std::vector<int>& steps_list);

private:
    ros::NodeHandle _nh;
    std::shared_ptr<can_driver::CanDriverCore> _can_driver_core;
    std::shared_ptr<ttl_driver::TtlDriverCore> _ttl_driver_core;

    std::vector<std::shared_ptr<common::model::JointState> > _joint_list;

    bool _calibration_in_progress{false};
    int _calibration_timeout{0};
    std::string _calibration_file_name;

    std::vector<int32_t> _motor_calibration_list;

    static constexpr int AUTO_CALIBRATION = 1;
    static constexpr int MANUAL_CALIBRATION = 2;

};

} // JointsInterface
#endif
