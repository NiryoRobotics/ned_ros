/*
    stepper_motor_state.cpp
    Copyright (C) 2017 Niryo
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

#include "common/model/stepper_motor_state.hpp"

// c++
#include <cassert>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "ros/time.h"

namespace common
{
namespace model
{

/**
 * @brief StepperMotorState::StepperMotorState
 * @param type
 * @param component_type
 * @param bus_proto
 * @param id
 */
StepperMotorState::StepperMotorState(EHardwareType type, EComponentType component_type, EBusProtocol bus_proto, uint8_t id)
    : StepperMotorState("unknown", type, component_type, bus_proto, id)
{
}

/**
 * @brief StepperMotorState::StepperMotorState
 * @param name
 * @param type
 * @param component_type
 * @param bus_proto
 * @param id
 */
StepperMotorState::StepperMotorState(std::string name, EHardwareType type, EComponentType component_type, EBusProtocol bus_proto, uint8_t id)
    : JointState(std::move(name), type, component_type, bus_proto, id)
{
    updateMultiplierRatio();
}

// *********************
//  JointState Interface
// ********************

/**
 * @brief StepperMotorState::reset
 */
void StepperMotorState::reset()
{
    JointState::reset();
    _last_time_read = 0.0;
    _hw_fail_counter = 0.0;
    _firmware_version.clear();
    _calibration_value = 0;
    _calibration_status = EStepperCalibrationStatus::UNINITIALIZED;
}

/**
 * @brief StepperMotorState::isValid
 * @return
 */
bool StepperMotorState::isValid() const { return (0 != _id) && (0.0 != _pos_multiplier_ratio); }

/**
 * @brief StepperMotorState::str
 * @return
 */
std::string StepperMotorState::str() const
{
    std::ostringstream ss;

    ss << "StepperMotorState :\n";
    ss << "firmware version: "
       << "\"" << _firmware_version << "\"";
    ss << ", last time read: " << _last_time_read << ", hw fail counter: " << _hw_fail_counter << "\n"
       << "max effort: " << _max_effort << ", "
       << "gear ratio: " << _gear_ratio << ", "
       << "micro steps: " << _micro_steps << ", "
       << "motor_ratio: " << _motor_ratio << ", "
       << "pos multiplier ratio: " << _pos_multiplier_ratio << ", "
       << "vel multiplier ratio: " << _vel_multiplier_ratio << "\n";

    ss << "velocity profile : ";
    for (auto const &d : getVelocityProfile().to_list())
        ss << d << ",";
    ss << "\n";

    ss << "calibration state: " << StepperCalibrationStatusEnum(_calibration_status).toString() << ", "
       << "calibration value: " << _calibration_value;

    ss << "\n---\n";
    ss << "\n";
    ss << JointState::str();

    return ss.str();
}

/**
 * @brief StepperMotorState::to_motor_pos
 * @param rad_pos
 * @return
 */
int StepperMotorState::to_motor_pos(double rad_pos)
{
    if (rad_pos > _limit_position_max)
        rad_pos = _limit_position_max;
    else if (rad_pos < _limit_position_min)
        rad_pos = _limit_position_min;

    int result = static_cast<int>(std::round((rad_pos - _offset_position) * _pos_multiplier_ratio * _direction));

    return result;
}

/**
 * @brief StepperMotorState::to_rad_pos
 * @param motor_pos
 * @return
 */
double StepperMotorState::to_rad_pos(int motor_pos)
{
    assert(0.0 != _pos_multiplier_ratio);

    return _offset_position + static_cast<double>(motor_pos * _direction / _pos_multiplier_ratio);
}

/**
 * @brief StepperMotorState::to_motor_vel
 * @param rad_vel
 * @return
 */
int StepperMotorState::to_motor_vel(double rad_vel)
{
    assert(0.0 != _vel_multiplier_ratio);
    return static_cast<int>(std::round(rad_vel / _vel_multiplier_ratio));
}

/**
 * @brief StepperMotorState::to_rad_vel
 * @param motor_vel
 * @return
 */
double StepperMotorState::to_rad_vel(int motor_vel) { return motor_vel * _vel_multiplier_ratio; }

// ****************
//  Setters
// ****************

/**
 * @brief StepperMotorState::setGearRatio
 * @param gear_ratio
 */
void StepperMotorState::setGearRatio(double gear_ratio) { _gear_ratio = gear_ratio; }
/**
 * @brief StepperMotorState::updateLastTimeRead
 */
void StepperMotorState::updateLastTimeRead() { _last_time_read = ros::Time::now().toSec(); }

/**
 * @brief StepperMotorState::setHwFailCounter
 * @param fail_counter
 */
void StepperMotorState::setHwFailCounter(double fail_counter) { _hw_fail_counter = fail_counter; }

/**
 * @brief StepperMotorState::setMaxEffort
 * @param max_effort
 */
void StepperMotorState::setMaxEffort(double max_effort) { _max_effort = max_effort; }

/**
 * @brief StepperMotorState::setMotorRatio
 * @param motor_ratio
 */
void StepperMotorState::setMotorRatio(double motor_ratio) { _motor_ratio = motor_ratio; }

/**
 * @brief StepperMotorState::setHomingAbsPosition
 * @param homing_abs_position
 */
void StepperMotorState::setHomingAbsPosition(int32_t homing_abs_position) { _homing_abs_position = homing_abs_position; }

/**
 * @brief StepperMotorState::setCalibration
 * @param calibration_state
 * @param calibration_value
 */
void StepperMotorState::setCalibration(const EStepperCalibrationStatus &calibration_state, const int32_t &calibration_value)
{
    _calibration_status = calibration_state;
    _calibration_value = calibration_value;
}

/**
 * @brief StepperMotorState::setCalibration
 * @param data
 */
void StepperMotorState::setCalibration(const std::tuple<EStepperCalibrationStatus, int32_t> &data)
{
    _calibration_status = std::get<0>(data);
    _calibration_value = std::get<1>(data);
}

/**
 * @brief StepperMotorState::setMicroSteps
 * @param micro_steps
 */
void StepperMotorState::setMicroSteps(double micro_steps)
{
    _micro_steps = micro_steps;

    updateMultiplierRatio();
}

/**
 * @brief StepperMotorState::setVelocityProfile
 * @param profile
 */
void StepperMotorState::setVelocityProfile(const VelocityProfile &profile) { _profile = profile; }

//**************
//    Private
//**************

/**
 * @brief StepperMotorState::updateMultiplierRatio
 */
void StepperMotorState::updateMultiplierRatio()
{
    double total_angle = 2 * M_PI;

    assert(0.0 != total_angle);

    if (common::model::EBusProtocol::CAN == _bus_proto)
    {
        _pos_multiplier_ratio = STEPPERS_MOTOR_STEPS_PER_REVOLUTION * _micro_steps * _gear_ratio / total_angle;
        _vel_multiplier_ratio = 1.0;
    }
    else
    {
        _pos_multiplier_ratio = 360 / (_motor_ratio * total_angle);
        _vel_multiplier_ratio = 0.01;
    }
}

}  // namespace model
}  // namespace common
