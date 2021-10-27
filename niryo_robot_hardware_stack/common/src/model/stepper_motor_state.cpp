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
#include <sstream>
#include <cassert>
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
StepperMotorState::StepperMotorState(EHardwareType type,
                                     EComponentType component_type,
                                     EBusProtocol bus_proto,
                                     uint8_t id) :
    StepperMotorState("unknown", type, component_type, bus_proto, id)
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
StepperMotorState::StepperMotorState(std::string name,
                                     EHardwareType type,
                                     EComponentType component_type,
                                     EBusProtocol bus_proto,
                                     uint8_t id) :
    JointState(std::move(name), type, component_type, bus_proto, id)
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
    _calibration_state = EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
}

/**
 * @brief StepperMotorState::isValid
 * @return
 */
bool StepperMotorState::isValid() const
{
    return (0 != _id) && ( 0.0 != _pos_multiplier_ratio);
}

/**
 * @brief StepperMotorState::str
 * @return
 */
std::string StepperMotorState::str() const
{
    std::ostringstream ss;

    ss << "StepperMotorState :\n";
    ss << "firmware version: " << "\"" << _firmware_version << "\"";
    ss << "last time read: " << _last_time_read << ", "
       << "hw fail counter: " << _hw_fail_counter << "\n"
       << "max effort: " << _max_effort << ", "
       << "gear ratio: " << _gear_ratio << ", "
       << "micro steps: " << _micro_steps << ", "
       << "pos multiplier ratio: " << _pos_multiplier_ratio << ", "
       << "vel multiplier ratio: " << _vel_multiplier_ratio << "\n";

    ss << "velocity profile : ";
    for (auto const& d : getVelocityProfile())
      ss << d << ",";
    ss << "\n";

    ss << "calibration state: " << StepperCalibrationStatusEnum(_calibration_state).toString() << ", "
       << "calibration stall threshold: " << std::to_string(_calibration_stall_threshold) << ", "
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
    int result =  static_cast<int>(std::round(_offset_position + (rad_pos * _pos_multiplier_ratio * _direction)));

    if (common::model::EBusProtocol::TTL == _bus_proto)
    {
        result = result > 0 ? result : 0;
    }

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

    return static_cast<double>( (motor_pos - _offset_position) * _direction / _pos_multiplier_ratio);
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
double StepperMotorState::to_rad_vel(int motor_vel)
{
    return motor_vel * _vel_multiplier_ratio;
}

  // ****************
  //  Setters
  // ****************

  /**
 * @brief StepperMotorState::setGearRatio
 * @param gear_ratio
 */
  void StepperMotorState::setGearRatio(double gear_ratio)
  {
    _gear_ratio = gear_ratio;

    updateMultiplierRatio();
}
/**
 * @brief StepperMotorState::updateLastTimeRead
 */
void StepperMotorState::updateLastTimeRead()
{
    _last_time_read = ros::Time::now().toSec();
}

/**
 * @brief StepperMotorState::setHwFailCounter
 * @param fail_counter
 */
void StepperMotorState::setHwFailCounter(double fail_counter)
{
    _hw_fail_counter = fail_counter;
}

/**
 * @brief StepperMotorState::setMaxEffort
 * @param max_effort
 */
void StepperMotorState::setMaxEffort(double max_effort)
{
    _max_effort = max_effort;
}

/**
 * @brief StepperMotorState::setCalibration
 * @param calibration_state
 * @param calibration_value
 */
void StepperMotorState::setCalibration(const EStepperCalibrationStatus &calibration_state,
                                       const int32_t &calibration_value)
{
    _calibration_state = calibration_state;
    _calibration_value = calibration_value;
}

/**
 * @brief StepperMotorState::setCalibration
 * @param data
 */
void StepperMotorState::setCalibration(const std::tuple<EStepperCalibrationStatus, int32_t> &data)
{
    _calibration_state = std::get<0>(data);
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
 * @brief StepperMotorState::setProfileVStart
 * @param profile_v_start
 */
void StepperMotorState::setProfileVStart(const uint32_t &profile_v_start)
{
  _profile_v_start = profile_v_start;
}

/**
 * @brief StepperMotorState::setProfileA1
 * @param profile_a_1
 */
void StepperMotorState::setProfileA1(const uint32_t &profile_a_1)
{
  _profile_a_1 = profile_a_1;
}

/**
 * @brief StepperMotorState::setProfileV1
 * @param profile_v_1
 */
void StepperMotorState::setProfileV1(const uint32_t &profile_v_1)
{
  _profile_v_1 = profile_v_1;
}

/**
 * @brief StepperMotorState::setProfileAMax
 * @param profile_a_max
 */
void StepperMotorState::setProfileAMax(const uint32_t &profile_a_max)
{
  _profile_a_max = profile_a_max;
}

/**
 * @brief StepperMotorState::setProfileVMax
 * @param profile_v_max
 */
void StepperMotorState::setProfileVMax(const uint32_t &profile_v_max)
{
  _profile_v_max = profile_v_max;
}

/**
 * @brief StepperMotorState::setProfileDMax
 * @param profile_d_max
 */
void StepperMotorState::setProfileDMax(const uint32_t &profile_d_max)
{
  _profile_d_max = profile_d_max;
}

/**
 * @brief StepperMotorState::setProfileD1
 * @param profile_d_1
 */
void StepperMotorState::setProfileD1(const uint32_t &profile_d_1)
{
  _profile_d_1 = profile_d_1;
}

/**
 * @brief StepperMotorState::setProfileVStop
 * @param profile_v_stop
 */
void StepperMotorState::setProfileVStop(const uint32_t &profile_v_stop)
{
  _profile_v_stop = profile_v_stop;
}

/**
 * @brief StepperMotorState::setCalibrationStallThreshold
 * @param calibration_stall_threshold
 */
void StepperMotorState::setCalibrationStallThreshold(const uint8_t &calibration_stall_threshold)
{
  _calibration_stall_threshold = calibration_stall_threshold;
}

//**************
//    Private
//**************

/**
 * @brief StepperMotorState::updateMultiplierRatio
 */
void StepperMotorState::updateMultiplierRatio()
{
    double total_angle =  2 * M_PI;

    assert(0.0 != total_angle);

    if (common::model::EBusProtocol::CAN == _bus_proto)
    {
        _pos_multiplier_ratio = STEPPERS_MOTOR_STEPS_PER_REVOLUTION * _micro_steps * _gear_ratio / total_angle;
        _vel_multiplier_ratio = 1.0;
    }
    else
    {
        _pos_multiplier_ratio = 360 / (0.088 * total_angle);
        _vel_multiplier_ratio = 0.01;
    }
}

}  // namespace model
}  // namespace common
