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
 */
StepperMotorState::StepperMotorState()
{
}

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
}

/**
 * @brief StepperMotorState::StepperMotorState : copy ctor
 * @param state
 */
StepperMotorState::StepperMotorState(const StepperMotorState &state) :
  JointState(state)
{
    _last_time_read = state._last_time_read;
    _hw_fail_counter = state._hw_fail_counter;

    _gear_ratio = state._gear_ratio;
    _max_effort = state._max_effort;
    _micro_steps = state._micro_steps;

    _calibration_state = state._calibration_state;
    _calibration_value = state._calibration_value;

    _profile_v_start = state._profile_v_start;
    _profile_a_1 = state._profile_a_1;
    _profile_v_1 = state._profile_v_1;
    _profile_a_max = state._profile_a_max;
    _profile_v_max = state._profile_v_max;
    _profile_d_max = state._profile_d_max;
    _profile_d_1 = state._profile_d_1;
    _profile_v_stop = state._profile_v_stop;
}

/**
 * @brief StepperMotorState::~StepperMotorState
 */
StepperMotorState::~StepperMotorState()
= default;

// ****************
//  Setters
// ****************

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
 * @brief StepperMotorState::setGearRatio
 * @param gear_ratio
 */
void StepperMotorState::setGearRatio(double gear_ratio)
{
    _gear_ratio = gear_ratio;
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
    return (0 != _id);
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
       << "gear ratio: " << _gear_ratio << ", "
       << "max effort: " << _max_effort << ", "
       << "micro steps: " << _micro_steps << "\n";

    ss << "velocity profile : ";
    for (auto const& d : getVelocityProfile())
      ss << d << ",";
    ss << "\n";

    ss << "calibration state: " << StepperCalibrationStatusEnum(_calibration_state).toString() << ", "
       << "calibration value: " << _calibration_value;
    ss << "\n---\n";
    ss << "\n";
    ss << JointState::str();

    return ss.str();
}

/**
 * @brief StepperMotorState::to_motor_pos
 * @param pos_rad
 * @return
 * TODO(CC) find a similar formula for both
 */
int StepperMotorState::to_motor_pos(double pos_rad)
{
    if (getBusProtocol() == common::model::EBusProtocol::CAN)
    {
        double numerator = (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * _micro_steps * _gear_ratio * pos_rad / (2*M_PI));
        return static_cast<int>(std::round( numerator * _direction));
    }
    
    
        int pos = static_cast<int>(std::round((pos_rad*180) / (M_PI * 0.088) * _direction + _offset_position));
        return pos > 0 ? pos : 0;
    
}

/**
 * @brief StepperMotorState::to_rad_pos
 * @param pos
 * @return
 * TODO(CC) find a similar formula for both
 */
double StepperMotorState::to_rad_pos(int pos)
{
    if (getBusProtocol() == common::model::EBusProtocol::CAN)
    {
        assert(0.0 != (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * _micro_steps * _gear_ratio * RADIAN_TO_DEGREE));
        return static_cast<double>(
                    (pos * 2*M_PI) /
                    (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * _micro_steps * _gear_ratio) *
                    _direction);
    }
    
    
        auto pos_rad = static_cast<double>((pos - _offset_position) * 0.088 * (M_PI / 180) * _direction);
        return pos_rad;
    
}

/**
 * @brief StepperMotorState::setMicroSteps
 * @param micro_steps
 */
void StepperMotorState::setMicroSteps(double micro_steps)
{
    _micro_steps = micro_steps;
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

}  // namespace model
}  // namespace common
