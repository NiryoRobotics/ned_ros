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

#include "ros/time.h"

namespace common
{
namespace model
{

/**
 * @brief StepperMotorState::StepperMotorState
 */
StepperMotorState::StepperMotorState() :
    JointState(),
    _isConveyor(false)
{
}

/**
 * @brief StepperMotorState::StepperMotorState
 * @param bus_proto
 * @param id
 * @param isConveyor
 */
StepperMotorState::StepperMotorState(EBusProtocol bus_proto, uint8_t id, bool isConveyor) :
    JointState("unknown", EHardwareType::STEPPER, bus_proto, id),
    _isConveyor(isConveyor)
{
}

/**
 * @brief StepperMotorState::StepperMotorState
 * @param name
 * @param type
 * @param bus_proto
 * @param id
 * @param isConveyor
 */
StepperMotorState::StepperMotorState(std::string name,
                                     EHardwareType type, EBusProtocol bus_proto,
                                     uint8_t id, bool isConveyor) :
    JointState(name, type, bus_proto, id),
    _isConveyor(isConveyor)
{
}

/**
 * @brief StepperMotorState::~StepperMotorState
 */
StepperMotorState::~StepperMotorState()
{
}

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

    ss << "StepperMotorState : ";
    ss << "\n---\n";
    ss << "last time read: " << _last_time_read << ", ";
    ss << "hw fail counter: " << _hw_fail_counter << ", ";
    ss << "firmware version: " << "\"" << _firmware_version << "\"";
    ss << "\n";
    ss << JointState::str();

    return ss.str();
}

int StepperMotorState::to_motor_pos(double pos_rad, common::model::EBusProtocol protocol)
{
    if (protocol == common::model::EBusProtocol::CAN)
    {
        double numerator = (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio * pos_rad / (2*M_PI));
        return std::round( numerator * _direction);
    }
    else
    {
        int pos = std::round((pos_rad*180) / (M_PI * 0.088) * _direction + _offset_position);
        return pos;
    }
}

double StepperMotorState::to_rad_pos(int pos, common::model::EBusProtocol protocol)
{
    if (protocol == common::model::EBusProtocol::CAN)
    {
        assert(0.0 != (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio * RADIAN_TO_DEGREE));
        return static_cast<double>(
                    (pos * 2*M_PI) /
                    (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio) *
                    _direction);
    }
    else
    {
        double pos_rad = static_cast<double>((pos - _offset_position) * 0.088 * (M_PI / 180) * _direction);
        return pos_rad;
    }
}


}  // namespace model
}  // namespace common
