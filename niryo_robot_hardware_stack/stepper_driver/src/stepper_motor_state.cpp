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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stepper_driver/stepper_motor_state.hpp"

namespace StepperDriver
{
    StepperMotorState::StepperMotorState() :
        utils::MotorState()
    {
    }

    StepperMotorState::StepperMotorState(uint8_t id) :
        utils::MotorState(id),
        _last_time_read(0.0),
        _hw_fail_counter(0.0),
        _firmware_version("")
    {
    }

    void StepperMotorState::reset()
    {
        _last_time_read = 0.0;
        _hw_fail_counter = 0.0;
        _firmware_version.clear();
    }

    void StepperMotorState::setLastTimeRead(double last_time)
    {
        _last_time_read = last_time;
    }

    void StepperMotorState::setHwFailCounter(double fail_counter)
    {
        _hw_fail_counter = fail_counter;
    }

    void StepperMotorState::setFirmwareVersion(std::string& firmware_version)
    {
        _firmware_version = firmware_version;
    }

    std::string StepperMotorState::str() const
    {
        return "stepper";
    }

    bool StepperMotorState::operator==(const StepperMotorState &other)
    {
        return (this->_id == other._id);
    }
}
