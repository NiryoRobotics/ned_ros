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
    bool StepperMotorState::operator==(const StepperMotorState& m)
    {
        return(this->_id == m._id);
    }

    StepperMotorState::StepperMotorState(uint8_t id) 
        : _id(id)
    {

    }

    uint8_t StepperMotorState::getId() const
    {
        return _id;
    }

    void StepperMotorState::setId(uint8_t motor_id)
    {
        _id = motor_id;
    }

    int32_t StepperMotorState::getPositionState() const
    {
        return _state_pos;
    }

    void StepperMotorState::setPositionState(int32_t pos)
    {
        _state_pos = pos;
    }

    int32_t StepperMotorState::getTemperatureState() const
    {
        return _state_temperature;
    }

    void StepperMotorState::setTemperatureState(int32_t temp)
    {
        _state_temperature = temp;
    }

    int32_t StepperMotorState::getHardwareErrorState() const
    {
        return _hw_error;
    }

    void StepperMotorState::setHardwareError(int32_t hw_error)
    {
        _hw_error = hw_error;
    }

    double StepperMotorState::getLastTimeRead() const
    {
        return _last_time_read;
    }

    void StepperMotorState::setLastTimeRead(double last_time)
    {
        _last_time_read = last_time;
    }

    int StepperMotorState::getHwFailCounter() const
    {
        return _hw_fail_counter;
    }

    void StepperMotorState::setHwFailCounter(double fail_counter)
    {
        _hw_fail_counter = fail_counter;
    }

    std::string StepperMotorState::getFirmwareVersion() const
    {
        return _firmware_version;
    }

    void StepperMotorState::setFirmwareVersion(std::string& firmware_version)
    {
        _firmware_version = firmware_version;
    }
}
