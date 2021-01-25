/*
    dxl_motor_state.cpp
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

#include "dynamixel_driver/dxl_motor_state.hpp"

namespace DynamixelDriver
{
    bool DxlMotorState::operator==(const DxlMotorState &m)
    {
        return ((this->_type == m._type) && (this->_id == m._id));
    }

    DxlMotorState::DxlMotorState(uint8_t id, DxlMotorType type)
        : _id(id), _type(type)
    {
    }

    uint8_t DxlMotorState::getId()
    {
        return _id;
    }

    void DxlMotorState::setId(uint8_t motor_id)
    {
        _id = motor_id;
    }

    DxlMotorType DxlMotorState::getType()
    {
        return _type;
    }

    void DxlMotorState::setType(DxlMotorType type)
    {
        _type = type;
    }


    // DxlMotorState::getters - state
    uint32_t DxlMotorState::getPositionState()
    {
        return _state_pos;
    }

    void DxlMotorState::setPositionState(uint32_t pos)
    {
        _state_pos = pos;
    }

    uint32_t DxlMotorState::getTemperatureState()
    {
        return _state_temperature;
    }

    void DxlMotorState::setTemperatureState(uint32_t temp)
    {
        _state_temperature = temp;
    }

    uint32_t DxlMotorState::getVoltageState()
    {
        return _state_voltage;
    }

    void DxlMotorState::setVoltageState(uint32_t volt)
    {
        _state_voltage = volt;
    }

    uint32_t DxlMotorState::getHardwareErrorState()
    {
        return _state_hw_error;
    }

    void DxlMotorState::setHardwareError(uint32_t hw_error)
    {
        _state_hw_error = hw_error;
    }

    std::string DxlMotorState::getHardwareErrorMessageState()
    {
        return _state_hw_error_msg;
    }

    void DxlMotorState::setHardwareError(std::string hw_error_msg)
    {
        _state_hw_error_msg = hw_error_msg;
    }

} // namespace DynamixelDriver