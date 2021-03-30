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
    DxlMotorState::DxlMotorState(uint8_t id, DxlMotorType type)
        : _id(id), _type(type),
          _state_pos(0),
          _state_temperature(0),
          _state_voltage(0),
          _state_hw_error(0),
          _state_hw_error_msg("")
    {
    }

    bool DxlMotorState::operator==(const DxlMotorState &m)
    {
        return ((this->_type == m._type) && (this->_id == m._id));
    }

    void DxlMotorState::setId(uint8_t motor_id)
    {
        _id = motor_id;
    }

    void DxlMotorState::setType(DxlMotorType type)
    {
        _type = type;
    }

    void DxlMotorState::setPositionState(uint32_t pos)
    {
        _position_state = pos;
    }

    void DxlMotorState::setTemperatureState(uint32_t temp)
    {
        _temperature_state = temp;
    }

    void DxlMotorState::setVoltageState(uint32_t volt)
    {
        _voltage_state = volt;
    }

    void DxlMotorState::setHardwareError(uint32_t hw_error)
    {
        _hw_error_state = hw_error;
    }

    void DxlMotorState::setHardwareError(std::string hw_error_msg)
    {
        _hw_error_message_state = hw_error_msg;
    }

} // namespace DynamixelDriver
