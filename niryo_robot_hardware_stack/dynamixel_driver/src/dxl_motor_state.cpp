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
#include <sstream>

using namespace std;

namespace DynamixelDriver
{
    DxlMotorState::DxlMotorState()
    {
        reset();
    }

    DxlMotorState::DxlMotorState(uint8_t id, DxlMotorType_t type, bool isTool)
        : _id(id), _type(type),
          _position_state(0),
          _temperature_state(0),
          _voltage_state(0),
          _hw_error_state(0),
          _hw_error_message_state(""),
          _isTool(isTool)
    {
    }

    void DxlMotorState::reset()
    {
        _id = 0;
        _type = DxlMotorType_t::MOTOR_TYPE_UNKNOWN;
        _position_state = 0;
        _temperature_state = 0;
        _voltage_state = 0;
        _hw_error_state = 0;
        _hw_error_message_state.clear();
        _isTool = false;
    }



    bool DxlMotorState::operator==(const DxlMotorState &m)
    {
        return ((this->_type == m._type) && (this->_id == m._id));
    }

    string DxlMotorState::str() const
    {
        ostringstream ss;

        ss << "DxlMotorState (" << (int)_id << ", " << static_cast<int>(_type) << ", " << _isTool << ")";
        ss << "\n---\n";

        ss << "position " << _position_state << "\n"
           << "temperature " << _temperature_state << "\n"
           << "voltage " << _voltage_state << "\n"
           << "hw_error " << _hw_error_state << "\n"
           << "hw_error_message \"" << _hw_error_message_state << "\"\n";

        ss << "\n";

        return ss.str();
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

    void DxlMotorState::setHardwareError(string hw_error_msg)
    {
        _hw_error_message_state = hw_error_msg;
    }

} // namespace DynamixelDriver
