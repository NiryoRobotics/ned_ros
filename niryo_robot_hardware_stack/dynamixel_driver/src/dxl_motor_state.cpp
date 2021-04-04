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
        : utils::MotorState()
    {
        reset();
    }

    DxlMotorState::DxlMotorState(uint8_t id, DxlMotorType_t type, bool isTool)
        : MotorState(id),
          _type(type),
          _isTool(isTool)
    {
    }

    void DxlMotorState::reset()
    {
        utils::MotorState::reset();
        _type = DxlMotorType_t::MOTOR_TYPE_UNKNOWN;
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

        ss << MotorState::str();

        return ss.str();
    }

} // namespace DynamixelDriver
