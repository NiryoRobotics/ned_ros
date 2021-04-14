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

#include "model/dxl_motor_state.hpp"

#include <sstream>

using namespace std;

namespace common {
    namespace model {

        DxlMotorState::DxlMotorState()
            : AbstractMotorState()
        {
            reset();
        }

        DxlMotorState::DxlMotorState(uint8_t id, EMotorType type, bool isTool)
            : AbstractMotorState(id),
              _type(type),
              _isTool(isTool)
        {
        }

        DxlMotorState::~DxlMotorState()
        {

        }

        void DxlMotorState::reset()
        {
            AbstractMotorState::reset();
            _type = EMotorType::MOTOR_TYPE_UNKNOWN;
            _isTool = false;
        }

        bool DxlMotorState::operator==(const DxlMotorState &m)
        {
            return ((this->_type == m._type) && (this->_id == m._id));
        }

        string DxlMotorState::str() const
        {
            ostringstream ss;

            ss << "DxlMotorState (" << static_cast<int>(_id) << ", "
               << static_cast<int>(_type) << ", " << _isTool << ")";
            ss << "\n---\n";

            ss << AbstractMotorState::str();

            return ss.str();
        }

        bool DxlMotorState::isValid() const
        {
            return (0 != _id && EMotorType::MOTOR_TYPE_UNKNOWN != _type);
        }

    } // namespace model
} // namespace common
