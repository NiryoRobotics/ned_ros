/*
    motor_state.cpp
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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include "common/model/abstract_motor_state.hpp"

#include <string>
#include <sstream>

using ::std::string;
using ::std::ostringstream;

namespace common
{
namespace model
{
/**
 * @brief AbstractMotorState::AbstractMotorState
 */
AbstractMotorState::AbstractMotorState() :
    AbstractHardwareState()
{
    reset();
}

/**
 * @brief AbstractMotorState::AbstractMotorState
 * @param type
 * @param bus_proto
 * @param id
 */
AbstractMotorState::AbstractMotorState(EHardwareType type, EComponentType component_type,
                                       EBusProtocol bus_proto, uint8_t id) :
      AbstractHardwareState(type, component_type, bus_proto, id),
      _position_state(0)
{
}

/**
 * @brief AbstractMotorState::AbstractMotorState
 * @param state
 */
AbstractMotorState::AbstractMotorState(const AbstractMotorState &state) :
  AbstractHardwareState(state)
{
  _position_state = state._position_state;
}

/**
 * @brief AbstractMotorState::~AbstractMotorState
 */
AbstractMotorState::~AbstractMotorState()
{
}

/**
 * @brief AbstractMotorState::reset
 */
void AbstractMotorState::reset()
{
    AbstractHardwareState::reset();
    _position_state = 0;
}

/**
 * @brief AbstractMotorState::str
 * @return
 */
string AbstractMotorState::str() const
{
    ostringstream ss;

    ss << "AbstractMotorState:\n";

    ss << "position " << _position_state;
    ss << "\n---\n";
    ss << "\n";
    ss << AbstractHardwareState::str();

    return ss.str();
}

/**
 * @brief AbstractMotorState::setPositionState
 * @param pos
 */
void AbstractMotorState::setPositionState(int pos)
{
    _position_state = pos;
}

}  // namespace model
}  // namespace common
