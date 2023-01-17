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

#include <sstream>
#include <string>

using ::std::ostringstream;
using ::std::string;

namespace common
{
namespace model
{
/**
 * @brief AbstractMotorState::AbstractMotorState
 */
AbstractMotorState::AbstractMotorState() { reset(); }

/**
 * @brief AbstractMotorState::AbstractMotorState
 * @param type
 * @param component_type
 * @param bus_proto
 * @param id
 */
AbstractMotorState::AbstractMotorState(EHardwareType type, EComponentType component_type, EBusProtocol bus_proto, uint8_t id)
    : AbstractHardwareState(type, component_type, bus_proto, id)
{
}

/**
 * @brief AbstractMotorState::reset
 */
void AbstractMotorState::reset()
{
    AbstractHardwareState::reset();
    _position = 0;
}

/**
 * @brief AbstractMotorState::str
 * @return
 */
string AbstractMotorState::str() const
{
    ostringstream ss;

    ss << "AbstractMotorState:\n";

    ss << "position: " << _position << ", ";
    ss << "velocity: " << _velocity << ", ";
    ss << "torque: " << _torque;
    ss << "\n---\n";
    ss << "\n";
    ss << AbstractHardwareState::str();

    return ss.str();
}

/**
 * @brief AbstractMotorState::setPosition
 * @param pos
 */
void AbstractMotorState::setPosition(int pos) { _position = pos; }

/**
 * @brief AbstractMotorState::setVelocity
 * @param vel
 */
void AbstractMotorState::setVelocity(int vel) { _velocity = vel; }

/**
 * @brief AbstractMotorState::setTorque
 * @param torque
 */
void AbstractMotorState::setTorque(int torque) { _torque = torque; }

}  // namespace model
}  // namespace common
