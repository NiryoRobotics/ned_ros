/*
    conveyor_state.cpp
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

#include "common/model/conveyor_state.hpp"

#include <sstream>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief ConveyorState::ConveyorState
 * @param bus_proto
 * @param id
 */
ConveyorState::ConveyorState(EBusProtocol bus_proto, uint8_t id)
    : StepperMotorState(bus_proto, id, true),
      _state(false),
      _speed(0)
{}

/**
 * @brief ConveyorState::~ConveyorState
 */
ConveyorState::~ConveyorState()
{}

/**
 * @brief ConveyorState::reset
 */
void ConveyorState::reset()
{
    StepperMotorState::reset();
    _direction = -1;
    _speed = 0;
    _state = false;
}

/**
 * @brief ConveyorState::setState
 * @param state
 */
void ConveyorState::setState(bool state)
{
    _state = state;
}

/**
 * @brief ConveyorState::setSpeed
 * @param speed
 */
void ConveyorState::setSpeed(int16_t speed)
{
    _speed = speed;
}

/**
 * @brief ConveyorState::operator ==
 * @param m
 * @return
 */
bool ConveyorState::operator==(const ConveyorState& m)
{
    return (this->_id == m._id);
}

/**
 * @brief ConveyorState::str
 * @return
 */
std::string ConveyorState::str() const
{
    std::ostringstream ss;

    ss << "DxlMotorState : ";
    ss << "\n---\n";
    ss << "state: " << (_state ? "true" : "false");
    ss << "speed: " << static_cast<int>(_speed);
    ss << "direction: " << static_cast<int>(_direction);
    ss << "\n";
    ss << StepperMotorState::str();

    return ss.str();
}

}  // namespace model
}  // namespace common
