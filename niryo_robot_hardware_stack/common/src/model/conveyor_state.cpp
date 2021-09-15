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
 */
ConveyorState::ConveyorState() :
  StepperMotorState ()
{

}

/**
 * @brief ConveyorState::ConveyorState
 * @param bus_proto
 */
ConveyorState::ConveyorState(EBusProtocol bus_proto)
    : StepperMotorState(EHardwareType::STEPPER, EComponentType::CONVEYOR, bus_proto, 1)
{}

/**
 * @brief ConveyorState::ConveyorState
 * @param type
 * @param bus_proto
 */
ConveyorState::ConveyorState(EHardwareType type, EBusProtocol bus_proto)
    : StepperMotorState(type, EComponentType::CONVEYOR, bus_proto, 1)
{}

/**
 * @brief ConveyorState::ConveyorState
 * @param type
 * @param bus_proto
 * @param id
 */
ConveyorState::ConveyorState(EHardwareType type, EBusProtocol bus_proto, uint8_t id)
    : StepperMotorState(type, EComponentType::CONVEYOR, bus_proto, id)
{}

/**
 * @brief ConveyorState::ConveyorState
 * @param state
 */
ConveyorState::ConveyorState(const ConveyorState &state) :
  StepperMotorState(state)
{
  _state = state._state;
  _speed = state._speed;
  _default_id = state._default_id;
}

/**
 * @brief ConveyorState::~ConveyorState
 */
ConveyorState::~ConveyorState()
{}

/**
 * @brief ConveyorState::initialize
 * @param default_id
 * @param max_effort
 * @param micro_steps
 */
void ConveyorState::initialize(uint8_t default_id, double max_effort, double micro_steps)
{
  _default_id = default_id;
  _max_effort = max_effort;
  _micro_steps = micro_steps;
}

/**
 * @brief ConveyorState::updateId
 * @param id
 */
void ConveyorState::updateId(uint8_t id)
{
  _id = id;
}

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

    ss << "ConveyorState : ";

    ss << "state: " << (_state ? "true" : "false")
       << "speed: " << _speed
       << "direction: " << _direction;

    ss << "\n---\n";
    ss << "\n";
    ss << StepperMotorState::str();

    return ss.str();
}

}  // namespace model
}  // namespace common
