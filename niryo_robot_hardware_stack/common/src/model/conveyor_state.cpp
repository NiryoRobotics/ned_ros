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
#include <tuple>

namespace common
{
namespace model
{

/**
 * @brief ConveyorState::ConveyorState
 */
ConveyorState::ConveyorState(uint8_t default_id) :
  StepperMotorState(),
  _default_id(default_id)
{}

/**
 * @brief ConveyorState::ConveyorState
 * @param bus_proto
 */
ConveyorState::ConveyorState(EBusProtocol bus_proto, uint8_t default_id)
    : ConveyorState(EHardwareType::STEPPER, bus_proto, 1, default_id)
{}

/**
 * @brief ConveyorState::ConveyorState
 * @param type
 * @param bus_proto
 */
ConveyorState::ConveyorState(EHardwareType type, EBusProtocol bus_proto, uint8_t default_id)
    : ConveyorState(type, bus_proto, 1, default_id)
{}

/**
 * @brief ConveyorState::ConveyorState
 * @param type
 * @param bus_proto
 * @param id
 */
ConveyorState::ConveyorState(EHardwareType type, EBusProtocol bus_proto, uint8_t id, uint8_t default_id)
    : StepperMotorState(type, EComponentType::CONVEYOR, bus_proto, id),
      _default_id(default_id)
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
 * @brief ConveyorState::updateId
 * @param id
 */
void ConveyorState::updateId(uint8_t id)
{
  _id = id;
}

/**
 * @brief ConveyorState::updateData : for conveniency
 */
void ConveyorState::updateData(const std::tuple<bool, uint8_t, uint16_t>& data)
{
    setState(std::get<0>(data));
    setSpeed(std::get<1>(data));
    setDirection(std::get<2>(data));
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
    _assembly_direction = 0;
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
 * @brief ConveyorState::setSpeed
 * @param direction
 */
void ConveyorState::setAssemblyDirection(int8_t direction)
{
    _assembly_direction = direction;
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
