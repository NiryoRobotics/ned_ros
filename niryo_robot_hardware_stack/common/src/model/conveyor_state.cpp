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

#include <cstdint>
#include <sstream>
#include <string>
#include <tuple>

namespace common
{
namespace model
{


/**
 * @brief ConveyorState::ConveyorState
 * @param type
 * @param bus_proto
 * @param id
 * @param default_id
 */
ConveyorState::ConveyorState(EHardwareType type, EBusProtocol bus_proto, uint8_t id, uint8_t default_id, std::string hardware_id)
    : StepperMotorState(type, EComponentType::CONVEYOR, bus_proto, id), _default_id(default_id), _hardware_id(hardware_id)
{
}

/**
 * @brief ConveyorState::updateId
 * @param id
 */
void ConveyorState::updateId(uint8_t id) { _id = id; }

/**
 * @brief ConveyorState::updateData : for conveniency
 */
void ConveyorState::updateData(const std::tuple<bool, uint8_t, int8_t> &data)
{
    setState(std::get<0>(data));
    setSpeed(std::get<1>(data));
    setGoalDirection(std::get<2>(data));
}

/**
 * @brief ConveyorState::reset
 */
void ConveyorState::reset()
{
    StepperMotorState::reset();
    _direction = 0;
    _speed = 0;
    _state = false;
    _goal_direction = -1;
}

/**
 * @brief ConveyorState::setState
 * @param state
 */
void ConveyorState::setState(bool state) { _state = state; }

/**
 * @brief ConveyorState::setSpeed
 * @param speed
 */
void ConveyorState::setSpeed(int16_t speed) { _speed = speed; }

/**
 * @brief ConveyorState::setGoalDirection
 * @param direction
 */
void ConveyorState::setGoalDirection(int8_t direction) { _goal_direction = direction; }

/**
 * @brief ConveyorState::operator ==
 * @param other
 * @return
 */
bool ConveyorState::operator==(const ConveyorState &other) { return (this->_id == other._id); }

/**
 * @brief ConveyorState::str
 * @return
 */
std::string ConveyorState::str() const
{
    std::ostringstream ss;

    ss << "ConveyorState : ";

    ss << "state: " << (_state ? "true" : "false") << ", speed: " << _speed << ", direction: " << static_cast<int>(_goal_direction);

    ss << "\n---\n";
    ss << "\n";
    ss << StepperMotorState::str();

    return ss.str();
}

}  // namespace model
}  // namespace common
