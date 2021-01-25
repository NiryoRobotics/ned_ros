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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stepper_driver/conveyor_state.hpp"

namespace StepperDriver
{
    bool ConveyorState::operator==(const ConveyorState& m)
    {
        return(this->_id == m._id);
    }

    ConveyorState::ConveyorState(uint8_t id) 
        : _id(id)
    {

    }

    uint8_t ConveyorState::getId()
    {
        return _id;
    }

    void ConveyorState::setId(uint8_t motor_id)
    {
        _id = motor_id;
    }

    bool ConveyorState::getState()
    {
        return _state;
    }

    void ConveyorState::setState(bool state)
    {
        _state = state;
    }

    int16_t ConveyorState::getSpeed()
    {
        return _speed;
    }

    void ConveyorState::setSpeed(int16_t speed)
    {
        _speed = speed;
    }

    int8_t ConveyorState::getDirection()
    {
        return _direction;
    }

    void ConveyorState::setDirection(int8_t direction)
    {
        _direction = direction;
    }

}
