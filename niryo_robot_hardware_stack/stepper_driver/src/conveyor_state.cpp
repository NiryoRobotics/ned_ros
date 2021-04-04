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

    ConveyorState::ConveyorState(uint8_t id)
        : StepperMotorState(id),
          _direction(-1),
          _speed(0),
          _state(false)
    {

    }

    void ConveyorState::reset()
    {
        utils::MotorState::reset();
        _direction = -1;
        _speed = 0;
        _state = false;
    }

    bool ConveyorState::isValid() const
    {
        return (_id != 0);
    }

    void ConveyorState::setState(bool state)
    {
        _state = state;
    }

    void ConveyorState::setSpeed(int16_t speed)
    {
        _speed = speed;
    }

    void ConveyorState::setDirection(int8_t direction)
    {
        _direction = direction;
    }

    bool ConveyorState::operator==(const ConveyorState& m)
    {
        return (this->_id == m._id);
    }

    std::string ConveyorState::str() const
    {
        return "conveyor state";
    }


}
