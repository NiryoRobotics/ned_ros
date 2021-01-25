/*
    conveyor_state.cpp
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

#include "conveyor_interface/conveyor_state.hpp"


ConveyorState::ConveyorState(uint8_t id) :
    _id(id)
{
    _connected = true;
    _activated = true;
    _velocity = 0;
    _direction = 0;
}

void ConveyorState::setId(uint8_t id)
{
    _id = id;
}

uint8_t ConveyorState::getId()
{
    return _id;
}

void ConveyorState::setConnectionState(bool state)
{
    _connected = state;
}

bool ConveyorState::isConnected()
{
    return _connected;
}

void ConveyorState::setState(bool state)
{
    _activated = state;
}

bool ConveyorState::isRunning()
{
    return _activated;
}

int16_t ConveyorState::getVelocity()
{
    return _velocity;
}

void ConveyorState::setVelocity(int16_t velocity)
{
    _velocity = velocity;
}

int8_t ConveyorState::getDirection()
{
    return _direction;
}

void ConveyorState::setDirection(int8_t direction)
{
    _direction = direction;
}
