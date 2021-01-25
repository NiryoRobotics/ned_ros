/*
    conveyor_state.hpp
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

#ifndef CONVEYOR_STATE_HPP
#define CONVEYOR_STATE_HPP

#include <stdint.h>
#include <string>

#include "stepper_driver/stepper_enum.hpp"

class ConveyorState
{
    public:
        ConveyorState(uint8_t id);

        void setId(uint8_t id);
        uint8_t getId();

        void setConnectionState(bool state);
        bool isConnected();

        void setState(bool state);
        bool isRunning();

        int16_t getVelocity();
        void setVelocity(int16_t velocity);

        int8_t getDirection();
        void setDirection(int8_t direction);

    private:

        uint8_t _id;

        bool _connected;
        bool _activated;

        int16_t _velocity;
        int8_t _direction;
};
#endif
