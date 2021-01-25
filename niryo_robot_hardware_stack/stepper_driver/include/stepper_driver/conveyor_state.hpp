/*
    conveyor_state.h
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

#ifndef CONVEYOR_STATE_H
#define CONVEYOR_STATE_H

#include <string>

namespace StepperDriver
{
    class ConveyorState {

        public:
        
            ConveyorState(uint8_t id);

            uint8_t getId();
            void setId(uint8_t motor_id);

            bool getState();
            void setState(bool state);

            int16_t getSpeed();
            void setSpeed(int16_t speed);

            int8_t getDirection();
            void setDirection(int8_t direction);

            bool operator==(const ConveyorState& other);

        private:

            uint8_t _id;
            bool _state;
            int16_t _speed;
            int8_t _direction;
    };
}

#endif
