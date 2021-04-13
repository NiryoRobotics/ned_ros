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
#include "stepper_motor_state.hpp"

namespace common {
    namespace model {

        class ConveyorState : public StepperMotorState {

            public:

                ConveyorState(uint8_t id);
                virtual ~ConveyorState() override;

                virtual void reset() override;
                virtual bool isValid() const override;

                void setState(bool state);
                void setSpeed(int16_t speed);
                void setDirection(int8_t direction);

                bool getState() const;
                int16_t getSpeed() const;
                int8_t getDirection() const;

                virtual bool operator==(const ConveyorState& other);

                virtual std::string str() const override;

            private:

                bool _state;
                int16_t _speed;
                int8_t _direction;
        };

        inline
        bool ConveyorState::getState() const
        {
            return _state;
        }

        inline
        int16_t ConveyorState::getSpeed() const
        {
            return _speed;
        }

        inline
        int8_t ConveyorState::getDirection() const
        {
            return _direction;
        }

    } // namespace model
} // namespace common

#endif
