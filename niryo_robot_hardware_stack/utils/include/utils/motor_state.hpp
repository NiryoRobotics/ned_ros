/*
    motor_state.h
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

#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <string>

namespace utils
{
    class MotorState
    {
        public:
            MotorState();
            MotorState(uint8_t id);

            virtual void reset();
            virtual bool isValid() const = 0; //to have an abstract class

            //getters
            uint8_t getId() const;

            uint32_t getPositionState() const;
            uint32_t getTemperatureState() const;
            uint32_t getVoltageState() const;
            uint32_t getHardwareErrorState() const;
            std::string getHardwareErrorMessageState() const;

            //setters
            void setPositionState(uint32_t pos);

            void setTemperatureState(uint32_t temp);
            void setVoltageState(uint32_t volt);
            void setHardwareError(uint32_t hw_error);
            void setHardwareError(std::string hw_error_msg);

            virtual bool operator==(const MotorState& other);

            virtual std::string str() const;

        protected:
            uint8_t _id;

            // read variables
            uint32_t _position_state;
            uint32_t _temperature_state;
            uint32_t _voltage_state;
            uint32_t _hw_error_state;
            std::string _hw_error_message_state;
    };

    inline
    uint8_t MotorState::getId() const
    {
        return _id;
    }

    inline
    uint32_t MotorState::getPositionState() const
    {
        return _position_state;
    }

    inline
    uint32_t MotorState::getTemperatureState() const
    {
        return _temperature_state;

    }

    inline
    uint32_t MotorState::getVoltageState() const
    {
        return _voltage_state;

    }

    inline
    uint32_t MotorState::getHardwareErrorState() const
    {
        return _hw_error_state;

    }

    inline
    std::string MotorState::getHardwareErrorMessageState() const
    {
        return _hw_error_message_state;
    }

} //utils

#endif
