/*
    motor_state.cpp
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

#include "model/abstract_motor_state.hpp"
#include <sstream>

using namespace std;

namespace common {
    namespace model
    {
        /**
         * @brief AbstractMotorState::AbstractMotorState
         */
        AbstractMotorState::AbstractMotorState() :
            _type(EMotorType::MOTOR_TYPE_UNKNOWN)
        {
            reset();
        }

        /**
         * @brief AbstractMotorState::AbstractMotorState
         * @param id
         * @param type
         */
        AbstractMotorState::AbstractMotorState(uint8_t id, EMotorType type) :
              _type(type),
              _id(id),
              _position_state(0),
              _temperature_state(0),
              _voltage_state(0),
              _hw_error_state(0),
              _hw_error_message_state("")
        {
        }

        /**
         * @brief AbstractMotorState::~AbstractMotorState
         */
        AbstractMotorState::~AbstractMotorState()
        {

        }

        /**
         * @brief AbstractMotorState::reset
         */
        void AbstractMotorState::reset()
        {
            _id = 0;
            _position_state = 0;
            _temperature_state = 0;
            _voltage_state = 0;
            _hw_error_state = 0;
            _hw_error_message_state.clear();
        }

        /**
         * @brief AbstractMotorState::operator ==
         * @param m
         * @return
         */
        bool AbstractMotorState::operator==(const AbstractMotorState &m)
        {
            return (this->_id == m._id);
        }

        /**
         * @brief AbstractMotorState::str
         * @return
         */
        string AbstractMotorState::str() const
        {
            ostringstream ss;

            ss << "AbstractMotorState (" << static_cast<int>(_id) << ")" << "\n";

            ss << "position " << _position_state << "\n"
               << "temperature " << _temperature_state << "\n"
               << "voltage " << _voltage_state << "\n"
               << "hw_error " << _hw_error_state << "\n"
               << "hw_error_message \"" << _hw_error_message_state << "\"";
            ss << "\n";

            return ss.str();
        }

        /**
         * @brief AbstractMotorState::setPositionState
         * @param pos
         */
        void AbstractMotorState::setPositionState(int pos)
        {
            _position_state = pos;
        }

        /**
         * @brief AbstractMotorState::setTemperatureState
         * @param temp
         */
        void AbstractMotorState::setTemperatureState(int temp)
        {
            _temperature_state = temp;
        }

        /**
         * @brief AbstractMotorState::setVoltageState
         * @param volt
         */
        void AbstractMotorState::setVoltageState(int volt)
        {
            _voltage_state = volt;
        }

        /**
         * @brief AbstractMotorState::setHardwareError
         * @param hw_error
         */
        void AbstractMotorState::setHardwareError(int hw_error)
        {
            _hw_error_state = hw_error;
        }

        /**
         * @brief AbstractMotorState::setHardwareError
         * @param hw_error_msg
         */
        void AbstractMotorState::setHardwareError(string hw_error_msg)
        {
            _hw_error_message_state = hw_error_msg;
        }

    } // namespace model
} // namespace common
