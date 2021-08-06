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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef ABSTRACT_MOTOR_STATE_H
#define ABSTRACT_MOTOR_STATE_H

#include "i_object.hpp"
#include <string>

#include "common/model/motor_type_enum.hpp"
#include "common/model/bus_protocol_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The AbstractMotorState class
 */
class AbstractMotorState : public IObject
{
    public:

        AbstractMotorState();
        AbstractMotorState(EMotorType type, EBusProtocol bus_proto, uint8_t id);
        virtual ~AbstractMotorState() override;

        // getters
        EMotorType getType() const;
        EBusProtocol getBusProtocol() const;
        uint8_t getId() const;

        int getPositionState() const;
        int getTemperatureState() const;
        int getVoltageState() const;
        int getHardwareErrorState() const;
        std::string getHardwareErrorMessageState() const;

        // setters
        void setPositionState(int pos);

        void setTemperatureState(int temp);
        void setVoltageState(int volt);
        void setHardwareError(int hw_error);
        void setHardwareError(std::string hw_error_msg);

        // tests
        bool isStepper() const;
        bool isDynamixel() const;

        // operators
        virtual bool operator==(const AbstractMotorState& other);

        // IObject interface
        virtual void reset() override;
        virtual std::string str() const override;
        virtual bool isValid() const override = 0; // not reimplemented to keep this class abstract

    protected:
        EMotorType _type;
        EBusProtocol _bus_proto;

        uint8_t _id;

        // read variables
        int _position_state;
        int _temperature_state;
        int _voltage_state;
        int _hw_error_state;
        std::string _hw_error_message_state;
};

/**
 * @brief AbstractMotorState::getType
 * @return
 */
inline
EMotorType AbstractMotorState::getType() const
{
    return _type;
}

/**
 * @brief AbstractMotorState::getBusProtocol
 * @return
 */
inline
EBusProtocol AbstractMotorState::getBusProtocol() const
{
    return _bus_proto;
}

/**
 * @brief AbstractMotorState::getId
 * @return
 */
inline
uint8_t AbstractMotorState::getId() const
{
    return _id;
}

/**
 * @brief AbstractMotorState::getPositionState
 * @return
 */
inline
int AbstractMotorState::getPositionState() const
{
    return _position_state;
}

/**
 * @brief AbstractMotorState::getTemperatureState
 * @return
 */
inline
int AbstractMotorState::getTemperatureState() const
{
    return _temperature_state;
}

/**
 * @brief AbstractMotorState::getVoltageState
 * @return
 */
inline
int AbstractMotorState::getVoltageState() const
{
    return _voltage_state;
}

/**
 * @brief AbstractMotorState::getHardwareErrorState
 * @return
 */
inline
int AbstractMotorState::getHardwareErrorState() const
{
    return _hw_error_state;
}

/**
 * @brief AbstractMotorState::getHardwareErrorMessageState
 * @return
 */
inline
std::string AbstractMotorState::getHardwareErrorMessageState() const
{
    return _hw_error_message_state;
}

/**
 * @brief AbstractMotorState::isStepper
 * @return
 */
inline
bool AbstractMotorState::isStepper() const
{
    return EMotorType::STEPPER == _type;
}

/**
 * @brief AbstractMotorState::isDynamixel
 * @return
 */
inline
bool AbstractMotorState::isDynamixel() const
{
    return (EMotorType::XC430 == _type) ||
           (EMotorType::XL320 == _type) ||
           (EMotorType::XL330 == _type) ||
           (EMotorType::XL430 == _type);
}

} // model
} // common

#endif // ABSTRACT_MOTOR_STATE_H
