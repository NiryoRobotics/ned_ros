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

#include "abstract_hardware_state.hpp"
#include <string>

#include "common/model/hardware_type_enum.hpp"
#include "common/model/bus_protocol_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The AbstractMotorState class
 */
class AbstractMotorState : public AbstractHardwareState
{
    public:

        AbstractMotorState();
        AbstractMotorState(EHardwareType type, EBusProtocol bus_proto, uint8_t id);
        virtual ~AbstractMotorState() override;

        // getters
        int getPositionState() const;

        // setters
        void setPositionState(int pos);

        // tests
        bool isStepper() const;
        bool isDynamixel() const;

        // IObject interface
        virtual void reset() override;
        virtual std::string str() const override;
        virtual bool isValid() const override = 0; // not reimplemented to keep this class abstract

    protected:
        // read variables
        int _position_state;
};

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
 * @brief AbstractMotorState::isStepper
 * @return
 */
inline
bool AbstractMotorState::isStepper() const
{
    return EHardwareType::STEPPER == _type;
}

/**
 * @brief AbstractMotorState::isDynamixel
 * @return
 */
inline
bool AbstractMotorState::isDynamixel() const
{
    return (EHardwareType::XC430 == _type) ||
           (EHardwareType::XL320 == _type) ||
           (EHardwareType::XL330 == _type) ||
           (EHardwareType::XL430 == _type);
}

} // model
} // common

#endif // ABSTRACT_MOTOR_STATE_H
