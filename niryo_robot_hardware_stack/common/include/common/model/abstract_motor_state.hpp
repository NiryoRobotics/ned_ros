/*
abstract_motor_state.h
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
#include "common/model/component_type_enum.hpp"
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
    AbstractMotorState(EHardwareType type, EComponentType component_type,
                       EBusProtocol bus_proto, uint8_t id);

    ~AbstractMotorState() override = default;

    // getters
    int getPosition() const;
    int getVelocity() const;
    int getTorque() const;

    // setters
    void setPosition(int pos);
    void setVelocity(int vel);
    void setTorque(int torque);

    // tests
    bool isStepper() const;
    bool isDynamixel() const;
    bool isClosed() const;

    // IObject interface
    void reset() override;
    std::string str() const override;
    bool isValid() const override = 0; // not reimplemented to keep this class abstract

protected:
    // read variables
    int _position{0};
    int _velocity{0};
    int _torque{0};

protected:
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    AbstractMotorState( const AbstractMotorState& ) = default;
    AbstractMotorState( AbstractMotorState&& ) = default;

    AbstractMotorState& operator= ( AbstractMotorState && ) = default;
    AbstractMotorState& operator= ( const AbstractMotorState& ) = default;
};

/**
 * @brief AbstractMotorState::getPositionState
 * @return
 */
inline
int AbstractMotorState::getPosition() const
{
    return _position;
}

/**
 * @brief AbstractMotorState::getVelocityState
 * @return
 */
inline
int AbstractMotorState::getVelocity() const
{
    return _velocity;
}

/**
 * @brief AbstractMotorState::getTorqueState
 * @return
 */
inline
int AbstractMotorState::getTorque() const
{
    return _torque;
}

/**
 * @brief AbstractMotorState::isStepper
 * @return
 */
inline
bool AbstractMotorState::isStepper() const
{
    return (EHardwareType::STEPPER == _hw_type || EHardwareType::NED3PRO_STEPPER == _hw_type ||
            EHardwareType::FAKE_STEPPER_MOTOR == _hw_type);
}

/**
 * @brief AbstractMotorState::isDynamixel
 * @return
 */
inline
bool AbstractMotorState::isDynamixel() const
{
    return (EHardwareType::XC430 == _hw_type) ||
           (EHardwareType::XL320 == _hw_type) ||
           (EHardwareType::XL330 == _hw_type) ||
           (EHardwareType::XL430 == _hw_type) ||
           (EHardwareType::XM430 == _hw_type) ||
           (EHardwareType::XH430 == _hw_type) ||
           (EHardwareType::FAKE_DXL_MOTOR == _hw_type);
}

} // model
} // common

#endif // ABSTRACT_MOTOR_STATE_H
