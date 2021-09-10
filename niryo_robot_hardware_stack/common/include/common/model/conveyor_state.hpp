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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef CONVEYOR_STATE_H
#define CONVEYOR_STATE_H

#include <string>
#include "stepper_motor_state.hpp"

namespace common
{
namespace model
{

/**
 * @brief The ConveyorState class
 */
class ConveyorState : public StepperMotorState {

    public:
        ConveyorState();
        ConveyorState(EBusProtocol bus_proto);
        ConveyorState(EHardwareType type,
                      EBusProtocol bus_proto);
        ConveyorState(EHardwareType type,
                      EBusProtocol bus_proto, uint8_t id);

        ConveyorState(const ConveyorState& state);
        virtual ~ConveyorState() override;

        void initialize(uint8_t default_id,
                        double max_effort,
                        double micro_steps);

        void updateId(uint8_t id);

        void setState(bool state);
        void setSpeed(int16_t speed);

        bool getState() const;
        int16_t getSpeed() const;

        // other getters
        uint8_t getDefaultId() const;
        int getMaxEffort() const;
        int getMicroSteps() const;

        virtual bool operator==(const ConveyorState& other);

        // StepperMotorState interface
        virtual std::string str() const override;
        virtual void reset() override;
        virtual bool isValid() const override;

private:
        bool _state{false};
        int16_t _speed{0};
        uint8_t _default_id{6};
};

/**
 * @brief ConveyorState::getState
 * @return
 */
inline
bool ConveyorState::getState() const
{
    return _state;
}

/**
 * @brief ConveyorState::getSpeed
 * @return
 */
inline
int16_t ConveyorState::getSpeed() const
{
    return _speed;
}

/**
 * @brief ConveyorState::isValid
 * @return
 */
inline
bool ConveyorState::isValid() const
{
    return (0 != getId());
}

/**
 * @brief ConveyorState::getDefaultId
 * @return
 */
inline
uint8_t ConveyorState::getDefaultId() const
{
  return _default_id;
}

/**
 * @brief ConveyorState::getMaxEffort
 * @return
 */
inline
int ConveyorState::getMaxEffort() const
{
  return _max_effort;
}

/**
 * @brief ConveyorState::getMicroSteps
 * @return
 */
inline
int ConveyorState::getMicroSteps() const
{
  return _micro_steps;
}

} // namespace model
} // namespace common

#endif
