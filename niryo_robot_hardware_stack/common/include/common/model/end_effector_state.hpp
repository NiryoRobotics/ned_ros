/*
end_effector_state.hpp
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

#ifndef END_EFFECTOR_STATE_HPP
#define END_EFFECTOR_STATE_HPP

#include "abstract_hardware_state.hpp"

#include <stdint.h>
#include <string>

#include "hardware_type_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EndEffectorState class
 */
class EndEffectorState : public AbstractHardwareState
{

    public:
        enum class EButtonType
        {
            FREE_DRIVE_BUTTON,
            SAVE_POS_BUTTON,
            CUSTOM_BUTTON
        };

        enum class EActionType
        {
            HANDLE_HELD_ACTION = 0,
            LONG_PUSH_ACTION = 1,
            SINGLE_PUSH_ACTION = 2,
            DOUBLE_PUSH_ACTION = 3,
            NO_ACTION = 100
        };

    private:
        struct Button
        {
          std::string name;
          EActionType action;
        };

    public:
        EndEffectorState();
        EndEffectorState(uint8_t id);

        virtual ~EndEffectorState() override;

        // AbstractHardwareState interface
        virtual std::string str() const override;

        // IObject interface
    public:
        virtual bool isValid() const override;

    public:

        void setButtonStatus(EButtonType button, EActionType action);
        std::map<EButtonType, Button> getButtonsStatus();
        EActionType getButtonStatus(EButtonType button);

        uint32_t getAccelerometerXValue() const;
        uint32_t getAccelerometerYValue() const;
        uint32_t getAccelerometerZValue() const;

        void setAccelerometerXValue(const uint32_t& getAccelerometerXValue);
        void setAccelerometerYValue(const uint32_t& getAccelerometerYValue);
        void setAccelerometerZValue(const uint32_t& getAccelerometerZValue);

        bool getCollisionStatus() const;
        void setCollisionStatus(bool getCollisionStatus);

private:
        std::map<EButtonType, Button> _buttons;

        uint32_t _accelerometer_x_value;
        uint32_t _accelerometer_y_value;
        uint32_t _accelerometer_z_value;

        bool _collision_status;
};


/**
 * @brief EndEffectorState::getAccelerometerXValue
 * @return
 */
inline
std::map<EndEffectorState::EButtonType, EndEffectorState::Button>
EndEffectorState::getButtonsStatus()
{
    return _buttons;
}

/**
 * @brief EndEffectorState::getAccelerometerXValue
 * @return
 */
inline
EndEffectorState::EActionType
EndEffectorState::getButtonStatus(EndEffectorState::EButtonType button)
{
    return _buttons.at(button).action;
}

inline
uint32_t EndEffectorState::getAccelerometerXValue() const
{
  return _accelerometer_x_value;
}

/**
 * @brief EndEffectorState::getAccelerometerYValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerYValue() const
{
  return _accelerometer_y_value;
}

/**
 * @brief EndEffectorState::getAccelerometerZValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerZValue() const
{
  return _accelerometer_z_value;
}

/**
 * @brief EndEffectorState::getCollisionStatus
 * @return
 */
inline
bool EndEffectorState::getCollisionStatus() const
{
  return _collision_status;
}

} // namespace model
} // namespace common

#endif // END_EFFECTOR_STATE_HPP
