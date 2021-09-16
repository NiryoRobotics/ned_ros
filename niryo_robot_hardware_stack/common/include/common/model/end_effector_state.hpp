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

#ifndef END_EFFECTOR_STATE_H
#define END_EFFECTOR_STATE_H

#include "abstract_hardware_state.hpp"

#include <stdint.h>
#include <string>
#include <cassert>
#include <sstream>

#include "hardware_type_enum.hpp"
#include "button_type_enum.hpp"
#include "action_type_enum.hpp"

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
        /**
         * @brief The Button struct describes the current state of a button (not its config)
         */
        struct Button
        {
          EButtonType type{EButtonType::UNKNOWN};
          EActionType action{EActionType::NO_ACTION};

          std::string str() const
          {
            std::ostringstream ss;
            ss << "Button (" << ButtonTypeEnum(type).toString() << ") : "
               << ActionTypeEnum(action).toString();
            return ss.str();
          }
        };

        struct Vector3D
        {
          uint32_t x{};
          uint32_t y{};
          uint32_t z{};

          std::string str() const
          {
            std::ostringstream ss;
            ss << "(" << x << "," << y << "," << z << ")";
            return ss.str();
          }
        };

    public:
        EndEffectorState();
        EndEffectorState(uint8_t id);
        EndEffectorState(const EndEffectorState& state);

        virtual ~EndEffectorState() override;

        void configureButton(uint8_t id, EButtonType button_type);

        // AbstractHardwareState interface
        virtual std::string str() const override;

        // IObject interface
    public:
        virtual bool isValid() const override;

    public:
        void setButtonStatus(uint8_t id, EActionType action);
        std::array<Button, 3> getButtonsStatus() const;

        uint32_t getAccelerometerXValue() const;
        uint32_t getAccelerometerYValue() const;
        uint32_t getAccelerometerZValue() const;

        void setAccelerometerXValue(const uint32_t& xValue);
        void setAccelerometerYValue(const uint32_t& yValue);
        void setAccelerometerZValue(const uint32_t& zValue);

        bool getCollisionStatus() const;
        void setCollisionStatus(bool getCollisionStatus);

    private:
        std::array<Button, 3> _buttons_list;
        Vector3D _accelerometer_values{};

        bool _collision_status{false};
};

/**
 * @brief EndEffectorState::getButtonsStatus
 * @return
 */
inline
std::array<EndEffectorState::Button, 3>
EndEffectorState::getButtonsStatus() const
{
  return _buttons_list;
}

/**
 * @brief EndEffectorState::getAccelerometerXValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerXValue() const
{
  return _accelerometer_values.x;
}

/**
 * @brief EndEffectorState::getAccelerometerYValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerYValue() const
{
  return _accelerometer_values.y;
}

/**
 * @brief EndEffectorState::getAccelerometerZValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerZValue() const
{
  return _accelerometer_values.z;
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

#endif // END_EFFECTOR_STATE_H
