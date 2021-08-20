/*
    end_effector_state.cpp
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
#include "common/model/end_effector_state.hpp"
#include <sstream>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief EndEffectorState::EndEffectorState
 */
EndEffectorState::EndEffectorState() :
  AbstractHardwareState()
{
}

EndEffectorState::EndEffectorState(uint8_t id) :
  AbstractHardwareState(EHardwareType::END_EFFECTOR, EBusProtocol::TTL, id)
{
  _buttons.insert(std::make_pair(EButtonType::FREE_DRIVE_BUTTON, Button{"Free Driver Button", EActionType::NO_ACTION}));
  _buttons.insert(std::make_pair(EButtonType::SAVE_POS_BUTTON, Button{"Save Position Button", EActionType::NO_ACTION}));
  _buttons.insert(std::make_pair(EButtonType::CUSTOM_BUTTON, Button{"Custom Button", EActionType::NO_ACTION}));
}

/**
 * @brief EndEffectorState::~EndEffectorState
 */
EndEffectorState::~EndEffectorState()
{
}

// ***********************
//  AbstractHardwareInterface intf
// ***********************

/**
 * @brief EndEffectorState::str
 * @return
 */
std::string EndEffectorState::str() const
{
    std::ostringstream ss;

    ss << "EndEffectorState : ";
    ss << "\n";
    ss << AbstractHardwareState::str();

    return ss.str();
}

/**
 * @brief common::model::EndEffectorState::isValid
 * @return
 */
bool common::model::EndEffectorState::isValid() const
{
    return (0 != getId() && EHardwareType::END_EFFECTOR == getType() && _buttons.size() == 3);
}

/**
 * @brief EndEffectorState::setButtonStatus
 * @param button
 * @param action
 */
void EndEffectorState::setButtonStatus(EButtonType button, EActionType action)
{
    _buttons.at(button).action = action;
}

/**
 * @brief EndEffectorState::setAccelerometerXValue
 * @param accelerometer_x_value
 */
void EndEffectorState::setAccelerometerXValue(const uint32_t &accelerometer_x_value)
{
  _accelerometer_x_value = accelerometer_x_value;
}

/**
 * @brief EndEffectorState::setAccelerometerYValue
 * @param accelerometer_y_value
 */
void EndEffectorState::setAccelerometerYValue(const uint32_t &accelerometer_y_value)
{
  _accelerometer_y_value = accelerometer_y_value;
}

/**
 * @brief EndEffectorState::setAccelerometerZValue
 * @param accelerometer_z_value
 */
void EndEffectorState::setAccelerometerZValue(const uint32_t &accelerometer_z_value)
{
  _accelerometer_z_value = accelerometer_z_value;
}

/**
 * @brief EndEffectorState::setCollisionStatus
 * @param collision_satus
 */
void EndEffectorState::setCollisionStatus(bool collision_satus)
{
  _collision_status = collision_satus;
}

}  // namespace model
}  // namespace common
