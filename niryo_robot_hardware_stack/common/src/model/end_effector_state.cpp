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
}

/**
 * @brief EndEffectorState::~EndEffectorState
 */
EndEffectorState::~EndEffectorState()
{
}

/**
 * @brief EndEffectorState::configureButton
 * @param id
 * @param button_type
 * @return
 */
void EndEffectorState::configureButton(uint8_t id, EButtonType button_type)
{
   assert(id <= 3);

  _buttons_list[id - 1].type = button_type;
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
  return (EHardwareType::END_EFFECTOR == getType() &&
          EButtonType::UNKNOWN != _buttons_list.at(0).type &&
          EButtonType::UNKNOWN != _buttons_list.at(1).type &&
          EButtonType::UNKNOWN != _buttons_list.at(2).type);
}

/**
 * @brief EndEffectorState::setButtonStatus
 * @param id
 * @param action
 */
void EndEffectorState::setButtonStatus(uint8_t id, EndEffectorState::EActionType action)
{
  assert(id <= 3);

  _buttons_list[id - 1].action = action;
}

/**
 * @brief EndEffectorState::setAccelerometerXValue
 * @param xValue
 */
void EndEffectorState::setAccelerometerXValue(const uint32_t& xValue)
{
    _accelerometer_values.x = xValue;
}

/**
 * @brief EndEffectorState::setAccelerometerYValue
 * @param yValue
 */
void EndEffectorState::setAccelerometerYValue(const uint32_t& yValue)
{
  _accelerometer_values.y = yValue;
}

/**
 * @brief EndEffectorState::setAccelerometerZValue
 * @param zValue
 */
void EndEffectorState::setAccelerometerZValue(const uint32_t &zValue)
{
  _accelerometer_values.z = zValue;
}

// ***************** set collision status ******************* //

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
