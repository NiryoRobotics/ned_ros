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
#include "common/model/action_type_enum.hpp"
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

/**
 * @brief EndEffectorState::EndEffectorState
 * @param id
 */
EndEffectorState::EndEffectorState(uint8_t id) :
  AbstractHardwareState(EHardwareType::END_EFFECTOR,
                        EComponentType::END_EFFECTOR,
                        EBusProtocol::TTL, id)
{
}

/**
 * @brief EndEffectorState::EndEffectorState : copy constructor
 * @param state
 */
EndEffectorState::EndEffectorState(const EndEffectorState &state) :
  AbstractHardwareState(state)
{
    _buttons_list = state._buttons_list;
    _accelerometer_values = state._accelerometer_values;
    _collision_status = state._collision_status;
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

  _buttons_list[id - 1].actions.push(EActionType::NO_ACTION);
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

    ss << "EndEffectorState : \n";

    ss << "Buttons : \n";
    for (size_t i = 0; i < _buttons_list.size(); ++i)
      ss << i << " " << _buttons_list.at(i).str();

    ss << "Acceleration: \n";
    ss << _accelerometer_values.str();

    ss << "\n---\n";
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
  return (EHardwareType::END_EFFECTOR == getHardwareType() &&
          EButtonType::UNKNOWN != _buttons_list.at(0).type &&
          EButtonType::UNKNOWN != _buttons_list.at(1).type &&
          EButtonType::UNKNOWN != _buttons_list.at(2).type);
}

/**
 * @brief EndEffectorState::setButtonStatus
 * @param id
 * @param action
 */
void EndEffectorState::setButtonStatus(uint8_t id, EActionType action)
{
  assert(id <= 3);

  common::model::Button button = _buttons_list[id - 1];
  // do not add 2 no action states consecutive
  if (button.actions.back() == EActionType::NO_ACTION &&
          action == EActionType::NO_ACTION)
      return;
  if (button.actions.back() == EActionType::SINGLE_PUSH_ACTION ||
        button.actions.back() == EActionType::DOUBLE_PUSH_ACTION ||
        button.actions.back() == EActionType::LONG_PUSH_ACTION)
  {
      button.actions.push(action);
      button.setDelay();
  }
  else if (!button.isNeedToSkip())
  {
      button.actions.push(action);
  }
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

/**
 * @brief EndEffectorState::setDigitalIn
 * @param digital_in
 */
void EndEffectorState::setDigitalIn(bool digital_in)
{
  _digital_in = digital_in;
}

/**
 * @brief EndEffectorState::setDigitalOut
 * @param digital_out
 */
void EndEffectorState::setDigitalOut(bool digital_out)
{
  _digital_out = digital_out;
}

}  // namespace model
}  // namespace common
