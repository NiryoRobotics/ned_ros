
// end_effector_state.hpp
// Copyright(C) 2020 Niryo
// All rights reserved.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http:// www.gnu.org/licenses/>.
//

#include "common/model/end_effector_state.hpp"

// std
#include <queue>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

// ros
#include <ros/ros.h>

// niryo
#include "common/model/abstract_hardware_state.hpp"
#include "common/model/action_type_enum.hpp"
#include "common/model/bus_protocol_enum.hpp"
#include "common/model/component_type_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief EndEffectorState::EndEffectorState
 */
EndEffectorState::EndEffectorState() : EndEffectorState(1) {}

/**
 * @brief EndEffectorState::EndEffectorState
 * @param id
 */
EndEffectorState::EndEffectorState(uint8_t id) : EndEffectorState(id, EHardwareType::END_EFFECTOR) {}

/**
 * @brief EndEffectorState::EndEffectorState
 */
EndEffectorState::EndEffectorState(uint8_t id, common::model::EHardwareType type) : AbstractHardwareState(type, EComponentType::END_EFFECTOR, EBusProtocol::TTL, id)
{
    _buttons_list.at(0) = std::make_shared<Button>();
    _buttons_list.at(1) = std::make_shared<Button>();
    _buttons_list.at(2) = std::make_shared<Button>();
}

/**
 * @brief EndEffectorState::configureButton
 * @param id
 * @param button_type
 * @return
 */
void EndEffectorState::configureButton(uint8_t id, EButtonType button_type)
{
    assert(id < 3);

    _buttons_list.at(id)->actions.push(EActionType::NO_ACTION);
    _buttons_list.at(id)->type = button_type;
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
        ss << i << " " << _buttons_list.at(i)->str() << "\n";

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
    return (EHardwareType::UNKNOWN != getHardwareType() && EButtonType::UNKNOWN != _buttons_list.at(0)->type && EButtonType::UNKNOWN != _buttons_list.at(1)->type &&
            EButtonType::UNKNOWN != _buttons_list.at(2)->type);
}

/**
 * @brief EndEffectorState::setButtonStatus
 * @param button_id
 * @param action
 */
void EndEffectorState::setButtonStatus(uint8_t button_id, EActionType action)
{
    assert(button_id < 3);

    auto button = _buttons_list.at(button_id);
    // do not add 2 no action states consecutive
    if (button->actions.back() == EActionType::NO_ACTION && action == EActionType::NO_ACTION)
        return;
    // add action as no action if last action is not no action state
    if (button->actions.back() != EActionType::NO_ACTION && action == EActionType::NO_ACTION)
    {
        button->actions.push(action);
    }
    // if action is single or double push, push to list
    else if (action == EActionType::SINGLE_PUSH_ACTION || action == EActionType::DOUBLE_PUSH_ACTION)
    {
        button->actions.push(action);
        button->setDelay();
    }
    else if (action == EActionType::LONG_PUSH_ACTION || (action == EActionType::HANDLE_HELD_ACTION && !button->needsToSkip()))
    {
        button->actions.push(action);
    }
}

/**
 * @brief EndEffectorState::setAccelerometerXValue
 * @param xValue
 */
void EndEffectorState::setAccelerometerXValue(const uint32_t &xValue) { _accelerometer_values.x = xValue; }

/**
 * @brief EndEffectorState::setAccelerometerYValue
 * @param yValue
 */
void EndEffectorState::setAccelerometerYValue(const uint32_t &yValue) { _accelerometer_values.y = yValue; }

/**
 * @brief EndEffectorState::setAccelerometerZValue
 * @param zValue
 */
void EndEffectorState::setAccelerometerZValue(const uint32_t &zValue) { _accelerometer_values.z = zValue; }

// ***************** set collision status ******************* //

/**
 * @brief EndEffectorState::setCollisionStatus
 * @param status
 */
void EndEffectorState::setCollisionStatus(bool status) { _collision_status = status; }

/**
 * @brief EndEffectorState::setCollisionThresh
 * @param thresh
 */
void EndEffectorState::setCollisionThresh(int thresh) { _collision_thresh = thresh; }

/**
 * @brief EndEffectorState::setCollisionThresh
 * @param thresh
 */
void EndEffectorState::setCollisionThreshAlgo2(int thresh) { _collision_thresh_algo_2 = thresh; }

/**
 * @brief EndEffectorState::setDigitalIn
 * @param digital_in
 */
void EndEffectorState::setDigitalIn(bool digital_in) { _digital_in = digital_in; }

/**
 * @brief EndEffectorState::setDigitalOut
 * @param digital_out
 */
void EndEffectorState::setDigitalOut(bool digital_out) { _digital_out = digital_out; }

//************************
//    Button subclass
//************************

/**
 * @brief EndEffectorState::Button::Button
 */
EndEffectorState::Button::Button() { actions.push(EActionType::NO_ACTION); }

/**
 * @brief EndEffectorState::Button::str
 * @return
 */
std::string EndEffectorState::Button::str() const
{
    std::ostringstream ss;
    ss << "Button (" << ButtonTypeEnum(type).toString() << ") : " << ActionTypeEnum(actions.front()).toString();
    return ss.str();
}

/**
 * @brief EndEffectorState::Button::isValid
 * @return
 */
bool EndEffectorState::Button::isValid() const { return (EButtonType::UNKNOWN != type); }

/**
 * @brief EndEffectorState::Button::reset
 */
void EndEffectorState::Button::reset()
{
    type = EButtonType::UNKNOWN;
    std::queue<EActionType> empty_queue;
    actions.swap(empty_queue);
}

/**
 * @brief EndEffectorState::Button::setDelay
 */
void EndEffectorState::Button::setDelay()
{
    _time_last_read_state = ros::Time::now().toSec();
    _need_delay = true;
}

/**
 * @brief EndEffectorState::Button::needsToSkip
 * @return
 */
bool EndEffectorState::Button::needsToSkip()
{
    if (_need_delay && (ros::Time::now().toSec() - _time_last_read_state) <= _time_avoid_duplicate_state)
        return true;

    _need_delay = false;
    return false;
}

}  // namespace model
}  // namespace common
