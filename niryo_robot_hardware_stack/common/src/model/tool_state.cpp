/*
    tool_state.cpp
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
#include "common/model/tool_state.hpp"
#include <sstream>
#include <string>
#include <utility>

namespace common
{
namespace model
{

/**
 * @brief ToolState::ToolState
 * @param name
 * @param type
 * @param id
 */
ToolState::ToolState(std::string name, EHardwareType type, uint8_t id) : DxlMotorState(type, EComponentType::TOOL, id), _tool_name(std::move(name))
{
    _led_state = 2;  // green if valid
}

/**
 * @brief ToolState::setName
 * @param name
 */
void ToolState::setName(std::string name) { _tool_name = std::move(name); }

/**
 * @brief ToolState::setLedState
 * @param led_state
 */
void ToolState::setLedState(int led_state) { _led_state = led_state; }

// ***********************
//  DxlMotorState intf
// ***********************

/**
 * @brief ToolState::reset
 */
void ToolState::reset()
{
    DxlMotorState::reset();
    _tool_name = "No Tool";
    _state = TOOL_STATE_PING_ERROR;
}

/**
 * @brief ToolState::str
 * @return
 */
std::string ToolState::str() const
{
    std::ostringstream ss;

    ss << "ToolState : ";
    ss << "name: "
       << "\"" << _tool_name << "\""
       << ", ";
    ss << "connected: " << (_connected ? "true" : "false") << ", ";
    ss << "\n---\n";
    ss << "\n";
    ss << DxlMotorState::str();

    return ss.str();
}

}  // namespace model
}  // namespace common
