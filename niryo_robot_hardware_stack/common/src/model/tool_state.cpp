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

namespace common
{
namespace model
{

/**
 * @brief ToolState::ToolState
 */
ToolState::ToolState() :
    DxlMotorState()
{
}

/**
 * @brief ToolState::ToolState
 * @param name
 * @param type
 * @param id
 */
ToolState::ToolState(std::string name, EHardwareType type, uint8_t id) :
    DxlMotorState(type, EBusProtocol::TTL, id, true),
    _tool_name(name),
    _connected(true),
    _position(0.0)
{
}

/**
 * @brief ToolState::~ToolState
 */
ToolState::~ToolState()
{
}

/**
 * @brief ToolState::setName
 * @param name
 */
void ToolState::setName(std::string name)
{
    _tool_name = name;
}

/**
 * @brief ToolState::setPosition
 * @param position
 */
void ToolState::setPosition(double position)
{
    _position = position;
}

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
    _position = 0.0;
}

/**
 * @brief ToolState::str
 * @return
 */
std::string ToolState::str() const
{
    std::ostringstream ss;

    ss << "ToolState : ";
    ss << "name: " << "\"" << _tool_name << "\"" << ", ";
    ss << "position: " << _position << ", ";
    ss << "connected: " << (_connected ? "true" : "false");
    ss << "\n";
    ss << DxlMotorState::str();

    return ss.str();
}


}  // namespace model
}  // namespace common
