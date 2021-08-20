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

bool common::model::EndEffectorState::isValid() const
{
    return (0 != getId() && EHardwareType::END_EFFECTOR == getType());
}


}  // namespace model
}  // namespace common
