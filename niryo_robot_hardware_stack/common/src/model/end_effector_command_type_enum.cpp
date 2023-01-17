/*
    end_effector_command_type_enum.hpp
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

#include "common/model/end_effector_command_type_enum.hpp"

#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief EndEffectorCommandTypeEnum::EndEffectorCommandTypeEnum
 * @param e
 */
EndEffectorCommandTypeEnum::EndEffectorCommandTypeEnum(EEndEffectorCommandType e) : AbstractEnum<EndEffectorCommandTypeEnum, EEndEffectorCommandType>(e) {}

/**
 * @brief EndEffectorCommandTypeEnum::EndEffectorCommandTypeEnum
 * @param str
 */
EndEffectorCommandTypeEnum::EndEffectorCommandTypeEnum(const char *const str) : AbstractEnum<EndEffectorCommandTypeEnum, EEndEffectorCommandType>(str) {}

/**
 * @brief EndEffectorCommandTypeEnum::initialize
 * @return
 */
std::map<EEndEffectorCommandType, std::string> EndEffectorCommandTypeEnum::initialize()
{
    std::map<EEndEffectorCommandType, std::string> m;

    m[EEndEffectorCommandType::CMD_TYPE_DIGITAL_OUTPUT] = "digit input cmd";
    m[EEndEffectorCommandType::CMD_TYPE_PING] = "ping";
    m[EEndEffectorCommandType::CMD_TYPE_SET_COLLISION_THRESH] = "set collision threshold cmd";
    m[EEndEffectorCommandType::CMD_TYPE_UNKNOWN] = "unknown type";

    return m;
}

}  // namespace model
}  // namespace common
