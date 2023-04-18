/*
    action_type_enum.cpp
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

#include "common/model/action_type_enum.hpp"
#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief ActionTypeEnum::ActionTypeEnum
 * @param e
 */
ActionTypeEnum::ActionTypeEnum(EActionType e) : AbstractEnum<ActionTypeEnum, EActionType>(e) {}

/**
 * @brief ActionTypeEnum::ActionTypeEnum
 * @param str
 */
ActionTypeEnum::ActionTypeEnum(const char *const str) : AbstractEnum<ActionTypeEnum, EActionType>(str) {}

/**
 * @brief ActionTypeEnum::initialize
 * @return
 */
std::map<EActionType, std::string> ActionTypeEnum::initialize()
{
    std::map<EActionType, std::string> m;

    m[EActionType::HANDLE_HELD_ACTION] = "handle held action";
    m[EActionType::LONG_PUSH_ACTION] = "long push action";
    m[EActionType::SINGLE_PUSH_ACTION] = "single push action";
    m[EActionType::DOUBLE_PUSH_ACTION] = "double push action";
    m[EActionType::NO_ACTION] = "no action";

    return m;
}

}  // namespace model
}  // namespace common
