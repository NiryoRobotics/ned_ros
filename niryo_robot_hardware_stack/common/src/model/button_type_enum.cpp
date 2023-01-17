/*
    button_type_enum.cpp
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

#include "common/model/button_type_enum.hpp"
#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief ButtonTypeEnum::ButtonTypeEnum
 * @param e
 */
ButtonTypeEnum::ButtonTypeEnum(EButtonType e) : AbstractEnum<ButtonTypeEnum, EButtonType>(e) {}

/**
 * @brief ButtonTypeEnum::ButtonTypeEnum
 * @param str
 */
ButtonTypeEnum::ButtonTypeEnum(const char *const str) : AbstractEnum<ButtonTypeEnum, EButtonType>(str) {}

/**
 * @brief ButtonTypeEnum::initialize
 * @return
 */
std::map<EButtonType, std::string> ButtonTypeEnum::initialize()
{
    std::map<EButtonType, std::string> m;

    m[EButtonType::FREE_DRIVE_BUTTON] = "free_drive";
    m[EButtonType::SAVE_POSITION_BUTTON] = "save_position";
    m[EButtonType::CUSTOM_BUTTON] = "custom";
    m[EButtonType::UNKNOWN] = "unknown";

    return m;
}

}  // namespace model
}  // namespace common
