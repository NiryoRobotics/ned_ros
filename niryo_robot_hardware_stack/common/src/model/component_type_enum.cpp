/*
    component_type_enum.hpp
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

#include "common/model/component_type_enum.hpp"
#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief ComponentTypeEnum::ComponentTypeEnum
 * @param e
 */
ComponentTypeEnum::ComponentTypeEnum(EComponentType e) : AbstractEnum<ComponentTypeEnum, EComponentType>(e) {}

/**
 * @brief ComponentTypeEnum::ComponentTypeEnum
 * @param str
 */
ComponentTypeEnum::ComponentTypeEnum(const char *const str) : AbstractEnum<ComponentTypeEnum, EComponentType>(str) {}

/**
 * @brief ComponentTypeEnum::initialize
 * @return
 */
std::map<EComponentType, std::string> ComponentTypeEnum::initialize()
{
    std::map<EComponentType, std::string> m;

    m[EComponentType::TOOL] = "tool";
    m[EComponentType::CONVEYOR] = "conveyor";
    m[EComponentType::JOINT] = "joint";
    m[EComponentType::END_EFFECTOR] = "end effector";
    m[EComponentType::UNKNOWN] = "unknown";

    return m;
}

}  // namespace model
}  // namespace common
