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

#ifndef COMPONENT_TYPE_ENUM_H
#define COMPONENT_TYPE_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EComponentType enum
 */
enum class EComponentType {
                        TOOL,
                        CONVEYOR,
                        JOINT,
                        END_EFFECTOR,
                        UNKNOWN
                      };

/**
 * @brief Specialization of AbstractEnum for Acknowledge status enum
 */
class ComponentTypeEnum : public AbstractEnum<ComponentTypeEnum, EComponentType>
{
public:
    ComponentTypeEnum(EComponentType e=EComponentType::UNKNOWN);
    ComponentTypeEnum(const char* str);

private:
    friend class AbstractEnum<ComponentTypeEnum, EComponentType>;
    static std::map<EComponentType, std::string> initialize();
};

} // model
} // common

#endif // COMPONENT_TYPE_ENUM_H
