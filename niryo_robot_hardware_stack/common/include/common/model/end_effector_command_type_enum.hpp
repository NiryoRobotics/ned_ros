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

#ifndef END_EFFECTOR_COMMAND_TYPE_ENUM_H
#define END_EFFECTOR_COMMAND_TYPE_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EEndEffectorCommandType enum
 */
enum class EEndEffectorCommandType { CMD_TYPE_DIGITAL_OUTPUT = 4,
                                     CMD_TYPE_PING=5,
                                     CMD_TYPE_SET_COLLISION_THRESH = 6,
                                     CMD_TYPE_SET_COLLISION_THRESH_ALGO_2 = 7,
                                     CMD_TYPE_UNKNOWN=100
                                   };

/**
 * @brief Specialization of AbstractEnum for Acknowledge status enum
 */
class EndEffectorCommandTypeEnum : public AbstractEnum<EndEffectorCommandTypeEnum, EEndEffectorCommandType>
{
public:
    EndEffectorCommandTypeEnum(EEndEffectorCommandType e=EEndEffectorCommandType::CMD_TYPE_UNKNOWN);
    EndEffectorCommandTypeEnum(const char* str);

private:
    friend class AbstractEnum<EndEffectorCommandTypeEnum, EEndEffectorCommandType>;
    static std::map<EEndEffectorCommandType, std::string> initialize();
};

} // model
} // common

#endif // END_EFFECTOR_COMMAND_TYPE_ENUM_H
