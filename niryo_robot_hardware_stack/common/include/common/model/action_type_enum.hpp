/*
    action_type_enum.hpp
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

#ifndef ACTION_TYPE_ENUM_H
#define ACTION_TYPE_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EActionType enum
 */
enum class EActionType {
                          HANDLE_HELD_ACTION = 0,
                          LONG_PUSH_ACTION = 1,
                          SINGLE_PUSH_ACTION = 2,
                          DOUBLE_PUSH_ACTION = 3,
                          NO_ACTION = 100
                       };

/**
 * @brief Specialization of AbstractEnum for Acknowledge status enum
 */
class ActionTypeEnum : public AbstractEnum<ActionTypeEnum, EActionType>
{
public:
    ActionTypeEnum(EActionType e=EActionType::NO_ACTION);
    ActionTypeEnum(const char* str);

private:
    friend class AbstractEnum<ActionTypeEnum, EActionType>;
    static std::map<EActionType, std::string> initialize();
};

} // model
} // common

#endif // ACTION_TYPE_ENUM_H
