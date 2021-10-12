/*
button_type_enum.hpp
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

#ifndef BUTTON_TYPE_ENUM_H
#define BUTTON_TYPE_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EButtonType enum
 */
enum class EButtonType {
                        UNKNOWN = 0,
                        FREE_DRIVE_BUTTON,
                        SAVE_POSITION_BUTTON,
                        CUSTOM_BUTTON,
                      };
/**
 * @brief Specialization of AbstractEnum for Acknowledge status enum
 */
class ButtonTypeEnum : public AbstractEnum<ButtonTypeEnum, EButtonType>
{
public:
    ButtonTypeEnum(EButtonType e=EButtonType::UNKNOWN);
    ButtonTypeEnum(const char* str);

private:
    friend class AbstractEnum<ButtonTypeEnum, EButtonType>;
    static std::map<EButtonType, std::string> initialize();
};

} // model
} // common

#endif // BUTTON_TYPE_ENUM_H
