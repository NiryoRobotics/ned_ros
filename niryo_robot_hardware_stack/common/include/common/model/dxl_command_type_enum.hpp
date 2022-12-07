/*
dxl_command_type_enum.hpp
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

#ifndef DXL_COMMAND_TYPE_ENUM_H
#define DXL_COMMAND_TYPE_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
    namespace model
    {

        /**
         * @brief The EDxlCommandType enum
         */
        enum class EDxlCommandType
        {
            CMD_TYPE_POSITION = 1,
            CMD_TYPE_VELOCITY = 2,
            CMD_TYPE_EFFORT = 3,
            CMD_TYPE_TORQUE = 4,
            CMD_TYPE_PING = 5,
            CMD_TYPE_LEARNING_MODE = 6,
            CMD_TYPE_PID = 7,
            CMD_TYPE_CONTROL_MODE = 8,
            CMD_TYPE_LED_STATE = 9,
            CMD_TYPE_PROFILE = 10,
            CMD_TYPE_STARTUP = 20,
            CMD_TYPE_TEMPERATURE_LIMIT = 21,
            CMD_TYPE_SHUTDOWN = 22,
            CMD_TYPE_UNKNOWN = 100
        };

        /**
         * @brief Specialization of AbstractEnum for Acknowledge status enum
         */
        class DxlCommandTypeEnum : public AbstractEnum<DxlCommandTypeEnum, EDxlCommandType>
        {
        public:
            DxlCommandTypeEnum(EDxlCommandType e = EDxlCommandType::CMD_TYPE_UNKNOWN);
            DxlCommandTypeEnum(const char *str);

        private:
            friend class AbstractEnum<DxlCommandTypeEnum, EDxlCommandType>;
            static std::map<EDxlCommandType, std::string> initialize();
        };

    } // model
} // common

#endif // DXL_COMMAND_TYPE_ENUM_H
