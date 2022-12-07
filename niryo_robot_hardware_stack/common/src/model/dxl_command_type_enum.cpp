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

#include "common/model/dxl_command_type_enum.hpp"

#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief DxlCommandTypeEnum::DxlCommandTypeEnum
 * @param e
 */
DxlCommandTypeEnum::DxlCommandTypeEnum(EDxlCommandType e) : AbstractEnum<DxlCommandTypeEnum, EDxlCommandType>(e) {}

/**
 * @brief DxlCommandTypeEnum::DxlCommandTypeEnum
 * @param str
 */
DxlCommandTypeEnum::DxlCommandTypeEnum(const char *const str) : AbstractEnum<DxlCommandTypeEnum, EDxlCommandType>(str) {}

/**
 * @brief DxlCommandTypeEnum::initialize
 * @return
 */
std::map<EDxlCommandType, std::string> DxlCommandTypeEnum::initialize()
{
    std::map<EDxlCommandType, std::string> m;

    m[EDxlCommandType::CMD_TYPE_POSITION] = "position";
    m[EDxlCommandType::CMD_TYPE_VELOCITY] = "velocity";
    m[EDxlCommandType::CMD_TYPE_EFFORT] = "effort";
    m[EDxlCommandType::CMD_TYPE_TORQUE] = "torque";
    m[EDxlCommandType::CMD_TYPE_PING] = "ping";
    m[EDxlCommandType::CMD_TYPE_LEARNING_MODE] = "learning mode";
    m[EDxlCommandType::CMD_TYPE_PID] = "PID";
    m[EDxlCommandType::CMD_TYPE_CONTROL_MODE] = "Control Mode";
    m[EDxlCommandType::CMD_TYPE_LED_STATE] = "Led State";
    m[EDxlCommandType::CMD_TYPE_PROFILE] = "Velocity and Acceleration profile";
    m[EDxlCommandType::CMD_TYPE_STARTUP] = "Startup Configuration";
    m[EDxlCommandType::CMD_TYPE_TEMPERATURE_LIMIT] = "Temperature limit";
    m[EDxlCommandType::CMD_TYPE_SHUTDOWN] = "Shutdown when error occurs";
    m[EDxlCommandType::CMD_TYPE_UNKNOWN] = "unknown type";

    return m;
}

}  // namespace model
}  // namespace common
