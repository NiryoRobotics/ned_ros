/*
    stepper_command_type_enum.hpp
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

#include "common/model/stepper_command_type_enum.hpp"

#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief StepperCommandTypeEnum::StepperCommandTypeEnum
 * @param e
 */
StepperCommandTypeEnum::StepperCommandTypeEnum(EStepperCommandType e) : AbstractEnum<StepperCommandTypeEnum, EStepperCommandType>(e) {}

/**
 * @brief StepperCommandTypeEnum::StepperCommandTypeEnum
 * @param str
 */
StepperCommandTypeEnum::StepperCommandTypeEnum(const char *const str) : AbstractEnum<StepperCommandTypeEnum, EStepperCommandType>(str) {}

/**
 * @brief StepperCommandTypeEnum::initialize
 * @return
 */
std::map<EStepperCommandType, std::string> StepperCommandTypeEnum::initialize()
{
    std::map<EStepperCommandType, std::string> m;

    m[EStepperCommandType::CMD_TYPE_NONE] = "none";
    m[EStepperCommandType::CMD_TYPE_POSITION] = "position";
    m[EStepperCommandType::CMD_TYPE_VELOCITY] = "velocity";
    m[EStepperCommandType::CMD_TYPE_EFFORT] = "effort";
    m[EStepperCommandType::CMD_TYPE_TORQUE] = "torque";
    m[EStepperCommandType::CMD_TYPE_SYNCHRONIZE] = "synchronize";
    m[EStepperCommandType::CMD_TYPE_RELATIVE_MOVE] = "relative move";
    m[EStepperCommandType::CMD_TYPE_MAX_EFFORT] = "max effort";
    m[EStepperCommandType::CMD_TYPE_MICRO_STEPS] = "micro steps";
    m[EStepperCommandType::CMD_TYPE_POSITION_OFFSET] = "position offset";
    m[EStepperCommandType::CMD_TYPE_CALIBRATION] = "calibration";
    m[EStepperCommandType::CMD_TYPE_CONVEYOR] = "conveyor";
    m[EStepperCommandType::CMD_TYPE_UPDATE_CONVEYOR] = "update conveyor";
    m[EStepperCommandType::CMD_TYPE_LEARNING_MODE] = "learning mode";
    m[EStepperCommandType::CMD_TYPE_PING] = "ping";
    m[EStepperCommandType::CMD_TYPE_CALIBRATION_SETUP] = "calibration setup";
    m[EStepperCommandType::CMD_TYPE_VELOCITY_PROFILE] = "velocity profile";
    m[EStepperCommandType::CMD_TYPE_READ_HOMING_ABS_POSITION] = "homing abs read position";
    m[EStepperCommandType::CMD_TYPE_WRITE_HOMING_ABS_POSITION] = "homing abs write position";
    m[EStepperCommandType::CMD_TYPE_UNKNOWN] = "unknown type";

    return m;
}

}  // namespace model
}  // namespace common
