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

#ifndef STEPPER_COMMAND_TYPE_ENUM_H
#define STEPPER_COMMAND_TYPE_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
    namespace model
    {

        /**
         * @brief The EStepperCommandType enum
         */
        enum class EStepperCommandType
        {
            CMD_TYPE_NONE = 0,
            CMD_TYPE_POSITION = 1,
            CMD_TYPE_VELOCITY = 2,
            CMD_TYPE_EFFORT = 3,
            CMD_TYPE_TORQUE = 4,
            CMD_TYPE_SYNCHRONIZE = 5,
            CMD_TYPE_RELATIVE_MOVE = 6,
            CMD_TYPE_MAX_EFFORT = 7,
            CMD_TYPE_MICRO_STEPS = 8,
            CMD_TYPE_POSITION_OFFSET = 9,
            CMD_TYPE_CALIBRATION = 10,
            CMD_TYPE_CONVEYOR = 11,
            CMD_TYPE_UPDATE_CONVEYOR = 12,
            CMD_TYPE_LEARNING_MODE = 13,
            CMD_TYPE_PING = 14,
            CMD_TYPE_CALIBRATION_SETUP = 15,
            CMD_TYPE_VELOCITY_PROFILE = 16,
            CMD_TYPE_WRITE_HOMING_ABS_POSITION = 17,
            CMD_TYPE_READ_HOMING_ABS_POSITION = 18,
            CMD_TYPE_FACTORY_CALIBRATION = 19,
            CMD_TYPE_OPERATING_MODE = 20,
            CMD_TYPE_UNKNOWN = 100
        };

        /**
         * @brief Specialization of AbstractEnum for Acknowledge status enum
         */
        class StepperCommandTypeEnum : public AbstractEnum<StepperCommandTypeEnum, EStepperCommandType>
        {
        public:
            StepperCommandTypeEnum(EStepperCommandType e = EStepperCommandType::CMD_TYPE_UNKNOWN);
            StepperCommandTypeEnum(const char *str);

        private:
            friend class AbstractEnum<StepperCommandTypeEnum, EStepperCommandType>;
            static std::map<EStepperCommandType, std::string> initialize();
        };

    } // model
} // common

#endif // STEPPER_COMMAND_TYPE_ENUM_H
