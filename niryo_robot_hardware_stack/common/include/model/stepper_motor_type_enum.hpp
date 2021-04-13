/*
    stepper_motor_type_enum.hpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_MOTOR_TYPE_ENUM
#define STEPPER_MOTOR_TYPE_ENUM

#include <map>
#include <string>

#include "common_defs.hpp"
#include "abstract_enum.hpp"

namespace common {
    namespace model {

        enum class EStepperMotorType {
                                       MOTOR_TYPE_STEPPER=1,
                                       MOTOR_TYPE_UNKNOWN=100
                                     };

        /**
         * @brief Specialization of AbstractEnum for Acknowledge status enum
         */
        class StepperMotorTypeEnum : public AbstractEnum<StepperMotorTypeEnum, EStepperMotorType>
        {
        public:
            StepperMotorTypeEnum(EStepperMotorType e=EStepperMotorType::MOTOR_TYPE_UNKNOWN);
            StepperMotorTypeEnum(const char* const str);
            ~StepperMotorTypeEnum() {}

        private:
            friend class AbstractEnum<StepperMotorTypeEnum, EStepperMotorType>;
            static std::map<EStepperMotorType, std::string> initialize();
        };

    } // model
} //common

#endif // STEPPER_MOTOR_TYPE_ENUM
