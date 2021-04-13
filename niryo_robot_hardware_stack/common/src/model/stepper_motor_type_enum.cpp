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

#include "model/stepper_motor_type_enum.hpp"

using namespace std;

namespace common {
    namespace model {

        /**
         *
         */
        StepperMotorTypeEnum::StepperMotorTypeEnum(EStepperMotorType e):
            AbstractEnum<StepperMotorTypeEnum, EStepperMotorType>(e)
        {}

        /**
         *
         */
        StepperMotorTypeEnum::StepperMotorTypeEnum(const char* const str):
            AbstractEnum<StepperMotorTypeEnum, EStepperMotorType>(str)
        {}

        /**
         *
         */
        map<EStepperMotorType, string>
        StepperMotorTypeEnum::initialize()
        {
            map<EStepperMotorType, string> m;

            m[EStepperMotorType::MOTOR_TYPE_STEPPER] = "stepper";
            m[EStepperMotorType::MOTOR_TYPE_UNKNOWN]  = "unknown";

            return m;
        }

    } // model
} //common

