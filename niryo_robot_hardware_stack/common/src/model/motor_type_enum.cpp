/*
    motor_type_enum.hpp
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

#include "model/motor_type_enum.hpp"

using namespace std;

namespace common {
    namespace model {

        /**
         * @brief MotorTypeEnum::MotorTypeEnum
         * @param e
         */
        MotorTypeEnum::MotorTypeEnum(EMotorType e):
            AbstractEnum<MotorTypeEnum, EMotorType>(e)
        {}

        /**
         * @brief MotorTypeEnum::MotorTypeEnum
         * @param str
         */
        MotorTypeEnum::MotorTypeEnum(const char* const str):
            AbstractEnum<MotorTypeEnum, EMotorType>(str)
        {}

        /**
         * @brief MotorTypeEnum::initialize
         * @return
         */
        map<EMotorType, string>
        MotorTypeEnum::initialize()
        {
            map<EMotorType, string> m;

            m[EMotorType::MOTOR_TYPE_STEPPER]    = "stepper";
            m[EMotorType::MOTOR_TYPE_XL430]    = "xl430";
            m[EMotorType::MOTOR_TYPE_XL320]    = "xl320";
            m[EMotorType::MOTOR_TYPE_XL330]    = "xl330";
            m[EMotorType::MOTOR_TYPE_XC430]    = "xc430";
            m[EMotorType::MOTOR_TYPE_UNKNOWN]  = "unknown";

            return m;
        }

    } // model
} //common

