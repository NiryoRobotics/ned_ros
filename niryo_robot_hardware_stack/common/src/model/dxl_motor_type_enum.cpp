/*
    dxl_motor_type_enum.hpp
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

#include "model/dxl_motor_type_enum.hpp"

using namespace std;

namespace common {
    namespace model {

        /**
         *
         */
        DxlMotorTypeEnum::DxlMotorTypeEnum(EDxlMotorType e):
            AbstractEnum<DxlMotorTypeEnum, EDxlMotorType>(e)
        {}

        /**
         *
         */
        DxlMotorTypeEnum::DxlMotorTypeEnum(const char* const str):
            AbstractEnum<DxlMotorTypeEnum, EDxlMotorType>(str)
        {}

        /**
         *
         */
        map<EDxlMotorType, string>
        DxlMotorTypeEnum::initialize()
        {
            map<EDxlMotorType, string> m;

            m[EDxlMotorType::MOTOR_TYPE_XL430]    = "xl430";
            m[EDxlMotorType::MOTOR_TYPE_XL320]    = "xl320";
            m[EDxlMotorType::MOTOR_TYPE_XL330]    = "xl330";
            m[EDxlMotorType::MOTOR_TYPE_XC430]    = "xc430";
            m[EDxlMotorType::MOTOR_TYPE_UNKNOWN]  = "unknown";

            return m;
        }

    } // model
} //common

