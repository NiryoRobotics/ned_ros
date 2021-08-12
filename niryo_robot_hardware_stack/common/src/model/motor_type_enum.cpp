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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include "common/model/motor_type_enum.hpp"
#include <map>
#include <string>

namespace common
{
namespace model
{

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
std::map<EMotorType, std::string>
MotorTypeEnum::initialize()
{
    std::map<EMotorType, std::string> m;

    m[EMotorType::STEPPER]          = "Niryo Stepper";
    m[EMotorType::XL430]            = "DXL XL430";
    m[EMotorType::XL320]            = "DXL XL-320";
    m[EMotorType::XL330]            = "DXL XL330";
    m[EMotorType::XC430]            = "DXL XC430";
    m[EMotorType::END_EFFECTOR]     = "End Effector";
    m[EMotorType::UNKNOWN]          = "DXL UNKOWN";

    return m;
}

}  // namespace model
}  // namespace common

