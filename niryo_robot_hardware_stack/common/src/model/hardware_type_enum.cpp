/*
    hardware_type_enum.hpp
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

#include "common/model/hardware_type_enum.hpp"
#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief HardwareTypeEnum::HardwareTypeEnum
 * @param e
 */
HardwareTypeEnum::HardwareTypeEnum(EHardwareType e):
    AbstractEnum<HardwareTypeEnum, EHardwareType>(e)
{}

/**
 * @brief HardwareTypeEnum::HardwareTypeEnum
 * @param str
 */
HardwareTypeEnum::HardwareTypeEnum(const char* const str):
    AbstractEnum<HardwareTypeEnum, EHardwareType>(str)
{}

/**
 * @brief HardwareTypeEnum::initialize
 * @return
 */
std::map<EHardwareType, std::string>
HardwareTypeEnum::initialize()
{
    std::map<EHardwareType, std::string> m;

    m[EHardwareType::STEPPER]          = "Niryo Stepper";
    m[EHardwareType::XL430]            = "DXL XL430";
    m[EHardwareType::XL320]            = "DXL XL-320";
    m[EHardwareType::XL330]            = "DXL XL330";
    m[EHardwareType::XC430]            = "DXL XC430";
    m[EHardwareType::END_EFFECTOR]     = "End Effector";
    m[EHardwareType::UNKNOWN]          = "DXL UNKOWN";

    return m;
}

}  // namespace model
}  // namespace common

