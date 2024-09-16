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

#ifndef HARDWARE_TYPE_ENUM_H
#define HARDWARE_TYPE_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EHardwareType enum
 */
enum class EHardwareType {
                            STEPPER=1,
                            XL430=2,
                            XL320=3,
                            XL330=4,
                            XC430=5,
                            XM430=6,
                            NED3PRO_STEPPER = 7,
                            XH430=8,
                            FAKE_DXL_MOTOR=20,
                            FAKE_STEPPER_MOTOR=21,
                            FAKE_END_EFFECTOR=22,
                            END_EFFECTOR=30,
                            NED3PRO_END_EFFECTOR=31,
                            PROGRAM_PLAYER=40,
                            UNKNOWN=100
                          };

/**
 * @brief Specialization of AbstractEnum for Acknowledge status enum
 */
class HardwareTypeEnum : public AbstractEnum<HardwareTypeEnum, EHardwareType>
{
public:
    HardwareTypeEnum(EHardwareType e=EHardwareType::UNKNOWN);
    HardwareTypeEnum(const char* str);


    static bool isMotor(EHardwareType hw_type){
        switch(hw_type) {
            case EHardwareType::STEPPER:
            case EHardwareType::XL430:
            case EHardwareType::XL320:
            case EHardwareType::XL330:
            case EHardwareType::XC430:
            case EHardwareType::XM430:
            case EHardwareType::NED3PRO_STEPPER:
            case EHardwareType::XH430:
                return true;
            default:
                return false;
        }
    }

private:
    friend class AbstractEnum<HardwareTypeEnum, EHardwareType>;
    static std::map<EHardwareType, std::string> initialize();


};

} // model
} // common

#endif // HARDWARE_TYPE_ENUM_H
