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

#ifndef DXL_MOTOR_TYPE_ENUM
#define DXL_MOTOR_TYPE_ENUM

#include <map>
#include <string>

#include "common_defs.hpp"
#include "abstract_enum.hpp"

namespace common {
    namespace model {

        enum class EDxlMotorType { MOTOR_TYPE_XL430=2,
                                   MOTOR_TYPE_XL320=3,
                                   MOTOR_TYPE_XL330=4,
                                   MOTOR_TYPE_XC430=5,
                                   MOTOR_TYPE_UNKNOWN=100
                                 };

        /**
         * @brief Specialization of AbstractEnum for Acknowledge status enum
         */
        class DxlMotorTypeEnum : public AbstractEnum<DxlMotorTypeEnum, EDxlMotorType>
        {
        public:
            DxlMotorTypeEnum(EDxlMotorType e=EDxlMotorType::MOTOR_TYPE_UNKNOWN);
            DxlMotorTypeEnum(const char* const str);
            ~DxlMotorTypeEnum() {}

        private:
            friend class AbstractEnum<DxlMotorTypeEnum, EDxlMotorType>;
            static std::map<EDxlMotorType, std::string> initialize();
        };

    } // model
} //common

#endif // DXL_MOTOR_TYPE_ENUM
