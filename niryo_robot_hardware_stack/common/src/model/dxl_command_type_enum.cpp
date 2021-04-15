/*
    dxl_command_type_enum.hpp
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

#include "model/dxl_command_type_enum.hpp"

using namespace std;

namespace common {
    namespace model {

        /**
         *
         */
        DxlCommandTypeEnum::DxlCommandTypeEnum(EDxlCommandType e):
            AbstractEnum<DxlCommandTypeEnum, EDxlCommandType>(e)
        {}

        /**
         *
         */
        DxlCommandTypeEnum::DxlCommandTypeEnum(const char* const str):
            AbstractEnum<DxlCommandTypeEnum, EDxlCommandType>(str)
        {}

        /**
         *
         */
        map<EDxlCommandType, string>
        DxlCommandTypeEnum::initialize()
        {
            map<EDxlCommandType, string> m;

            m[EDxlCommandType::CMD_TYPE_POSITION] = "position";
            m[EDxlCommandType::CMD_TYPE_VELOCITY]    = "velocity";
            m[EDxlCommandType::CMD_TYPE_EFFORT]     = "effort";
            m[EDxlCommandType::CMD_TYPE_TORQUE]    = "torque";
            m[EDxlCommandType::CMD_TYPE_PING] = "ping";
            m[EDxlCommandType::CMD_TYPE_LEARNING_MODE]  = "learning mode";
            m[EDxlCommandType::CMD_TYPE_P_GAIN] = "P Gain";
            m[EDxlCommandType::CMD_TYPE_I_GAIN] = "I Gain";
            m[EDxlCommandType::CMD_TYPE_D_GAIN] = "D Gain";
            m[EDxlCommandType::CMD_TYPE_FF1_GAIN] = "FF1 Gain";
            m[EDxlCommandType::CMD_TYPE_FF2_GAIN] = "F2 Gain";
            m[EDxlCommandType::CMD_TYPE_UNKNOWN]  = "unknown type";

            return m;
        }

    } // model
} //common

