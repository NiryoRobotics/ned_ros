/*
    single_motor_cmd.hpp
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

#ifndef DXL_SINGLE_MOTOR_CMD_H
#define DXL_SINGLE_MOTOR_CMD_H

#include <string>
#include <vector>

#include "dynamixel_driver/dxl_enum.hpp"

namespace DynamixelDriver
{
    class SingleMotorCmd {

        public:

            SingleMotorCmd( );
            SingleMotorCmd(DxlCommandType type, uint8_t motor_id, uint32_t param);

            DxlCommandType getType() const;
            void setType(DxlCommandType type);

            void setId(uint8_t id);
            uint8_t getId() const;

            uint32_t getParam() const;
            void setParam(uint32_t param);            

        private:

            uint8_t _id;
            DxlCommandType _type;
            uint32_t _param;

    };
}

#endif
