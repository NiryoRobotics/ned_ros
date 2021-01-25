/*
    synchronize_motor_cmd.hpp
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

#ifndef DXL_SYNCHRONIZE_MOTOR_CMD_H
#define DXL_SYNCHRONIZE_MOTOR_CMD_H

#include <string>
#include <vector>

#include "dynamixel_driver/dxl_enum.hpp"
namespace DynamixelDriver
{
    class SynchronizeMotorCmd {

        public:

            SynchronizeMotorCmd( );
            SynchronizeMotorCmd(DxlCommandType type, std::vector<uint8_t> motor_id, std::vector<uint32_t> params);
            
            DxlCommandType getType();
            void setType(DxlCommandType type);
            
            std::vector<uint8_t>& getMotorsId();
            void setMotorsId(std::vector<uint8_t> motor_id);

            std::vector<uint32_t>& getParams();
            void setParams(std::vector<uint32_t> params);

        private:

            DxlCommandType _type;
            std::vector<uint8_t> _motor_id_list;
            std::vector<uint32_t> _param_list;
    };
}

#endif
