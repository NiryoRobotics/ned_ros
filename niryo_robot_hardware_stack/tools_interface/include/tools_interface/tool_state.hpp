/*
    tool_state.hpp
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

#ifndef END_EFFECTOR_STATE_HPP
#define END_EFFECTOR_STATE_HPP

#include <stdint.h>
#include <string>

#include "dynamixel_driver/dxl_enum.hpp"

class ToolState
{
    public:
        ToolState(uint8_t id, DynamixelDriver::DxlMotorType type);

        void setId(uint8_t id);
        uint8_t getId();

        void setType(DynamixelDriver::DxlMotorType type);
        DynamixelDriver::DxlMotorType getType();

        bool isConnected();

        void setPosition(double position);
        double getPosition();

    private:
    
        uint8_t _id;
        DynamixelDriver::DxlMotorType _type;
        std::string _name;

        bool _connected;
        double _position; 
};
#endif
