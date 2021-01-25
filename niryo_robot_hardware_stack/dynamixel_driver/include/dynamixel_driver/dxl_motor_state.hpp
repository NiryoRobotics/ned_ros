/*
    dxl_motor_state.h
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

#ifndef DXL_MOTOR_STATE_H
#define DXL_MOTOR_STATE_H

#include <string>
#include "dynamixel_driver/dxl_enum.hpp"

#define TOOL_STATE_PING_OK       0x01
#define TOOL_STATE_PING_ERROR    0x02
#define TOOL_STATE_WRONG_ID      0x03
#define TOOL_STATE_TIMEOUT       0x04

#define GRIPPER_STATE_OPEN       0x10 
#define GRIPPER_STATE_CLOSE      0x11

#define VACUUM_PUMP_STATE_PULLED 0x20
#define VACUUM_PUMP_STATE_PUSHED 0x21

namespace DynamixelDriver
{
    class DxlMotorState
    {
        public:
            DxlMotorState(uint8_t id, DxlMotorType type);
            uint8_t getId();
            void setId(uint8_t motor_id);
            DxlMotorType getType();
            void setType(DxlMotorType type);
            
            // getters - state
            uint32_t getPositionState();
            void setPositionState(uint32_t pos);

            uint32_t getTemperatureState();
            void setTemperatureState(uint32_t temp);
            uint32_t getVoltageState();
            void setVoltageState(uint32_t volt);
            uint32_t getHardwareErrorState();
            void setHardwareError(uint32_t hw_error);
            std::string getHardwareErrorMessageState();
            void setHardwareError(std::string hw_error_msg);

            bool operator==(const DxlMotorState& other);


        private:
            uint8_t _id;
            DxlMotorType _type;

            // read variables
            
            uint32_t _state_pos=0; 
            uint32_t _state_temperature=0;
            uint32_t _state_voltage=0;
            uint32_t _state_hw_error=0;
            std::string _state_hw_error_msg="";
    };
}

#endif
