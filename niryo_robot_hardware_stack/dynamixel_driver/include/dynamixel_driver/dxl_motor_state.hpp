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
#include "utils/motor_state.hpp"
#include "dynamixel_driver/dxl_enum.hpp"

// CC nothing to do here
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
    class DxlMotorState : public utils::MotorState
    {
        public:
            DxlMotorState();
            DxlMotorState(uint8_t id, DxlMotorType_t type, bool isTool = false);

            virtual void reset();
            virtual bool isValid() const;

            //getters
            DxlMotorType_t getType() const;
            bool isTool() const;

            //setters
            bool operator==(const DxlMotorState& other);

            virtual std::string str() const;

        protected:
            DxlMotorType_t _type;

            bool _isTool;
    };

    inline
    bool DxlMotorState::isValid() const
    {
        return (0 != _id && DxlMotorType_t::MOTOR_TYPE_UNKNOWN != _type);
    }

    inline
    DxlMotorType_t DxlMotorState::getType() const
    {
        return _type;
    }

    inline
    bool DxlMotorState::isTool() const
    {
        return _isTool;
    }

} //DynamixelDriver

#endif
