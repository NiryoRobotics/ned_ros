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

#include <string>

#include "dynamixel_driver/dxl_enum.hpp"
#include "dynamixel_driver/dxl_motor_state.hpp"

namespace ToolsInterface {

    class ToolState : public DynamixelDriver::DxlMotorState
    {
        public:
            ToolState();
            ToolState(uint8_t id, std::string name,
                      DynamixelDriver::DxlMotorType_t type);

            void reset();

            void setName(std::string name);
            void setPosition(double position);

            std::string getName() const;
            double getPosition() const;

            bool isConnected() const;

        protected:
            std::string _name;

            bool _connected;
            double _position;
    };

    inline
    std::string ToolState::getName() const
    {
        return _name;
    }

    inline
    double ToolState::getPosition() const
    {
        return _position;
    }

    inline
    bool ToolState::isConnected() const
    {
        return _connected;
    }

} //ToolsInterface

#endif
