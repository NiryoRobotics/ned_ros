/*
    tool_state.cpp
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
#include "model/tool_state.hpp"

using namespace std;

namespace common  {
    namespace model {

        ToolState::ToolState() :
            DxlMotorState()
        {
        }

        ToolState::ToolState(uint8_t id, std::string name, EDxlMotorType type) :
            DxlMotorState(id, type, true),
            _name(name),
            _connected(true),
            _position(0.0)
        {
        }

        ToolState::~ToolState()
        {

        }

        void ToolState::reset()
        {
            DxlMotorState::reset();
            _name = "No Tool";
            _position = 0.0;
        }

        void ToolState::setName(std::string name)
        {
            _name = name;
        }

        void ToolState::setPosition(double position)
        {
            _position = position;
        }

    } // namespace model
} // namespace common
