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

#include "model/single_motor_cmd.hpp"
#include <sstream>

using namespace std;

namespace common {
    namespace model {

    SingleMotorCmd::SingleMotorCmd() :
        _type(EDxlCommandType::CMD_TYPE_UNKNOWN),
        _id(0),
        _param(0)
    {

    }

    SingleMotorCmd::SingleMotorCmd(EDxlCommandType type,
                                       uint8_t motor_id,
                                       uint32_t param) :
            _type(type),
            _id(motor_id),
            _param(param)
        {}

        void SingleMotorCmd::setType(EDxlCommandType type)
        {
            _type = type;
        }

        void SingleMotorCmd::setId(uint8_t id)
        {
            _id = id;
        }

        void SingleMotorCmd::setParam(uint32_t param)
        {
            _param = param;
        }

        string SingleMotorCmd::str() const
        {
            ostringstream ss;
            ss << "Single motor cmd - ";

            ss << DxlCommandTypeEnum(_type).toString();

            ss << ": ";
            ss << "motor " << _id << ": " << _param;

            return ss.str();
        }

        EDxlCommandType SingleMotorCmd::getType() const
        {
            return _type;
        }

    } // namespace model
} // namespace common
