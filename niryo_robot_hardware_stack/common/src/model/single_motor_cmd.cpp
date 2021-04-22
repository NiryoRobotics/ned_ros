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
            AbstractMotorCmd<EDxlCommandType>(EDxlCommandType::CMD_TYPE_UNKNOWN)
        {
            reset();
        }

        SingleMotorCmd::SingleMotorCmd(EDxlCommandType type,
                                       uint8_t motor_id,
                                       uint32_t param) :
            AbstractMotorCmd<EDxlCommandType>(type),
            _id(motor_id),
            _param(param)
        {}


        void SingleMotorCmd::setId(uint8_t id)
        {
            _id = id;
        }

        void SingleMotorCmd::setParam(uint32_t param)
        {
            _param = param;
        }

        void SingleMotorCmd::reset()
        {
            setType(EDxlCommandType::CMD_TYPE_UNKNOWN);
            clear();
        }

        void SingleMotorCmd::clear()
        {
            _id = 0;
            _param = 0;
        }

        string SingleMotorCmd::str() const
        {
            ostringstream ss;
            ss << "Single motor cmd - ";

            ss << DxlCommandTypeEnum(_type).toString();

            ss << ": ";
            ss << "motor " << static_cast<int>(_id) << ": " << static_cast<int>(_param);

            return ss.str();
        }

        bool SingleMotorCmd::isValid() const
        {
            return (EDxlCommandType::CMD_TYPE_UNKNOWN != _type) &&
                   (0 != _id);
        }

    } // namespace model
} // namespace common
