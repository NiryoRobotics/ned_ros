/*
    synchronize_motor_cmd.cpp
    Copyright (C) 2017 Niryo
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

#include "dynamixel_driver/synchronize_motor_cmd.hpp"
#include <sstream>

using namespace std;

namespace DynamixelDriver
{
    SynchronizeMotorCmd::SynchronizeMotorCmd()
    {
    }

    SynchronizeMotorCmd::SynchronizeMotorCmd(DxlCommandType type,
                                             vector<uint8_t> motor_id,
                                             vector<uint32_t> params)
        : _type(type), _motor_id_list(motor_id), _param_list(params)
    {
    }

    void SynchronizeMotorCmd::setType(DxlCommandType type)
    {
        _type = type;
    }

    void SynchronizeMotorCmd::setMotorsId(vector<uint8_t> motor_id)
    {
        _motor_id_list = motor_id;
    }

    void SynchronizeMotorCmd::setParams(vector<uint32_t> params)
    {
        _param_list = params;
    }

    /**
     * @brief SynchronizeMotorCmd::to_string
     * @return
     */
    string SynchronizeMotorCmd::str() const
    {
        string string_info;

        ostringstream ss;
        ss << "Sync motor cmd - ";
        switch(_type)
        {
            case DxlCommandType::CMD_TYPE_POSITION:
                ss << "Position";
                break;
            case DxlCommandType::CMD_TYPE_VELOCITY:
                ss << "Velocity";
                break;
            case DxlCommandType::CMD_TYPE_EFFORT:
                ss << "Effort";
                break;
            case DxlCommandType::CMD_TYPE_TORQUE:
                ss << "Torque";
                break;
            case DxlCommandType::CMD_TYPE_PING:
                ss << "Ping";
                break;
            case DxlCommandType::CMD_TYPE_LEARNING_MODE:
                ss << "Learning mode";
                break;
            default:
                ss << "Unknown type " << static_cast<int>(_type);
            break;
        }

        ss << ": ";

        if(!isValid())
            ss << "Corrupted command : motors id list and params list size mismatch";
        else {
            for(int i = 0; i < _motor_id_list.size(); ++i)
                ss << " motor " << _motor_id_list.at(i) << ": " << _param_list.at(i) << ",";
        }

        string_info = ss.str();
        string_info.pop_back();

        return string_info;
    }

} // namespace DynamixelDriver
