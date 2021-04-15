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

#include "model/synchronize_motor_cmd.hpp"
#include <sstream>

using namespace std;

namespace common {
    namespace model {

        SynchronizeMotorCmd::SynchronizeMotorCmd() :
            AbstractMotorCmd<EDxlCommandType>(EDxlCommandType::CMD_TYPE_UNKNOWN)

        {
            reset();
        }

        SynchronizeMotorCmd::SynchronizeMotorCmd(EDxlCommandType type,
                                                 vector<uint8_t> motor_id,
                                                 vector<uint32_t> params) :
            AbstractMotorCmd<EDxlCommandType>(type),
            _motor_id_list(motor_id),
            _param_list(params)
        {
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
         * @brief SynchronizeMotorCmd::reset
         */
        void SynchronizeMotorCmd::reset()
        {
            setType(EDxlCommandType::CMD_TYPE_UNKNOWN);
            _motor_id_list.clear();
            _param_list.clear();
        }

        /**
         * @brief SynchronizeMotorCmd::str
         * @return
         */
        string SynchronizeMotorCmd::str() const
        {
            string string_info;

            ostringstream ss;
            ss << "Sync motor cmd - ";

            ss << DxlCommandTypeEnum(_type).toString();

            ss << ": ";

            if(!isValid()) {
                ss << "Corrupted command : motors id list and params list size mismatch";
            }
            else {
                ss << "(";
                for(size_t i = 0; i < _motor_id_list.size(); ++i)
                    ss << " motor " << static_cast<int>(_motor_id_list.at(i)) << ": " <<
                          _param_list.at(i) << ",";

                string_info.pop_back();
            }

            string_info = ss.str();

            string_info += ")";

            return string_info;
        }

        /**
         * @brief SynchronizeMotorCmd::isValid
         * @return
         */
        bool SynchronizeMotorCmd::isValid() const
        {
            return (EDxlCommandType::CMD_TYPE_UNKNOWN != _type) &&
                   (!_motor_id_list.empty()) &&
                   (_motor_id_list.size() == _param_list.size());
        }

    } // namespace model
} // namespace common
