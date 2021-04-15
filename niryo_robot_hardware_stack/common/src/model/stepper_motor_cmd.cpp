/*
    stepper_motor_cmd.cpp
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

#include "model/stepper_motor_cmd.hpp"

#include <sstream>

using namespace std;

namespace common {
    namespace model {

        StepperMotorCmd::StepperMotorCmd() :
            AbstractMotorCmd<EStepperCommandType>(EStepperCommandType::CMD_TYPE_NONE)
        {
            reset();
        }

        StepperMotorCmd::StepperMotorCmd(EStepperCommandType type,
                                         vector<uint8_t> motor_id,
                                         vector<int32_t> params) :
            AbstractMotorCmd<EStepperCommandType>(type),
            _motor_id_list(motor_id),
            _param_list(params)
        {
        }

        void StepperMotorCmd::setMotorsId(vector<uint8_t> motor_id)
        {
            _motor_id_list = motor_id;
        }

        void StepperMotorCmd::setParams(vector<int32_t> params)
        {
            _param_list = params;
        }

        /**
         * @brief StepperMotorCmd::reset
         */
        void StepperMotorCmd::reset()
        {
            setType(EStepperCommandType::CMD_TYPE_NONE);
            _motor_id_list.clear();
            _param_list.clear();
        }

        /**
         * @brief StepperMotorCmd::str
         * @return
         */
        string StepperMotorCmd::str() const
        {
            ostringstream ss;
            ss << "Single motor cmd - ";

            ss << StepperCommandTypeEnum(_type).toString() << " ";

            ss << "Motors id: ";
            for (uint8_t m_id : getMotorsId())
                ss << std::to_string(m_id) << " ";

            ss << "Params: ";
            for (int32_t param : getParams())
                ss << std::to_string(param) << " ";

            return ss.str();
        }

        /**
         * @brief StepperMotorCmd::isValid
         * @return
         */
        bool StepperMotorCmd::isValid() const
        {
            return (EStepperCommandType::CMD_TYPE_NONE != _type) &&
                   (EStepperCommandType::CMD_TYPE_UNKNOWN != _type) &&
                   (!_motor_id_list.empty()) &&
                   (_motor_id_list.size() == _param_list.size());
        }

    } // namespace model
} // namespace common
