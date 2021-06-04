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

        /**
         * @brief StepperMotorCmd::StepperMotorCmd
         */
        StepperMotorCmd::StepperMotorCmd() :
            AbstractMotorCmd<EStepperCommandType>(EStepperCommandType::CMD_TYPE_NONE)
        {
            reset();
        }

        /**
         * @brief StepperMotorCmd::StepperMotorCmd
         * @param type
         */
        StepperMotorCmd::StepperMotorCmd(EStepperCommandType type) :
            AbstractMotorCmd<EStepperCommandType>(type)
        {
            clear();
        }

        /**
         * @brief StepperMotorCmd::StepperMotorCmd
         * @param type
         * @param motor_id
         * @param params
         */
        StepperMotorCmd::StepperMotorCmd(EStepperCommandType type,
                                         uint8_t motor_id,
                                         std::vector<int32_t> params) :
            AbstractMotorCmd<EStepperCommandType>(type),
            _id(motor_id),
            _param_list(params)
        {
        }

        /**
         * @brief StepperMotorCmd::setId
         * @param id
         */
        void StepperMotorCmd::setId(uint8_t id)
        {
            _id = id;
        }

        /**
         * @brief StepperMotorCmd::setParams
         * @param params
         */
        void StepperMotorCmd::setParams(std::vector<int32_t> params)
        {
            _param_list = params;
        }


        //***********************
        //  AbstractMotorCmd intf
        //***********************

        /**
         * @brief StepperMotorCmd::reset
         */
        void StepperMotorCmd::reset()
        {
            setType(EStepperCommandType::CMD_TYPE_NONE);
            clear();
        }

        /**
         * @brief StepperMotorCmd::clear
         */
        void StepperMotorCmd::clear()
        {
            _param_list.clear();
        }

        /**
         * @brief StepperMotorCmd::str
         * @return
         */
        string StepperMotorCmd::str() const
        {
            ostringstream ss;
            ss << "Stepper motor cmd - ";

            ss << StepperCommandTypeEnum(_type).toString() << " ";

            ss << "Motor id: ";
                ss << std::to_string(_id) << " ";

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
            if ((EStepperCommandType::CMD_TYPE_NONE == _type) ||
               (EStepperCommandType::CMD_TYPE_UNKNOWN == _type) ||
               (_id == 0) ||
               (_param_list.empty()))
                    return false;

            switch(_type) {
                case EStepperCommandType::CMD_TYPE_RELATIVE_MOVE:
                    return (_param_list.size() == 2);
                case EStepperCommandType::CMD_TYPE_CALIBRATION:
                    return (_param_list.size() == 4);
                case EStepperCommandType::CMD_TYPE_POSITION_OFFSET:
                    return (_param_list.size() == 4);
                case EStepperCommandType::CMD_TYPE_CONVEYOR:
                    return (_param_list.size() == 3);
                default:
                    return (_param_list.size() == 1);
            }
        }

    } // namespace model
} // namespace common
