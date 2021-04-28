/*
    synchronize_stepper_motor_cmd.cpp
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

#include "model/synchronize_stepper_motor_cmd.hpp"
#include <sstream>

using namespace std;

namespace common {
    namespace model {

        SynchronizeStepperMotorCmd::SynchronizeStepperMotorCmd() :
            AbstractMotorCmd<EStepperCommandType>(EStepperCommandType::CMD_TYPE_NONE)

        {
            reset();
        }

        SynchronizeStepperMotorCmd::SynchronizeStepperMotorCmd(EStepperCommandType type) :
            AbstractMotorCmd<EStepperCommandType>(type)
        {
        }

        /**
         * @brief SynchronizeStepperMotorCmd::addMotorParam
         * @param type
         * @param motor_id
         * @param param
         */
        void SynchronizeStepperMotorCmd::addMotorParam(uint8_t motor_id, int32_t param)
        {
            _motor_params_map[motor_id] = param;
        }

        /**
         * @brief SynchronizeStepperMotorCmd::reset
         */
        void SynchronizeStepperMotorCmd::reset()
        {
            setType(EStepperCommandType::CMD_TYPE_NONE);
            clear();
        }

        /**
         * @brief SynchronizeStepperMotorCmd::str
         * @return
         */
        string SynchronizeStepperMotorCmd::str() const
        {
            string string_info;

            ostringstream ss;
            ss << "Sync motor cmd - ";

            ss << StepperCommandTypeEnum(_type).toString();

            ss << ": ";

            if(!isValid()) {
                ss << "Corrupted command : motors id list and params list size mismatch ";
                string_info = ss.str();
            }
            else {
                ss << "[";

                for(auto const& param: _motor_params_map) {
                    ss << " " << param.second << ",";
                }

                string_info = ss.str();
                string_info.pop_back();

                string_info += "]";
            }

            return string_info;
        }

        /**
         * @brief SynchronizeStepperMotorCmd::isValid
         * @return
         */
        bool SynchronizeStepperMotorCmd::isValid() const
        {
            return (EStepperCommandType::CMD_TYPE_UNKNOWN != _type &&
                    EStepperCommandType::CMD_TYPE_NONE != _type &&
                    !_motor_params_map.empty());
        }

        std::vector<uint8_t>
        SynchronizeStepperMotorCmd::getMotorsId() const
        {
            std::vector<uint8_t> ids;
            for(auto const& it_map: _motor_params_map) {
                ids.emplace_back(it_map.first);
            }

            return ids;
        }

        int32_t
        SynchronizeStepperMotorCmd::getParam(uint8_t motor_id) const
        {
            if(!_motor_params_map.count(motor_id))
                throw std::out_of_range("SynchronizeStepperMotorCmd::getParam : Unknown motor id");

            return _motor_params_map.at(motor_id);
        }


        /**
         * @brief SynchronizeStepperMotorCmd::clear : clears the data (keep the cmd type)
         */
        void SynchronizeStepperMotorCmd::clear()
        {
            _motor_params_map.clear();
        }

    } // namespace model
} // namespace common
