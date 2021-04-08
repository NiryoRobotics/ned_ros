/*
    stepper_motor_cmd.hpp
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

#ifndef STEPPER_MOTOR_CMD_H
#define STEPPER_MOTOR_CMD_H

#include <string>
#include <vector>
#include "stepper_driver/stepper_enum.hpp"

namespace StepperDriver
{
    class StepperMotorCmd {

        public:

            StepperMotorCmd();
            StepperMotorCmd(StepperCommandType_t type,
                            std::vector<uint8_t> motor_id,
                            std::vector<int32_t> params);
            
            StepperCommandType_t getType() const;
            std::vector<uint8_t> getMotorsId() const;
            std::vector<int32_t> getParams() const;

            void setType(StepperCommandType_t type);
            void setMotorsId(std::vector<uint8_t> motor_id);
            void setParams(std::vector<int32_t> params);

        private:

            StepperCommandType_t _type{StepperCommandType_t::CMD_TYPE_NONE};
            std::vector<uint8_t> _motor_id_list;
            std::vector<int32_t> _param_list;
    };

    inline
    StepperCommandType_t StepperMotorCmd::getType() const
    {
        return _type;
    }

    inline
    std::vector<uint8_t> StepperMotorCmd::getMotorsId() const
    {
        return _motor_id_list;
    }

    inline
    std::vector<int32_t> StepperMotorCmd::getParams() const
    {
        return _param_list;
    }
} //stepper driver

#endif
