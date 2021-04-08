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

#include "stepper_driver/stepper_motor_cmd.hpp"

namespace StepperDriver
{
    StepperMotorCmd::StepperMotorCmd()
    {
    }

    StepperMotorCmd::StepperMotorCmd(StepperCommandType_t type,
                                     std::vector<uint8_t> motor_id,
                                     std::vector<int32_t> params) :
        _type(type), _motor_id_list(motor_id), _param_list(params)
    {
    }

    void StepperMotorCmd::setType(StepperCommandType_t type)
    {
        _type = type;
    }

    void StepperMotorCmd::setMotorsId(std::vector<uint8_t> motor_id)
    {
        _motor_id_list = motor_id;
    }

    void StepperMotorCmd::setParams(std::vector<int32_t> params)
    {
        _param_list = params;
    }

} //stepper driver
