/*
can_driver/stepper_reg.hpp
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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef CAN_STEPPER_REG_HPP
#define CAN_STEPPER_REG_HPP

#include <memory>
#include "common/model/hardware_type_enum.hpp"

namespace can_driver
{
struct StepperReg
{
    static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::STEPPER;
    static constexpr int CAN_MODEL_NUMBER                       = 10000;

    static constexpr int CAN_CMD_POSITION                       = 0x03;
    static constexpr int CAN_CMD_TORQUE                         = 0x04;
    static constexpr int CAN_CMD_MODE                           = 0x07;
    static constexpr int CAN_CMD_MICRO_STEPS                    = 0x13;
    static constexpr int CAN_CMD_OFFSET                         = 0x14;
    static constexpr int CAN_CMD_CALIBRATE                      = 0x15;
    static constexpr int CAN_CMD_SYNCHRONIZE                    = 0x16;
    static constexpr int CAN_CMD_MAX_EFFORT                     = 0x17;
    static constexpr int CAN_CMD_MOVE_REL                       = 0x18;
    static constexpr int CAN_CMD_RESET                          = 0x19; // not yet implemented

    static constexpr int CAN_STEPPERS_CALIBRATION_MODE_AUTO     = 1;
    static constexpr int CAN_STEPPERS_CALIBRATION_MODE_MANUAL   = 2;

    static constexpr int CAN_STEPPERS_WRITE_OFFSET_FAIL         = -3;

    static constexpr int STEPPER_CONTROL_MODE_RELAX             = 0;
    static constexpr int STEPPER_CONTROL_MODE_STANDARD          = 1;
    static constexpr int STEPPER_CONTROL_MODE_PID_POS           = 2;
    static constexpr int STEPPER_CONTROL_MODE_TORQUE            = 3;

    static constexpr int STEPPER_CONVEYOR_OFF                   = 20;
    static constexpr int STEPPER_CONVEYOR_ON                    = 21;
    static constexpr int CAN_UPDATE_CONVEYOR_ID                 = 23;

};
} // can_driver

#endif // CAN_STEPPER_REG_HPP
