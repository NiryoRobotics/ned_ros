/*
stepper_reg.hpp
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

#ifndef STEPPER_REG_HPP
#define STEPPER_REG_HPP

#include <memory>
#include "common/model/motor_type_enum.hpp"

namespace ttl_driver
{
struct StepperReg
{
    static constexpr common::model::EMotorType motor_type = common::model::EMotorType::STEPPER;

    static constexpr int PROTOCOL_VERSION                       = 2.0;
    static constexpr int MODEL_NUMBER                           = 0.0;

    // EEPROM

    static constexpr std::uint8_t ADDR_ID                       = 7;
    static constexpr std::uint8_t SIZE_ID                       = 1;

    static constexpr std::uint8_t ADDR_BAUDRATE                 = 8;
    static constexpr std::uint8_t SIZE_BAUDRATE                 = 1;

    static constexpr std::uint8_t ADDR_MAX_POSITION_LIMIT       = 48;
    static constexpr std::uint8_t SIZE_MAX_POSITION_LIMIT       = 4;

    static constexpr std::uint8_t ADDR_MIN_POSITION_LIMIT       = 52;
    static constexpr std::uint8_t SIZE_MIN_POSITION_LIMIT       = 4;

    static constexpr std::uint8_t ADDR_FIRMWARE_VERSION         = 59;
    static constexpr std::uint8_t SIZE_FIRMWARE_VERSION         = 4;

    // RAM

    static constexpr std::uint8_t ADDR_TORQUE_ENABLE            = 64;
    static constexpr std::uint8_t SIZE_TORQUE_ENABLE            = 1;

    static constexpr std::uint8_t ADDR_HW_ERROR_STATUS          = 70;
    static constexpr std::uint8_t SIZE_HW_ERROR_STATUS          = 1;

    static constexpr std::uint8_t ADDR_GOAL_VELOCITY            = 104;
    static constexpr std::uint8_t SIZE_GOAL_VELOCITY            = 4;

    static constexpr std::uint8_t ADDR_GOAL_POSITION            = 116;
    static constexpr std::uint8_t SIZE_GOAL_POSITION            = 4;

    static constexpr std::uint8_t ADDR_PRESENT_POSITION         = 132;
    static constexpr std::uint8_t SIZE_PRESENT_POSITION         = 4;

    static constexpr std::uint8_t ADDR_PRESENT_VOLTAGE          = 144;
    static constexpr std::uint8_t SIZE_PRESENT_VOLTAGE          = 2;

    static constexpr std::uint8_t ADDR_PRESENT_TEMPERATURE      = 146;
    static constexpr std::uint8_t SIZE_PRESENT_TEMPERATURE      = 1;

    static constexpr std::uint8_t ADDR_COMMAND                  = 147;
    static constexpr std::uint8_t SIZE_COMMAND                  = 1;

    static constexpr std::uint8_t ADDR_HOMING_STATUS            = 148;
    static constexpr std::uint8_t SIZE_HOMING_STATUS            = 1;

    static constexpr std::uint8_t ADDR_HOMING_DIRECTION         = 149;
    static constexpr std::uint8_t SIZE_HOMING_DIRECTION         = 1;
};
} // DynamixelDriver

#endif // STEPPER_REG_HPP
