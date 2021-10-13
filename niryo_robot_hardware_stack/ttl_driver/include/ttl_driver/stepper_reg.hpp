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
#include "common/model/hardware_type_enum.hpp"

namespace ttl_driver
{
struct StepperReg
{
    static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::STEPPER;

    static constexpr float PROTOCOL_VERSION                     = 2.0;
    static constexpr int MODEL_NUMBER                           = 2000;
    static constexpr int VOLTAGE_CONVERSION                     = 1000;

    // EEPROM
    static constexpr uint16_t ADDR_MODEL_NUMBER             = 0;
    static constexpr uint8_t SIZE_MODEL_NUMBER             = 2;

    static constexpr uint16_t ADDR_ID                       = 7;
    static constexpr uint8_t SIZE_ID                       = 1;

    static constexpr uint16_t ADDR_BAUDRATE                 = 8;
    static constexpr uint8_t SIZE_BAUDRATE                 = 1;

    static constexpr uint16_t ADDR_MAX_POSITION_LIMIT       = 48;
    static constexpr uint8_t SIZE_MAX_POSITION_LIMIT       = 4;

    static constexpr uint16_t ADDR_MIN_POSITION_LIMIT       = 52;
    static constexpr uint8_t SIZE_MIN_POSITION_LIMIT       = 4;

    static constexpr uint16_t ADDR_FIRMWARE_VERSION         = 59;
    static constexpr uint8_t SIZE_FIRMWARE_VERSION         = 4;

    // RAM

    static constexpr uint16_t ADDR_TORQUE_ENABLE            = 64;
    static constexpr uint8_t SIZE_TORQUE_ENABLE            = 1;

    static constexpr uint16_t ADDR_HW_ERROR_STATUS          = 70;
    static constexpr uint8_t SIZE_HW_ERROR_STATUS          = 1;

    static constexpr uint16_t ADDR_GOAL_VELOCITY            = 104;
    static constexpr uint8_t SIZE_GOAL_VELOCITY            = 4;

    static constexpr uint16_t ADDR_GOAL_POSITION            = 116;
    static constexpr uint8_t SIZE_GOAL_POSITION            = 4;

    static constexpr uint16_t ADDR_PRESENT_POSITION         = 132;
    static constexpr uint8_t SIZE_PRESENT_POSITION         = 4;

    static constexpr uint16_t ADDR_PRESENT_VOLTAGE          = 144;
    static constexpr uint8_t SIZE_PRESENT_VOLTAGE          = 2;

    static constexpr uint16_t ADDR_PRESENT_TEMPERATURE      = 146;
    static constexpr uint8_t SIZE_PRESENT_TEMPERATURE      = 1;

    static constexpr uint16_t ADDR_COMMAND                  = 147;
    static constexpr uint8_t SIZE_COMMAND                  = 1;

    static constexpr uint16_t ADDR_HOMING_STATUS            = 148;
    static constexpr uint8_t SIZE_HOMING_STATUS            = 1;

    static constexpr uint16_t ADDR_HOMING_DIRECTION         = 149;
    static constexpr uint8_t SIZE_HOMING_DIRECTION         = 1;

    // acceleration profile

    static constexpr uint16_t ADDR_VSTART = 1024;
    static constexpr uint8_t SIZE_VSTART = 4;
    
    static constexpr uint16_t ADDR_A1 = 1028;
    static constexpr uint8_t SIZE_A1 = 4;
    
    static constexpr uint16_t ADDR_V1 = 1032;
    static constexpr uint8_t SIZE_V1 = 4;
    
    static constexpr uint16_t ADDR_AMAX = 1036;
    static constexpr uint8_t SIZE_AMAX = 4;
    
    static constexpr uint16_t ADDR_VMAX = 1040;
    static constexpr uint8_t SIZE_VMAX = 4;
    
    static constexpr uint16_t ADDR_DMAX = 1044;
    static constexpr uint8_t SIZE_DMAX = 4;
    
    static constexpr uint16_t ADDR_D1 = 1048;
    static constexpr uint8_t SIZE_D1 = 4;
    
    static constexpr uint16_t ADDR_VSTOP = 1052;
    static constexpr uint8_t SIZE_VSTOP = 4;

    // Firmware status

    static constexpr uint16_t ADDR_FIRMWARE_RUNNING = 8192;
    static constexpr uint8_t SIZE_FIRMWARE_RUNNING = 1;

    static constexpr uint16_t ADDR_ENTER_BOOTLOADER = 8193;
    static constexpr uint8_t SIZE_ENTER_BOOTLOADER = 4;
};
} // ttl_driver

#endif // STEPPER_REG_HPP
