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
#include "common/model/stepper_calibration_status_enum.hpp"

namespace ttl_driver
{
    struct StepperReg
    {
        static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::STEPPER;

        static constexpr float PROTOCOL_VERSION = 2.0;
        static constexpr int MODEL_NUMBER = 2000;
        static constexpr int VOLTAGE_CONVERSION = 1000;

        // EEPROM
        static constexpr uint16_t ADDR_MODEL_NUMBER = 0;
        using TYPE_MODEL_NUMBER = uint16_t;

        static constexpr uint16_t ADDR_ID = 7;
        using TYPE_ID = uint8_t;

        static constexpr uint16_t ADDR_BAUDRATE = 8;
        using TYPE_BAUDRATE = uint8_t;

        static constexpr uint16_t ADDR_OPERATING_MODE = 11;
        using TYPE_OPERATING_MODE = uint8_t;

        static constexpr uint16_t ADDR_MAX_POSITION_LIMIT = 48;
        using TYPE_MAX_POSITION_LIMIT = uint32_t;

        static constexpr uint16_t ADDR_MIN_POSITION_LIMIT = 52;
        using TYPE_MIN_POSITION_LIMIT = uint32_t;

        static constexpr uint16_t ADDR_FIRMWARE_VERSION = 59;
        using TYPE_FIRMWARE_VERSION = uint32_t;

        // RAM

        static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
        using TYPE_TORQUE_ENABLE = uint8_t;

        static constexpr uint16_t ADDR_HW_ERROR_STATUS = 70;
        using TYPE_HW_ERROR_STATUS = uint8_t;

        static constexpr float VELOCITY_UNIT = 0.01;
        static constexpr uint16_t ADDR_GOAL_VELOCITY = 104;
        using TYPE_GOAL_VELOCITY = uint32_t;

        // unit = 0.088 deg
        static constexpr uint16_t ADDR_GOAL_POSITION = 116;
        using TYPE_GOAL_POSITION = uint32_t;

        // unit = 0.01 RPM
        static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
        using TYPE_PRESENT_VELOCITY = uint32_t;

        // unit = 0.088 deg
        static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
        using TYPE_PRESENT_POSITION = uint32_t;

        // unit = 1mV
        static constexpr uint16_t ADDR_PRESENT_VOLTAGE = 144;
        using TYPE_PRESENT_VOLTAGE = uint16_t;

        static constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 146;
        using TYPE_PRESENT_TEMPERATURE = uint8_t;

        static constexpr uint16_t ADDR_COMMAND = 147;
        using TYPE_COMMAND = uint8_t;

        static constexpr uint16_t ADDR_HOMING_STATUS = 148;
        using TYPE_HOMING_STATUS = uint8_t;

        static constexpr uint16_t ADDR_HOMING_DIRECTION = 149;
        using TYPE_HOMING_DIRECTION = uint8_t;

        static constexpr uint16_t ADDR_HOMING_STALL_THRESHOLD = 150;
        using TYPE_HOMING_STALL_THRESHOLD = uint8_t;

        static constexpr uint16_t ADDR_HOMING_ABS_POSITION = 151;
        using TYPE_HOMING_ABS_POSITION = uint32_t;

        // acceleration profile
        using TYPE_PROFILE = uint32_t;
        static constexpr uint16_t ADDR_VSTART = 1024;
        static constexpr uint16_t ADDR_A1 = 1028;
        static constexpr uint16_t ADDR_V1 = 1032;
        static constexpr uint16_t ADDR_AMAX = 1036;
        static constexpr uint16_t ADDR_VMAX = 1040;
        static constexpr uint16_t ADDR_DMAX = 1044;
        static constexpr uint16_t ADDR_D1 = 1048;
        static constexpr uint16_t ADDR_VSTOP = 1052;

        // Firmware status

        static constexpr uint16_t ADDR_FIRMWARE_RUNNING = 8192;
        using TYPE_FIRMWARE_RUNNING = uint8_t;

        static constexpr uint16_t ADDR_ENTER_BOOTLOADER = 8193;
        using TYPE_ENTER_BOOTLOADER = uint32_t;
    };
} // ttl_driver

#endif // STEPPER_REG_HPP
