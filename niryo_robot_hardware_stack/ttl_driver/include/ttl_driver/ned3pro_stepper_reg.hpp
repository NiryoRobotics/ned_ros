/*
ned3pro_stepper_reg.hpp
Copyright (C) 2024 Niryo
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

#ifndef NED3PRO_STEPPER_REG_HPP
#define NED3PRO_STEPPER_REG_HPP

#include <memory>
#include "common/model/hardware_type_enum.hpp"

namespace ttl_driver
{
    struct Ned3ProStepperReg
    {
        static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::STEPPER;

        static constexpr float PROTOCOL_VERSION = 2.0;
        static constexpr int MODEL_NUMBER = 2100;
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

        static constexpr uint16_t ADDR_FIRMWARE_VERSION = 59;
        using TYPE_FIRMWARE_VERSION = uint32_t;

        // RAM

        static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
        using TYPE_TORQUE_ENABLE = uint8_t;

        static constexpr uint16_t ADDR_HW_ERROR_STATUS = 70;
        using TYPE_HW_ERROR_STATUS = uint8_t;

        static constexpr float VELOCITY_UNIT = 0.001;
        static constexpr uint16_t ADDR_GOAL_VELOCITY = 104;
        using TYPE_GOAL_VELOCITY = uint32_t;

        // unit = 214.577 RPM2
        static constexpr uint16_t ADDR_PROFILE_ACCELERATION = 108;
        using TYPE_PROFILE_ACCELERATION = uint32_t;

        // unit = 0.001 RPM
        static constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
        using TYPE_PROFILE_VELOCITY = uint32_t;

        // unit = 0.001 deg
        static constexpr uint16_t ADDR_GOAL_POSITION = 116;
        using TYPE_GOAL_POSITION = uint32_t;

        // not used
        static constexpr uint16_t ADDR_MOVING_STATUS = 123;
        using TYPE_MOVING_STATUS = uint8_t;

        // unit = 0.001 RPM
        static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
        using TYPE_PRESENT_VELOCITY = uint32_t;

        // unit = 0.001 deg
        static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
        using TYPE_PRESENT_POSITION = uint32_t;

        // unit = 1mV
        static constexpr uint16_t ADDR_PRESENT_VOLTAGE = 144;
        using TYPE_PRESENT_VOLTAGE = uint16_t;

        static constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 146;
        using TYPE_PRESENT_TEMPERATURE = uint8_t;

        static constexpr uint16_t ADDR_CONTROL = 1536;
        using TYPE_CONTROL = uint32_t;

        static constexpr uint16_t ADDR_STATUS = 1540;
        using TYPE_STATUS = uint32_t;

        static constexpr uint16_t ADDR_ENC_ANGLE = 1544;
        using TYPE_ENC_ANGLE = uint32_t;

        // Firmware status

        static constexpr uint16_t ADDR_FIRMWARE_RUNNING = 8192;
        using TYPE_FIRMWARE_RUNNING = uint8_t;

        static constexpr uint16_t ADDR_ENTER_BOOTLOADER = 8193;
        using TYPE_ENTER_BOOTLOADER = uint32_t;

        static constexpr uint16_t ADDR_OTA_BEGIN = 8197;
        using TYPE_OTA_BEGIN = uint8_t;

        static constexpr uint16_t ADDR_OTA_WRITE = 8198;
        using TYPE_OTA_WRITE = uint8_t[520];

        static constexpr uint16_t ADDR_OTA_END = 8718;
        using TYPE_OTA_END = uint32_t;

        static constexpr uint16_t ADDR_OTA_ERASE = 8722;
        using TYPE_OTA_ERASE = uint32_t;
    };
} // ttl_driver

#endif // NED3PRO_STEPPER_REG_HPP
