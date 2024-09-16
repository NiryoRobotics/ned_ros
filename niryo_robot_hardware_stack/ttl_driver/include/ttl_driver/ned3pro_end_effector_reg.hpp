/*
ned3pro_end_effector_reg.hpp
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

#ifndef NED3PRO_END_EFFECTOR_REG_HPP
#define NED3PRO_END_EFFECTOR_REG_HPP

#include <memory>
#include "common/model/hardware_type_enum.hpp"

namespace ttl_driver
{
    struct Ned3ProEndEffectorReg
    {
        static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::END_EFFECTOR;

        static constexpr float PROTOCOL_VERSION = 2.0;
        static constexpr int MODEL_NUMBER = 2101;
        static constexpr int VOLTAGE_CONVERSION = 1000;

        // see table here : http:// support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
        // EEPROM
        static constexpr uint16_t ADDR_MODEL_NUMBER = 0;
        using TYPE_MODEL_NUMBER = uint16_t;

        static constexpr uint16_t ADDR_ID = 7;
        using TYPE_ID = uint8_t;

        static constexpr uint16_t ADDR_BAUDRATE = 8;
        using TYPE_BAUDRATE = uint8_t;

        static constexpr uint16_t ADDR_FIRMWARE_VERSION = 59;
        using TYPE_FIRMWARE_VERSION = uint32_t;

        static constexpr uint16_t ADDR_HW_ERROR_STATUS = 70;
        using TYPE_HW_ERROR_STATUS = uint8_t;

        static constexpr uint16_t ADDR_PRESENT_VOLTAGE = 144;
        using TYPE_PRESENT_VOLTAGE = uint16_t;

        static constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 146;
        using TYPE_PRESENT_TEMPERATURE = uint8_t;

        // specific to EE
        static constexpr uint16_t ADDR_COLLISION_STATUS = 2052;
        using TYPE_COLLISION_STATUS = uint8_t;

        using TYPE_BUTTON_STATUS = uint8_t;
        static constexpr uint16_t ADDR_STATE_BUTTON_FREEDRIVE = 2053;
        static constexpr uint16_t ADDR_STATE_BUTTON_SAVE = 2054;
        static constexpr uint16_t ADDR_STATE_BUTTON_CUSTOM = 2055;

        static constexpr uint16_t ADDR_DIGITAL_INPUT = 2056;
        using TYPE_DIGITAL_IN = uint8_t;

        static constexpr uint16_t ADDR_DIGITAL_OUTPUT = 2057;
        using TYPE_DIGITAL_OUT = uint8_t;

        static constexpr uint16_t ADDR_COLLISION_DETECT_THRESH_ALGO_1 = 2058;
        using TYPE_COLLISION_THRESHOLD_ALGO_1 = uint32_t;

        static constexpr uint16_t ADDR_COLLISION_DETECT_THRESH_ALGO_2 = 2062;
        using TYPE_COLLISION_THRESHOLD_ALGO_2 = uint32_t;

        static constexpr uint16_t ADDR_ACCELERO_VALUE_X = 2066;
        using TYPE_ACCELERO_VALUE_X = uint32_t;

        static constexpr uint16_t ADDR_ACCELERO_VALUE_Y = 2070;
        using TYPE_ACCELERO_VALUE_Y = uint32_t;

        static constexpr uint16_t ADDR_ACCELERO_VALUE_Z = 2074;
        using TYPE_ACCELERO_VALUE_Z = uint32_t;

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

#endif // NED3PRO_END_EFFECTOR_REG_HPP
