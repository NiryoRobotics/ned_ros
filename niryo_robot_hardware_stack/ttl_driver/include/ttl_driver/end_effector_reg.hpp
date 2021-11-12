/*
xc430_reg.hpp
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

#ifndef END_EFFECTOR_REG_HPP
#define END_EFFECTOR_REG_HPP

#include <memory>
#include "common/model/hardware_type_enum.hpp"

namespace ttl_driver
{
struct EndEffectorReg
{
    static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::END_EFFECTOR;

    static constexpr float PROTOCOL_VERSION                     = 2.0;
    static constexpr int MODEL_NUMBER                           = 2001;
    static constexpr int VOLTAGE_CONVERSION                     = 1000;

    // see table here : http:// support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
    // EEPROM
    static constexpr uint16_t ADDR_MODEL_NUMBER             = 0;
    static constexpr uint8_t SIZE_MODEL_NUMBER              = 2;


    static constexpr uint16_t ADDR_ID                       = 7;
    static constexpr uint8_t SIZE_ID                        = 1;

    static constexpr uint16_t ADDR_BAUDRATE                 = 8;
    static constexpr uint8_t SIZE_BAUDRATE                  = 1;

    static constexpr uint16_t ADDR_FIRMWARE_VERSION         = 59;
    using TYPE_FIRMWARE_VERSION = uint32_t;

    static constexpr uint16_t ADDR_HW_ERROR_STATUS          = 70;
    using TYPE_HW_ERROR_STATUS = uint8_t;

    static constexpr uint16_t ADDR_PRESENT_VOLTAGE          = 144;
    using TYPE_PRESENT_VOLTAGE = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_TEMPERATURE      = 146;
    using TYPE_PRESENT_TEMPERATURE = uint8_t;

    // specific to EE
    static constexpr uint16_t ADDR_BUTTON_0_STATUS          = 1024;
    static constexpr uint16_t ADDR_BUTTON_1_STATUS          = 1025;
    static constexpr uint16_t ADDR_BUTTON_2_STATUS          = 1026;
    using TYPE_BUTTON_STATUS = uint8_t;

    static constexpr uint16_t ADDR_DIGITAL_IN               = 1040;
    static constexpr uint8_t SIZE_DIGITAL_IN                = 1;

    static constexpr uint16_t ADDR_DIGITAL_OUT              = 1041;
    static constexpr uint8_t SIZE_DIGITAL_OUT               = 1;


    static constexpr uint16_t ADDR_COLLISION_STATUS         = 1048;
    static constexpr uint8_t SIZE_COLLISION_STATUS          = 1;

    static constexpr uint16_t ADDR_COLLISION_THRESHOLD      = 1049;
    static constexpr uint8_t SIZE_COLLISION_THRESHOLD       = 1;


    static constexpr uint16_t ADDR_ACCELERO_VALUE_X         = 1053;
    static constexpr uint8_t SIZE_ACCELERO_VALUE_X         = 4;

    static constexpr uint16_t ADDR_ACCELERO_VALUE_Y         = 1057;
    static constexpr uint8_t SIZE_ACCELERO_VALUE_Y         = 4;

    static constexpr uint16_t ADDR_ACCELERO_VALUE_Z         = 1061;
    static constexpr uint8_t SIZE_ACCELERO_VALUE_Z         = 4;

    // Firmware status

    static constexpr uint16_t ADDR_FIRMWARE_RUNNING         = 8192;
    static constexpr uint8_t SIZE_FIRMWARE_RUNNING          = 1;

    static constexpr uint16_t ADDR_ENTER_BOOTLOADER         = 8193;
    static constexpr uint8_t SIZE_ENTER_BOOTLOADER          = 4;

};
} // ttl_driver

#endif // END_EFFECTOR_REG_HPP
