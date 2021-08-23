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

    static constexpr int PROTOCOL_VERSION                       = 2.0;
    static constexpr int MODEL_NUMBER                           = 3000;

    // see table here : http:// support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
    // EEPROM
    static constexpr std::uint8_t ADDR_MODEL_NUMBER             = 0;
    static constexpr std::uint8_t SIZE_MODEL_NUMBER             = 2;

    static constexpr std::uint8_t ADDR_FIRMWARE_VERSION         = 6;
    static constexpr std::uint8_t SIZE_FIRMWARE_VERSION         = 1;

    static constexpr std::uint8_t ADDR_ID                       = 7;
    static constexpr std::uint8_t SIZE_ID                       = 1;

    static constexpr std::uint8_t ADDR_BAUDRATE                 = 8;
    static constexpr std::uint8_t SIZE_BAUDRATE                 = 1;

    static constexpr std::uint8_t ADDR_BUTTON_1_CONFIG          = 9;
    static constexpr std::uint8_t SIZE_BUTTON_1_CONFIG          = 1;

    static constexpr std::uint8_t ADDR_BUTTON_2_CONFIG          = 10;
    static constexpr std::uint8_t SIZE_BUTTON_2_CONFIG          = 1;

    static constexpr std::uint8_t ADDR_BUTTON_3_CONFIG          = 12;
    static constexpr std::uint8_t SIZE_BUTTON_3_CONFIG          = 1;

    static constexpr std::uint8_t ADDR_BUTTON_1_STATUS          = 13;
    static constexpr std::uint8_t SIZE_BUTTON_1_STATUS          = 1;

    static constexpr std::uint8_t ADDR_BUTTON_2_STATUS          = 14;
    static constexpr std::uint8_t SIZE_BUTTON_2_STATUS          = 1;

    static constexpr std::uint8_t ADDR_BUTTON_3_STATUS          = 15;
    static constexpr std::uint8_t SIZE_BUTTON_3_STATUS          = 1;

    static constexpr std::uint8_t ADDR_ACCELERO_VALUE_X         = 16;
    static constexpr std::uint8_t SIZE_ACCELERO_VALUE_X         = 4;

    static constexpr std::uint8_t ADDR_ACCELERO_VALUE_Y         = 20;
    static constexpr std::uint8_t SIZE_ACCELERO_VALUE_Y         = 4;

    static constexpr std::uint8_t ADDR_ACCELERO_VALUE_Z         = 24;
    static constexpr std::uint8_t SIZE_ACCELERO_VALUE_Z         = 4;

    static constexpr std::uint8_t ADDR_DIGITAL_IN               = 28;
    static constexpr std::uint8_t SIZE_DIGITAL_IN               = 1;

    static constexpr std::uint8_t ADDR_DIGITAL_OUT              = 29;
    static constexpr std::uint8_t SIZE_DIGITAL_OUT              = 1;

    static constexpr std::uint8_t ADDR_PRESENT_VOLTAGE          = 144;
    static constexpr std::uint8_t SIZE_PRESENT_VOLTAGE          = 2;

    static constexpr std::uint8_t ADDR_PRESENT_TEMPERATURE      = 146;
    static constexpr std::uint8_t SIZE_PRESENT_TEMPERATURE      = 1;
};
} // ttl_driver

#endif // END_EFFECTOR_REG_HPP
