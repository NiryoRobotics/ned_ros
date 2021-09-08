/*
xl30_reg.hpp
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

#ifndef XL320_REG_HPP
#define XL320_REG_HPP

#include <memory>
#include "common/model/hardware_type_enum.hpp"

namespace ttl_driver
{
struct XL320Reg
{
    static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::XL320;

    static constexpr int PROTOCOL_VERSION                  = 2.0;
    static constexpr int MODEL_NUMBER                      = 350;
    static constexpr double VOLTAGE_UNIT                   = 0.1;

    // see table here : http:// support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
    // EEPROM
    static constexpr std::uint8_t ADDR_MODEL_NUMBER        = 0;
    static constexpr std::uint8_t SIZE_MODEL_NUMBER        = 2;

    static constexpr std::uint8_t ADDR_FIRMWARE_VERSION    = 2;
    static constexpr std::uint8_t SIZE_FIRMWARE_VERSION    = 1;

    static constexpr std::uint8_t ADDR_ID                  = 3;
    static constexpr std::uint8_t SIZE_ID                  = 1;

    static constexpr std::uint8_t ADDR_BAUDRATE            = 4;
    static constexpr std::uint8_t SIZE_BAUDRATE            = 1;

    static constexpr std::uint8_t ADDR_RETURN_DELAY_TIME   = 5;
    static constexpr std::uint8_t SIZE_RETURN_DELAY_TIME   = 1;

    static constexpr std::uint8_t ADDR_CW_ANGLE_LIMIT      = 6;
    static constexpr std::uint8_t SIZE_CW_ANGLE_LIMIT      = 2;

    static constexpr std::uint8_t ADDR_CCW_ANGLE_LIMIT     = 8;
    static constexpr std::uint8_t SIZE_CCW_ANGLE_LIMIT     = 2;

    static constexpr std::uint8_t ADDR_CONTROL_MODE        = 11;
    static constexpr std::uint8_t SIZE_CONTROL_MODE        = 1;

    static constexpr std::uint8_t ADDR_TEMPERATURE_LIMIT   = 12;
    static constexpr std::uint8_t SIZE_TEMPERATURE_LIMIT   = 1;

    static constexpr std::uint8_t ADDR_MIN_VOLTAGE_LIMIT   = 13;
    static constexpr std::uint8_t SIZE_MIN_VOLTAGE_LIMIT   = 1;

    static constexpr std::uint8_t ADDR_MAX_VOLTAGE_LIMIT   = 14;
    static constexpr std::uint8_t SIZE_MAX_VOLTAGE_LIMIT   = 1;

    static constexpr std::uint8_t ADDR_MAX_TORQUE          = 15;
    static constexpr std::uint8_t SIZE_MAX_TORQUE          = 2;

    static constexpr std::uint8_t ADDR_STATUS_RETURN_LEVEL = 17;
    static constexpr std::uint8_t SIZE_STATUS_RETURN_LEVEL = 1;

    static constexpr std::uint8_t ADDR_ALARM_SHUTDOWN      = 18;
    static constexpr std::uint8_t SIZE_ALARM_SHUTDOWN      = 1;

    // RAM
    static constexpr std::uint8_t ADDR_TORQUE_ENABLE       = 24;
    static constexpr std::uint8_t SIZE_TORQUE_ENABLE       = 1;

    static constexpr std::uint8_t ADDR_LED                 = 25;
    static constexpr std::uint8_t SIZE_LED                 = 1;

    static constexpr std::uint8_t ADDR_POSITION_D_GAIN     = 27;
    static constexpr std::uint8_t SIZE_POSITION_D_GAIN     = 1;

    static constexpr std::uint8_t ADDR_POSITION_I_GAIN     = 28;
    static constexpr std::uint8_t SIZE_POSITION_I_GAIN     = 1;

    static constexpr std::uint8_t ADDR_POSITION_P_GAIN     = 29;
    static constexpr std::uint8_t SIZE_POSITION_P_GAIN     = 1;

    static constexpr std::uint8_t ADDR_GOAL_POSITION       = 30;
    static constexpr std::uint8_t SIZE_GOAL_POSITION       = 2;

    static constexpr std::uint8_t ADDR_GOAL_VELOCITY       = 32;
    static constexpr std::uint8_t SIZE_GOAL_VELOCITY       = 2;

    static constexpr std::uint8_t ADDR_GOAL_TORQUE         = 35;
    static constexpr std::uint8_t SIZE_GOAL_TORQUE         = 2;

    static constexpr std::uint8_t ADDR_PRESENT_POSITION    = 37;
    static constexpr std::uint8_t SIZE_PRESENT_POSITION    = 2;

    static constexpr std::uint8_t ADDR_PRESENT_VELOCITY    = 39;
    static constexpr std::uint8_t SIZE_PRESENT_VELOCITY    = 2;

    static constexpr std::uint8_t ADDR_PRESENT_LOAD        = 41;
    static constexpr std::uint8_t SIZE_PRESENT_LOAD        = 2;

    static constexpr std::uint8_t ADDR_PRESENT_VOLTAGE     = 45;
    static constexpr std::uint8_t SIZE_PRESENT_VOLTAGE     = 1;

    static constexpr std::uint8_t ADDR_PRESENT_TEMPERATURE = 46;
    static constexpr std::uint8_t SIZE_PRESENT_TEMPERATURE = 1;

    static constexpr std::uint8_t ADDR_REGISTERED          = 47;
    static constexpr std::uint8_t SIZE_REGISTERED          = 1;

    static constexpr std::uint8_t ADDR_MOVING              = 49;
    static constexpr std::uint8_t SIZE_MOVING              = 1;

    static constexpr std::uint8_t ADDR_HW_ERROR_STATUS     = 50;
    static constexpr std::uint8_t SIZE_HW_ERROR_STATUS     = 1;

    static constexpr std::uint8_t ADDR_PUNCH               = 51;
    static constexpr std::uint8_t SIZE_PUNCH               = 2;
};
} // ttl_driver

#endif // XL320_REG_HPP
