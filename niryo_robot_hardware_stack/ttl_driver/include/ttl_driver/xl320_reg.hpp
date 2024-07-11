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

    static constexpr float PROTOCOL_VERSION                = 2.0;
    static constexpr int MODEL_NUMBER                      = 350;
    static constexpr int VOLTAGE_CONVERSION                = 10;

    // see table here : http:// support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
    // EEPROM
    static constexpr uint16_t ADDR_MODEL_NUMBER        = 0;
    using TYPE_MODEL_NUMBER = uint16_t;

    static constexpr uint16_t ADDR_FIRMWARE_VERSION    = 2;
    using TYPE_FIRMWARE_VERSION = uint8_t;

    static constexpr uint16_t ADDR_ID                  = 3;
    using TYPE_ID = uint8_t;

    static constexpr uint16_t ADDR_BAUDRATE            = 4;
    using TYPE_BAUDRATE = uint8_t;

    static constexpr uint16_t ADDR_RETURN_DELAY_TIME   = 5;
    using TYPE_RETURN_DELAY_TIME = uint8_t;

    static constexpr uint16_t ADDR_CW_ANGLE_LIMIT      = 6;
    using TYPE_CW_ANGLE_LIMIT = uint16_t;

    static constexpr uint16_t ADDR_CCW_ANGLE_LIMIT     = 8;
    using TYPE_CCW_ANGLE_LIMIT = uint16_t;

    static constexpr uint16_t ADDR_OPERATING_MODE        = 11;
    using TYPE_OPERATING_MODE = uint8_t;

    static constexpr uint16_t ADDR_TEMPERATURE_LIMIT   = 12;
    using TYPE_TEMPERATURE_LIMIT = uint8_t;

    static constexpr uint16_t ADDR_MIN_VOLTAGE_LIMIT   = 13;
    using TYPE_MIN_VOLTAGE_LIMIT = uint8_t;

    static constexpr uint16_t ADDR_MAX_VOLTAGE_LIMIT   = 14;
    using TYPE_MAX_VOLTAGE_LIMIT = uint8_t;

    static constexpr uint16_t ADDR_MAX_TORQUE          = 15;
    using TYPE_MAX_TORQUE = uint16_t;

    static constexpr uint16_t ADDR_STATUS_RETURN_LEVEL = 17;
    using TYPE_STATUS_RETURN_LEVEL = uint8_t;

    static constexpr uint16_t ADDR_ALARM_SHUTDOWN      = 18;
    using TYPE_ALARM_SHUTDOWN = uint8_t;

    // RAM
    static constexpr uint16_t ADDR_TORQUE_ENABLE       = 24;
    using TYPE_TORQUE_ENABLE = uint8_t;

    static constexpr uint16_t ADDR_LED                 = 25;
    using TYPE_LED = uint8_t;

    using TYPE_PID_GAIN = uint8_t;
    static constexpr uint16_t ADDR_POSITION_D_GAIN     = 27;
    static constexpr uint16_t ADDR_POSITION_I_GAIN     = 28;
    static constexpr uint16_t ADDR_POSITION_P_GAIN     = 29;

    static constexpr uint16_t ADDR_GOAL_POSITION       = 30;
    using TYPE_GOAL_POSITION = uint16_t;

    static constexpr uint16_t ADDR_GOAL_VELOCITY       = 32;
    using TYPE_GOAL_VELOCITY = uint16_t;

    static constexpr uint16_t ADDR_GOAL_TORQUE         = 35;
    using TYPE_GOAL_TORQUE = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_POSITION    = 37;
    using TYPE_PRESENT_POSITION = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_VELOCITY    = 39;
    using TYPE_PRESENT_VELOCITY = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_LOAD        = 41;
    using TYPE_PRESENT_LOAD = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_VOLTAGE     = 45;
    using TYPE_PRESENT_VOLTAGE = uint8_t;

    static constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 46;
    using TYPE_PRESENT_TEMPERATURE = uint8_t;

    static constexpr uint16_t ADDR_REGISTERED          = 47;
    using TYPE_REGISTERED = uint8_t;

    static constexpr uint16_t ADDR_MOVING              = 49;
    using TYPE_MOVING = uint8_t;

    static constexpr uint16_t ADDR_HW_ERROR_STATUS     = 50;
    using TYPE_HW_ERROR_STATUS = uint8_t;

    static constexpr uint16_t ADDR_PUNCH               = 51;
    using TYPE_PUNCH = uint16_t;
};
} // ttl_driver

#endif // XL320_REG_HPP
