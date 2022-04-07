/*
xl330_reg.hpp
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

#ifndef XL330_REG_HPP
#define XL330_REG_HPP

#include <memory>
#include "common/model/hardware_type_enum.hpp"

namespace ttl_driver
{
struct XL330Reg
{
    static constexpr common::model::EHardwareType motor_type = common::model::EHardwareType::XL330;

    static constexpr float PROTOCOL_VERSION                     = 2.0;
    static constexpr int MODEL_NUMBER                           = 1200;
    static constexpr int VOLTAGE_CONVERSION                     = 10;

    // see table here : http:// support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
    // EEPROM
    static constexpr uint16_t ADDR_MODEL_NUMBER             = 0;
    using TYPE_MODEL_NUMBER = uint16_t;

    static constexpr uint16_t ADDR_MODEL_INFORMATION        = 2;
    using TYPE_MODEL_INFORMATION = uint32_t;

    static constexpr uint16_t ADDR_FIRMWARE_VERSION         = 6;
    using TYPE_FIRMWARE_VERSION = uint8_t;

    static constexpr uint16_t ADDR_ID                       = 7;
    using TYPE_ID = uint8_t;

    static constexpr uint16_t ADDR_BAUDRATE                 = 8;
    using TYPE_BAUDRATE = uint8_t;

    static constexpr uint16_t ADDR_RETURN_DELAY_TIME        = 9;
    using TYPE_RETURN_DELAY_TIME = uint8_t;

    static constexpr uint16_t ADDR_DRIVE_MODE               = 10;
    using TYPE_DRIVE_MODE = uint8_t;

    static constexpr uint16_t ADDR_OPERATING_MODE           = 11;
    using TYPE_OPERATING_MODE = uint8_t;

    static constexpr uint16_t ADDR_HOMING_OFFSET            = 20;
    using TYPE_HOMING_OFFSET = uint32_t;

    static constexpr uint16_t ADDR_TEMPERATURE_LIMIT        = 31;
    using TYPE_TEMPERATURE_LIMIT = uint8_t;

    static constexpr uint16_t ADDR_MAX_VOLTAGE_LIMIT        = 32;
    using TYPE_MAX_VOLTAGE_LIMIT = uint16_t;

    static constexpr uint16_t ADDR_MIN_VOLTAGE_LIMIT        = 34;
    using TYPE_MIN_VOLTAGE_LIMIT = uint16_t;

    static constexpr uint16_t ADDR_PWM_LIMIT                = 36;
    using TYPE_PWM_LIMIT = uint16_t;

    static constexpr uint16_t ADDR_CURRENT_LIMIT            = 38;
    using TYPE_CURRENT_LIMIT = uint16_t;

    static constexpr uint16_t ADDR_VELOCITY_LIMIT           = 44;
    using TYPE_VELOCITY_LIMIT = uint32_t;

    static constexpr uint16_t ADDR_MAX_POSITION_LIMIT       = 48;
    using TYPE_MAX_POSITION_LIMIT = uint32_t;

    static constexpr uint16_t ADDR_MIN_POSITION_LIMIT       = 52;
    using TYPE_MIN_POSITION_LIMIT = uint32_t;

    static constexpr uint16_t ADDR_STARTUP_CONFIGURATION    = 60;
    static constexpr uint8_t VERSION_STARTUP_CONFIGURATION  = 46;
    using TYPE_STARTUP_CONFIGURATION = uint8_t;

    static constexpr uint16_t ADDR_ALARM_SHUTDOWN           = 63;
    using TYPE_ALARM_SHUTDOWN = uint8_t;

    // RAM
    static constexpr uint16_t ADDR_TORQUE_ENABLE            = 64;
    using TYPE_TORQUE_ENABLE = uint8_t;

    static constexpr uint16_t ADDR_LED                      = 65;
    using TYPE_LED = uint8_t;

    static constexpr uint16_t ADDR_STATUS_RETURN_LEVEL      = 68;
    using TYPE_STATUS_RETURN_LEVEL = uint8_t;

    static constexpr uint16_t ADDR_REGISTERED_INSTRUCTION   = 69;
    using TYPE_REGISTERED_INSTRUCTION = uint8_t;

    static constexpr uint16_t ADDR_HW_ERROR_STATUS          = 70;
    using TYPE_HW_ERROR_STATUS = uint8_t;

    using TYPE_PID_GAIN = uint16_t;
    static constexpr uint16_t ADDR_VELOCITY_I_GAIN          = 76;
    static constexpr uint16_t ADDR_VELOCITY_P_GAIN          = 78;
    static constexpr uint16_t ADDR_POSITION_D_GAIN          = 80;
    static constexpr uint16_t ADDR_POSITION_I_GAIN          = 82;
    static constexpr uint16_t ADDR_POSITION_P_GAIN          = 84;
    static constexpr uint16_t ADDR_FF2_GAIN                 = 88;
    static constexpr uint16_t ADDR_FF1_GAIN                 = 90;

    static constexpr uint16_t ADDR_BUS_WATCHDOG             = 98;
    using TYPE_BUS_WATCHDOG = uint8_t;

    static constexpr uint16_t ADDR_GOAL_PWM                 = 100;
    using TYPE_GOAL_PWM = uint16_t;

    static constexpr uint16_t ADDR_GOAL_CURRENT             = 102;
    using TYPE_GOAL_CURRENT = uint16_t;

    static constexpr uint16_t ADDR_GOAL_VELOCITY            = 104;
    using TYPE_GOAL_VELOCITY = uint32_t;

    using TYPE_PROFILE = uint32_t;
    static constexpr uint16_t ADDR_PROFILE_ACCELERATION     = 108;
    static constexpr uint16_t ADDR_PROFILE_VELOCITY         = 112;

    static constexpr uint16_t ADDR_GOAL_POSITION            = 116;
    using TYPE_GOAL_POSITION = uint32_t;

    static constexpr uint16_t ADDR_MOVING                   = 122;
    using TYPE_MOVING = uint8_t;

    static constexpr uint16_t ADDR_MOVING_STATUS            = 123;
    using TYPE_MOVING_STATUS = uint8_t;

    static constexpr uint16_t ADDR_PRESENT_PWM              = 124;
    using TYPE_PRESENT_PWM = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_CURRENT          = 126;
    using TYPE_PRESENT_CURRENT = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_VELOCITY         = 128;
    using TYPE_PRESENT_VELOCITY = uint32_t;

    static constexpr uint16_t ADDR_PRESENT_POSITION         = 132;
    using TYPE_PRESENT_POSITION = uint32_t;

    static constexpr uint16_t ADDR_VELOCITY_TRAJECTORY      = 136;
    using TYPE_VELOCITY_TRAJECTORY = uint32_t;

    static constexpr uint16_t ADDR_POSITION_TRAJECTORY      = 140;
    using TYPE_POSITION_TRAJECTORY = uint32_t;

    static constexpr uint16_t ADDR_PRESENT_VOLTAGE          = 144;
    using TYPE_PRESENT_VOLTAGE = uint16_t;

    static constexpr uint16_t ADDR_PRESENT_TEMPERATURE      = 146;
    using TYPE_PRESENT_TEMPERATURE = uint8_t;
};

} // ttl_driver

#endif // XL330_REG_HPP
