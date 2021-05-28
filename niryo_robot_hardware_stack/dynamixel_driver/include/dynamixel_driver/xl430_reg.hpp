/*
    xl430_reg.hpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef XL430REG_HPP
#define XL430REG_HPP

#include <memory>
#include "model/motor_type_enum.hpp"

namespace DynamixelDriver
{
    struct XL430Reg
    {
        static constexpr common::model::EMotorType motor_type = common::model::EMotorType::XL430;

        static constexpr int PROTOCOL_VERSION                       = 2.0;
        static constexpr int MODEL_NUMBER                           = 1060;

        //see table here : http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
        // EEPROM
        static constexpr std::uint8_t ADDR_MODEL_NUMBER             = 0;
        static constexpr std::uint8_t SIZE_MODEL_NUMBER             = 2;

        static constexpr std::uint8_t ADDR_MODEL_INFORMATION        = 2;
        static constexpr std::uint8_t SIZE_MODEL_INFORMATION        = 4;

        static constexpr std::uint8_t ADDR_FIRMWARE_VERSION         = 6;
        static constexpr std::uint8_t SIZE_FIRMWARE_VERSION         = 1;

        static constexpr std::uint8_t ADDR_ID                       = 7;
        static constexpr std::uint8_t SIZE_ID                       = 1;

        static constexpr std::uint8_t ADDR_BAUDRATE                 = 8;
        static constexpr std::uint8_t SIZE_BAUDRATE                 = 1;

        static constexpr std::uint8_t ADDR_RETURN_DELAY_TIME        = 9;
        static constexpr std::uint8_t SIZE_RETURN_DELAY_TIME        = 1;

        static constexpr std::uint8_t ADDR_DRIVE_MODE               = 10;
        static constexpr std::uint8_t SIZE_DRIVE_MODE               = 1;

        static constexpr std::uint8_t ADDR_OPERATING_MODE           = 11;
        static constexpr std::uint8_t SIZE_OPERATING_MODE           = 1;

        static constexpr std::uint8_t ADDR_HOMING_OFFSET            = 20;
        static constexpr std::uint8_t SIZE_HOMING_OFFSET            = 4;

        static constexpr std::uint8_t ADDR_TEMPERATURE_LIMIT        = 31;
        static constexpr std::uint8_t SIZE_TEMPERATURE_LIMIT        = 1;

        static constexpr std::uint8_t ADDR_MAX_VOLTAGE_LIMIT        = 32;
        static constexpr std::uint8_t SIZE_MAX_VOLTAGE_LIMIT        = 2;

        static constexpr std::uint8_t ADDR_MIN_VOLTAGE_LIMIT        = 34;
        static constexpr std::uint8_t SIZE_MIN_VOLTAGE_LIMIT        = 2;

        static constexpr std::uint8_t ADDR_PWM_LIMIT                = 36;
        static constexpr std::uint8_t SIZE_PWM_LIMIT                = 2;

        static constexpr std::uint8_t ADDR_VELOCITY_LIMIT           = 44;
        static constexpr std::uint8_t SIZE_VELOCITY_LIMIT           = 4;

        static constexpr std::uint8_t ADDR_MAX_POSITION_LIMIT       = 48;
        static constexpr std::uint8_t SIZE_MAX_POSITION_LIMIT       = 4;

        static constexpr std::uint8_t ADDR_MIN_POSITION_LIMIT       = 52;
        static constexpr std::uint8_t SIZE_MIN_POSITION_LIMIT       = 4;

        static constexpr std::uint8_t ADDR_ALARM_SHUTDOWN           = 63;
        static constexpr std::uint8_t SIZE_ALARM_SHUTDOWN           = 1;

        // RAM
        static constexpr std::uint8_t ADDR_TORQUE_ENABLE            = 64;
        static constexpr std::uint8_t SIZE_TORQUE_ENABLE            = 1;

        static constexpr std::uint8_t ADDR_LED                      = 65;
        static constexpr std::uint8_t SIZE_LED                      = 1;

        static constexpr std::uint8_t ADDR_STATUS_RETURN_LEVEL      = 68;
        static constexpr std::uint8_t SIZE_STATUS_RETURN_LEVEL      = 1;

        static constexpr std::uint8_t ADDR_REGISTERED_INSTRUCTION   = 69;
        static constexpr std::uint8_t SIZE_REGISTERED_INSTRUCTION   = 1;

        static constexpr std::uint8_t ADDR_HW_ERROR_STATUS          = 70;
        static constexpr std::uint8_t SIZE_HW_ERROR_STATUS          = 1;

        static constexpr std::uint8_t ADDR_VELOCITY_I_GAIN          = 76;
        static constexpr std::uint8_t SIZE_VELOCITY_I_GAIN          = 2;

        static constexpr std::uint8_t ADDR_VELOCITY_P_GAIN          = 78;
        static constexpr std::uint8_t SIZE_VELOCITY_P_GAIN          = 2;

        static constexpr std::uint8_t ADDR_POSITION_D_GAIN          = 80;
        static constexpr std::uint8_t SIZE_POSITION_D_GAIN          = 2;

        static constexpr std::uint8_t ADDR_POSITION_I_GAIN          = 82;
        static constexpr std::uint8_t SIZE_POSITION_I_GAIN          = 2;

        static constexpr std::uint8_t ADDR_POSITION_P_GAIN          = 84;
        static constexpr std::uint8_t SIZE_POSITION_P_GAIN          = 2;

        static constexpr std::uint8_t ADDR_FF2_GAIN                 = 88;
        static constexpr std::uint8_t SIZE_FF2_GAIN                 = 2;

        static constexpr std::uint8_t ADDR_FF1_GAIN                 = 90;
        static constexpr std::uint8_t SIZE_FF1_GAIN                 = 2;

        static constexpr std::uint8_t ADDR_BUS_WATCHDOG             = 98;
        static constexpr std::uint8_t SIZE_BUS_WATCHDOG             = 1;

        static constexpr std::uint8_t ADDR_GOAL_PWM                 = 100;
        static constexpr std::uint8_t SIZE_GOAL_PWM                 = 2;

        static constexpr std::uint8_t ADDR_GOAL_VELOCITY            = 104;
        static constexpr std::uint8_t SIZE_GOAL_VELOCITY            = 4;

        static constexpr std::uint8_t ADDR_PROFILE_ACCELERATION     = 108;
        static constexpr std::uint8_t SIZE_PROFILE_ACCELERATION     = 4;

        static constexpr std::uint8_t ADDR_PROFILE_VELOCITY         = 112;
        static constexpr std::uint8_t SIZE_PROFILE_VELOCITY         = 4;

        static constexpr std::uint8_t ADDR_GOAL_POSITION            = 116;
        static constexpr std::uint8_t SIZE_GOAL_POSITION            = 4;

        static constexpr std::uint8_t ADDR_MOVING                   = 122;
        static constexpr std::uint8_t SIZE_MOVING                   = 1;

        static constexpr std::uint8_t ADDR_MOVING_STATUS            = 123;
        static constexpr std::uint8_t SIZE_MOVING_STATUS            = 1;

        static constexpr std::uint8_t ADDR_PRESENT_PWM              = 124;
        static constexpr std::uint8_t SIZE_PRESENT_PWM              = 2;

        static constexpr std::uint8_t ADDR_PRESENT_LOAD             = 126;
        static constexpr std::uint8_t SIZE_PRESENT_LOAD             = 2;

        static constexpr std::uint8_t ADDR_PRESENT_VELOCITY         = 128;
        static constexpr std::uint8_t SIZE_PRESENT_VELOCITY         = 4;

        static constexpr std::uint8_t ADDR_PRESENT_POSITION         = 132;
        static constexpr std::uint8_t SIZE_PRESENT_POSITION         = 4;

        static constexpr std::uint8_t ADDR_VELOCITY_TRAJECTORY      = 136;
        static constexpr std::uint8_t SIZE_VELOCITY_TRAJECTORY      = 4;

        static constexpr std::uint8_t ADDR_POSITION_TRAJECTORY      = 140;
        static constexpr std::uint8_t SIZE_POSITION_TRAJECTORY      = 4;

        static constexpr std::uint8_t ADDR_PRESENT_VOLTAGE          = 144;
        static constexpr std::uint8_t SIZE_PRESENT_VOLTAGE          = 2;

        static constexpr std::uint8_t ADDR_PRESENT_TEMPERATURE      = 146;
        static constexpr std::uint8_t SIZE_PRESENT_TEMPERATURE      = 1;
    };
} //DynamixelDriver

#endif // XL430REG_HPP
