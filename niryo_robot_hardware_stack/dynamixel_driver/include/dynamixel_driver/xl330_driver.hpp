/*
    xl330_driver.hpp
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

#ifndef XL330_DRIVER_HPP
#define XL330_DRIVER_HPP

#include <memory>
#include <vector>

#include "dynamixel_driver/xdriver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"



namespace DynamixelDriver
{
    /**
     * @brief The XL330Driver class
     */
    class XL330Driver : public XDriver
    {
    public:
        XL330Driver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                    std::shared_ptr<dynamixel::PacketHandler> packetHandler);

        std::string interpreteErrorState(uint32_t hw_state);

        //from XDriver interface
        int checkModelNumber(uint8_t id);

        // eeprom write
        int changeId(uint8_t id, uint8_t new_id);
        int changeBaudRate(uint8_t id, uint32_t new_baudrate);
        int setReturnDelayTime(uint8_t id, uint32_t return_delay_time);
        int setLimitTemperature(uint8_t id, uint32_t temperature);
        int setMaxTorque(uint8_t id, uint32_t torque);
        int setReturnLevel(uint8_t id, uint32_t return_level);
        int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown);

        // eeprom read
        int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time);
        int readLimitTemperature(uint8_t id, uint32_t *limit_temperature);
        int readMaxTorque(uint8_t id, uint32_t *max_torque);
        int readReturnLevel(uint8_t id, uint32_t *return_level);
        int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown);

        // ram write
        int setTorqueEnable(uint8_t id, uint32_t torque_enable);
        int setLed(uint8_t id, uint32_t led_value);
        int setGoalPosition(uint8_t id, uint32_t position);
        int setGoalVelocity(uint8_t id, uint32_t velocity);
        int setGoalTorque(uint8_t id, uint32_t torque);

        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list);
        int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list);
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list);
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list);
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list);

        // ram read
        int readPosition(uint8_t id, uint32_t *present_position);
        int readVelocity(uint8_t id, uint32_t *present_velocity);
        int readLoad(uint8_t id, uint32_t *present_load);
        int readTemperature(uint8_t id, uint32_t *temperature);
        int readVoltage(uint8_t id, uint32_t *voltage);
        int readHardwareStatus(uint8_t id, uint32_t *hardware_status);

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list);
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list);
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list);
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list);
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list);
        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list);

    private:
        static constexpr int XL330_PROTOCOL_VERSION         = 2.0;
        static constexpr int XL330_MODEL_NUMBER             = 1200;

        //see table here : https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
        static constexpr int XL330_ADDR_MODEL_NUMBER        = 0;
        static constexpr int XL330_ADDR_FIRMWARE_VERSION    = 6;
        static constexpr int XL330_ADDR_ID                  = 7;
        static constexpr int XL330_ADDR_BAUDRATE            = 8;
        static constexpr int XL330_ADDR_RETURN_DELAY_TIME   = 9;
        static constexpr int XL330_ADDR_DRIVE_MODE          = 10;
        static constexpr int XL330_ADDR_OPERATING_MODE      = 11;
        static constexpr int XL330_ADDR_HOMING_OFFSET       = 20; // EEPROM (not all addresses)
        static constexpr int XL330_ADDR_TEMPERATURE_LIMIT   = 31;
        static constexpr int XL330_ADDR_MAX_VOLTAGE_LIMIT   = 32;
        static constexpr int XL330_ADDR_MIN_VOLTAGE_LIMIT   = 34;
        static constexpr int XL330_ADDR_CURRENT_LIMIT       = 38;
        static constexpr int XL330_ADDR_MAX_POSITION_LIMIT  = 48;
        static constexpr int XL330_ADDR_MIN_POSITION_LIMIT  = 52;
        static constexpr int XL330_ADDR_ALARM_SHUTDOWN      = 63;

        static constexpr int XL330_ADDR_TORQUE_ENABLE       = 64;
        static constexpr int XL330_ADDR_LED                 = 65;
        static constexpr int XL330_ADDR_STATUS_RETURN_LEVEL = 68;
        static constexpr int XL330_ADDR_HW_ERROR_STATUS     = 70;
        static constexpr int XL330_ADDR_GOAL_PWM            = 100;
        static constexpr int XL330_ADDR_GOAL_CURRENT        = 102;
        static constexpr int XL330_ADDR_GOAL_VELOCITY       = 104;
        static constexpr int XL330_ADDR_GOAL_POSITION       = 116; // RAM (not all addresses)
        static constexpr int XL330_ADDR_MOVING              = 122;
        static constexpr int XL330_ADDR_PRESENT_PWM         = 124;
        static constexpr int XL330_ADDR_PRESENT_CURRENT     = 126;
        static constexpr int XL330_ADDR_PRESENT_VELOCITY    = 128;
        static constexpr int XL330_ADDR_PRESENT_POSITION    = 132;
        static constexpr int XL330_ADDR_PRESENT_VOLTAGE     = 144;
        static constexpr int XL330_ADDR_PRESENT_TEMPERATURE = 146;

    public:

        // according to xl-330 datasheet : 1 speed ~ 0.229 rpm ~ 3.9083 dxl position per second
        static constexpr double XL330_STEPS_FOR_1_SPEED     = 15.6331; // 0.229 * 4096 / 60

        static constexpr int    TOTAL_RANGE_POSITION     = 4095;
        static constexpr int    MIDDLE_POSITION       = 2047;
        static constexpr double TOTAL_ANGLE           = 296.67;
    };
} //DynamixelDriver

#endif
