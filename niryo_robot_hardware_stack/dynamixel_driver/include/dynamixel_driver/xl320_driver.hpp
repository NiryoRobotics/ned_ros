/*
    xl320_driver.hpp
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

#ifndef XL320_DRIVER_HPP
#define XL320_DRIVER_HPP

#include <memory>
#include <vector>

#include "dynamixel_driver/xdriver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"


namespace DynamixelDriver
{
    /**
     * @brief The XL320Driver class
     */
    class XL320Driver : public XDriver
    {
    public:
        XL320Driver(std::shared_ptr<dynamixel::PortHandler> portHandler,
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

        int customWrite(uint8_t id, uint8_t reg_address, uint32_t value, uint8_t byte_number);
        int customRead(uint8_t id, uint8_t reg_address, uint32_t &value, uint8_t byte_number);

    private:
        static constexpr int XL320_PROTOCOL_VERSION         = 2.0;
        static constexpr int XL320_MODEL_NUMBER             = 350;

        //see table here : http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
        static constexpr int XL320_ADDR_MODEL_NUMBER        = 0;
        static constexpr int XL320_ADDR_FIRMWARE_VERSION    = 2;
        static constexpr int XL320_ADDR_ID                  = 3;
        static constexpr int XL320_ADDR_BAUDRATE            = 4;
        static constexpr int XL320_ADDR_RETURN_DELAY_TIME   = 5;
        static constexpr int XL320_ADDR_CW_ANGLE_LIMIT      = 6;
        static constexpr int XL320_ADDR_CCW_ANGLE_LIMIT     = 8; // EEPROM
        static constexpr int XL320_ADDR_CONTROL_MODE        = 11;
        static constexpr int XL320_ADDR_LIMIT_TEMPERATURE   = 12;
        static constexpr int XL320_ADDR_LOWER_LIMIT_VOLTAGE = 13;
        static constexpr int XL320_ADDR_UPPER_LIMIT_VOLTAGE = 14;
        static constexpr int XL320_ADDR_MAX_TORQUE          = 15;
        static constexpr int XL320_ADDR_RETURN_LEVEL        = 17;
        static constexpr int XL320_ADDR_ALARM_SHUTDOWN      = 18;

        static constexpr int XL320_ADDR_TORQUE_ENABLE       = 24;
        static constexpr int XL320_ADDR_LED                 = 25;
        static constexpr int XL320_ADDR_D_GAIN              = 27;
        static constexpr int XL320_ADDR_I_GAIN              = 28;
        static constexpr int XL320_ADDR_P_GAIN              = 29;
        static constexpr int XL320_ADDR_GOAL_POSITION       = 30;
        static constexpr int XL320_ADDR_GOAL_SPEED          = 32;
        static constexpr int XL320_ADDR_GOAL_TORQUE         = 35;
        static constexpr int XL320_ADDR_PRESENT_POSITION    = 37; // RAM
        static constexpr int XL320_ADDR_PRESENT_SPEED       = 39;
        static constexpr int XL320_ADDR_PRESENT_LOAD        = 41;
        static constexpr int XL320_ADDR_PRESENT_VOLTAGE     = 45;
        static constexpr int XL320_ADDR_PRESENT_TEMPERATURE = 46;
        static constexpr int XL320_ADDR_REGISTERED          = 47;
        static constexpr int XL320_ADDR_MOVING              = 49;
        static constexpr int XL320_ADDR_HW_ERROR_STATUS     = 50;
        static constexpr int XL320_ADDR_PUNCH               = 51;

    public:

        // according to xl-320 datasheet : 1 speed ~ 0.111 rpm ~ 1.8944 dxl position per second
        static constexpr double XL320_STEPS_FOR_1_SPEED = 1.8944; // 0.111 * 1024 / 60

        // we stop at 1022 instead of 1023, to get an odd number of positions (1023)
        // --> so we can get a middle point (511)
        static constexpr int    TOTAL_RANGE_POSITION    = 1023;
        static constexpr int    MIDDLE_POSITION         = 511;
        static constexpr double TOTAL_ANGLE             = 296.67;
    };
} //DynamixelDriver

#endif
