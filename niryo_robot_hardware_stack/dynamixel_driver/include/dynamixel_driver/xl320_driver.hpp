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

        std::string interpreteErrorState(uint32_t hw_state) override;

        //from XDriver interface
        int checkModelNumber(uint8_t id) override;

        // eeprom write
        int changeId(uint8_t id, uint8_t new_id) override;
        int changeBaudRate(uint8_t id, uint32_t new_baudrate) override;
        int setReturnDelayTime(uint8_t id, uint32_t return_delay_time) override;
        int setLimitTemperature(uint8_t id, uint32_t temperature) override;
        int setMaxTorque(uint8_t id, uint32_t torque) override;
        int setReturnLevel(uint8_t id, uint32_t return_level) override;
        int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown) override;

        // eeprom read
        int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time) override;
        int readLimitTemperature(uint8_t id, uint32_t *limit_temperature) override;
        int readMaxTorque(uint8_t id, uint32_t *max_torque) override;
        int readReturnLevel(uint8_t id, uint32_t *return_level) override;
        int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown) override;

        // ram write
        int setTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        int setLed(uint8_t id, uint32_t led_value) override;
        int setGoalPosition(uint8_t id, uint32_t position) override;
        int setGoalVelocity(uint8_t id, uint32_t velocity) override;
        int setGoalTorque(uint8_t id, uint32_t torque) override;

        int setPGain(uint8_t id, uint32_t gain) override;
        int setIGain(uint8_t id, uint32_t gain) override;
        int setDGain(uint8_t id, uint32_t gain) override;
        int setff1Gain(uint8_t, uint32_t ) override;
        int setff2Gain(uint8_t, uint32_t ) override;

        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) override;
        int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) override;

        // ram read
        int readPosition(uint8_t id, uint32_t *present_position) override;
        int readVelocity(uint8_t id, uint32_t *present_velocity) override;
        int readLoad(uint8_t id, uint32_t *present_load) override;
        int readTemperature(uint8_t id, uint32_t *temperature) override;
        int readVoltage(uint8_t id, uint32_t *voltage) override;
        int readHardwareStatus(uint8_t id, uint32_t *hardware_status) override;

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) override;
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) override;
        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

    private:
        static constexpr int XL320_PROTOCOL_VERSION         = 2.0;
        static constexpr int XL320_MODEL_NUMBER             = 350;

        //see table here : http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
        static constexpr int XL320_ADDR_MODEL_NUMBER        = 0; // EEPROM
        static constexpr int XL320_ADDR_FIRMWARE_VERSION    = 2;
        static constexpr int XL320_ADDR_ID                  = 3;
        static constexpr int XL320_ADDR_BAUDRATE            = 4;
        static constexpr int XL320_ADDR_RETURN_DELAY_TIME   = 5;
        static constexpr int XL320_ADDR_CW_ANGLE_LIMIT      = 6;
        static constexpr int XL320_ADDR_CCW_ANGLE_LIMIT     = 8;
        static constexpr int XL320_ADDR_CONTROL_MODE        = 11;
        static constexpr int XL320_ADDR_LIMIT_TEMPERATURE   = 12;
        static constexpr int XL320_ADDR_LOWER_LIMIT_VOLTAGE = 13;
        static constexpr int XL320_ADDR_UPPER_LIMIT_VOLTAGE = 14;
        static constexpr int XL320_ADDR_MAX_TORQUE          = 15;
        static constexpr int XL320_ADDR_RETURN_LEVEL        = 17;
        static constexpr int XL320_ADDR_ALARM_SHUTDOWN      = 18;

        static constexpr int XL320_ADDR_TORQUE_ENABLE       = 24; // RAM
        static constexpr int XL320_ADDR_LED                 = 25;

        static constexpr int XL320_ADDR_D_GAIN              = 27;
        static constexpr int XL320_ADDR_I_GAIN              = 28;
        static constexpr int XL320_ADDR_P_GAIN              = 29;

        static constexpr int XL320_ADDR_GOAL_POSITION       = 30;
        static constexpr int XL320_ADDR_GOAL_SPEED          = 32;
        static constexpr int XL320_ADDR_GOAL_TORQUE         = 35;
        static constexpr int XL320_ADDR_PRESENT_POSITION    = 37;
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

    };
} //DynamixelDriver

#endif
