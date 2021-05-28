/*
    xc430_driver.hpp
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

#ifndef XC430_DRIVER_HPP
#define XC430_DRIVER_HPP

#include <memory>

#include <vector>
#include <string>
#include <iostream>

#include "dynamixel_driver/xdriver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"


namespace DynamixelDriver {
    /**
     * @brief The XC430Driver class
     */
    class XC430Driver : public XDriver
    {
    public:
        XC430Driver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                    std::shared_ptr<dynamixel::PacketHandler> packetHandler);

        // XDriver interface
    public:
        std::string interpreteErrorState(uint32_t hw_state)  override;

         int checkModelNumber(uint8_t id)  override;

        // eeprom write
         int changeId(uint8_t id, uint8_t new_id)  override;
         int changeBaudRate(uint8_t id, uint32_t new_baudrate)  override;

         int setReturnDelayTime(uint8_t id, uint32_t return_delay_time)  override;
         int setLimitTemperature(uint8_t id, uint32_t temperature)  override;
         int setMaxTorque(uint8_t id, uint32_t torque)  override;
         int setReturnLevel(uint8_t id, uint32_t return_level)  override;
         int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)  override;

        // eeprom read
         int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)  override;
         int readLimitTemperature(uint8_t id, uint32_t *limit_temperature)  override;
         int readMaxTorque(uint8_t id, uint32_t *max_torque)  override;
         int readReturnLevel(uint8_t id, uint32_t *return_level)  override;
         int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)  override;

        // ram write
         int setTorqueEnable(uint8_t id, uint32_t torque_enable)  override;
         int setLed(uint8_t id, uint32_t led_value)  override;
         int setGoalPosition(uint8_t id, uint32_t position)  override;
         int setGoalVelocity(uint8_t id, uint32_t velocity)  override;
         int setGoalTorque(uint8_t id, uint32_t torque)  override;

         int setPGain(uint8_t id, uint32_t gain) override;
         int setIGain(uint8_t id, uint32_t gain) override;
         int setDGain(uint8_t id, uint32_t gain) override;
         int setff1Gain(uint8_t id, uint32_t gain) override;
         int setff2Gain(uint8_t id, uint32_t gain) override;

         int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list)  override;
         int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)  override;
         int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)  override;
         int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)  override;
         int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)  override;

        // ram read
         int readPosition(uint8_t id, uint32_t *present_position)  override;
         int readVelocity(uint8_t id, uint32_t *present_velocity)  override;
         int readLoad(uint8_t id, uint32_t *present_load)  override;
         int readTemperature(uint8_t id, uint32_t *temperature)  override;
         int readVoltage(uint8_t id, uint32_t *voltage)  override;
         int readHardwareStatus(uint8_t id, uint32_t *hardware_status)  override;

         int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)  override;
         int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)  override;
         int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)  override;
         int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)  override;
         int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)  override;
         int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)  override;
    };

} //DynamixelDriver

#endif
