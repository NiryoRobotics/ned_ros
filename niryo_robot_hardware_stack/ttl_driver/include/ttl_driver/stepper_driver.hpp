/*
    stepper_driver.hpp
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

#ifndef TTLSTEPPERDRIVER_HPP
#define TTLSTEPPERDRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "abstract_motor_driver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common/common_defs.hpp"
#include "common/model/motor_type_enum.hpp"

#include "stepper_reg.hpp"

namespace ttl_driver
{
    /**
     * @brief The StepperDriver class
     */
    class StepperDriver : public AbstractMotorDriver
    {
    public:
        StepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                       std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        virtual ~StepperDriver() override;

        // TTLMotorDriver interface
    public:
        virtual std::string str() const override;
        virtual std::string interpreteErrorState(uint32_t hw_state) override;
        virtual int checkModelNumber(uint8_t id) override;
        virtual int changeId(uint8_t id, uint8_t new_id) override;
        virtual int changeBaudRate(uint8_t id, uint32_t new_baudrate) override;
        virtual int setReturnDelayTime(uint8_t id, uint32_t return_delay_time) override;
        virtual int setLimitTemperature(uint8_t id, uint32_t temperature) override;
        virtual int setMaxTorque(uint8_t id, uint32_t torque) override;
        virtual int setReturnLevel(uint8_t id, uint32_t return_level) override;
        virtual int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown) override;
        virtual int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time) override;
        virtual int readLimitTemperature(uint8_t id, uint32_t *limit_temperature) override;
        virtual int readMaxTorque(uint8_t id, uint32_t *max_torque) override;
        virtual int readReturnLevel(uint8_t id, uint32_t *return_level) override;
        virtual int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown) override;
        virtual int setTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        virtual int setLed(uint8_t id, uint32_t led_value) override;
        virtual int setGoalPosition(uint8_t id, uint32_t position) override;
        virtual int setGoalVelocity(uint8_t id, uint32_t velocity) override;
        virtual int setGoalTorque(uint8_t id, uint32_t torque) override;
        virtual int setPGain(uint8_t id, uint32_t gain) override;
        virtual int setIGain(uint8_t id, uint32_t gain) override;
        virtual int setDGain(uint8_t id, uint32_t gain) override;
        virtual int setff1Gain(uint8_t id, uint32_t gain) override;
        virtual int setff2Gain(uint8_t id, uint32_t gain) override;
        virtual int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) override;
        virtual int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        virtual int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        virtual int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;
        virtual int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) override;
        virtual int readPosition(uint8_t id, uint32_t *present_position) override;
        virtual int readVelocity(uint8_t id, uint32_t *present_velocity) override;
        virtual int readLoad(uint8_t id, uint32_t *present_load) override;
        virtual int readTemperature(uint8_t id, uint32_t *temperature) override;
        virtual int readVoltage(uint8_t id, uint32_t *voltage) override;
        virtual int readHardwareStatus(uint8_t id, uint32_t *hardware_status) override;
        virtual int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        virtual int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
        virtual int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) override;
        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) override;
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;
    };

}

#endif //TTLSTEPPERDRIVER_HPP
