/*
ttl_motor_driver.hpp
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

#ifndef TTLMOTORDRIVER_HPP
#define TTLMOTORDRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common/common_defs.hpp"
#include "common/model/motor_type_enum.hpp"

namespace ttl_driver
{

/**
 * @brief The XDriver class
 */
// CC add list of associated motors ? this would remove the need for a map and for some params
class AbstractMotorDriver
{

public:
    AbstractMotorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
            std::shared_ptr<dynamixel::PacketHandler> packetHandler);
    virtual ~AbstractMotorDriver();

    int ping(uint8_t id);
    int getModelNumber(uint8_t id,
                       uint16_t *dxl_model_number);
    int scan(std::vector<uint8_t> &id_list);
    int reboot(uint8_t id);

    virtual std::string str() const;

    virtual std::string interpreteErrorState(uint32_t hw_state) = 0;

    // get model number - specific to a child
    virtual int checkModelNumber(uint8_t id) = 0;

    // eeprom write
    virtual int changeId(uint8_t id, uint8_t new_id) = 0;

    virtual int changeBaudRate(uint8_t id, uint32_t new_baudrate) = 0;

    virtual int setReturnDelayTime(uint8_t id, uint32_t return_delay_time) = 0;
    virtual int setLimitTemperature(uint8_t id, uint32_t temperature) = 0;
    virtual int setMaxTorque(uint8_t id, uint32_t torque) = 0;
    virtual int setReturnLevel(uint8_t id, uint32_t return_level) = 0;
    virtual int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown) = 0;

    // eeprom read
    virtual int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time) = 0;
    virtual int readLimitTemperature(uint8_t id, uint32_t *limit_temperature) = 0;
    virtual int readMaxTorque(uint8_t id, uint32_t *max_torque) = 0;
    virtual int readReturnLevel(uint8_t id, uint32_t *return_level) = 0;
    virtual int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown) = 0;

    // ram write
    virtual int setTorqueEnable(uint8_t id, uint32_t torque_enable) = 0;
    virtual int setLed(uint8_t id, uint32_t led_value) = 0;
    virtual int setGoalPosition(uint8_t id, uint32_t position) = 0;
    virtual int setGoalVelocity(uint8_t id, uint32_t velocity) = 0;
    virtual int setGoalTorque(uint8_t id, uint32_t torque) = 0;

    virtual int setPGain(uint8_t id, uint32_t gain) = 0;
    virtual int setIGain(uint8_t id, uint32_t gain) = 0;
    virtual int setDGain(uint8_t id, uint32_t gain) = 0;
    virtual int setff1Gain(uint8_t id, uint32_t gain) = 0;
    virtual int setff2Gain(uint8_t id, uint32_t gain) = 0;

    virtual int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) = 0;
    virtual int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) = 0;
    virtual int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) = 0;
    virtual int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) = 0;
    virtual int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) = 0;

    // ram read
    virtual int readPosition(uint8_t id, uint32_t *present_position) = 0;
    virtual int readVelocity(uint8_t id, uint32_t *present_velocity) = 0;
    virtual int readLoad(uint8_t id, uint32_t *present_load) = 0;
    virtual int readTemperature(uint8_t id, uint32_t *temperature) = 0;
    virtual int readVoltage(uint8_t id, uint32_t *voltage) = 0;
    virtual int readHardwareStatus(uint8_t id, uint32_t *hardware_status) = 0;

    virtual int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) = 0;
    virtual int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) = 0;
    virtual int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) = 0;
    virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) = 0;
    virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) = 0;
    virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) = 0;

    // custom write and read
    int read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t *data);
    int write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data);

protected:
    int syncRead(uint8_t address, uint8_t data_len, const std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);
    int syncWrite(uint8_t address, uint8_t data_len, const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &data_list);

    static constexpr int PING_WRONG_MODEL_NUMBER = 30;

private:
    std::shared_ptr<dynamixel::PortHandler> _dxlPortHandler;
    std::shared_ptr<dynamixel::PacketHandler> _dxlPacketHandler;

    static constexpr uint8_t DXL_LEN_ONE_BYTE    = 1;
    static constexpr uint8_t DXL_LEN_TWO_BYTES   = 2;
    static constexpr uint8_t DXL_LEN_FOUR_BYTES  = 4;

    static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
    static constexpr int GROUP_SYNC_READ_RX_FAIL = 11;
    static constexpr int LEN_ID_DATA_NOT_SAME    = 20;
};
} // DynamixelDriver

#endif // TTLMOTORDRIVER_HPP
