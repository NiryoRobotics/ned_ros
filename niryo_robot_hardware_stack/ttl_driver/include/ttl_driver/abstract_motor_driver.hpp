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

#ifndef ABSTRACT_MOTOR_DRIVER_HPP
#define ABSTRACT_MOTOR_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common/common_defs.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/abstract_synchronize_motor_cmd.hpp"

#include "ttl_driver/abstract_ttl_driver.hpp"

namespace ttl_driver
{

/**
 * @brief The XDriver class
 */
// CC add list of associated motors ? this would remove the need for a map and for some params
// generic driver -> write and write for a given address and lenght
class AbstractMotorDriver : public AbstractTtlDriver
{

public:
    AbstractMotorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                      std::shared_ptr<dynamixel::PacketHandler> packetHandler);
    virtual ~AbstractMotorDriver();


    virtual int writeSingleCmd(std::shared_ptr<common::model::AbstractTtlSingleMotorCmd >& cmd) = 0;
    virtual int writeSyncCmd(int type, const std::vector<uint8_t>& ids, const std::vector<uint32_t>& params) = 0;

public:
    // AbstractTtlDriver interface
    virtual std::string str() const override;

    virtual std::string interpreteErrorState(uint32_t hw_state) = 0;

    // eeprom read
    virtual int checkModelNumber(uint8_t id) = 0;
    virtual int readFirmwareVersion(uint8_t id, uint32_t& version) = 0;

    // ram read
    virtual int readTemperature(uint8_t id, uint32_t& temperature) = 0;
    virtual int readVoltage(uint8_t id, uint32_t& voltage) = 0;
    virtual int readHwErrorStatus(uint8_t id, uint32_t& hardware_status) = 0;

    virtual int syncReadTemperature(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& temperature_list) = 0;
    virtual int syncReadVoltage(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& voltage_list) = 0;
    virtual int syncReadHwErrorStatus(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& hw_error_list) = 0;

public:
    // here are only common TTL commands found in both Steppers and DXl

    // eeprom write
    virtual int changeId(uint8_t id, uint8_t new_id) = 0;

    // eeprom read
    virtual int readMinPosition(uint8_t id, uint32_t& min_pos) = 0;
    virtual int readMaxPosition(uint8_t id, uint32_t& max_pos) = 0;

    // ram write
    virtual int setTorqueEnable(uint8_t id, uint32_t torque_enable) = 0;
    virtual int setGoalPosition(uint8_t id, uint32_t position) = 0;
    virtual int setGoalVelocity(uint8_t id, uint32_t velocity) = 0;

    virtual int syncWriteTorqueEnable(const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& torque_enable_list) = 0;
    virtual int syncWritePositionGoal(const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& position_list) = 0;
    virtual int syncWriteVelocityGoal(const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& velocity_list) = 0;

    // ram read
    virtual int readPosition(uint8_t id, uint32_t& present_position) = 0;

    virtual int syncReadPosition(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& position_list) = 0;
};

} // ttl_driver

#endif // ABSTRACT_MOTOR_DRIVER_HPP
