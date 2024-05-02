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

#include "ttl_driver/abstract_ttl_driver.hpp"

namespace ttl_driver
{

/**
 * @brief The XDriver class
 */
// CC add list of associated motors ? this would remove the need for a map and for some params
class AbstractMotorDriver : public AbstractTtlDriver
{
public:
    AbstractMotorDriver() = default;
    AbstractMotorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                        std::shared_ptr<dynamixel::PacketHandler> packetHandler);

public:
    // AbstractTtlDriver interface
    std::string str() const override;

public:
    // here are only common TTL commands found in both Steppers and DXl

    // eeprom write
    virtual int changeId(uint8_t id, uint8_t new_id) = 0;

    // eeprom read
    virtual int readMinPosition(uint8_t id, uint32_t& min_pos) = 0;
    virtual int readMaxPosition(uint8_t id, uint32_t& max_pos) = 0;

    // ram write
    
    virtual int writeControlMode(uint8_t id, uint8_t mode) = 0;
    virtual int writeVelocityProfile(uint8_t id, const std::vector<uint32_t>& data_list) = 0;

    virtual int writeTorquePercentage(uint8_t id, uint8_t torque_percentage) = 0;
    virtual int writePositionGoal(uint8_t id, uint32_t position) = 0;
    virtual int writeVelocityGoal(uint8_t id, uint32_t velocity) = 0;

    virtual int syncWriteTorquePercentage(const std::vector<uint8_t>& id_list, const std::vector<uint8_t>& torque_percentage_list) = 0;
    virtual int syncWritePositionGoal(const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& position_list) = 0;
    virtual int syncWriteVelocityGoal(const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& velocity_list) = 0;

    // ram read
    virtual int readControlMode(uint8_t id, uint8_t& mode) = 0;
    virtual int readVelocityProfile(uint8_t id, std::vector<uint32_t>& data_list) = 0;

    virtual int readPosition(uint8_t id, uint32_t& present_position) = 0;
    virtual int readVelocity(uint8_t id, uint32_t& present_velocity) = 0;

    virtual int syncReadPosition(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& position_list) = 0;
    virtual int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t>& velocity_list) = 0;
    virtual int syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2> >& data_array_list) = 0;
};

} // ttl_driver

#endif // ABSTRACT_MOTOR_DRIVER_HPP
