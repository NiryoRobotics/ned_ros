/*
abstract_can_driver.hpp
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

#ifndef ABSTRACT_CAN_DRIVER_HPP
#define ABSTRACT_CAN_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "ros/ros.h"

#include "mcp_can_rpi/mcp_can_rpi.h"
#include "common/common_defs.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"

namespace can_driver
{

/**
 * @brief The AbstractCanDriver class
 */
class AbstractCanDriver
{

public:
    static constexpr int MAX_MESSAGE_LENGTH                     = 8;
    static constexpr int MESSAGE_LENGTH                         = 4;

    static constexpr double STEPPER_MOTOR_TIMEOUT_VALUE         = 1.0;

public:
    AbstractCanDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can);
    virtual ~AbstractCanDriver();

    bool canReadData() const;

    virtual int ping(uint8_t id);
    virtual int scan(const std::set<uint8_t>& motors_to_find, std::vector<uint8_t> &id_list);

    virtual int writeSingleCmd(const std::shared_ptr<common::model::AbstractCanSingleMotorCmd >& cmd) = 0;

public:
    virtual std::string str() const;

    // here are only common TTL commands found in both Steppers and DXl
    // write
    virtual uint8_t sendUpdateConveyorId(uint8_t old_id, uint8_t new_id) = 0;
    virtual uint8_t sendTorqueOnCommand(uint8_t id, int torque_on) = 0;
    virtual uint8_t sendRelativeMoveCommand(uint8_t id, int steps, int delay) = 0;

    virtual uint8_t sendPositionCommand(uint8_t id, int cmd) = 0;
    virtual uint8_t sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position) = 0;
    virtual uint8_t sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout) = 0;
    virtual uint8_t sendSynchronizePositionCommand(uint8_t id, bool begin_traj) = 0;
    virtual uint8_t sendMicroStepsCommand(uint8_t id, int micro_steps) = 0;
    virtual uint8_t sendMaxEffortCommand(uint8_t id, int effort) = 0;
    virtual uint8_t sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction) = 0;

    // read
    uint8_t readData(uint8_t& id, int& control_byte,
                     std::array<uint8_t, MAX_MESSAGE_LENGTH>& rxBuf,
                     std::string& error_message);
protected:
    uint8_t read(INT32U *id, uint8_t *len, std::array<uint8_t, MAX_MESSAGE_LENGTH> &buf);
    uint8_t write(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);

private:
    std::shared_ptr<mcp_can_rpi::MCP_CAN> _mcp_can;

};

/**
 * @brief CanManager::canReadData
 * @return
 */
inline
bool AbstractCanDriver::canReadData() const
{
  return _mcp_can->canReadData();
}

} // can_driver

#endif // ABSTRACT_CAN_DRIVER_HPP
