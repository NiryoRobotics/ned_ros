/*
    abstract_motor_driver.cpp
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

#include "ttl_driver/abstract_dxl_driver.hpp"

#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"

#include <cassert>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <utility>
#include <vector>

using ::common::model::EDxlCommandType;

namespace ttl_driver
{

/**
 * @brief AbstractDxlDriver::AbstractDxlDriver
 */
AbstractDxlDriver::AbstractDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler, std::shared_ptr<dynamixel::PacketHandler> packetHandler)
    : AbstractMotorDriver(std::move(portHandler), std::move(packetHandler))
{
}

std::string AbstractDxlDriver::str() const { return "Dynamixel Driver (" + AbstractMotorDriver::str() + ")"; }

/**
 * @brief AbstractDxlDriver::writeSingleCmd
 * @param cmd
 */
int AbstractDxlDriver::writeSingleCmd(const std::unique_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd)
{
    if (cmd && cmd->isValid() && cmd->isDxlCmd())
    {
        switch (EDxlCommandType(cmd->getCmdType()))
        {
        case EDxlCommandType::CMD_TYPE_VELOCITY:
            return writeVelocityGoal(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_POSITION:
            return writePositionGoal(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_EFFORT:
            return writeTorqueGoal(cmd->getId(), static_cast<uint16_t>(cmd->getParam()));
        case EDxlCommandType::CMD_TYPE_TORQUE:
            return writeTorquePercentage(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        case EDxlCommandType::CMD_TYPE_LEARNING_MODE:
            return writeTorquePercentage(cmd->getId(), static_cast<uint8_t>(!cmd->getParam()));
        case EDxlCommandType::CMD_TYPE_PING:
            return ping(cmd->getId());
        case EDxlCommandType::CMD_TYPE_PID:
        {
            std::vector<uint16_t> params_conv;
            for (auto p : cmd->getParams())
            {
                params_conv.emplace_back(static_cast<uint16_t>(p));
            }
            return writePID(cmd->getId(), params_conv);
        }
        case EDxlCommandType::CMD_TYPE_PROFILE:
            return writeVelocityProfile(cmd->getId(), cmd->getParams());
        case EDxlCommandType::CMD_TYPE_CONTROL_MODE:
            return writeControlMode(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        case EDxlCommandType::CMD_TYPE_LED_STATE:
            return writeLed(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        case EDxlCommandType::CMD_TYPE_STARTUP:
            return writeStartupConfiguration(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        case EDxlCommandType::CMD_TYPE_TEMPERATURE_LIMIT:
            return writeTemperatureLimit(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        case EDxlCommandType::CMD_TYPE_SHUTDOWN:
            return writeShutdownConfiguration(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        default:
            std::cout << "Command not implemented " << cmd->getCmdType() << std::endl;
        }
    }

    std::cout << "AbstractDxlDriver::writeSingleCmd : Command not validated: " << cmd->str() << std::endl;
    return COMM_RX_CORRUPT;
}

/**
 * @brief AbstractDxlDriver::writeSyncCmd
 * @param type
 * @param ids
 * @param params
 */
int AbstractDxlDriver::writeSyncCmd(int type, const std::vector<uint8_t> &ids, const std::vector<uint32_t> &params)
{
    assert(!ids.empty() && "AbstractDxlDriver::writeSyncCmd: ids is empty");
    assert(!params.empty() && "AbstractDxlDriver::writeSyncCmd: params is empty");

    switch (EDxlCommandType(type))
    {
    case EDxlCommandType::CMD_TYPE_POSITION:
        return syncWritePositionGoal(ids, params);
    case EDxlCommandType::CMD_TYPE_VELOCITY:
        return syncWriteVelocityGoal(ids, params);
    case EDxlCommandType::CMD_TYPE_EFFORT:
    {
        std::vector<uint16_t> params_conv;
        params_conv.reserve(params.size());
        for (auto const &p : params)
        {
            params_conv.emplace_back(static_cast<uint16_t>(p));
        }
        return syncWriteTorqueGoal(ids, params_conv);
    }
    case EDxlCommandType::CMD_TYPE_TORQUE:
    {
        std::vector<uint8_t> params_conv;
        params_conv.reserve(params.size());
        for (auto const &p : params)
        {
            params_conv.emplace_back(static_cast<uint8_t>(p));
        }
        return syncWriteTorquePercentage(ids, params_conv);
    }
    case EDxlCommandType::CMD_TYPE_LEARNING_MODE:
    {
        std::vector<uint8_t> params_inv;
        params_inv.reserve(params.size());
        for (auto const &p : params)
        {
            params_inv.emplace_back(!p);
        }
        return syncWriteTorquePercentage(ids, params_inv);
    }
    default:
        std::cout << "Command not implemented " << type << std::endl;
    }

    std::cout << "AbstractDxlDriver::writeSyncCmd : Command not validated: " << type << std::endl;
    return -1;
}

}  // namespace ttl_driver
