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
#include <utility>
#include <vector>
#include <string>

using ::common::model::EDxlCommandType;

namespace ttl_driver
{

/**
 * @brief AbstractDxlDriver::AbstractDxlDriver
*/
AbstractDxlDriver::AbstractDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                     std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractMotorDriver(std::move(portHandler), std::move(packetHandler))
{}

std::string AbstractDxlDriver::str() const
{
    return "Dynamixel Driver (" + AbstractMotorDriver::str() + ")";
}

/**
 * @brief AbstractDxlDriver::writeSingleCmd
 * @param cmd
*/
int AbstractDxlDriver::writeSingleCmd(const std::unique_ptr<common::model::AbstractTtlSingleMotorCmd >& cmd)
{
    if (cmd && cmd->isValid() && cmd->isDxlCmd())
    {
        switch (EDxlCommandType(cmd->getCmdType()))
        {
        case EDxlCommandType::CMD_TYPE_VELOCITY:
            return writeGoalVelocity(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_POSITION:
            return writeGoalPosition(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_EFFORT:
            return writeGoalTorque(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_TORQUE:
            return writeTorqueEnable(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_LEARNING_MODE:
            return writeTorqueEnable(cmd->getId(), !cmd->getParam());
        case EDxlCommandType::CMD_TYPE_PING:
            return ping(cmd->getId());
        case EDxlCommandType::CMD_TYPE_PID:
            return writePID(cmd->getId(), cmd->getParams());
        default:
            std::cout << "Command not implemented " << cmd->getCmdType() << std::endl;
        }
    }

    std::cout << "Command not validated" << std::endl;
    return COMM_RX_CORRUPT;
}

/**
 * @brief AbstractDxlDriver::writeSyncCmd
 * @param type
 * @param ids
 * @param params
*/
int AbstractDxlDriver::writeSyncCmd(int type, const std::vector<uint8_t>& ids, const std::vector<uint32_t>& params)
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
        return syncWriteTorqueGoal(ids, params);
    case EDxlCommandType::CMD_TYPE_TORQUE:
        return syncWriteTorqueEnable(ids, params);
    case EDxlCommandType::CMD_TYPE_LEARNING_MODE:
    {
        std::vector<uint32_t> params_inv;
        params_inv.reserve(params.size());
        for (auto const& p : params)
        {
            params_inv.emplace_back(!p);
        }
        return syncWriteTorqueEnable(ids, params_inv);
    }
    default:
        std::cout << "Command not implemented " << type << std::endl;
    }

    std::cout << "Command not validated" << std::endl;
    return -1;
}

}  // namespace ttl_driver
