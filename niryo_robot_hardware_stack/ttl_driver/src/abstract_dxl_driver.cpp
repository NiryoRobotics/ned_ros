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
#include <vector>

using ::common::model::EDxlCommandType;

namespace ttl_driver
{

/**
 * @brief AbstractDxlDriver::AbstractDxlDriver
*/
AbstractDxlDriver::AbstractDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                     std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractTtlDriver(portHandler, packetHandler)
{}

/**
 * @brief AbstractDxlDriver::~AbstractDxlDriver
*/
AbstractDxlDriver::~AbstractDxlDriver()
{}

/**
 * @brief AbstractDxlDriver::writeSingleCmd
 * @param cmd
*/
int AbstractDxlDriver::writeSingleCmd(const std::shared_ptr<common::model::AbstractTtlSingleMotorCmd > &cmd)
{
    if (cmd && cmd->isValid() && cmd->isDxlCmd())
    {
        switch (EDxlCommandType(cmd->getCmdType()))
        {
        case EDxlCommandType::CMD_TYPE_VELOCITY:
            return setGoalVelocity(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_POSITION:
            return setGoalPosition(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_EFFORT:
            return setGoalTorque(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_TORQUE:
            return setTorqueEnable(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_PING:
        case EDxlCommandType::CMD_TYPE_LEARNING_MODE:
            return ping(cmd->getId());
        case EDxlCommandType::CMD_TYPE_POSITION_P_GAIN:
            return setPositionPGain(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_POSITION_I_GAIN:
            return setPositionIGain(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_POSITION_D_GAIN:
            return setPositionDGain(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_VELOCITY_P_GAIN:
            return setVelocityPGain(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_VELOCITY_I_GAIN:
            return setVelocityIGain(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_FF1_GAIN:
            return setff1Gain(cmd->getId(), cmd->getParam());
        case EDxlCommandType::CMD_TYPE_FF2_GAIN:
            return setff2Gain(cmd->getId(), cmd->getParam());
        default:
            std::cout << "Command not implemented" << std::endl;
        }
    }
    
    std::cout << "Command not validated" << std::endl;
    return -1;
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
    case EDxlCommandType::CMD_TYPE_LEARNING_MODE:
        return syncWriteTorqueEnable(ids, params);
    default:
        std::cout << "Command not implemented" << std::endl;
    }

    std::cout << "Command not validated" << std::endl;
    return -1;
}

}  // namespace ttl_driver
