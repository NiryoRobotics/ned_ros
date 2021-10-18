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

#include "ttl_driver/abstract_stepper_driver.hpp"
#include "common/model/stepper_command_type_enum.hpp"

#include <cassert>
#include <string>
#include <utility>
#include <vector>
#include <ros/ros.h>

using ::common::model::EStepperCommandType;

namespace ttl_driver
{

/**
 * @brief AbstractStepperDriver::AbstractStepperDriver
*/
AbstractStepperDriver::AbstractStepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                             std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractMotorDriver(std::move(portHandler), std::move(packetHandler))
{}

std::string AbstractStepperDriver::str() const
{
    return "Abstract Stepper Driver  (" + AbstractMotorDriver::str() + ")";
}

/**
 * @brief AbstractStepperDriver::writeSingleCmd
 * @param cmd
*/
int AbstractStepperDriver::writeSingleCmd(std::unique_ptr<common::model::AbstractTtlSingleMotorCmd> && cmd)
{
    if (cmd->isValid() && cmd->isStepperCmd())
    {
        switch (EStepperCommandType(cmd->getCmdType()))
        {
        case EStepperCommandType::CMD_TYPE_VELOCITY:
            return setGoalVelocity(cmd->getId(), cmd->getParam());
        case EStepperCommandType::CMD_TYPE_POSITION:
            return setGoalPosition(cmd->getId(), cmd->getParam());
        case EStepperCommandType::CMD_TYPE_TORQUE:
            return setTorqueEnable(cmd->getId(), cmd->getParam());
        case EStepperCommandType::CMD_TYPE_LEARNING_MODE:
            return setTorqueEnable(cmd->getId(), !cmd->getParam());
        case EStepperCommandType::CMD_TYPE_CALIBRATION:
            return startHoming(cmd->getId());
        case EStepperCommandType::CMD_TYPE_CALIBRATION_DIRECTION:
            return writeHomingDirection(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        case EStepperCommandType::CMD_TYPE_PING:
            return ping(cmd->getId());
        case EStepperCommandType::CMD_TYPE_CONVEYOR:
        {
            std::vector<uint32_t> params = cmd->getParams();
            if (!params[0])
            {
                return setGoalVelocity(cmd->getId(), 0);
            }

            // convert direction and speed into signed speed
            int8_t dir = static_cast<int8_t>(cmd->getParams().at(2));
            // normal warning : we need to put an int32 inside an uint32_t
            uint32_t speed = static_cast<uint32_t>(static_cast<int>(cmd->getParams().at(1)) * dir);
            return setGoalVelocity(cmd->getId(), speed);
        }
        case EStepperCommandType::CMD_TYPE_VELOCITY_PROFILE:
            return writeVelocityProfile(cmd->getId(), cmd->getParams());
        default:
            std::cout << "Command not implemented " << cmd->getCmdType() << std::endl;
        }
    }

    std::cout << "Command not validated" << std::endl;
    return -1;
}

/**
 * @brief AbstractStepperDriver::writeSyncCmd
 * @param type
 * @param ids
 * @param params
*/
int AbstractStepperDriver::writeSyncCmd(int type, const std::vector<uint8_t>& ids, const std::vector<uint32_t>& params)
{
    assert(!ids.empty() && "AbstractStepperDriver::writeSyncCmdwriteSyncCmd: ids is empty");
    assert(!params.empty() && "AbstractStepperDriver::writeSyncCmdwriteSyncCmd: params is empty");

    switch (EStepperCommandType(type))
    {
    case EStepperCommandType::CMD_TYPE_POSITION:
        return syncWritePositionGoal(ids, params);
    case EStepperCommandType::CMD_TYPE_VELOCITY:
        return syncWriteVelocityGoal(ids, params);
    case EStepperCommandType::CMD_TYPE_TORQUE:
        return syncWriteTorqueEnable(ids, params);
    case EStepperCommandType::CMD_TYPE_LEARNING_MODE:
    {
        std::vector<uint32_t> params_inv;
        params_inv.reserve(params.size());
for (auto const& p : params)
        {
            params_inv.emplace_back(!p);
        }
        return syncWriteTorqueEnable(ids, params);
    }
    default:
        std::cout << "Command not implemented " << type << std::endl;
    }

    std::cout << "Command not validated" << std::endl;
    return -1;
}

/**
 * @brief AbstractStepperDriver::interpreteFirmwareVersion
 * @param fw_version
 * @return
 */
std::string AbstractStepperDriver::interpreteFirmwareVersion(uint32_t fw_version) const
{
    auto v_major = static_cast<uint8_t>(fw_version >> 24);
    auto v_minor = static_cast<uint16_t>(fw_version >> 8);
    auto v_patch = static_cast<uint8_t>(fw_version >> 0);

    std::ostringstream ss;
    ss << std::to_string(v_major) << "."
       << std::to_string(v_minor) << "."
       << std::to_string(v_patch);
    std::string version = ss.str();

    return version;
}

}  // namespace ttl_driver
