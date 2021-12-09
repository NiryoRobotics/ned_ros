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
using ::common::model::EStepperCalibrationStatus;

namespace ttl_driver
{

/**
 * @brief AbstractStepperDriver::AbstractStepperDriver
*/
AbstractStepperDriver::AbstractStepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                             std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractMotorDriver(std::move(portHandler), std::move(packetHandler))
{}

/**
 * @brief AbstractStepperDriver::str
 * @return
 */
std::string AbstractStepperDriver::str() const
{
    return "Abstract Stepper Driver  (" + AbstractMotorDriver::str() + ")";
}

/**
 * @brief AbstractStepperDriver::writeSingleCmd
 * @param cmd
*/
int AbstractStepperDriver::writeSingleCmd(const std::unique_ptr<common::model::AbstractTtlSingleMotorCmd>& cmd)
{
    if (cmd->isValid() && cmd->isStepperCmd())
    {
        switch (EStepperCommandType(cmd->getCmdType()))
        {
        case EStepperCommandType::CMD_TYPE_VELOCITY:
            return writeVelocityGoal(cmd->getId(), cmd->getParam());
        case EStepperCommandType::CMD_TYPE_POSITION:
            return writePositionGoal(cmd->getId(), cmd->getParam());
        case EStepperCommandType::CMD_TYPE_TORQUE:
            return writeTorqueEnable(cmd->getId(), static_cast<uint8_t>(cmd->getParam()));
        case EStepperCommandType::CMD_TYPE_LEARNING_MODE:
            return writeTorqueEnable(cmd->getId(), static_cast<uint8_t>(!cmd->getParam()));
        case EStepperCommandType::CMD_TYPE_CALIBRATION:
            return startHoming(cmd->getId());
        case EStepperCommandType::CMD_TYPE_CALIBRATION_SETUP:
            return writeHomingSetup(cmd->getId(), static_cast<uint8_t>(cmd->getParams().at(0)),
                                                  static_cast<uint8_t>(cmd->getParams().at(1)));
        case EStepperCommandType::CMD_TYPE_PING:
            return ping(cmd->getId());
        case EStepperCommandType::CMD_TYPE_CONVEYOR:
        {
            std::vector<uint32_t> params = cmd->getParams();
            if (!params[0])
            {
                return writeVelocityGoal(cmd->getId(), 0);
            }

            // convert direction and speed into signed speed
            int8_t dir = static_cast<int8_t>(cmd->getParams().at(2));
            // normal warning : we need to put an int32 inside an uint32_t
            // param received from user/app is in percentage. It have to be converted to speed (unit 0.01 rpm) accepted by ttl conveyor
            // TODO(Thuc) avoid hardcode 6000 here
            uint32_t speed = static_cast<uint32_t>(static_cast<int>(cmd->getParams().at(1)) * dir * 6000 / 100);
            return writeVelocityGoal(cmd->getId(), speed);
        }
        case EStepperCommandType::CMD_TYPE_VELOCITY_PROFILE:
            return writeVelocityProfile(cmd->getId(), cmd->getParams());
        default:
            std::cout << "Command not implemented " << cmd->getCmdType() << std::endl;
        }
    }

    std::cout << "AbstractStepperDriver::writeSingleCmd: Command not validated : " << cmd->str() << std::endl;
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
    {
        std::vector<uint8_t> params_conv;
        params_conv.reserve(params.size());
        for (auto const& p : params)
        {
            params_conv.emplace_back(static_cast<uint8_t>(p));
        }
        return syncWriteTorqueEnable(ids, params_conv);
    }
    case EStepperCommandType::CMD_TYPE_LEARNING_MODE:
    {
        std::vector<uint8_t> params_inv;
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

    std::cout << "AbstractStepperDriver::writeSyncCmd : Command not validated : " << type << std::endl;
    return -1;
}

/**
 * @brief AbstractStepperDriver::interpretFirmwareVersion
 * @param fw_version
 * @return
 */
std::string AbstractStepperDriver::interpretFirmwareVersion(uint32_t fw_version) const
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

/**
 * @brief AbstractStepperDriver::interpretErrorState
 * @param hw_state
 * @return
 */
std::string AbstractStepperDriver::interpretErrorState(uint32_t hw_state) const
{
    std::string hardware_message;

    if (hw_state & 1<<0)    // 0b00000001
    {
        hardware_message += "Input Voltage";
    }
    if (hw_state & 1<<2)    // 0b00000100
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "OverHeating";
    }
    if (hw_state & 1<<3)    // 0b00001000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Motor Encoder";
    }
    if (hw_state & 1<<7)    // 0b10000000 => added by us : disconnected error
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Disconnection";
    }

    if (!hardware_message.empty())
        hardware_message += " Error";

    return hardware_message;
}

/**
 * @brief AbstractStepperDriver::interpretHomingData
 * @param status
 * @return
 */
common::model::EStepperCalibrationStatus
AbstractStepperDriver::interpretHomingData(uint8_t status) const
{
    EStepperCalibrationStatus homing_status{EStepperCalibrationStatus::UNINITIALIZED};

    switch (status)
    {
    case 0:
      homing_status = EStepperCalibrationStatus::UNINITIALIZED;
      break;
    case 1:
      homing_status = EStepperCalibrationStatus::IN_PROGRESS;
      break;
    case 2:
      homing_status = EStepperCalibrationStatus::OK;
      break;
    case 3:
      homing_status = EStepperCalibrationStatus::FAIL;
      break;
    default:
      homing_status = EStepperCalibrationStatus::FAIL;
      break;
    }

    return homing_status;
}

}  // namespace ttl_driver
