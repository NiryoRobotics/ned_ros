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

#include "can_driver/abstract_stepper_driver.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include <cassert>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using ::common::model::EStepperCommandType;

namespace can_driver
{

/**
 * @brief AbstractStepperDriver::AbstractStepperDriver
 * @param mcp_can
 */
AbstractStepperDriver::AbstractStepperDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can) : AbstractCanDriver(std::move(mcp_can)) {}

/**
 * @brief AbstractStepperDriver::str
 * @return
 */
std::string AbstractStepperDriver::str() const { return "Abstract Stepper Driver (" + AbstractCanDriver::str() + ")"; }

/**
 * @brief AbstractStepperDriver::writeSingleCmd
 * @param cmd
 * @return
 */
int AbstractStepperDriver::writeSingleCmd(const std::unique_ptr<common::model::AbstractCanSingleMotorCmd> &cmd)
{
    if (cmd->isValid() && cmd->isStepperCmd())
    {
        switch (EStepperCommandType(cmd->getCmdType()))
        {
        case EStepperCommandType::CMD_TYPE_POSITION:
            return sendPositionCommand(cmd->getId(), cmd->getParams().front());
        case EStepperCommandType::CMD_TYPE_TORQUE:
            return sendTorqueOnCommand(cmd->getId(), cmd->getParams().front());
        case EStepperCommandType::CMD_TYPE_LEARNING_MODE:
            return sendTorqueOnCommand(cmd->getId(), !cmd->getParams().front());
        case EStepperCommandType::CMD_TYPE_SYNCHRONIZE:
            return sendSynchronizePositionCommand(cmd->getId(), cmd->getParams().front());
        case EStepperCommandType::CMD_TYPE_RELATIVE_MOVE:
            return sendRelativeMoveCommand(cmd->getId(), cmd->getParams().at(0), cmd->getParams().at(1));
        case EStepperCommandType::CMD_TYPE_MAX_EFFORT:
            return sendMaxEffortCommand(cmd->getId(), cmd->getParams().front());
        case EStepperCommandType::CMD_TYPE_MICRO_STEPS:
            return sendMicroStepsCommand(cmd->getId(), cmd->getParams().front());
        case EStepperCommandType::CMD_TYPE_CALIBRATION:
            return sendCalibrationCommand(cmd->getId(), cmd->getParams().at(0), cmd->getParams().at(1), cmd->getParams().at(2), cmd->getParams().at(3));

        case EStepperCommandType::CMD_TYPE_POSITION_OFFSET:
            return sendPositionOffsetCommand(cmd->getId(), cmd->getParams().at(0), cmd->getParams().at(1));
        case EStepperCommandType::CMD_TYPE_CONVEYOR:
            return sendConveyorOnCommand(cmd->getId(), cmd->getParams().at(0), static_cast<uint8_t>(cmd->getParams().at(1)), static_cast<uint8_t>(cmd->getParams().at(2)));
        default:
            std::cout << "Command not implemented " << cmd->getCmdType() << std::endl;
        }
    }

    std::cout << "AbstractStepperDriver::writeSingleCmd : Command not validated: " << cmd->str() << std::endl;
    return -1;
}

/**
 * @brief AbstractStepperDriver::interpretPositionStatus
 * @param data
 * @return
 */
int32_t AbstractStepperDriver::interpretPositionStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    int32_t pos = (data[1] << 16) + (data[2] << 8) + data[3];
    pos = (pos & (1 << 15)) ? -1 * ((~pos + 1) & 0xFFFF) : pos;

    return pos;
}

/**
 * @brief StepperDriver::interpretTemperatureStatus
 * @param data
 * @return
 */
uint8_t AbstractStepperDriver::interpretTemperatureStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    int driver_temp_raw = (data[2] << 8) + data[3];
    double a = -0.00316;
    double b = -12.924;
    double c = 2367.7;
    double v_temp = driver_temp_raw * 3.3 / 1024.0 * 1000.0;
    uint8_t driver_temp = static_cast<uint8_t>((-b - std::sqrt(b * b - 4 * a * (c - v_temp))) / (2 * a) + 30);

    return driver_temp;
}

/**
 * @brief StepperDriver::interpretFirmwareVersion
 * @param data
 * @return
 */
std::string AbstractStepperDriver::interpretFirmwareVersion(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    int v_major = data[1];
    int v_minor = data[2];
    int v_patch = data[3];
    std::ostringstream ss;
    ss << v_major << "." << v_minor << "." << v_patch;
    std::string version = ss.str();

    return version;
}

/**
 * @brief StepperDriver::interpretHomingData
 * @param data
 * @return
 */
std::pair<common::model::EStepperCalibrationStatus, int32_t> AbstractStepperDriver::interpretHomingData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    auto status = static_cast<common::model::EStepperCalibrationStatus>(data[1]);
    int32_t value = (data[2] << 8) + data[3];

    return std::make_pair(status, value);
}

/**
 * @brief StepperDriver::interpretConveyorData
 * @param data
 * @return
 */
std::tuple<bool, uint8_t, uint16_t> AbstractStepperDriver::interpretConveyorData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    bool state = data[1];
    int16_t speed = data[2];
    auto direction = static_cast<int8_t>(data[3]);

    return std::make_tuple(state, speed, direction);
}

}  // namespace can_driver
