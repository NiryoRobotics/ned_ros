/*
    mock_stepper_driver.cpp
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

#include "can_driver/mock_stepper_driver.hpp"

#include <boost/exception/exception.hpp>
#include <cstdint>
#include <random>
#include <sstream>
#include <vector>
#include <string>
#include <tuple>
#include <utility>
#include <set>

#include "common/model/stepper_calibration_status_enum.hpp"
#include "mcp_can_rpi/mcp_can_dfs_rpi.h"
#include "ros/ros.h"

#include "common/model/stepper_command_type_enum.hpp"

using ::std::shared_ptr;
using ::std::string;
using ::std::ostringstream;
using ::common::model::EStepperCommandType;

namespace can_driver
{

/**
 * @brief MockStepperDriver::MockStepperDriver
 */
MockStepperDriver::MockStepperDriver(FakeCanData data)
{
    // steppers
    for (auto fdata : data.stepper_registers)
    {
        FakeCanData::FakeRegister tmp;
        tmp.id = fdata.id;
        tmp.position = fdata.position;
        tmp.temperature = fdata.temperature;
        tmp.voltage = fdata.voltage;
        tmp.model_number = fdata.model_number;
        tmp.firmware = fdata.firmware;
        if (!_map_fake_registers.count(tmp.id))
            _map_fake_registers.insert(std::pair<uint8_t, FakeCanData::FakeRegister>(tmp.id, tmp));
        _id_list.push_back(tmp.id);
    }
    _fake_conveyor.id = data.conveyor.id;

    _id_list.push_back(_fake_conveyor.id);
}

/**
 * @brief MockStepperDriver::~StepperDriver
 */
MockStepperDriver::~MockStepperDriver()
{
}

/**
 * @brief MockStepperDriver::ping
 * @param id
 * @return
 */
int MockStepperDriver::ping(uint8_t id)
{
    if (std::find(_id_list.begin(), _id_list.end(), id) != _id_list.end())
        return CAN_OK;
    else
        return CAN_FAIL;
}

/**
 * @brief MockStepperDriver::scan : try to find "motors_to_find" list of motors for a given time
 * @param motors_to_find
 * @param id_list
 * @return
 */
int MockStepperDriver::scan(const std::set<uint8_t>& motors_to_find, std::vector<uint8_t> &id_list)
{
    int result = CAN_FAIL;

    std::set<uint8_t> motors_unfound = motors_to_find;
    id_list.clear();

    for (auto id : motors_to_find)
    {
        if (std::find(_id_list.begin(), _id_list.end(), id) != _id_list.end())
        {
            id_list.push_back(id);
            motors_unfound.erase(id);
        }
    }
    // if found everything
    if (motors_unfound.empty())
        result = CAN_OK;

    return result;
}

/**
 * @brief MockStepperDriver::writeSingleCmd
 * @param cmd
 * @return
 */
int MockStepperDriver::writeSingleCmd(const std::shared_ptr<common::model::AbstractCanSingleMotorCmd> &cmd)
{
    if (cmd->isValid() && cmd->isStepperCmd())
    {
        switch (EStepperCommandType(cmd->getCmdType()))
        {
            case EStepperCommandType::CMD_TYPE_POSITION:
                return sendPositionCommand(cmd->getId(),
                                           cmd->getParams().front());
            case EStepperCommandType::CMD_TYPE_TORQUE:
                return sendTorqueOnCommand(cmd->getId(),
                                           cmd->getParams().front());
            case EStepperCommandType::CMD_TYPE_LEARNING_MODE:
                return sendTorqueOnCommand(cmd->getId(),
                                           !cmd->getParams().front());
            case EStepperCommandType::CMD_TYPE_SYNCHRONIZE:
                return sendSynchronizePositionCommand(cmd->getId(),
                                                      cmd->getParams().front());
            case EStepperCommandType::CMD_TYPE_RELATIVE_MOVE:
                return sendRelativeMoveCommand(cmd->getId(),
                                               cmd->getParams().at(0),
                                               cmd->getParams().at(1));
            case EStepperCommandType::CMD_TYPE_MAX_EFFORT:
                return sendMaxEffortCommand(cmd->getId(),
                                            cmd->getParams().front());
            case EStepperCommandType::CMD_TYPE_MICRO_STEPS:
                return sendMicroStepsCommand(cmd->getId(),
                                             cmd->getParams().front());
            case EStepperCommandType::CMD_TYPE_CALIBRATION:
                return sendCalibrationCommand(cmd->getId(),
                                              cmd->getParams().at(0),
                                              cmd->getParams().at(1),
                                              cmd->getParams().at(2),
                                              cmd->getParams().at(3));

            case EStepperCommandType::CMD_TYPE_POSITION_OFFSET:
                    return sendPositionOffsetCommand(cmd->getId(),
                                                     cmd->getParams().at(0),
                                                     cmd->getParams().at(1));
            case EStepperCommandType::CMD_TYPE_CONVEYOR:
                    return sendConveyorOnCommand(cmd->getId(),
                                                 cmd->getParams().at(0),
                                                 static_cast<uint8_t>(cmd->getParams().at(1)),
                                                 cmd->getParams().at(2) > 0 ? 1 : 0);
            default:
                std::cout << "Command not implemented " << cmd->getCmdType() << std::endl;
        }
    }

    std::cout << "Command not validated" << std::endl;
    return -1;
}

/**
 * @brief MockStepperDriver::readData a fake can driver have to make fake events.
 * This function generate generate control byte and id for each loop to send all type of events
 * @param id
 * @param control_byte
 * @param rxBuf
 * @param error_message
 * @return
 */
uint8_t MockStepperDriver::readData(uint8_t& id, int& control_byte,
                                    std::array<uint8_t, MAX_MESSAGE_LENGTH>& rxBuf,
                                    std::string& error_message)
{
    // change ID to generate event on the next id
    _current_id = _id_list[_next_index];
    id = _current_id;
    _next_index++;
    if (_next_index > MAX_IDX + 1)
    {
        _next_index = 0;
    }

    // change control byte to all type of data can be read
    if (id == _fake_conveyor.id && _next_control_byte != CAN_DATA_CONVEYOR_STATE)
    {
        control_byte = CAN_DATA_CONVEYOR_STATE;
        return CAN_OK;
    }
    else if (id != _fake_conveyor.id && _next_control_byte == CAN_DATA_CONVEYOR_STATE)
    {
        control_byte = CAN_DATA_POSITION;
    }
    else
    {
        control_byte = _next_control_byte;
        switch (control_byte) {
        case CAN_DATA_POSITION:
            // set next control byte
            if (_next_index == MAX_IDX)
                _next_control_byte = CAN_DATA_DIAGNOSTICS;
            break;
        case CAN_DATA_DIAGNOSTICS:
            if (_next_index == MAX_IDX)
                _next_control_byte = CAN_DATA_FIRMWARE_VERSION;
            break;
        case CAN_DATA_FIRMWARE_VERSION:
            if (_next_index == MAX_IDX)
                _next_control_byte = CAN_DATA_CALIBRATION_RESULT;
            break;
        case CAN_DATA_CALIBRATION_RESULT:
            if (_next_index == MAX_IDX)
                _next_control_byte = CAN_DATA_POSITION;
            break;
        default:
            break;
        }
    }
    return CAN_OK;
}

/**
 * @brief MockStepperDriver::canReadData
 */
bool MockStepperDriver::canReadData() const
{
    return true;
}

/**
 * @brief MockStepperDriver::str
 * @return
 */
std::string MockStepperDriver::str() const
{
  return "Stepper Driver (" + AbstractCanDriver::str() + ")";
}

/**
 * @brief MockStepperDriver::sendConveyorOnCommand
 * @param id
 * @param conveyor_on
 * @param conveyor_speed
 * @param direction
 * @return
 */
uint8_t MockStepperDriver::sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction)
{
    ROS_DEBUG("MockStepperDriver::scanMotorId - Send conveyor id %d enabled (%d) at speed %d on direction %d",
              id, static_cast<int>(conveyor_on), conveyor_speed, direction);

    _fake_conveyor.speed = conveyor_speed;
    _fake_conveyor.direction = direction;
    if (_fake_conveyor.speed == 0)
        _fake_conveyor.state = false;
    else
        _fake_conveyor.state = conveyor_on;

    if (!conveyor_on)
    {
        _fake_conveyor.speed = 0;
    }

    return CAN_OK;
}

/**
 * @brief MockStepperDriver::sendPositionCommand
 * @param id
 * @param cmd
 * @return
 */
uint8_t MockStepperDriver::sendPositionCommand(uint8_t id, int cmd)
{
    if (_map_fake_registers.count(id))
    {
        _map_fake_registers.at(id).position = cmd;
    }
    return CAN_OK;
}

/**
 * @brief MockStepperDriver::sendRelativeMoveCommand
 * @param id
 * @param steps
 * @param delay
 * @return
 */
uint8_t MockStepperDriver::sendRelativeMoveCommand(uint8_t id, int steps, int delay)
{
    if (_map_fake_registers.count(id))
    {
        return CAN_OK;
    }
    return CAN_FAIL;
}

/**
 * @brief MockStepperDriver::sendTorqueOnCommand
 * @param id
 * @param torque_on
 * @return
 */
uint8_t MockStepperDriver::sendTorqueOnCommand(uint8_t id, int torque_on)
{
    return CAN_OK;
}

/**
 * @brief MockStepperDriver::sendPositionOffsetCommand
 * @param id
 * @param cmd
 * @param absolute_steps_at_offset_position
 * @return
 */
uint8_t MockStepperDriver::sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position)
{
    if (_map_fake_registers.count(id))
    {
        _map_fake_registers.at(id).position -= cmd;
    }
    return CAN_OK;
}

/**
 * @brief MockStepperDriver::sendSynchronizePositionCommand
 * @param id
 * @param begin_traj
 * @return
 */
uint8_t MockStepperDriver::sendSynchronizePositionCommand(uint8_t id, bool begin_traj)
{
    if (_map_fake_registers.count(id))
        return CAN_OK;
    return CAN_FAIL;
}

/**
 * @brief MockStepperDriver::sendMicroStepsCommand
 * @param id
 * @param micro_steps
 * @return
 */
uint8_t MockStepperDriver::sendMicroStepsCommand(uint8_t id, int micro_steps)
{
    return CAN_OK;
}

/**
 * @brief MockStepperDriver::sendMaxEffortCommand
 * @param id
 * @param effort
 * @return
 */
uint8_t MockStepperDriver::sendMaxEffortCommand(uint8_t id, int effort)
{
    return CAN_OK;
}

/**
 * @brief MockStepperDriver::sendCalibrationCommand
 * @param id
 * @param offset
 * @param delay
 * @param direction
 * @param timeout
 * @return
 */
uint8_t MockStepperDriver::sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout)
{
    direction = (direction > 0) ? 1 : 0;

    if (_map_fake_registers.count(id))
    {
        if (_calibration_status.count(id))
        {
            std::get<0>(_calibration_status.at(id)) = common::model::EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS;
            std::get<1>(_calibration_status.at(id)) = offset;
        }
        else {
            _calibration_status.insert(std::pair<uint8_t, std::tuple<common::model::EStepperCalibrationStatus, int32_t>>(
                                        id, std::make_tuple(common::model::EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS, static_cast<int32_t>(offset))));
        }
        _fake_time = 10;
    }
    return CAN_OK;
}

/**
 * @brief MockStepperDriver::sendUpdateConveyorId
 * @param old_id
 * @param new_id
 * @return
 */
uint8_t MockStepperDriver::sendUpdateConveyorId(uint8_t old_id, uint8_t new_id)
{
    ROS_DEBUG("MockStepperDriver::sendUpdateConveyorId - Send update conveyor id from %d to %d", old_id, new_id);
    if (_fake_conveyor.id == old_id)
    {
        _fake_conveyor.id = new_id;
        _id_list.erase(std::remove(_id_list.begin(), _id_list.end(), old_id), _id_list.end());
        _id_list.push_back(new_id);
        return CAN_OK;
    }
    else
        return CAN_FAIL;
}

// ***************
//  Private
// ***************

/**
 * @brief MockStepperDriver::interpretePositionStatus
 * The mission of readData in fake driver is no longer getting datat. It will generate only event.
 * So interpretePositionStatus or interprete* send final value corresponding to the event
 * @param data
 * @return
 */
int32_t MockStepperDriver::interpretePositionStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    if (_map_fake_registers.count(_current_id))
        return _map_fake_registers.at(_current_id).position;
    return 0;
}

/**
 * @brief MockStepperDriver::interpreteTemperatureStatus
 * @param data
 * @return
 */
uint32_t MockStepperDriver::interpreteTemperatureStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    if (_map_fake_registers.count(_current_id))
        return _map_fake_registers.at(_current_id).temperature;
    return 0;
}

/**
 * @brief MockStepperDriver::interpreteFirmwareVersion
 * @param data
 * @return
 */
std::string MockStepperDriver::interpreteFirmwareVersion(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data)
{
    if (_map_fake_registers.count(_current_id))
        return _map_fake_registers.at(_current_id).firmware;
    return "";
}

/**
 * @brief MockStepperDriver::interpreteCalibrationData
 * @param data
 * @return
 */
std::tuple<common::model::EStepperCalibrationStatus, int32_t>
MockStepperDriver::interpreteCalibrationData(const std::array<uint8_t, MockStepperDriver::MAX_MESSAGE_LENGTH> &data)
{
    if (_map_fake_registers.count(_current_id) && _calibration_status.count(_current_id))
    {
        // need to try send calibration status more than 1 time to make sure the "ok" calibration status is got
        if (_fake_time <= 0 && _fake_time >= -3)
        {
            _fake_time--;
            std::get<0>(_calibration_status.at(_current_id))  = common::model::EStepperCalibrationStatus::CALIBRATION_OK;
            std::tuple<common::model::EStepperCalibrationStatus, int32_t> current_calib_status = _calibration_status.at(_current_id);
            std::get<0>(_calibration_status.at(_current_id))  = common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
            return current_calib_status;
        }
        else
        {
            _fake_time--;
            return _calibration_status.at(_current_id);
        }
     }

    return std::make_tuple(common::model::EStepperCalibrationStatus::CALIBRATION_BAD_PARAM, 0);
}

/**
 * @brief MockStepperDriver::interpreteConveyorData
 * @param data
 * @return
 */
std::tuple<bool, uint8_t, uint16_t>
MockStepperDriver::interpreteConveyorData(const std::array<uint8_t, MockStepperDriver::MAX_MESSAGE_LENGTH> &data)
{
    if (_fake_conveyor.id == _current_id)
    {
        int direction = _fake_conveyor.direction ? 1 : -1;
        return std::make_tuple(_fake_conveyor.state, _fake_conveyor.speed, static_cast<uint16_t>(direction));
    }

    return std::make_tuple(false, 0, 1);
}

}  // namespace can_driver
