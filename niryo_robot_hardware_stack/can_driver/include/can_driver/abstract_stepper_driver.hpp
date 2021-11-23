/*
abstract_dxl_driver.hpp
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

#ifndef CAN_ABSTRACT_STEPPER_DRIVER_HPP
#define CAN_ABSTRACT_STEPPER_DRIVER_HPP

#include "abstract_can_driver.hpp"

//std
#include <memory>

// common
#include "common/model/single_motor_cmd.hpp"
#include "common/model/synchronize_motor_cmd.hpp"

namespace can_driver
{

class AbstractStepperDriver : public AbstractCanDriver
{
public:
    AbstractStepperDriver() = default;
    AbstractStepperDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can);

public:
    // AbstractCanDriver interface
    std::string str() const override;

    int writeSingleCmd(const std::unique_ptr<common::model::AbstractCanSingleMotorCmd> &cmd) override;

    int32_t interpretPositionStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    uint8_t interpretTemperatureStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    std::string interpretFirmwareVersion(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    std::pair<common::model::EStepperCalibrationStatus, int32_t> interpretHomingData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    std::tuple<bool, uint8_t, uint16_t> interpretConveyorData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;

public:

    // here are only common can stepper commands
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

public:
    static constexpr int CAN_DATA_POSITION                      = 0x03;
    static constexpr int CAN_DATA_DIAGNOSTICS                   = 0x08;
    static constexpr int CAN_DATA_FIRMWARE_VERSION              = 0x10;
    static constexpr int CAN_DATA_CONVEYOR_STATE                = 0x07;
    static constexpr int CAN_DATA_CALIBRATION_RESULT            = 0x09;
};

} // can_driver

#endif // CAN_ABSTRACT_STEPPER_DRIVER_HPP
