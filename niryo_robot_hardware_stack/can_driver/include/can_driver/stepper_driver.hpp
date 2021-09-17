/*
    stepper_driver.hpp
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

#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <memory>
#include <set>
#include <vector>

#include "abstract_can_driver.hpp"

#include "mcp_can_rpi/mcp_can_rpi.h"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/abstract_single_motor_cmd.hpp"
#include "common/model/conveyor_state.hpp"

namespace can_driver
{

class StepperDriver : public AbstractCanDriver
{

public:
  static constexpr int CAN_DATA_POSITION                      = 0x03;
  static constexpr int CAN_DATA_DIAGNOSTICS                   = 0x08;
  static constexpr int CAN_DATA_FIRMWARE_VERSION              = 0x10;
  static constexpr int CAN_DATA_CONVEYOR_STATE                = 0x07;
  static constexpr int CAN_DATA_CALIBRATION_RESULT            = 0x09;

public:
    StepperDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can);
    virtual ~StepperDriver() override;

    int writeSingleCmd(const std::shared_ptr<common::model::AbstractCanSingleMotorCmd >& cmd) override;

public:
    std::string str() const override;

    // here are only common CAN commands
    // write
    uint8_t sendUpdateConveyorId(uint8_t old_id, uint8_t new_id) override;
    uint8_t sendTorqueOnCommand(uint8_t id, int torque_on) override;
    uint8_t sendRelativeMoveCommand(uint8_t id, int steps, int delay) override;

    uint8_t sendPositionCommand(uint8_t id, int cmd) override;
    uint8_t sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position) override;
    uint8_t sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout) override;
    uint8_t sendSynchronizePositionCommand(uint8_t id, bool begin_traj) override;
    uint8_t sendMicroStepsCommand(uint8_t id, int micro_steps) override;
    uint8_t sendMaxEffortCommand(uint8_t id, int effort) override;
    uint8_t sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction) override;

    static int32_t interpretePositionStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data);
    static uint32_t interpreteTemperatureStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data);
    static std::string interpreteFirmwareVersion(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data);
    static std::tuple<common::model::EStepperCalibrationStatus, int32_t> interpreteCalibrationData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data);
    static std::tuple<bool, uint8_t, uint16_t> interpreteConveyorData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data);

private:
    static constexpr int CAN_CMD_POSITION                       = 0x03;
    static constexpr int CAN_CMD_TORQUE                         = 0x04;
    static constexpr int CAN_CMD_MODE                           = 0x07;
    static constexpr int CAN_CMD_MICRO_STEPS                    = 0x13;
    static constexpr int CAN_CMD_OFFSET                         = 0x14;
    static constexpr int CAN_CMD_CALIBRATE                      = 0x15;
    static constexpr int CAN_CMD_SYNCHRONIZE                    = 0x16;
    static constexpr int CAN_CMD_MAX_EFFORT                     = 0x17;
    static constexpr int CAN_CMD_MOVE_REL                       = 0x18;
    static constexpr int CAN_CMD_RESET                          = 0x19; // not yet implemented


    static constexpr int CAN_STEPPERS_CALIBRATION_MODE_AUTO     = 1;
    static constexpr int CAN_STEPPERS_CALIBRATION_MODE_MANUAL   = 2;

    static constexpr int CAN_STEPPERS_WRITE_OFFSET_FAIL         = -3;

    static constexpr int STEPPER_CONTROL_MODE_RELAX             = 0;
    static constexpr int STEPPER_CONTROL_MODE_STANDARD          = 1;
    static constexpr int STEPPER_CONTROL_MODE_PID_POS           = 2;
    static constexpr int STEPPER_CONTROL_MODE_TORQUE            = 3;

    static constexpr int STEPPER_CONVEYOR_OFF                   = 20;
    static constexpr int STEPPER_CONVEYOR_ON                    = 21;
    static constexpr int CAN_UPDATE_CONVEYOR_ID                 = 23;

    static constexpr int CAN_MODEL_NUMBER                       = 10000;

};

} // namespace can_driver

#endif // STEPPER_DRIVER_H
