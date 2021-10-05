/*
    mock_stepper_driver.hpp
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

#ifndef MOCK_STEPPER_DRIVER_H
#define MOCK_STEPPER_DRIVER_H

#include <cstdint>
#include <map>
#include <memory>
#include <vector>
#include <thread>

#include "can_driver/abstract_can_driver.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/abstract_single_motor_cmd.hpp"
#include "common/model/conveyor_state.hpp"
#include "can_driver/fake_can_data.hpp"

namespace can_driver
{

class MockStepperDriver : public AbstractCanDriver
{
public:
    static constexpr int CAN_DATA_POSITION                      = 0x03;
    static constexpr int CAN_DATA_DIAGNOSTICS                   = 0x08;
    static constexpr int CAN_DATA_FIRMWARE_VERSION              = 0x10;
    static constexpr int CAN_DATA_CONVEYOR_STATE                = 0x07;
    static constexpr int CAN_DATA_CALIBRATION_RESULT            = 0x09;

public:
    MockStepperDriver(FakeCanData data);
    virtual ~MockStepperDriver() override;
    
    virtual int ping(uint8_t id) override;
    virtual int scan(const std::set<uint8_t>& motors_to_find, std::vector<uint8_t> &id_list) override;

    int writeSingleCmd(const std::shared_ptr<common::model::AbstractCanSingleMotorCmd>& cmd) override;
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

    // read
    virtual uint8_t readData(uint8_t& id, int& control_byte,
                     std::array<uint8_t, MAX_MESSAGE_LENGTH>& rxBuf,
                     std::string& error_message) override;
    virtual bool canReadData() const override;

    virtual int32_t interpretePositionStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    virtual uint32_t interpreteTemperatureStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    virtual std::string interpreteFirmwareVersion(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    virtual std::tuple<common::model::EStepperCalibrationStatus, int32_t> interpreteCalibrationData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;
    virtual std::tuple<bool, uint8_t, uint16_t> interpreteConveyorData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) override;

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

    std::map<uint8_t, FakeCanData::FakeRegister> _map_fake_registers;
    std::vector<uint8_t> _id_list;
    FakeCanData::FakeConveyor _fake_conveyor;
    
    // using for fake event can
    static constexpr uint8_t MAX_IDX = 2; // index for joints in _id_list
    static constexpr uint8_t MAX_ID_JOINT = 3;
    uint8_t _current_id;
    uint8_t _next_index = 0;
    uint8_t _next_control_byte = CAN_DATA_POSITION;
    std::map<uint8_t, std::tuple<common::model::EStepperCalibrationStatus, int32_t>> _calibration_status;
    // fake time for calibration
    int _fake_time = 0;
};  // class MockStepperDriver
}  // namespace can_driver
#endif  // MOCK_STEPPER_DRIVER_H

