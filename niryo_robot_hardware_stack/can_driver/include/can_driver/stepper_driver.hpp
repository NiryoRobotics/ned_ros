/*
    can_driver/stepper_driver.hpp
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

#ifndef CAN_STEPPER_DRIVER_H
#define CAN_STEPPER_DRIVER_H

#include <memory>
#include <set>
#include <vector>
#include <ros/ros.h>
#include "abstract_stepper_driver.hpp"

#include "mcp_can_rpi/mcp_can_rpi.h"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/abstract_single_motor_cmd.hpp"
#include "common/model/conveyor_state.hpp"

#include "stepper_reg.hpp"

namespace can_driver
{

template<typename reg_type = StepperReg>
class StepperDriver : public AbstractStepperDriver
{

public:
    StepperDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can);

public:
    std::string str() const override;


    // AbstractStepperDriver interface
public:
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
};

/**
 * @brief StepperDriver::StepperDriver
 * @param mcp_can
 */
template<typename reg_type>
StepperDriver<reg_type>::StepperDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can) :
    AbstractStepperDriver(std::move(mcp_can))
{
}

/**
 * @brief StepperDriver::str
 * @return
 */
template<typename reg_type>
std::string StepperDriver<reg_type>::str() const
{
  return "Stepper Driver (" + can_driver::AbstractStepperDriver::str() + ")";
}

/**
 * @brief StepperDriver::sendConveyorOnCommand
 * @param id
 * @param conveyor_on
 * @param conveyor_speed
 * @param direction
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction)
{
    ROS_DEBUG("StepperDriver::scanMotorId - Send conveyor id %d enabled (%d) at speed %d on direction %d",
              id, static_cast<int>(conveyor_on), conveyor_speed, direction);
    uint8_t data[4] = {0};
    data[0] = reg_type::CAN_CMD_MODE;
    if (conveyor_on)
    {
        data[1] = reg_type::STEPPER_CONVEYOR_ON;
    }
    else
    {
        data[1] = reg_type::STEPPER_CONVEYOR_OFF;
    }
    data[2] = conveyor_speed;
    data[3] = direction;

    return write(id, 0, 4, data);
}

/**
 * @brief template<typename reg_type>
StepperDriver<reg_type>::sendPositionCommand
 * @param id
 * @param cmd
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendPositionCommand(uint8_t id, int cmd)
{
    uint8_t data[4] = {reg_type::CAN_CMD_POSITION, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                       static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF)};
    return write(id, 0, 4, data);
}

/**
 * @brief StepperDriver::sendRelativeMoveCommand
 * @param id
 * @param steps
 * @param delay
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendRelativeMoveCommand(uint8_t id, int steps, int delay)
{
    uint8_t data[7] = {reg_type::CAN_CMD_MOVE_REL,
                       static_cast<uint8_t>((steps >> 16) & 0xFF),
                       static_cast<uint8_t>((steps >> 8) & 0xFF),
                       static_cast<uint8_t>(steps & 0XFF),
                       static_cast<uint8_t>((delay >> 16) & 0xFF),
                       static_cast<uint8_t>((delay >> 8) & 0xFF),
                       static_cast<uint8_t>(delay & 0XFF)};
    return write(id, 0, 7, data);
}

/**
 * @brief StepperDriver::sendTorqueOnCommand
 * @param id
 * @param torque_on
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendTorqueOnCommand(uint8_t id, int torque_on)
{
    uint8_t data[2] = {0};
    data[0] = reg_type::CAN_CMD_MODE;
    data[1] = (torque_on) ? reg_type::STEPPER_CONTROL_MODE_STANDARD : reg_type::STEPPER_CONTROL_MODE_RELAX;
    return write(id, 0, 2, data);
}

/**
 * @brief StepperDriver::sendPositionOffsetCommand
 * @param id
 * @param cmd
 * @param absolute_steps_at_offset_position
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position)
{
    uint8_t data[6] = {reg_type::CAN_CMD_OFFSET, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                       static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF),
                       static_cast<uint8_t>((absolute_steps_at_offset_position >> 8) & 0xFF),
                       static_cast<uint8_t>(absolute_steps_at_offset_position & 0xFF)};
    return write(id, 0, 6, data);
}

/**
 * @brief StepperDriver::sendSynchronizePositionCommand
 * @param id
 * @param begin_traj
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendSynchronizePositionCommand(uint8_t id, bool begin_traj)
{
    uint8_t data[2] = {reg_type::CAN_CMD_SYNCHRONIZE, static_cast<uint8_t>(begin_traj)};
    return write(id, 0, 2, data);
}

/**
 * @brief StepperDriver::sendMicroStepsCommand
 * @param id
 * @param micro_steps
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendMicroStepsCommand(uint8_t id, int micro_steps)
{
    uint8_t data[2] = {reg_type::CAN_CMD_MICRO_STEPS, static_cast<uint8_t>(micro_steps)};
    return write(id, 0, 2, data);
}

/**
 * @brief StepperDriver::sendMaxEffortCommand
 * @param id
 * @param effort
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendMaxEffortCommand(uint8_t id, int effort)
{
    uint8_t data[2] = {reg_type::CAN_CMD_MAX_EFFORT, static_cast<uint8_t>(effort)};
    return write(id, 0, 2, data);
}

/**
 * @brief StepperDriver::sendCalibrationCommand
 * @param id
 * @param offset
 * @param delay
 * @param direction
 * @param timeout
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout)
{
    direction = (direction > 0) ? 1 : 0;

    uint8_t data[8] = {reg_type::CAN_CMD_CALIBRATE, static_cast<uint8_t>((offset >> 16) & 0xFF),
                       static_cast<uint8_t>((offset >> 8) & 0xFF), static_cast<uint8_t>(offset & 0XFF),
                       static_cast<uint8_t>((delay >> 8) & 0xFF), static_cast<uint8_t>(delay & 0xFF),
                       static_cast<uint8_t>(direction), static_cast<uint8_t>(timeout)};
    return write(id, 0, 8, data);
}

/**
 * @brief StepperDriver::sendUpdateConveyorId
 * @param old_id
 * @param new_id
 * @return
 */
template<typename reg_type>
uint8_t StepperDriver<reg_type>::sendUpdateConveyorId(uint8_t old_id, uint8_t new_id)
{
    ROS_DEBUG("StepperDriver::sendUpdateConveyorId - Send update conveyor id from %d to %d", old_id, new_id);
    uint8_t data[3] = {0};
    data[0] = reg_type::CAN_CMD_MODE;
    data[1] = reg_type::CAN_UPDATE_CONVEYOR_ID;
    data[2] = new_id;
    return write(old_id, 0, 3, data);
}

// ***************
//  Private
// ***************


} // namespace can_driver

#endif // CAN_STEPPER_DRIVER_HPP
