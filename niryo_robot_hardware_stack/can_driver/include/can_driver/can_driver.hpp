/*
    can_driver.hpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CAN_DRIVER_HPP
#define CAN_DRIVER_HPP

//std
#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <map>
#include <mutex>

//ros
#include <ros/ros.h>

//niryo
#include "model/idriver.hpp"
#include "model/stepper_motor_state.hpp"
#include "model/stepper_motor_cmd.hpp"
#include "model/conveyor_state.hpp"
#include "model/stepper_calibration_status_enum.hpp"
#include "model/synchronize_stepper_motor_cmd.hpp"

#include "can_driver/StepperMotorCommand.h"
#include "can_driver/StepperCmd.h"

#include "mcp_can_rpi/mcp_can_rpi.h"


namespace CanDriver
{

    /**
     * @brief The CanDriver class
     */
    class CanDriver : public common::model::IDriver
    {
        public:

            CanDriver();
            virtual ~CanDriver() override;

            //commands
            void addMotor(uint8_t id, bool isConveyor = false);

            int readSingleCommand(common::model::StepperMotorCmd cmd);
            void executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, int32_t> > cmd_vec);

            void startCalibration();
            void resetCalibration();

            uint8_t sendTorqueOnCommand(uint8_t id, int torque_on);
            uint8_t sendRelativeMoveCommand(uint8_t id, int steps, int delay);
            uint8_t sendUpdateConveyorId(uint8_t old_id, uint8_t new_id);

            void readStatus();

            //getters
            int32_t getPosition(uint8_t motor_id) const;

            common::model::StepperMotorState getMotorState(uint8_t motor_id) const;
            std::vector<std::shared_ptr<common::model::StepperMotorState> > getMotorsStates() const;

            int32_t getCalibrationResult(uint8_t id) const;
            common::model::EStepperCalibrationStatus getCalibrationStatus() const;

            bool isCalibrationInProgress() const;

            // IDriver Interface
            void removeMotor(uint8_t id) override;
            bool isConnectionOk() const override;

            int scanAndCheck() override;
            bool ping(uint8_t motor_to_find) override;

            size_t getNbMotors() const override;
            void getBusState(bool& connection_status, std::vector<uint8_t>& motor_list, std::string& error) const override;
            std::string getErrorMessage() const override;


    private:
            bool init() override;
            void initParameters();
            int setupCAN();

            bool hasMotors() override;
            bool canReadData() const;

            uint8_t readMsgBuf(unsigned long *id, uint8_t *len, std::array<uint8_t, 8> &buf);
            uint8_t sendCanMsgBuf(unsigned long id, uint8_t ext, uint8_t len, uint8_t *buf);

            uint8_t sendPositionCommand(uint8_t id, int cmd);
            uint8_t sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position);
            uint8_t sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout);
            uint8_t sendSynchronizePositionCommand(uint8_t id, bool begin_traj);
            uint8_t sendMicroStepsCommand(uint8_t id, int micro_steps);
            uint8_t sendMaxEffortCommand(uint8_t id, int effort);
            uint8_t sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction);

            void fillPositionStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillTemperatureStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillFirmwareVersion(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillConveyorState(uint8_t motor_id, const std::array<uint8_t, 8> &data);
            void fillCalibrationState(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);

            void _verifyMotorTimeoutLoop();
            void _refreshLastTimeRead();

            void updateCurrentCalibrationStatus();
            double getCurrentTimeout() const;

        private:
            ros::NodeHandle _nh;

            std::unique_ptr<MCP_CAN_RPI::MCP_CAN> mcp_can;

            std::mutex _stepper_timeout_mutex;
            std::thread _stepper_timeout_thread;

            // cc use a set ?? -> pb with publish
            std::vector<uint8_t> _all_motor_connected;

            std::map<uint8_t, std::shared_ptr<common::model::StepperMotorState> > _state_map;

            double _calibration_timeout;
            common::model::EStepperCalibrationStatus _calibration_status;

            // for hardware control
            bool _is_connection_ok;
            std::string _debug_error_message;

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

            static constexpr int CAN_DATA_POSITION                      = 0x03;
            static constexpr int CAN_DATA_DIAGNOSTICS                   = 0x08;
            static constexpr int CAN_DATA_FIRMWARE_VERSION              = 0x10;
            static constexpr int CAN_DATA_CONVEYOR_STATE                = 0x07;
            static constexpr int CAN_DATA_CALIBRATION_RESULT            = 0x09;

            static constexpr int STEPPER_CONTROL_MODE_RELAX             = 0;
            static constexpr int STEPPER_CONTROL_MODE_STANDARD          = 1;
            static constexpr int STEPPER_CONTROL_MODE_PID_POS           = 2;
            static constexpr int STEPPER_CONTROL_MODE_TORQUE            = 3;

            static constexpr int STEPPER_CONVEYOR_OFF                   = 20;
            static constexpr int STEPPER_CONVEYOR_ON                    = 21;
            static constexpr int CAN_UPDATE_CONVEYOR_ID                 = 23;

            static constexpr uint8_t MESSAGE_POSITION_LENGTH            = 4;
            static constexpr uint8_t MESSAGE_DIAGNOSTICS_LENGTH         = 4;
            static constexpr uint8_t MESSAGE_FIRMWARE_LENGTH            = 4;

            static constexpr int CAN_MODEL_NUMBER                       = 10000;

            static constexpr double STEPPER_MOTOR_TIMEOUT_VALUE         = 1.0;
    };

    inline
    size_t CanDriver::getNbMotors() const
    {
        return _state_map.size();
    }

    inline
    bool CanDriver::canReadData() const
    {
        return mcp_can->canReadData();
    }

    inline
    common::model::EStepperCalibrationStatus CanDriver::getCalibrationStatus() const
    {
        return _calibration_status;
    }

    inline
    bool CanDriver::isCalibrationInProgress() const {
        return common::model::EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS == _calibration_status;
    }

    inline
    bool CanDriver::hasMotors()
    {
        return _state_map.size() > 0;
    }

} // namespace CanDriver

#endif
