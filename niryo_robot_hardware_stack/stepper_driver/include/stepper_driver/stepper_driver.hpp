/*
    StepperDriver.hpp
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

#ifndef STEPPER_DRIVER_HPP
#define STEPPER_DRIVER_HPP

//std
#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <map>

//ros
#include <ros/ros.h>

//niryo
#include "model/idriver.hpp"
#include "model/stepper_motor_state.hpp"
#include "model/stepper_motor_cmd.hpp"
#include "model/conveyor_state.hpp"
#include "model/stepper_calibration_status_enum.hpp"
#include "model/synchronize_stepper_motor_cmd.hpp"

#include "stepper_driver/StepperMotorCommand.h"
#include "stepper_driver/StepperCmd.h"
#include "stepper_driver/calibration_stepper_data.hpp"

#include "mcp_can_rpi/mcp_can_rpi.h"


namespace StepperDriver
{


    /**
     * @brief The StepperDriver class
     */
    class StepperDriver : public common::model::IDriver
    {
        public:

            StepperDriver();
            virtual ~StepperDriver() override;

            void scanAndCheck();

            void addConveyor(uint8_t conveyor_id);
            void removeConveyor(uint8_t conveyor_id);

            //commands

            bool scanMotorId(int motor_to_find);

            int readSynchronizeCommand(const common::model::SynchronizeStepperMotorCmd& cmd);
            int readSingleCommand(common::model::StepperMotorCmd cmd);

            void readMotorsState();

            //setters
            void startCalibration();
            void stopCalibration();

            //getters
            int32_t getStepperPose(uint8_t motor_id) const;

            common::model::StepperMotorState getMotorState(uint8_t motor_id) const;
            std::vector<common::model::StepperMotorState> getMotorsStates() const;

            common::model::ConveyorState getConveyorState(uint8_t motor_id) const;
            std::vector<common::model::ConveyorState> getConveyorsStates() const;

            int32_t getCalibrationResult(uint8_t id) const;
            common::model::EStepperCalibrationStatus getCalibrationStatus() const;

            void addMotor(uint8_t id);

            uint8_t sendTorqueOnCommand(uint8_t id, int torque_on);
            uint8_t sendRelativeMoveCommand(uint8_t id, int steps, int delay);
            uint8_t sendUpdateConveyorId(uint8_t old_id, uint8_t new_id);
            bool isCalibrationInProgress() const;

            // IDriver interface
            void removeMotor(uint8_t id) override;
            bool isConnectionOk() const override;
            size_t getNbMotors() const override;
            void getBusState(bool& connection_status, std::vector<uint8_t>& motor_list, std::string& error) const override;
            std::string getErrorMessage() const override;

    private:
            bool init() override;
            void initParameters();
            bool hasMotors() override;

            int setupCAN();

            bool canReadData() const;
            uint8_t readMsgBuf(unsigned long *id, uint8_t *len, std::array<uint8_t, 8> &buf);

            uint8_t sendPositionCommand(uint8_t id, int cmd);
            uint8_t sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position);
            uint8_t sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout);
            uint8_t sendSynchronizePositionCommand(uint8_t id, bool begin_traj);
            uint8_t sendMicroStepsCommand(uint8_t id, int micro_steps);
            uint8_t sendMaxEffortCommand(uint8_t id, int effort);

            uint8_t sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction);

            uint8_t sendCanMsgBuf(unsigned long id, uint8_t ext, uint8_t len, uint8_t *buf);

            void readCalibrationStates();
            bool checkMessageLength(const uint8_t &message_length, int message_type);
            bool checkMotorsId(uint8_t motor_id);

            void fillPositionStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillTemperatureStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillFirmwareVersion(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);

            void fillConveyorState(uint8_t motor_id, const std::array<uint8_t, 8> &data);

            void _verifyMotorTimeoutLoop();
            void _refreshMotorTimeout();


        private:
            ros::NodeHandle _nh;

            std::map<uint8_t, common::model::ConveyorState> _conveyor_map;
            std::map<uint8_t, common::model::StepperMotorState> _state_map;

            std::vector<uint8_t> _all_motor_connected;

            std::map<uint8_t, CalibrationStepperData> _calibration_map;

            std::unique_ptr<MCP_CAN_RPI::MCP_CAN> mcp_can;

            std::thread _calibration_thread;
            std::thread _stepper_timeout_thread;

            bool _is_can_connection_ok;
            std::string _debug_error_message;

            int _calibration_timeout;

            common::model::EStepperCalibrationStatus _calibration_result;

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

            static constexpr int STEPPER_CONTROL_MODE_RELAX             = 0;
            static constexpr int STEPPER_CONTROL_MODE_STANDARD          = 1;
            static constexpr int STEPPER_CONTROL_MODE_PID_POS           = 2;
            static constexpr int STEPPER_CONTROL_MODE_TORQUE            = 3;

            static constexpr int STEPPER_CONVEYOR_OFF                   = 20;
            static constexpr int STEPPER_CONVEYOR_ON                    = 21;
            static constexpr int CAN_UPDATE_CONVEYOR_ID                 = 23;

            static constexpr int MESSAGE_POSITION_LENGTH                = 4;
            static constexpr int MESSAGE_DIAGNOSTICS_LENGTH             = 4;
            static constexpr int MESSAGE_FIRMWARE_LENGTH                = 4;

            static constexpr int CAN_MODEL_NUMBER                       = 10000;

            static constexpr int CAN_SCAN_OK                            = 0;
            static constexpr int CAN_SCAN_TIMEOUT                       =  -10003;

            static constexpr double STEPPER_MOTOR_TIMEOUT_VALUE         = 1.0;
    };

    inline
    size_t StepperDriver::getNbMotors() const
    {
        return _state_map.size();
    }

    inline
    bool StepperDriver::canReadData() const
    {
        return mcp_can->canReadData();
    }

    inline
    bool StepperDriver::isCalibrationInProgress() const {
        return common::model::EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS == _calibration_result;
    }

    inline
    bool StepperDriver::hasMotors()
    {
        return _state_map.size() > 0;
    }

} // namespace StepperDriver

#endif
