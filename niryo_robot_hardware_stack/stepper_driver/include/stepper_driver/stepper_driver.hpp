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

#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <map>
#include <ros/ros.h>
#include "mcp_can_rpi/mcp_can_rpi.h"
#include "stepper_driver/stepper_motor_state.hpp"
#include "stepper_driver/stepper_motor_cmd.hpp"
#include "stepper_driver/StepperMotorCommand.h"
#include "stepper_driver/StepperCmd.h"
#include "stepper_driver/conveyor_state.hpp"


namespace StepperDriver
{


    constexpr int CAN_CMD_POSITION    = 0x03;
    constexpr int CAN_CMD_TORQUE      = 0x04;
    constexpr int CAN_CMD_MODE        = 0x07;
    constexpr int CAN_CMD_MICRO_STEPS = 0x13;
    constexpr int CAN_CMD_OFFSET      = 0x14;
    constexpr int CAN_CMD_CALIBRATE   = 0x15;
    constexpr int CAN_CMD_SYNCHRONIZE = 0x16;
    constexpr int CAN_CMD_MAX_EFFORT  = 0x17;
    constexpr int CAN_CMD_MOVE_REL    = 0x18;
    constexpr int CAN_CMD_RESET       = 0x19; // not yet implemented

    constexpr int CAN_STEPPERS_CALIBRATION_MODE_AUTO   = 1;
    constexpr int CAN_STEPPERS_CALIBRATION_MODE_MANUAL = 2;

    constexpr int CAN_STEPPERS_WRITE_OFFSET_FAIL = -3;

    constexpr int CAN_DATA_POSITION             = 0x03;
    constexpr int CAN_DATA_DIAGNOSTICS          = 0x08;
    constexpr int CAN_DATA_CALIBRATION_RESULT   = 0x09;
    constexpr int CAN_DATA_FIRMWARE_VERSION     = 0x10;
    constexpr int CAN_DATA_CONVEYOR_STATE       = 0x07;

    constexpr int STEPPER_CONTROL_MODE_RELAX    = 0;
    constexpr int STEPPER_CONTROL_MODE_STANDARD = 1;
    constexpr int STEPPER_CONTROL_MODE_PID_POS  = 2;
    constexpr int STEPPER_CONTROL_MODE_TORQUE   = 3;

    constexpr int STEPPER_CONVEYOR_OFF          = 20;
    constexpr int STEPPER_CONVEYOR_ON           = 21;
    constexpr int CAN_UPDATE_CONVEYOR_ID        = 23;

    constexpr int MESSAGE_POSITION_LENGTH       = 4;
    constexpr int MESSAGE_DIAGNOSTICS_LENGTH    = 4;
    constexpr int MESSAGE_FIRMWARE_LENGTH       = 4;

    constexpr int CAN_MODEL_NUMBER              = 10000;

    constexpr int CAN_SCAN_OK = 0;
    constexpr int CAN_SCAN_TIMEOUT   =  -10003;
    constexpr double TIME_TO_WAIT_IF_BUSY = 0.0005;

    constexpr float STEPPER_MOTOR_TIMEOUT_VALUE = 1.0f; // s

    enum class e_CanStepperCalibrationStatus {
        CAN_STEPPERS_CALIBRATION_UNINITIALIZED = 0,
        CAN_STEPPERS_CALIBRATION_OK = 1,
        CAN_STEPPERS_CALIBRATION_TIMEOUT = 2,
        CAN_STEPPERS_CALIBRATION_BAD_PARAM = 3,
        CAN_STEPPERS_CALIBRATION_FAIL = 4,
        CAN_STEPPERS_CALIBRATION_WAITING_USER_INPUT = 5,
        CAN_STEPPERS_CALIBRATION_IN_PROGRESS = 6,
    };

    struct CalibrationStepperData
    {
        unsigned long rxId;
        uint8_t len;
        std::array<uint8_t, 8> rxBuf;
    };

    struct CalibrationStepperCmdStatus
    {
        StepperMotorCmd cmd;
        ros::Time cmd_time;
    };

    class StepperDriver
    {
        public:

            StepperDriver();
            ~StepperDriver();

            void scanAndCheck();

            void addConveyor(uint8_t conveyor_id);
            void removeConveyor(uint8_t conveyor_id);

            //commands

            bool scanMotorId(int motor_to_find);

            uint8_t sendTorqueOnCommand(int id, int torque_on);
            uint8_t sendRelativeMoveCommand(int id, int steps, int delay);
            uint8_t sendUpdateConveyorId(uint8_t old_id, uint8_t new_id);

            void executeJointTrajectoryCmd(std::vector<int32_t> &cmd);

            int readCommand(StepperMotorCmd cmd);
            void readMotorsState();

            // tests
            bool isConnectionOk() const;

            //setters
            void setCalibrationInProgress(bool in_progress);
            void clearCalibrationTab();

            //getters
            int getStepperPose(int32_t motor_id) const;
            const std::vector<int32_t>& getJointTrajectoryState() const;
            const std::vector<StepperMotorState>& getMotorsState() const;
            const std::vector<ConveyorState>& getConveyorsState() const;
            void getBusState(bool& connection_status, std::vector<uint8_t>& motor_list, std::string& error) const;

            e_CanStepperCalibrationStatus getCalibrationResult(uint8_t id, int32_t &result) const;
            std::string getErrorMessage() const;

    private:
            int init();
            bool setupInterruptGpio();
            bool setupSpi();

            void updateMotorList();
            void addMotor(uint8_t motor_id);
            void removeMotor(uint8_t motor_id);

            bool canReadData();
            uint8_t readMsgBuf(unsigned long *id, uint8_t *len, std::array<uint8_t, 8> &buf);

            uint8_t sendPositionCommand(int id, int cmd);
            uint8_t sendPositionOffsetCommand(int id, int cmd, int absolute_steps_at_offset_position);
            uint8_t sendCalibrationCommand(int i, int offset, int delay, int direction, int timeout);
            uint8_t sendSynchronizePositionCommand(int id, bool begin_traj);
            uint8_t sendMicroStepsCommand(int id, int micro_steps);
            uint8_t sendMaxEffortCommand(int id, int effort);

            uint8_t sendConveyorOnCommand(int id, bool conveyor_on, int conveyor_speed, int8_t direction);

            uint8_t sendCanMsgBuf(unsigned long id, uint8_t ext, uint8_t len, uint8_t *buf);

            void readCalibrationStates();
            bool checkMessageLength(const uint8_t &message_length, int message_type);
            bool checkMotorsId(int motor_id);
            void fillMotorPosition(int motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillMotorDiagnostics(int motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillMotorFirmware(int motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data);
            void fillConveyorState(int motor_id, const std::array<uint8_t, 8> &data);
            void interpreteCalibrationCommand();

            void _verifyMotorTimeoutLoop();
            void _refreshMotorTimeout();

        private:
            ros::NodeHandle _nh;
            std::vector<int> _arm_id_list;
            std::vector<int> _motor_id_list;

            std::vector<ConveyorState> _conveyor_list;
            std::vector<StepperMotorState> _motor_list;

            std::vector<uint8_t> _all_motor_connected;
            std::vector<uint8_t> _calibration_motor_list;
            std::map<uint8_t, int> _motor_calibration_map;
            std::map<uint8_t, CalibrationStepperCmdStatus> _motor_calibration_map_cmd;

            std::shared_ptr<MCP_CAN_RPI::MCP_CAN> mcp_can;
            std::thread _calibration_thread;
            std::thread _stepper_timeout_thread;
            std::vector<CalibrationStepperData> _calibration_readed_datas;

            bool _is_can_connection_ok;
            bool _calibration_in_progress;

            std::string _debug_error_message;

            std::vector<int32_t> _stepper_states;
            e_CanStepperCalibrationStatus _calibration_result;
    };
}

#endif
