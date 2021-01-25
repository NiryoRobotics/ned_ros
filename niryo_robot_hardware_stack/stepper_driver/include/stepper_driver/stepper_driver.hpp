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

#include <boost/shared_ptr.hpp>
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

#define CAN_CMD_POSITION     0x03
#define CAN_CMD_TORQUE       0x04
#define CAN_CMD_MODE         0x07
#define CAN_CMD_MICRO_STEPS  0x13
#define CAN_CMD_OFFSET       0x14
#define CAN_CMD_CALIBRATE    0x15
#define CAN_CMD_SYNCHRONIZE  0x16
#define CAN_CMD_MAX_EFFORT   0x17
#define CAN_CMD_MOVE_REL     0x18
#define CAN_CMD_RESET        0x19 // not yet implemented

enum class e_CanStepperCalibrationStatus {
    CAN_STEPPERS_CALIBRATION_UNINITIALIZED = 0,
    CAN_STEPPERS_CALIBRATION_OK = 1,
    CAN_STEPPERS_CALIBRATION_TIMEOUT = 2,
    CAN_STEPPERS_CALIBRATION_BAD_PARAM = 3,
    CAN_STEPPERS_CALIBRATION_FAIL = 4,
    CAN_STEPPERS_CALIBRATION_WAITING_USER_INPUT = 5,
    CAN_STEPPERS_CALIBRATION_IN_PROGRESS = 6,
};

#define CAN_STEPPERS_CALIBRATION_MODE_AUTO   1
#define CAN_STEPPERS_CALIBRATION_MODE_MANUAL 2

#define CAN_STEPPERS_WRITE_OFFSET_FAIL -3

#define CAN_DATA_POSITION    0x03
#define CAN_DATA_DIAGNOSTICS 0x08
#define CAN_DATA_CALIBRATION_RESULT 0x09
#define CAN_DATA_FIRMWARE_VERSION 0x10
#define CAN_DATA_CONVEYOR_STATE 0x07

#define STEPPER_CONTROL_MODE_RELAX    0
#define STEPPER_CONTROL_MODE_STANDARD 1
#define STEPPER_CONTROL_MODE_PID_POS  2 
#define STEPPER_CONTROL_MODE_TORQUE   3

#define STEPPER_CONVEYOR_OFF 20
#define STEPPER_CONVEYOR_ON 21
#define CAN_UPDATE_CONVEYOR_ID 23
#define STEPPER_CONTROL_MODE_PID_POS  2
#define STEPPER_CONTROL_MODE_TORQUE   3

#define MESSAGE_POSITION_LENGTH 4
#define MESSAGE_DIAGNOSTICS_LENGTH 4
#define MESSAGE_FIRMWARE_LENGTH 4

#define CAN_MODEL_NUMBER 10000

#define CAN_SCAN_OK 0
#define CAN_SCAN_TIMEOUT     -10003
#define TIME_TO_WAIT_IF_BUSY 0.0005

#define STEPPER_MOTOR_TIMEOUT_VALUE 1.0f // s

namespace StepperDriver
{
    struct CalibrationStepperData
    {
        INT32U rxId;
        INT8U len;
        std::array<INT8U, 8> rxBuf;
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

            bool setupInterruptGpio();
            bool setupSpi();
            INT8U init();

            bool isConnectionOk() const;

            void updateMotorList();
            void addMotor(uint8_t motor_id);
            void removeMotor(uint8_t motor_id);

            void addConveyor(uint8_t conveyor_id);
            void removeConveyor(uint8_t conveyor_id);
            std::vector<ConveyorState>& getConveyorsState();

            std::vector<StepperMotorState>& getMotorsState();
            void getBusState(bool& connection_status, std::vector<uint8_t>& motor_list, std::string& error);

            void readMotorsState();
            void clearCalibrationTab();
            e_CanStepperCalibrationStatus getCalibrationResult(uint8_t id, boost::shared_ptr<int32_t> &result);

            void executeJointTrajectoryCmd(std::vector<int32_t> &cmd);
            std::vector<int32_t> &getJointTrajectoryState();
            int32_t getStepperPose(int32_t motor_id);
            int readCommand(StepperMotorCmd cmd);

            bool canReadData();
            INT8U readMsgBuf(INT32U *id, INT8U *len, std::array<INT8U, 8> &buf);
        
            void scanAndCheck();
            
            INT8U sendPositionCommand(int id, int cmd);
            INT8U sendRelativeMoveCommand(int id, int steps, int delay);
            INT8U sendTorqueOnCommand(int id, int torque_on);
            INT8U sendPositionOffsetCommand(int id, int cmd, int absolute_steps_at_offset_position);
            INT8U sendCalibrationCommand(int i, int offset, int delay, int direction, int timeout);
            INT8U sendSynchronizePositionCommand(int id, bool begin_traj);
            INT8U sendMicroStepsCommand(int id, int micro_steps);
            INT8U sendMaxEffortCommand(int id, int effort);

            INT8U sendConveyoOnCommand(int id, bool conveyor_on, int conveyor_speed, int8_t direction);
            INT8U sendUpdateConveyorId(uint8_t old_id, uint8_t new_id);

            INT8U sendCanMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);

            std::string getErrorMessage();
            bool scanMotorId(int motor_to_find);

            std::vector<uint8_t> getConnectedMotors();
            void setCalibrationInProgress(bool in_progress);

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

            boost::shared_ptr<MCP_CAN> mcp_can;
            std::thread _calibration_thread;
            std::thread _stepper_timeout_thread;
            std::vector<CalibrationStepperData> _calibration_readed_datas;

            void readCalibrationStates();
            bool checkMessageLength(const INT8U &message_length, int message_type);
            bool checkMotorsId(int motor_id);
            void fillMotorPosition(int motor_id, const INT8U &len, const std::array<INT8U, 8> &data);
            void fillMotorDiagnostics(int motor_id, const INT8U &len, const std::array<INT8U, 8> &data);
            void fillMotorFirmware(int motor_id, const INT8U &len, const std::array<INT8U, 8> &data);
            void fillConveyorState(int motor_id, const std::array<INT8U, 8> &data);
            void interpreteCalibrationCommand();

            void _verifyMotorTimeoutLoop();
            void _refreshMotorTimeout();

            bool _is_can_connection_ok;
            bool _calibration_in_progress;

            bool _debug_mode;

            std::string _debug_error_message;

            std::vector<int32_t> _stepper_states;
            e_CanStepperCalibrationStatus _calibration_result;
    };
}

#endif
