/*
    stepper_driver.cpp
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

#include "stepper_driver/stepper_driver.hpp"

namespace StepperDriver
{
    StepperDriver::StepperDriver() : _calibration_result(e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_UNINITIALIZED), _calibration_in_progress(false), _stepper_timeout_thread(boost::bind(&StepperDriver::_verifyMotorTimeoutLoop, this))
    {
        _nh.getParam("/niryo_robot_hardware_interface/debug", _debug_mode);
        if (_debug_mode)
        {
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            {
                ros::console::notifyLoggerLevelsChanged();
            }
        }

        int spi_channel, spi_baudrate, gpio_can_interrupt;

        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_bus/spi_channel", spi_channel);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_bus/spi_baudrate", spi_baudrate);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_bus/gpio_can_interrupt", gpio_can_interrupt);

        if (_nh.hasParam("/niryo_robot_hardware_interface/stepper_driver/motors_params/stepper_motor_id_list"))
        {
            _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/motors_params/stepper_motor_id_list", _motor_id_list);
        }
        else
        {
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/stepper_motor_id_list", _motor_id_list);
        }

        _conveyor_list.clear();
        _arm_id_list.clear();
        _arm_id_list = _motor_id_list;
        ROS_DEBUG("Stepper Driver - Can bus parameters: spi_channel : %d", spi_channel);
        ROS_DEBUG("Stepper Driver - Can bus parameters: spi_baudrate : %d", spi_baudrate);
        ROS_DEBUG("Stepper Driver - Can bus parameters: spi_baudrate : %d", gpio_can_interrupt);

        _is_can_connection_ok = false;
        _debug_error_message = "";

        mcp_can.reset(new MCP_CAN(spi_channel, spi_baudrate, gpio_can_interrupt));
        _calibration_motor_list.clear();
        setupInterruptGpio();
        setupSpi();
        init();
        updateMotorList();

        _stepper_states.clear();
        _stepper_states.push_back(0);
        _stepper_states.push_back(0);
        _stepper_states.push_back(0);
    }

    StepperDriver::~StepperDriver()
    {
        if (_stepper_timeout_thread.joinable())
        {
            _stepper_timeout_thread.join();
        }
    }

    bool StepperDriver::setupInterruptGpio()
    {
        if (!mcp_can->setupInterruptGpio())
        {
            ROS_WARN("Stepper Driver - Failed to start gpio");
            _debug_error_message = "Failed to start gpio";
            return CAN_GPIO_FAILINIT;
        }
        return CAN_OK;
    }

    bool StepperDriver::setupSpi()
    {
        if (!mcp_can->setupSpi())
        {
            ROS_WARN("Stepper Driver - Failed to start spi");
            _debug_error_message = "Failed to start spi";
            return CAN_SPI_FAILINIT;
        }
        return CAN_OK;
    }

    INT8U StepperDriver::init()
    {
        // no mask or filter used, receive all messages from CAN bus
        // messages with ids != motor_id will be sent to another ROS interface
        // so we can use many CAN devices with this only driver
        int result = mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
        ROS_DEBUG("Stepper Driver - Result begin can : %d", result);

        if (result != CAN_OK)
        {
            ROS_ERROR("Stepper Driver - Failed to init MCP2515 (CAN bus)");
            _debug_error_message = "Failed to init MCP2515 (CAN bus)";
            return result;
        }

        // set mode to normal
        mcp_can->setMode(MCP_NORMAL);
        _is_can_connection_ok = false;
        ros::Duration(0.05).sleep();
        return result;
    }

    bool StepperDriver::isConnectionOk() const
    {
        return _is_can_connection_ok;
    }

    void StepperDriver::addMotor(uint8_t motor_id)
    {
        ROS_DEBUG("Stepper Driver - Add motor id: %d", motor_id);
        _motor_id_list.push_back(motor_id);
        updateMotorList();
    }

    void StepperDriver::removeMotor(uint8_t motor_id)
    {
        ROS_DEBUG("Stepper Driver - Remove motor id: %d", motor_id);
        for (int i = 0; i < _motor_id_list.size(); i++)
        {
            if (_motor_id_list.at(i) == motor_id)
            {
                _motor_id_list.erase(_motor_id_list.begin() + i);
                break;
            }
        }
        updateMotorList();
    }

    void StepperDriver::updateMotorList()
    {
        _motor_list.clear();
        for (int i = 0; i < _motor_id_list.size(); i++)
        {
            StepperMotorState m(_motor_id_list.at(i));
            ROS_DEBUG("Stepper Driver - Push back id %d", _motor_id_list.at(i));
            _motor_list.push_back(m);
        }

        std::string motor_string_list = "[";
        for (int i = 0; i < _motor_id_list.size(); i++)
        {
            if (i != 0)
                motor_string_list += ", ";
            motor_string_list += std::to_string(_motor_id_list.at(i));
        }
        motor_string_list += "]";
        ROS_INFO("Stepper Driver - Motor list: %s ", motor_string_list.c_str());
    }

    void StepperDriver::addConveyor(uint8_t conveyor_id)
    {
        ConveyorState c(conveyor_id);
        c.setDirection(-1);
        c.setSpeed(0);
        c.setState(false);
        _conveyor_list.push_back(c);
    }
    void StepperDriver::removeConveyor(uint8_t conveyor_id)
    {
        for (int i = 0; i < _conveyor_list.size(); i++)
        {
            if (_conveyor_list.at(i).getId() == conveyor_id)
            {
                _conveyor_list.erase(_conveyor_list.begin() + i);
            }
        }
    }

    std::vector<StepperMotorState> &StepperDriver::getMotorsState()
    {
        return _motor_list;
    }

    void StepperDriver::getBusState(bool &connection_status, std::vector<uint8_t> &motor_list, std::string &error)
    {
        error = _debug_error_message;
        motor_list = _all_motor_connected;
        connection_status = isConnectionOk();
    }

    void StepperDriver::executeJointTrajectoryCmd(std::vector<int32_t> &cmd)
    {
        int result;
        for (int i = 0; i < _arm_id_list.size(); i++)
        {
            result = sendPositionCommand(_arm_id_list.at(i), cmd.at(i));
            if (result != CAN_OK)
            {
                ROS_WARN("Stepper Driver - send positions to motor id %d failed", _arm_id_list.at(i));
            }
        }
    }

    int32_t StepperDriver::getStepperPose(int32_t motor_id)
    {
        for (std::vector<StepperMotorState>::iterator motor_it = _motor_list.begin(); motor_it != _motor_list.end(); motor_it++)
        {
            if (motor_it->getId() == motor_id)
            {
                return motor_it->getPositionState();
            }
        }
        ROS_WARN("Stepper Driver - Get positionsfrom motor id %d failed because doesn't exist", motor_id);
        return -1;
    }

    std::vector<int32_t> &StepperDriver::getJointTrajectoryState()
    {
        return _stepper_states;
    }

    int StepperDriver::readCommand(StepperMotorCmd cmd)
    {
        int result = CAN_INVALID_CMD;
        ROS_DEBUG("Stepper Driver - Received stepper cmd with type %d", int(cmd.getType()));

        std::string ids_string = "";
        for (int i = 0; i < cmd.getMotorsId().size(); i++)
            ids_string += std::to_string(cmd.getMotorsId().at(i)) + " ";
        ROS_DEBUG("Stepper Driver - Received stepper cmd with ids %s", ids_string.c_str());

        std::string params_string = "";
        for (int i = 0; i < cmd.getParams().size(); i++)
            params_string += std::to_string(cmd.getParams().at(i)) + " ";
        ROS_DEBUG("Stepper Driver - Received stepper cmd with params %s", params_string.c_str());

        if (cmd.getType() == StepperCommandType::CMD_TYPE_POSITION)
        {
            if (cmd.getMotorsId().size() <= cmd.getParams().size())
            {
                for (int i = 0; i < cmd.getMotorsId().size(); i++)
                {
                    result = sendPositionCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                }
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_TORQUE)
        {
            if (cmd.getMotorsId().size() <= cmd.getParams().size())
            {
                for (int i = 0; i < cmd.getMotorsId().size(); i++)
                {
                    result = sendTorqueOnCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                }
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_SYNCHRONIZE)
        {
            if (cmd.getMotorsId().size() <= cmd.getParams().size())
            {
                for (int i = 0; i < cmd.getMotorsId().size(); i++)
                {
                    result = sendSynchronizePositionCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                }
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_RELATIVE_MOVE)
        {

            if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 2)
            {
                result = sendRelativeMoveCommand(cmd.getMotorsId().at(0), cmd.getParams().at(0), cmd.getParams().at(1));
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_MAX_EFFORT)
        {
            if (cmd.getMotorsId().size() <= cmd.getParams().size())
            {
                for (int i = 0; i < cmd.getMotorsId().size(); i++)
                {
                    result = sendMaxEffortCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                }
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_MICRO_STEPS)
        {
            if (cmd.getMotorsId().size() <= cmd.getParams().size())
            {
                for (int i = 0; i < cmd.getMotorsId().size(); i++)
                {
                    result = sendMicroStepsCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                }
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_CALIBRATION)
        {
            if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 4)
            {
                _calibration_motor_list.push_back(cmd.getMotorsId().at(0));
                _motor_calibration_map[cmd.getMotorsId().at(0)] = 0;
                _motor_calibration_map_cmd[cmd.getMotorsId().at(0)] = {cmd, ros::Time::now()};
                result = sendCalibrationCommand(cmd.getMotorsId().at(0), cmd.getParams().at(0), cmd.getParams().at(1), cmd.getParams().at(2), cmd.getParams().at(3));
                if (result == CAN_OK && _calibration_result != e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_IN_PROGRESS)
                {
                    // Join the previous calibration thread (otherwise we cannot reassign the thread)
                    if (_calibration_thread.joinable())
                        _calibration_thread.join();
                    _calibration_thread = std::thread(boost::bind(&StepperDriver::readCalibrationStates, this));
                }
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_POSITION_OFFSET)
        {
            if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 2)
            {
                result = sendPositionOffsetCommand(cmd.getMotorsId().at(0), cmd.getParams().at(0), cmd.getParams().at(1));
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_CONVEYOR)
        {
            if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 3)
            {
                result = sendConveyoOnCommand(cmd.getMotorsId().at(0), cmd.getParams().at(0), cmd.getParams().at(1), cmd.getParams().at(2));
            }
        }
        else if (cmd.getType() == StepperCommandType::CMD_TYPE_UPDATE_CONVEYOR)
        {
            if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 1)
            {
                result = sendUpdateConveyorId(cmd.getMotorsId().at(0), cmd.getParams().at(0));
                if (result == CAN_OK)
                {
                    removeConveyor(cmd.getMotorsId().at(0));
                    addConveyor(cmd.getParams().at(0));
                }
            }
        }
        ROS_DEBUG("Stepper Driver - Received stepper cmd finished");
        return result;
    }

    bool StepperDriver::canReadData()
    {
        return mcp_can->canReadData();
    }

    INT8U StepperDriver::readMsgBuf(INT32U *id, INT8U *len, std::array<INT8U, 8> &buf)
    {
        int hw_fail_counter_read = 0;
        INT8U status;

        while (hw_fail_counter_read < 10)
        {
            status = mcp_can->readMsgBuf(id, len, buf.data());
            if (status == CAN_OK)
            {
                break;
            }
            else
            {
                _is_can_connection_ok = false;
                ROS_WARN_THROTTLE(2, "Stepper Driver - Reading Stepper message on CAN Bus failed");
            }
        }
        return status;
    }

    void StepperDriver::scanAndCheck()
    {
        double time_begin_scan = ros::Time::now().toSec();
        double min_time_to_wait = 0.25;
        double timeout = 0.5;

        _all_motor_connected.clear();

        std::vector<uint8_t> motors_unfound(_motor_id_list.begin(), _motor_id_list.end());
        while ((motors_unfound.size() != 0) || (ros::Time::now().toSec() - time_begin_scan < min_time_to_wait))
        {
            ros::Duration(0.001).sleep(); // check at 1000 Hz
            if (canReadData())
            {
                INT32U rxId;
                INT8U len;
                std::array<INT8U, 8> rxBuf;
                readMsgBuf(&rxId, &len, rxBuf);
                int motor_id = rxId & 0x0F;

                auto it = std::find(motors_unfound.begin(), motors_unfound.end(), motor_id);
                if (it != motors_unfound.end())
                {
                    _all_motor_connected.push_back(*it);
                    motors_unfound.erase(it);
                }
            }

            if (ros::Time::now().toSec() - time_begin_scan > timeout)
            {
                ROS_ERROR_THROTTLE(2, "Stepper Driver - CAN scan Timeout");
                _debug_error_message = "CAN bus scan failed : motors ";
                for (int i = 0; i < motors_unfound.size(); i++)
                {
                    _debug_error_message += std::to_string(motors_unfound.at(i));
                    _debug_error_message += ", ";
                }
                _debug_error_message += "are not connected";
                _is_can_connection_ok = false;
                ROS_ERROR_THROTTLE(2, "Stepper Driver - %s", _debug_error_message.c_str());
            }
        }
        
        this->_refreshMotorTimeout();
        
        _is_can_connection_ok = true;
        _debug_error_message = "";
    }

    void StepperDriver::readMotorsState()
    {
        if (canReadData())
        {
            INT32U rxId;
            INT8U len;
            std::array<INT8U, 8> rxBuf;
            readMsgBuf(&rxId, &len, rxBuf);
            int motor_id = rxId & 0x0F;
            bool motor_known;

            motor_known = checkMotorsId(motor_id);
            if (!motor_known)
            {
                return;
            }

            if (len < 1)
            {
                ROS_ERROR_THROTTLE(1, "Stepper Driver - Received can frame with empty data");
                return;
            }

            int control_byte = rxBuf[0];

            if (control_byte == CAN_DATA_POSITION)
            {
                fillMotorPosition(motor_id, len, rxBuf);
            }

            else if (control_byte == CAN_DATA_DIAGNOSTICS)
            {
                fillMotorDiagnostics(motor_id, len, rxBuf);
            }

            else if (control_byte == CAN_DATA_FIRMWARE_VERSION)
            {
                fillMotorFirmware(motor_id, len, rxBuf);
            }

            else if (control_byte == CAN_DATA_CONVEYOR_STATE)
            {
                fillConveyorState(motor_id, rxBuf);
            }
        }
    }

    void StepperDriver::fillMotorPosition(int motor_id, const INT8U &len, const std::array<INT8U, 8> &data)
    {
        if (!checkMessageLength(len, MESSAGE_POSITION_LENGTH))
        {
            ROS_ERROR("Stepper Driver - Position can frame should contain 4 data bytes");
            return;
        }

        int32_t pos = (data[1] << 16) + (data[2] << 8) + data[3];
        if (pos & (1 << 15))
        {
            pos = -1 * ((~pos + 1) & 0xFFFF);
        }
        if (motor_id == 1)
        {
            _stepper_states[0] = pos;
        }
        else if (motor_id == 2)
        {
            _stepper_states[1] = pos;
        }
        else if (motor_id == 3)
        {
            _stepper_states[2] = pos;
        }
        for (int i = 0; i < _motor_list.size(); i++)
        {
            if (motor_id == _motor_list.at(i).getId())
            {
                _motor_list.at(i).setPositionState(pos);
                break;
            }
        }
        return;
    }

    void StepperDriver::fillMotorDiagnostics(int motor_id, const INT8U &len, const std::array<INT8U, 8> &data)
    {
        if (!checkMessageLength(len, MESSAGE_DIAGNOSTICS_LENGTH))
        {
            ROS_ERROR("Stepper Driver - Diagnostic can frame should contain 4 data bytes");
        }

        int driver_temp_raw = (data[2] << 8) + data[3];
        double a = -0.00316;
        double b = -12.924;
        double c = 2367.7;
        double v_temp = driver_temp_raw * 3.3 / 1024.0 * 1000.0;
        int driver_temp = int((-b - std::sqrt(b * b - 4 * a * (c - v_temp))) / (2 * a) + 30);

        // fill data
        for (int i = 0; i < _motor_list.size(); i++)
        {
            if (motor_id == _motor_list.at(i).getId())
            {
                _motor_list.at(i).setTemperatureState(driver_temp);
                break;
            }
        }
    }

    void StepperDriver::fillMotorFirmware(int motor_id, const INT8U &len, const std::array<INT8U, 8> &data)
    {
        if (!checkMessageLength(len, MESSAGE_FIRMWARE_LENGTH))
        {
            ROS_ERROR("Stepper Driver - Firmware version frame should contain 4 data bytes");
        }

        int v_major = data[1];
        int v_minor = data[2];
        int v_patch = data[3];
        std::string version = "";
        version += std::to_string(v_major);
        version += ".";
        version += std::to_string(v_minor);
        version += ".";
        version += std::to_string(v_patch);

        // fill data
        for (int i = 0; i < _motor_list.size(); i++)
        {
            if (motor_id == _motor_list.at(i).getId())
            {
                _motor_list.at(i).setFirmwareVersion(version);
                break;
            }
        }
    }

    void StepperDriver::fillConveyorState(int motor_id, const std::array<INT8U, 8> &data)
    {
        bool state = data[1];
        int16_t speed = data[2];
        int8_t direction = data[3];

        for (int i = 0; i < _conveyor_list.size(); i++)
        {
            if (_conveyor_list.at(i).getId() == motor_id)
            {
                _conveyor_list.at(i).setDirection(direction);
                _conveyor_list.at(i).setSpeed(speed);
                _conveyor_list.at(i).setState(state);
                break;
            }
        }
    }

    void StepperDriver::_verifyMotorTimeoutLoop()
    {
        while (this->_nh.ok())
        {
            // Only check when motors seems connected and not in calibration (not state received)
            if (this->_is_can_connection_ok && !this->_calibration_in_progress)
            {
                for (const StepperMotorState &motor : this->_motor_list)
                {
                    const double &last_time_read = motor.getLastTimeRead();
                    if (last_time_read > 0)
                    {
                        double sec_elapsed = ros::Time::now().toSec() - last_time_read;
                        if (sec_elapsed > STEPPER_MOTOR_TIMEOUT_VALUE)
                        {
                            // If it's the first motor that seems disconnected
                            if (this->_is_can_connection_ok)
                            {
                                this->_debug_error_message = "Disconnected stepper motor(s): " + std::to_string(motor.getId());
                            }
                            // If an another motor seems already disconnected
                            else
                            {
                                this->_debug_error_message += ", " + std::to_string(motor.getId());
                            }
                            this->_is_can_connection_ok = false;
                        }
                    }
                }
            }
            ros::Duration(0.1).sleep();
        }
    }

    void StepperDriver::_refreshMotorTimeout()
    {
        // Refresh time read value for timeout thread
        for (StepperMotorState &motor_stepper_state : _motor_list)
        {
            motor_stepper_state.setLastTimeRead(-1);
        }
    }

    std::vector<ConveyorState> &StepperDriver::getConveyorsState()
    {
        return _conveyor_list;
    }

    bool StepperDriver::checkMotorsId(int motor_id)
    {
        bool motor_found = false;
        for (int i = 0; i < _motor_list.size(); i++)
        {
            if (motor_id == _motor_list.at(i).getId())
            {
                _motor_list.at(i).setLastTimeRead(ros::Time::now().toSec());
                motor_found = true;
                break;
            }
        }
        for (int i = 0; i < _conveyor_list.size(); i++)
        {
            if (motor_id == _conveyor_list.at(i).getId())
            {
                motor_found = true;
                break;
            }
        }
        if (!motor_found)
        {
            // ROS_WARN("Received can frame with unknow id : %d", motor_id);
            _debug_error_message = "Unknow connected motor : ";
            _debug_error_message += std::to_string(motor_id);
        }
        return motor_found;
    }

    bool StepperDriver::checkMessageLength(const INT8U &message_length, int message_type)
    {
        if (message_length != message_type)
        {
            return false;
        }
        return true;
    }

    INT8U StepperDriver::sendPositionCommand(int id, int cmd)
    {
        uint8_t data[4] = {CAN_CMD_POSITION, (uint8_t)((cmd >> 16) & 0xFF),
                           (uint8_t)((cmd >> 8) & 0xFF), (uint8_t)(cmd & 0XFF)};
        return sendCanMsgBuf(id, 0, 4, data);
    }

    INT8U StepperDriver::sendRelativeMoveCommand(int id, int steps, int delay)
    {
        uint8_t data[7] = {CAN_CMD_MOVE_REL,
                           (uint8_t)((steps >> 16) & 0xFF), (uint8_t)((steps >> 8) & 0xFF), (uint8_t)(steps & 0XFF),
                           (uint8_t)((delay >> 16) & 0xFF), (uint8_t)((delay >> 8) & 0xFF), (uint8_t)(delay & 0XFF)};
        return sendCanMsgBuf(id, 0, 7, data);
    }

    INT8U StepperDriver::sendTorqueOnCommand(int id, int torque_on)
    {
        uint8_t data[2] = {0};
        data[0] = CAN_CMD_MODE;
        data[1] = (torque_on) ? STEPPER_CONTROL_MODE_STANDARD : STEPPER_CONTROL_MODE_RELAX;
        return sendCanMsgBuf(id, 0, 2, data);
    }

    INT8U StepperDriver::sendPositionOffsetCommand(int id, int cmd, int absolute_steps_at_offset_position)
    {
        uint8_t data[6] = {CAN_CMD_OFFSET, (uint8_t)((cmd >> 16) & 0xFF),
                           (uint8_t)((cmd >> 8) & 0xFF), (uint8_t)(cmd & 0XFF),
                           (uint8_t)((absolute_steps_at_offset_position >> 8) & 0xFF), (uint8_t)(absolute_steps_at_offset_position & 0xFF)};
        return sendCanMsgBuf(id, 0, 6, data);
    }

    INT8U StepperDriver::sendSynchronizePositionCommand(int id, bool begin_traj)
    {
        uint8_t data[2] = {CAN_CMD_SYNCHRONIZE, (uint8_t)begin_traj};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    INT8U StepperDriver::sendMicroStepsCommand(int id, int micro_steps)
    {
        uint8_t data[2] = {CAN_CMD_MICRO_STEPS, (uint8_t)micro_steps};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    INT8U StepperDriver::sendMaxEffortCommand(int id, int effort)
    {
        uint8_t data[2] = {CAN_CMD_MAX_EFFORT, (uint8_t)effort};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    std::string StepperDriver::getErrorMessage()
    {
        return _debug_error_message;
    }

    INT8U StepperDriver::sendCalibrationCommand(int id, int offset, int delay, int direction, int timeout)
    {
        direction = (direction > 0 ) ? 1 : 0;

        uint8_t data[8] = {CAN_CMD_CALIBRATE, (uint8_t)((offset >> 16) & 0xFF),
                           (uint8_t)((offset >> 8) & 0xFF), (uint8_t)(offset & 0XFF),
                           (uint8_t)((delay >> 8) & 0xFF), (uint8_t)(delay & 0xFF),
                           (uint8_t)direction, (uint8_t)timeout};
        return sendCanMsgBuf(id, 0, 8, data);
    }

    e_CanStepperCalibrationStatus StepperDriver::getCalibrationResult(uint8_t id, boost::shared_ptr<int32_t> &result)
    {
        (*result) = _motor_calibration_map[id];
        return _calibration_result;
    }

    void StepperDriver::clearCalibrationTab()
    {
        _motor_calibration_map.clear();
    }

    void StepperDriver::setCalibrationInProgress(bool in_progress)
    {
        this->_calibration_in_progress = in_progress;
        this->_refreshMotorTimeout();
    }

    void StepperDriver::readCalibrationStates()
    {
        _calibration_result = e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_IN_PROGRESS;
        double time_thread_begin = ros::Time::now().toSec();

        int calibration_timeout;
        _nh.getParam("/niryo_robot_hardware_interface/calibration_timeout", calibration_timeout);
        std::thread reading_data_thread(boost::bind(&StepperDriver::interpreteCalibrationCommand, this));
        while (_calibration_motor_list.size() != 0)
        {
            if (canReadData())
            {
                INT32U rxId;
                INT8U len;
                std::array<INT8U, 8> rxBuf;

                readMsgBuf(&rxId, &len, rxBuf);
                CalibrationStepperData calib_data = {rxId, len, rxBuf};
                _calibration_readed_datas.push_back(calib_data);
            }
            else if (ros::Time::now().toSec() - time_thread_begin > calibration_timeout)
            {
                ROS_ERROR("Stepper Driver - Calibration timeout after: %lf s", ros::Time::now().toSec() - time_thread_begin);
                for (int i = 0; i < _calibration_motor_list.size(); i++)
                {
                    ROS_ERROR("Stepper Driver - Motor %d timeout: may be disconnected", _calibration_motor_list.at(i));
                }
                _calibration_result = e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_TIMEOUT;
                _calibration_motor_list.clear();
                reading_data_thread.join();
                return;
            }
        }
        _calibration_motor_list.clear();
        reading_data_thread.join();
        _calibration_result = e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_OK;
        ROS_DEBUG("Stepper Driver - Calibration thread ended with success");
    }

    void StepperDriver::interpreteCalibrationCommand()
    {
        while(_calibration_motor_list.size() > 0)
        {
            if (_calibration_readed_datas.size() > 0)
            {
                
//                std::string data_str = "";
//                for(int byte_nb=0; byte_nb<_calibration_readed_datas[0].len; byte_nb++ )
//                {
//                    data_str+= " " + std::to_string((int)_calibration_readed_datas[0].rxBuf[byte_nb]);
//                }
//                ROS_WARN("motor calib: %d, %d, %s", (int)(_calibration_readed_datas[0].rxId & 0x0F), _calibration_readed_datas[0].len, data_str.c_str());

                if (_calibration_readed_datas[0].len == 4)
                {
                    INT32U rxId = _calibration_readed_datas[0].rxId ;
                    std::array<INT8U, 8> rxBuf = _calibration_readed_datas[0].rxBuf;

                    int motor_id = rxId & 0x0F;
                    for (int i = 0; i < _calibration_motor_list.size(); i++)
                    {
                        if (_calibration_motor_list.at(i) == motor_id)
                        {
                            // 3. Check control byte
                            int control_byte = rxBuf[0];
                            if (control_byte == CAN_DATA_CALIBRATION_RESULT)
                            { // only check this frame
                                int result = rxBuf[1];
                                e_CanStepperCalibrationStatus can_enum_result = static_cast<e_CanStepperCalibrationStatus>(result);
                                if (can_enum_result == e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_TIMEOUT)
                                {
                                    ROS_ERROR("Stepper Driver - Motor %d had calibration timeout", motor_id);
                                    _calibration_result = can_enum_result;
                                    return;
                                }
                                else if (can_enum_result == e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_BAD_PARAM)
                                {
                                    ROS_ERROR("Stepper Driver - Bad params given to motor %d", motor_id);
                                    _calibration_result = can_enum_result;
                                    return;
                                }
                                else if (can_enum_result == e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_OK)
                                {
                                    ROS_INFO("Stepper Driver - Motor %d calibration OK", motor_id);
                                    int steps_at_offset_pos = (rxBuf[2] << 8) + rxBuf[3];
                                    ROS_INFO("Stepper Driver - Motor %d - Absolute steps at offset position : %d", motor_id, steps_at_offset_pos);
                                    _motor_calibration_map[motor_id] = (int32_t)steps_at_offset_pos;
                                    _calibration_motor_list.erase(std::remove(_calibration_motor_list.begin(), _calibration_motor_list.end(), motor_id), _calibration_motor_list.end());
                                    // keep torque ON for axis 1
                                    if (motor_id == 1)
                                    {
                                        sendTorqueOnCommand(1, true);
                                    }
                                }
                            }
                            else if ((ros::Time::now() - _motor_calibration_map_cmd[motor_id].cmd_time).toSec() > 0.5)
                            {
                                StepperMotorCmd cmd = _motor_calibration_map_cmd[motor_id].cmd;
                                _motor_calibration_map_cmd[motor_id].cmd_time = ros::Time::now();
                                sendCalibrationCommand(motor_id, cmd.getParams().at(0), cmd.getParams().at(1), cmd.getParams().at(2), cmd.getParams().at(3));
                            }
                        }
                    }
                }
                _calibration_readed_datas.erase(_calibration_readed_datas.begin());
            }
        }
    }

    bool StepperDriver::scanMotorId(int motor_to_find)
    {
        double time_begin_scan = ros::Time::now().toSec();
        double min_time_to_wait = 0.25;
        double timeout = 0.5;
        bool motor_found = false;

        while (!motor_found || (ros::Time::now().toSec() - time_begin_scan < min_time_to_wait))
        {
            ros::Duration(0.001).sleep(); // check at 1000 Hz
            if (canReadData())
            {
                INT32U rxId;
                INT8U len;
                std::array<INT8U, 8> rxBuf;
                readMsgBuf(&rxId, &len, rxBuf);
                int motor_id = rxId & 0x0F;
                if (motor_id == motor_to_find)
                {
                    motor_found = true;
                }
            }
            if (ros::Time::now().toSec() - time_begin_scan > timeout)
            {
                ROS_ERROR("Stepper Driver - Motor with id %d not found", motor_to_find);
                return motor_found;
            }
        }
        return motor_found;
    }

    INT8U StepperDriver::sendConveyoOnCommand(int id, bool conveyor_on, int conveyor_speed, int8_t direction)
    {
        ROS_DEBUG("Stepper Driver - Send conveyor id %d enabled (%d) at speed %d on direction %d",
                  id, static_cast<int>(conveyor_on), conveyor_speed, direction);
        uint8_t data[3] = {0};
        data[0] = CAN_CMD_MODE;
        if (conveyor_on)
        {
            data[1] = STEPPER_CONVEYOR_ON;
        }
        else
        {
            data[1] = STEPPER_CONVEYOR_OFF;
        }
        data[2] = conveyor_speed;
        data[3] = direction;

        return sendCanMsgBuf(id, 0, 4, data);
    }

    INT8U StepperDriver::sendUpdateConveyorId(uint8_t old_id, uint8_t new_id)
    {
        ROS_DEBUG("Stepper Driver - Send update conveyor id from %d to %d", old_id, new_id);
        uint8_t data[3] = {0};
        data[0] = CAN_CMD_MODE;
        data[1] = CAN_UPDATE_CONVEYOR_ID;
        data[2] = new_id;
        return sendCanMsgBuf(old_id, 0, 3, data);
    }

    INT8U StepperDriver::sendCanMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf)
    {
        INT8U status;
        int hw_fail_counter_send = 0;
        while (hw_fail_counter_send < 10)
        {
            status = mcp_can->sendMsgBuf(id, ext, len, buf);
            if (status == CAN_OK)
            {
                break;
            }
            else
            {
                _is_can_connection_ok = false;
                ROS_WARN_THROTTLE(0.5, "Stepper Driver - Sending Stepper message on CAN Bus failed");
            }
        }
        return status;
    }

} // namespace StepperDriver
