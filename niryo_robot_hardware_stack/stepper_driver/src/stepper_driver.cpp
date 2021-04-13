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
#include "model/conveyor_state.hpp"
#include "model/stepper_command_type_enum.hpp"

#include <functional>

namespace StepperDriver
{
    StepperDriver::StepperDriver() :
        _stepper_timeout_thread(&StepperDriver::_verifyMotorTimeoutLoop, this),
        _calibration_in_progress(false),
        _calibration_result(e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_UNINITIALIZED)
    {

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
        ROS_DEBUG("StepperDriver::StepperDriver - Can bus parameters: spi_channel : %d", spi_channel);
        ROS_DEBUG("StepperDriver::StepperDriver - Can bus parameters: spi_baudrate : %d", spi_baudrate);
        ROS_DEBUG("StepperDriver::StepperDriver - Can bus parameters: spi_baudrate : %d", gpio_can_interrupt);

        _is_can_connection_ok = false;
        _debug_error_message = "";

        mcp_can = std::make_shared<MCP_CAN_RPI::MCP_CAN>(spi_channel, spi_baudrate, gpio_can_interrupt);

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
            ROS_WARN("StepperDriver::setupInterruptGpio - Failed to start gpio");
            _debug_error_message = "Failed to start gpio";
            return CAN_GPIO_FAILINIT;
        }
        return CAN_OK;
    }

    bool StepperDriver::setupSpi()
    {
        if (!mcp_can->setupSpi())
        {
            ROS_WARN("StepperDriver::setupSpi - Failed to start spi");
            _debug_error_message = "Failed to start spi";
            return CAN_SPI_FAILINIT;
        }
        return CAN_OK;
    }

    int StepperDriver::init()
    {
        // no mask or filter used, receive all messages from CAN bus
        // messages with ids != motor_id will be sent to another ROS interface
        // so we can use many CAN devices with this only driver
        int result = mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
        ROS_DEBUG("StepperDriver::init - Result begin can : %d", result);

        if (result != CAN_OK)
        {
            ROS_ERROR("StepperDriver::init - Failed to init MCP2515 (CAN bus)");
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
        ROS_DEBUG("StepperDriver::addMotor - Add motor id: %d", motor_id);
        _motor_id_list.push_back(motor_id);
        updateMotorList();
    }

    void StepperDriver::removeMotor(uint8_t motor_id)
    {
        ROS_DEBUG("StepperDriver::removeMotor - Remove motor id: %d", motor_id);
        for (size_t i = 0; i < _motor_id_list.size(); i++)
        {
            if (_motor_id_list.at(i) == motor_id)
            {
                _motor_id_list.erase(_motor_id_list.begin() + static_cast<int>(i));
                break;
            }
        }
        updateMotorList();
    }

    void StepperDriver::updateMotorList()
    {
        _motor_list.clear();
        for (size_t i = 0; i < _motor_id_list.size(); i++)
        {
            common::model::StepperMotorState m(_motor_id_list.at(i));
            ROS_DEBUG("StepperDriver::updateMotorList - Push back id %d", _motor_id_list.at(i));
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
        ROS_INFO("StepperDriver::updateMotorList - Motor list: %s ", motor_string_list.c_str());
    }

    void StepperDriver::addConveyor(uint8_t conveyor_id)
    {
        common::model::ConveyorState c(conveyor_id);
        _conveyor_list.push_back(c);
    }

    void StepperDriver::removeConveyor(uint8_t conveyor_id)
    {
        for (size_t i = 0; i < _conveyor_list.size(); i++)
        {
            if (_conveyor_list.at(i).getId() == conveyor_id)
            {
                _conveyor_list.erase(_conveyor_list.begin() + i);
            }
        }
    }

    const std::vector<common::model::StepperMotorState> &StepperDriver::getMotorsState() const
    {
        return _motor_list;
    }

    void StepperDriver::getBusState(bool &connection_status, std::vector<uint8_t> &motor_list, std::string &error) const
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
                ROS_WARN("StepperDriver::executeJointTrajectoryCmd - send positions to motor id %d failed", _arm_id_list.at(i));
            }
        }
    }

    int StepperDriver::getStepperPose(int32_t motor_id) const
    {
        for (auto const& motor_it : _motor_list)
        {
            if (motor_it.getId() == motor_id)
            {
                return motor_it.getPositionState();
            }
        }
        ROS_WARN("StepperDriver::getStepperPose - Get positionsfrom motor id %d failed because doesn't exist", motor_id);
        return -1;
    }

    const std::vector<int32_t> &StepperDriver::getJointTrajectoryState() const
    {
        return _stepper_states;
    }

    int StepperDriver::readCommand(common::model::StepperMotorCmd cmd)
    {
        int result = CAN_INVALID_CMD;
        ROS_DEBUG("StepperDriver::readCommand - Received stepper cmd with type %d", int(cmd.getType()));

        std::string ids_string = "";
        for (int i = 0; i < cmd.getMotorsId().size(); i++)
            ids_string += std::to_string(cmd.getMotorsId().at(i)) + " ";
        ROS_DEBUG("StepperDriver::readCommand - Received stepper cmd with ids %s", ids_string.c_str());

        std::string params_string = "";
        for (int i = 0; i < cmd.getParams().size(); i++)
            params_string += std::to_string(cmd.getParams().at(i)) + " ";
        ROS_DEBUG("StepperDriver::readCommand - Received stepper cmd with params %s", params_string.c_str());

        switch(cmd.getType())
        {
            case common::model::EStepperCommandType::CMD_TYPE_POSITION:
                if (cmd.getMotorsId().size() <= cmd.getParams().size()) {
                    for (int i = 0; i < cmd.getMotorsId().size(); i++) {
                        result = sendPositionCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                    }
                }
            break;
            case common::model::EStepperCommandType::CMD_TYPE_TORQUE:
                if (cmd.getMotorsId().size() <= cmd.getParams().size()) {
                    for (int i = 0; i < cmd.getMotorsId().size(); i++) {
                        result = sendTorqueOnCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                    }
                }
            break;
            case common::model::EStepperCommandType::CMD_TYPE_SYNCHRONIZE:
                if (cmd.getMotorsId().size() <= cmd.getParams().size())
                {
                    for (int i = 0; i < cmd.getMotorsId().size(); i++)
                    {
                        result = sendSynchronizePositionCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                    }
                }
            break;
            case common::model::EStepperCommandType::CMD_TYPE_RELATIVE_MOVE:
                if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 2)
                    result = sendRelativeMoveCommand(cmd.getMotorsId().at(0), cmd.getParams().at(0), cmd.getParams().at(1));
            break;
            case common::model::EStepperCommandType::CMD_TYPE_MAX_EFFORT:
                if (cmd.getMotorsId().size() <= cmd.getParams().size()) {
                    for (int i = 0; i < cmd.getMotorsId().size(); i++) {
                        result = sendMaxEffortCommand(cmd.getMotorsId().at(i), cmd.getParams().at(i));
                    }
                }
            break;
            case common::model::EStepperCommandType::CMD_TYPE_MICRO_STEPS:
                if (cmd.getMotorsId().size() <= cmd.getParams().size()) {
                    for (int i = 0; i < cmd.getMotorsId().size(); i++) {
                        result = sendMicroStepsCommand(cmd.getMotorsId().at(i),
                                                       cmd.getParams().at(i));
                    }
                }
            break;
            case common::model::EStepperCommandType::CMD_TYPE_CALIBRATION:
                if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 4)
                {
                    _calibration_motor_list.push_back(cmd.getMotorsId().at(0));
                    _motor_calibration_map[cmd.getMotorsId().at(0)] = 0;
                    _motor_calibration_map_cmd[cmd.getMotorsId().at(0)] = {cmd, ros::Time::now()};
                    result = sendCalibrationCommand(cmd.getMotorsId().at(0), cmd.getParams().at(0), cmd.getParams().at(1), cmd.getParams().at(2), cmd.getParams().at(3));
                    if (result == CAN_OK &&
                            _calibration_result != e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_IN_PROGRESS)
                    {
                        // Join the previous calibration thread (otherwise we cannot reassign the thread)
                        if (_calibration_thread.joinable())
                            _calibration_thread.join();
                        _calibration_thread = std::thread(&StepperDriver::readCalibrationStates, this);
                    }
                }
            break;
            case common::model::EStepperCommandType::CMD_TYPE_POSITION_OFFSET:
                if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 2)
                    result = sendPositionOffsetCommand(cmd.getMotorsId().at(0),
                                                       cmd.getParams().at(0),
                                                       cmd.getParams().at(1));
                break;
            case common::model::EStepperCommandType::CMD_TYPE_CONVEYOR:
                if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 3)
                    result = sendConveyorOnCommand(cmd.getMotorsId().at(0),
                                                  cmd.getParams().at(0),
                                                  cmd.getParams().at(1),
                                                  cmd.getParams().at(2));
                break;
            case common::model::EStepperCommandType::CMD_TYPE_UPDATE_CONVEYOR:
                if (cmd.getMotorsId().size() > 0 && cmd.getParams().size() >= 1)
                {
                    result = sendUpdateConveyorId(cmd.getMotorsId().at(0), cmd.getParams().at(0));
                    if (result == CAN_OK)
                    {
                        removeConveyor(cmd.getMotorsId().at(0));
                        addConveyor(cmd.getParams().at(0));
                    }
                }
            break;
            case common::model::EStepperCommandType::CMD_TYPE_UNKNOWN:
            default:
                break;
        }

        ROS_DEBUG("StepperDriver::readCommand - Received stepper cmd finished");
        return result;
    }

    bool StepperDriver::canReadData()
    {
        return mcp_can->canReadData();
    }

    uint8_t StepperDriver::readMsgBuf(unsigned long *id, uint8_t *len, std::array<uint8_t, 8> &buf)
    {
        int hw_fail_counter_read = 0;
        uint8_t status = 0;

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
                ROS_WARN_THROTTLE(2, "StepperDriver::readMsgBuf - Reading Stepper message on CAN Bus failed");
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
                unsigned long rxId;
                uint8_t len;
                std::array<uint8_t, 8> rxBuf;
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
                ROS_ERROR_THROTTLE(2, "StepperDriver::scanAndCheck - CAN scan Timeout");
                _debug_error_message = "CAN bus scan failed : motors ";
                for (int i = 0; i < motors_unfound.size(); i++)
                {
                    _debug_error_message += std::to_string(motors_unfound.at(i));
                    _debug_error_message += ", ";
                }
                _debug_error_message += "are not connected";
                _is_can_connection_ok = false;
                ROS_ERROR_THROTTLE(2, "StepperDriver::scanAndCheck - %s", _debug_error_message.c_str());
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
            unsigned long rxId;
            uint8_t len;
            std::array<uint8_t, 8> rxBuf;
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
                ROS_ERROR_THROTTLE(1, "StepperDriver::readMotorsState - Received can frame with empty data");
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

    void StepperDriver::fillMotorPosition(int motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
    {
        if (!checkMessageLength(len, MESSAGE_POSITION_LENGTH))
        {
            ROS_ERROR("StepperDriver::fillMotorPosition - Position can frame should contain 4 data bytes");
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

    void StepperDriver::fillMotorDiagnostics(int motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
    {
        if (!checkMessageLength(len, MESSAGE_DIAGNOSTICS_LENGTH))
        {
            ROS_ERROR("StepperDriver::fillMotorDiagnostics - Diagnostic can frame should contain 4 data bytes");
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

    void StepperDriver::fillMotorFirmware(int motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
    {
        if (!checkMessageLength(len, MESSAGE_FIRMWARE_LENGTH))
        {
            ROS_ERROR("StepperDriver::fillMotorFirmware - Firmware version frame should contain 4 data bytes");
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

    void StepperDriver::fillConveyorState(int motor_id, const std::array<uint8_t, 8> &data)
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
                for (const auto &motor : this->_motor_list)
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
        for (common::model::StepperMotorState &motor_stepper_state : _motor_list)
        {
            motor_stepper_state.setLastTimeRead(-1);
        }
    }

    const std::vector<common::model::ConveyorState> &StepperDriver::getConveyorsState() const
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

    bool StepperDriver::checkMessageLength(const uint8_t &message_length, int message_type)
    {
        if (message_length != message_type)
        {
            return false;
        }
        return true;
    }

    uint8_t StepperDriver::sendPositionCommand(int id, int cmd)
    {
        uint8_t data[4] = {CAN_CMD_POSITION, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                           static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF)};
        return sendCanMsgBuf(id, 0, 4, data);
    }

    uint8_t StepperDriver::sendRelativeMoveCommand(int id, int steps, int delay)
    {
        uint8_t data[7] = {CAN_CMD_MOVE_REL,
                           static_cast<uint8_t>((steps >> 16) & 0xFF), static_cast<uint8_t>((steps >> 8) & 0xFF), static_cast<uint8_t>(steps & 0XFF),
                           static_cast<uint8_t>((delay >> 16) & 0xFF), static_cast<uint8_t>((delay >> 8) & 0xFF), static_cast<uint8_t>(delay & 0XFF)};
        return sendCanMsgBuf(id, 0, 7, data);
    }

    uint8_t StepperDriver::sendTorqueOnCommand(int id, int torque_on)
    {
        uint8_t data[2] = {0};
        data[0] = CAN_CMD_MODE;
        data[1] = (torque_on) ? STEPPER_CONTROL_MODE_STANDARD : STEPPER_CONTROL_MODE_RELAX;
        return sendCanMsgBuf(id, 0, 2, data);
    }

    uint8_t StepperDriver::sendPositionOffsetCommand(int id, int cmd, int absolute_steps_at_offset_position)
    {
        uint8_t data[6] = {CAN_CMD_OFFSET, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                           static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF),
                           static_cast<uint8_t>((absolute_steps_at_offset_position >> 8) & 0xFF), static_cast<uint8_t>(absolute_steps_at_offset_position & 0xFF)};
        return sendCanMsgBuf(id, 0, 6, data);
    }

    uint8_t StepperDriver::sendSynchronizePositionCommand(int id, bool begin_traj)
    {
        uint8_t data[2] = {CAN_CMD_SYNCHRONIZE, static_cast<uint8_t>(begin_traj)};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    uint8_t StepperDriver::sendMicroStepsCommand(int id, int micro_steps)
    {
        uint8_t data[2] = {CAN_CMD_MICRO_STEPS, static_cast<uint8_t>(micro_steps)};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    uint8_t StepperDriver::sendMaxEffortCommand(int id, int effort)
    {
        uint8_t data[2] = {CAN_CMD_MAX_EFFORT, static_cast<uint8_t>(effort)};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    std::string StepperDriver::getErrorMessage() const
    {
        return _debug_error_message;
    }

    uint8_t StepperDriver::sendCalibrationCommand(int id, int offset, int delay, int direction, int timeout)
    {
        direction = (direction > 0 ) ? 1 : 0;

        uint8_t data[8] = {CAN_CMD_CALIBRATE, static_cast<uint8_t>((offset >> 16) & 0xFF),
                           static_cast<uint8_t>((offset >> 8) & 0xFF), static_cast<uint8_t>(offset & 0XFF),
                           static_cast<uint8_t>((delay >> 8) & 0xFF), static_cast<uint8_t>(delay & 0xFF),
                           static_cast<uint8_t>(direction), static_cast<uint8_t>(timeout)};
        return sendCanMsgBuf(id, 0, 8, data);
    }

    e_CanStepperCalibrationStatus StepperDriver::getCalibrationResult(uint8_t id, int32_t &result) const
    {
        result = _motor_calibration_map.at(id);
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
        std::thread reading_data_thread(&StepperDriver::interpreteCalibrationCommand, this);
        while (_calibration_motor_list.size() != 0)
        {
            if (canReadData())
            {
                unsigned long rxId;
                uint8_t len;
                std::array<uint8_t, 8> rxBuf;

                readMsgBuf(&rxId, &len, rxBuf);
                CalibrationStepperData calib_data = {rxId, len, rxBuf};
                _calibration_readed_datas.push_back(calib_data);
            }
            else if (ros::Time::now().toSec() - time_thread_begin > calibration_timeout)
            {
                ROS_ERROR("StepperDriver::readCalibrationStates - Calibration timeout after: %lf s", ros::Time::now().toSec() - time_thread_begin);
                for (int i = 0; i < _calibration_motor_list.size(); i++)
                {
                    ROS_ERROR("StepperDriver::readCalibrationStates - Motor %d timeout: may be disconnected", _calibration_motor_list.at(i));
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
        ROS_DEBUG("StepperDriver::readCalibrationStates - Calibration thread ended with success");
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
//                    data_str+= " " + std::to_string(static_cast<int>(_calibration_readed_datas[0].rxBuf[byte_nb]));
//                }
//                ROS_WARN("motor calib: %d, %d, %s", static_cast<int>(_calibration_readed_datas[0].rxId & 0x0F), _calibration_readed_datas[0].len, data_str.c_str());

                if (_calibration_readed_datas[0].len == 4)
                {
                    unsigned long rxId = _calibration_readed_datas[0].rxId ;
                    std::array<uint8_t, 8> rxBuf = _calibration_readed_datas[0].rxBuf;

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
                                    ROS_ERROR("StepperDriver::interpreteCalibrationCommand - Motor %d had calibration timeout", motor_id);
                                    _calibration_result = can_enum_result;
                                    return;
                                }
                                else if (can_enum_result == e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_BAD_PARAM)
                                {
                                    ROS_ERROR("StepperDriver::interpreteCalibrationCommand - Bad params given to motor %d", motor_id);
                                    _calibration_result = can_enum_result;
                                    return;
                                }
                                else if (can_enum_result == e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_OK)
                                {
                                    ROS_INFO("StepperDriver::interpreteCalibrationCommand - Motor %d calibration OK", motor_id);
                                    int steps_at_offset_pos = (rxBuf[2] << 8) + rxBuf[3];
                                    ROS_INFO("StepperDriver::interpreteCalibrationCommand - Motor %d - Absolute steps at offset position : %d", motor_id, steps_at_offset_pos);
                                    _motor_calibration_map[motor_id] = static_cast<int32_t>(steps_at_offset_pos);
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
                                common::model::StepperMotorCmd cmd = _motor_calibration_map_cmd[motor_id].cmd;
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
                unsigned long rxId;
                uint8_t len;
                std::array<uint8_t, 8> rxBuf;
                readMsgBuf(&rxId, &len, rxBuf);
                int motor_id = rxId & 0x0F;
                if (motor_id == motor_to_find)
                {
                    motor_found = true;
                }
            }
            if (ros::Time::now().toSec() - time_begin_scan > timeout)
            {
                ROS_ERROR("StepperDriver::scanMotorId - Motor with id %d not found", motor_to_find);
                return motor_found;
            }
        }
        return motor_found;
    }

    uint8_t StepperDriver::sendConveyorOnCommand(int id, bool conveyor_on, int conveyor_speed, int8_t direction)
    {
        ROS_DEBUG("StepperDriver::scanMotorId - Send conveyor id %d enabled (%d) at speed %d on direction %d",
                  id, static_cast<int>(conveyor_on), conveyor_speed, direction);
        uint8_t data[4] = {0};
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

    uint8_t StepperDriver::sendUpdateConveyorId(uint8_t old_id, uint8_t new_id)
    {
        ROS_DEBUG("StepperDriver::sendUpdateConveyorId - Send update conveyor id from %d to %d", old_id, new_id);
        uint8_t data[3] = {0};
        data[0] = CAN_CMD_MODE;
        data[1] = CAN_UPDATE_CONVEYOR_ID;
        data[2] = new_id;
        return sendCanMsgBuf(old_id, 0, 3, data);
    }

    uint8_t StepperDriver::sendCanMsgBuf(unsigned long id, uint8_t ext, uint8_t len, uint8_t *buf)
    {
        uint8_t status = 0;
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
                ROS_WARN_THROTTLE(0.5, "StepperDriver::sendCanMsgBuf - Sending Stepper message on CAN Bus failed");
            }
        }
        return status;
    }

} // namespace StepperDriver
