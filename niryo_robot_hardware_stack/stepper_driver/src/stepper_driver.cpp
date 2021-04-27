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
#include "stepper_driver/calibration_stepper_data.hpp"

#include <functional>

using namespace common::model;

namespace StepperDriver
{
    StepperDriver::StepperDriver() :
        _stepper_timeout_thread(&StepperDriver::_verifyMotorTimeoutLoop, this),
        _is_can_connection_ok(false),
        _calibration_result(EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED)
    {
        ROS_DEBUG("StepperDriver - ctor");

        initParameters();
        init();

        if(CAN_OK != setupCAN())
            ROS_WARN("StepperDriver - Stepper setup Failed");
    }

    StepperDriver::~StepperDriver()
    {
        if (_stepper_timeout_thread.joinable())
            _stepper_timeout_thread.join();


        if(_calibration_thread.joinable())
            _stepper_timeout_thread.join();
    }

    /**
     * @brief StepperDriver::init : initialize the internal data (map, vectors) based on conf
     * @return
     */
    bool StepperDriver::init()
    {
        std::vector<int> idList;

        if (_nh.hasParam("/niryo_robot_hardware_interface/stepper_driver/motors_params/stepper_motor_id_list"))
            _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/motors_params/stepper_motor_id_list", idList);
        else
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/stepper_motor_id_list", idList);

        //debug - display info
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < idList.size(); ++i)
            ss << " id " << idList.at(i) << ",";

        std::string motor_string_list = ss.str();
        motor_string_list.pop_back(); //remove last ","
        motor_string_list += "]";

        ROS_INFO("StepperDriver::init - Stepper motor list: %s ", motor_string_list.c_str());

        //put everything in maps
        for(size_t i = 0; i < idList.size(); ++i) {

            uint8_t id = static_cast<uint8_t>(idList.at(i));

            if(0 == _state_map.count(id))
                addMotor(id);
            else
                ROS_ERROR("StepperDriver::init - duplicate id %d. Please check your configuration file (niryo_robot_hardware_stack/dynamixel_driver/config/motors_config.yaml)", id);
        }

        //display internal data for debug
        for(auto const &s : _state_map)
            ROS_DEBUG("StepperDriver::init - State map: %d => %s", s.first, s.second.str().c_str());



        return true;
    }

    /**
     * @brief StepperDriver::initParameters : initiliaze parameters that are direct attribute of this class
     */
    void StepperDriver::initParameters()
    {
        _nh.getParam("/niryo_robot_hardware_interface/calibration_timeout", _calibration_timeout);

    }

    int StepperDriver::setupCAN()
    {
        int result = CAN_FAILINIT;
        int spi_channel = 0;
        int spi_baudrate = 0;
        int gpio_can_interrupt = 0;

        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_bus/spi_channel", spi_channel);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_bus/spi_baudrate", spi_baudrate);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_bus/gpio_can_interrupt", gpio_can_interrupt);

        ROS_DEBUG("StepperDriver::StepperDriver - Can bus parameters: spi_channel : %d", spi_channel);
        ROS_DEBUG("StepperDriver::StepperDriver - Can bus parameters: spi_baudrate : %d", spi_baudrate);
        ROS_DEBUG("StepperDriver::StepperDriver - Can bus parameters: spi_baudrate : %d", gpio_can_interrupt);

        mcp_can.reset(new MCP_CAN_RPI::MCP_CAN(spi_channel, spi_baudrate, gpio_can_interrupt));

        if(mcp_can) {
            // no mask or filter used, receive all messages from CAN bus
            // messages with ids != motor_id will be sent to another ROS interface
            // so we can use many CAN devices with this only driver
            result = mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
            ROS_DEBUG("StepperDriver::setupCAN - Result begin can : %d", result);

            if (CAN_OK == result) {

                // set mode to normal
                mcp_can->setMode(MCP_NORMAL);
                _is_can_connection_ok = false;
                ros::Duration(0.05).sleep();

                if(mcp_can->setupInterruptGpio())
                {
                    ROS_DEBUG("StepperDriver::setupInterruptGpio - setup successfull");
                    if(mcp_can->setupSpi()) {
                        ROS_DEBUG("StepperDriver::setupSpi - setup successfull");
                    }
                    else
                    {
                        ROS_WARN("StepperDriver::setupSpi - Failed to start spi");
                        _debug_error_message = "Failed to start spi";
                        result = CAN_SPI_FAILINIT;
                    }
                }
                else
                {
                    ROS_WARN("StepperDriver::setupInterruptGpio - Failed to start gpio");
                    _debug_error_message = "Failed to start gpio";
                    result = CAN_GPIO_FAILINIT;
                }
            }
            else
            {
                ROS_ERROR("StepperDriver::init - Failed to init MCP2515 (CAN bus)");
                _debug_error_message = "Failed to init MCP2515 (CAN bus)";
            }
        }

        return result;
    }

    /**
     * @brief StepperDriver::isConnectionOk
     * @return
     */
    bool StepperDriver::isConnectionOk() const
    {
        return _is_can_connection_ok;
    }

    /**
     * @brief StepperDriver::addMotor
     * @param id
     */
    void StepperDriver::addMotor(uint8_t id)
    {
        ROS_DEBUG("DxlDriver::addMotor - Add motor id: %d", id);

        //add id to _state_map
        _state_map.insert(std::make_pair(id, StepperMotorState(id)));
        _calibration_map.insert(std::make_pair(id, CalibrationStepperData()));
    }

    /**
     * @brief StepperDriver::removeMotor
     * @param id
     */
    void StepperDriver::removeMotor(uint8_t id)
    {
        ROS_DEBUG("DxlDriver::removeMotor - Remove motor id: %d", id);

        if(_state_map.count(id)) {
            _state_map.erase(id);
        }
    }

    /**
     * @brief StepperDriver::addConveyor
     * @param conveyor_id
     */
    void StepperDriver::addConveyor(uint8_t conveyor_id)
    {
        ROS_DEBUG("DxlDriver::addConveyor - Add motor id: %d", conveyor_id);

        //add id to _state_map
        _conveyor_map.insert(std::make_pair(conveyor_id, ConveyorState(conveyor_id)));
    }

    /**
     * @brief StepperDriver::removeConveyor
     * @param conveyor_id
     */
    void StepperDriver::removeConveyor(uint8_t conveyor_id)
    {
        ROS_DEBUG("DxlDriver::removeConveyor - Remove motor id: %d", conveyor_id);

        if(_state_map.count(conveyor_id)) {
            _state_map.erase(conveyor_id);
        }
    }

    void StepperDriver::getBusState(bool &connection_status, std::vector<uint8_t> &motor_list, std::string &error) const
    {
        error = _debug_error_message;
        motor_list = _all_motor_connected;
        connection_status = isConnectionOk();
    }

    int StepperDriver::readSynchronizeCommand(const SynchronizeStepperMotorCmd& cmd)
    {
        int result = CAN_INVALID_CMD;
        ROS_DEBUG("StepperDriver::executeJointTrajectoryCmd - Received synchronize stepper cmd %s", cmd.str().c_str());

        std::vector<uint8_t> ids = cmd.getMotorsId();
        if(cmd.isValid()) // certifies that params is not empty
        {
            switch(cmd.getType())
            {
                case EStepperCommandType::CMD_TYPE_POSITION:
                    for(auto&& id : ids) {
                        result = sendPositionCommand(id, cmd.getParam(id));
                        if (result != CAN_OK)
                            ROS_WARN("Stepper Driver - send positions to motor id %d failed", id);
                    }
                break;
                case EStepperCommandType::CMD_TYPE_TORQUE:
                    for(auto&& id : ids) {
                        result = sendTorqueOnCommand(id, cmd.getParam(id));
                        if (result != CAN_OK)
                            ROS_WARN("Stepper Driver - send positions to motor id %d failed", id);
                    }
                break;
                case EStepperCommandType::CMD_TYPE_MAX_EFFORT:
                    for(auto&& id : ids) {
                        result = sendMaxEffortCommand(id, cmd.getParam(id));
                        if (result != CAN_OK)
                            ROS_WARN("Stepper Driver - send positions to motor id %d failed", id);
                    }
                break;
                case EStepperCommandType::CMD_TYPE_MICRO_STEPS:
                    for(auto&& id : ids) {

                        result = sendMicroStepsCommand(id, cmd.getParam(id));
                        if (result != CAN_OK)
                            ROS_WARN("Stepper Driver - send positions to motor id %d failed", id);
                    }
                break;
                default:
                break;
            }
        }

        if(CAN_OK != result)
        {
            ROS_ERROR_THROTTLE(0.5, "StepperDriver::readCommand - Failed to read stepper cmd");
            _debug_error_message = "StepperDriver - Failed to read stepper cmd";
        }

        ROS_DEBUG_THROTTLE(0.5, "StepperDriver::readCommand - Received stepper cmd finished");
        return result;
    }

    /**
     * @brief StepperDriver::getStepperPose
     * @param motor_id
     * @return
     */
    uint32_t StepperDriver::getStepperPose(uint8_t motor_id) const
    {
        if(!_state_map.count(motor_id)) {
            throw std::out_of_range("DxlDriver::getStepperPose: Unknown motor id");
        }

        return _state_map.at(motor_id).getPositionState();
    }


    /**
     * @brief StepperDriver::readCommand
     * @param cmd
     * @return
     */
    int StepperDriver::readSingleCommand(StepperMotorCmd cmd)
    {
        int result = CAN_INVALID_CMD;
        ROS_DEBUG("StepperDriver::readCommand - Received stepper cmd %s", cmd.str().c_str());

        if(cmd.isValid()) // certifies that params is not empty
        {
            switch(cmd.getType())
            {
                case EStepperCommandType::CMD_TYPE_POSITION:
                    result = sendPositionCommand(cmd.getId(),
                                                 cmd.getParams().front());
                break;
                case EStepperCommandType::CMD_TYPE_TORQUE:
                    result = sendTorqueOnCommand(cmd.getId(),
                                                 cmd.getParams().front());
                break;
                case EStepperCommandType::CMD_TYPE_SYNCHRONIZE:
                    result = sendSynchronizePositionCommand(cmd.getId(),
                                                            cmd.getParams().front());
                break;
                case EStepperCommandType::CMD_TYPE_RELATIVE_MOVE:
                    result = sendRelativeMoveCommand(cmd.getId(),
                                                     cmd.getParams().at(0),
                                                     cmd.getParams().at(1));
                break;
                case EStepperCommandType::CMD_TYPE_MAX_EFFORT:
                    result = sendMaxEffortCommand(cmd.getId(),
                                                  cmd.getParams().front());
                break;
                case EStepperCommandType::CMD_TYPE_MICRO_STEPS:
                    result = sendMicroStepsCommand(cmd.getId(),
                                                   cmd.getParams().front());
                break;
                case EStepperCommandType::CMD_TYPE_CALIBRATION:
                    result = sendCalibrationCommand(cmd.getId(),
                                                    cmd.getParams().at(0),
                                                    cmd.getParams().at(1),
                                                    cmd.getParams().at(2),
                                                    cmd.getParams().at(3));

                break;
                case EStepperCommandType::CMD_TYPE_POSITION_OFFSET:
                        result = sendPositionOffsetCommand(cmd.getId(),
                                                           cmd.getParams().at(0),
                                                           cmd.getParams().at(1));
                    break;
                case EStepperCommandType::CMD_TYPE_CONVEYOR:
                        result = sendConveyorOnCommand(cmd.getId(),
                                                      cmd.getParams().at(0),
                                                      cmd.getParams().at(1),
                                                      static_cast<int8_t>(cmd.getParams().at(2)));
                    break;
                case EStepperCommandType::CMD_TYPE_UPDATE_CONVEYOR:
                        result = sendUpdateConveyorId(cmd.getId(),
                                                      static_cast<uint8_t>(cmd.getParams().front()));
                        if (result == CAN_OK)
                        {
                            removeConveyor(cmd.getId());
                            addConveyor(static_cast<uint8_t>(cmd.getParams().front()));
                        }
                break;
                default:
                    break;
            }
        }

        if(CAN_OK != result)
        {
            ROS_ERROR_THROTTLE(0.5, "StepperDriver::readCommand - Failed to read stepper cmd");
            _debug_error_message = "StepperDriver - Failed to read stepper cmd";
        }

        ROS_DEBUG_THROTTLE(0.5, "StepperDriver::readCommand - Received stepper cmd finished");
        return result;
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

        std::vector<uint8_t> motors_unfound;
        for(auto const& it: _state_map)
            motors_unfound.emplace_back(it.first);

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
                for (uint8_t m_id : motors_unfound)
                {
                    _debug_error_message += std::to_string(m_id);
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

    /**
     * @brief StepperDriver::readMotorsState
     */
    void StepperDriver::readMotorsState()
    {
        if (canReadData())
        {
            unsigned long rxId;
            uint8_t len;
            std::array<uint8_t, 8> rxBuf;
            readMsgBuf(&rxId, &len, rxBuf);
            uint8_t motor_id = rxId & 0x0F;

            if (checkMotorsId(motor_id))
            {
                int control_byte = rxBuf[0];

                switch(control_byte)
                {
                    case CAN_DATA_POSITION:
                        fillPositionStatus(motor_id, len, rxBuf);
                        break;
                    case CAN_DATA_DIAGNOSTICS:
                        fillTemperatureStatus(motor_id, len, rxBuf);
                        break;
                    case CAN_DATA_FIRMWARE_VERSION:
                        fillFirmwareVersion(motor_id, len, rxBuf);
                        break;
                    case CAN_DATA_CONVEYOR_STATE:
                        fillConveyorState(motor_id, rxBuf);
                        break;
                    default:
                        ROS_ERROR("StepperDriver::readMotorsState : unknown control byte value");
                    break;
                }
            }
        }
    }

    /**
     * @brief StepperDriver::fillPositionStatus
     * @param motor_id
     * @param len
     * @param data
     */
    void StepperDriver::fillPositionStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
    {
        if (checkMessageLength(len, MESSAGE_DIAGNOSTICS_LENGTH))
        {
            if(_state_map.count(motor_id)) {

                int32_t pos = (data[1] << 16) + (data[2] << 8) + data[3];
                pos = (pos & (1 << 15)) ? -1 * ((~pos + 1) & 0xFFFF) : pos;

                _state_map.at(motor_id).setPositionState(pos);
            }
            else {
                ROS_ERROR("StepperDriver::fillPositionStatus - unknown motor id %d", static_cast<int>(motor_id));
            }
        }
        else {
            ROS_ERROR("StepperDriver::fillPositionStatus - frame should contain 4 data bytes");
        }
    }

    /**
     * @brief StepperDriver::fillTemperatureStatus
     * @param motor_id
     * @param len
     * @param data
     */
    void StepperDriver::fillTemperatureStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
    {
        if (checkMessageLength(len, MESSAGE_DIAGNOSTICS_LENGTH))
        {
            if(_state_map.count(motor_id)) {

                int driver_temp_raw = (data[2] << 8) + data[3];
                double a = -0.00316;
                double b = -12.924;
                double c = 2367.7;
                double v_temp = driver_temp_raw * 3.3 / 1024.0 * 1000.0;
                int driver_temp = int((-b - std::sqrt(b * b - 4 * a * (c - v_temp))) / (2 * a) + 30);

                // fill data
                _state_map.at(motor_id).setTemperatureState(driver_temp);
            }
            else {
                ROS_ERROR("StepperDriver::fillTemperatureStatus - unknown motor id %d", static_cast<int>(motor_id));
            }
        }
        else {
            ROS_ERROR("StepperDriver::fillTemperatureStatus - frame should contain 4 data bytes");
        }
    }

    /**
     * @brief StepperDriver::fillFirmwareVersion
     * @param motor_id
     * @param len
     * @param data
     */
    void StepperDriver::fillFirmwareVersion(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
    {
        if (checkMessageLength(len, MESSAGE_DIAGNOSTICS_LENGTH))
        {
            if(_state_map.count(motor_id)) {

                int v_major = data[1];
                int v_minor = data[2];
                int v_patch = data[3];
                std::ostringstream ss;
                ss << v_major << "." << v_minor << "." << v_patch;
                std::string version = ss.str();

                // fill data
                _state_map.at(motor_id).setFirmwareVersion(version);
            }
            else {
                ROS_ERROR("StepperDriver::fillFirmwareVersion - unknown motor id %d", static_cast<int>(motor_id));
            }
        }
        else {
            ROS_ERROR("StepperDriver::fillFirmwareVersion - frame should contain 4 data bytes");
        }
    }

    /**
     * @brief StepperDriver::fillConveyorState
     * @param motor_id
     * @param data
     */
    void StepperDriver::fillConveyorState(uint8_t motor_id, const std::array<uint8_t, 8> &data)
    {
        if(_conveyor_map.count(motor_id)) {
            bool state = data[1];
            int16_t speed = data[2];
            int8_t direction = static_cast<int8_t>(data[3]);

            _conveyor_map.at(motor_id).setDirection(direction);
            _conveyor_map.at(motor_id).setSpeed(speed);
            _conveyor_map.at(motor_id).setState(state);
        }
    }

    void StepperDriver::_verifyMotorTimeoutLoop()
    {
        while (this->_nh.ok())
        {
            // Only check when motors seems connected and not in calibration (not state received)
            if (this->_is_can_connection_ok && !this->isCalibrationInProgress())
            {
                for (const auto &motor : this->_state_map)
                {
                    const double &last_time_read = motor.second.getLastTimeRead();
                    if (last_time_read > 0)
                    {
                        double sec_elapsed = ros::Time::now().toSec() - last_time_read;
                        if (sec_elapsed > STEPPER_MOTOR_TIMEOUT_VALUE)
                        {
                            // If it's the first motor that seems disconnected
                            if (this->_is_can_connection_ok)
                            {
                                this->_debug_error_message = "Disconnected stepper motor(s): " + std::to_string(motor.second.getId());
                            }
                            // If an another motor seems already disconnected
                            else
                            {
                                this->_debug_error_message += ", " + std::to_string(motor.second.getId());
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
        for (auto &it : _state_map)
        {
            it.second.setLastTimeRead(-1);
        }
    }


    bool StepperDriver::checkMotorsId(uint8_t motor_id)
    {
        bool motor_found = false;
        if(_state_map.count(motor_id))
        {
            _state_map.at(motor_id).setLastTimeRead(ros::Time::now().toSec());
            motor_found = true;
        }
        else if(_conveyor_map.count(motor_id))
        {
            motor_found = true;
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

    uint8_t StepperDriver::sendPositionCommand(uint8_t id, int cmd)
    {
        uint8_t data[4] = {CAN_CMD_POSITION, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                           static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF)};
        return sendCanMsgBuf(id, 0, 4, data);
    }

    uint8_t StepperDriver::sendRelativeMoveCommand(uint8_t id, int steps, int delay)
    {
        uint8_t data[7] = {CAN_CMD_MOVE_REL,
                           static_cast<uint8_t>((steps >> 16) & 0xFF), static_cast<uint8_t>((steps >> 8) & 0xFF), static_cast<uint8_t>(steps & 0XFF),
                           static_cast<uint8_t>((delay >> 16) & 0xFF), static_cast<uint8_t>((delay >> 8) & 0xFF), static_cast<uint8_t>(delay & 0XFF)};
        return sendCanMsgBuf(id, 0, 7, data);
    }

    uint8_t StepperDriver::sendTorqueOnCommand(uint8_t id, int torque_on)
    {
        uint8_t data[2] = {0};
        data[0] = CAN_CMD_MODE;
        data[1] = (torque_on) ? STEPPER_CONTROL_MODE_STANDARD : STEPPER_CONTROL_MODE_RELAX;
        return sendCanMsgBuf(id, 0, 2, data);
    }

    uint8_t StepperDriver::sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position)
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

    uint8_t StepperDriver::sendMicroStepsCommand(uint8_t id, int micro_steps)
    {
        uint8_t data[2] = {CAN_CMD_MICRO_STEPS, static_cast<uint8_t>(micro_steps)};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    uint8_t StepperDriver::sendMaxEffortCommand(uint8_t id, int effort)
    {
        uint8_t data[2] = {CAN_CMD_MAX_EFFORT, static_cast<uint8_t>(effort)};
        return sendCanMsgBuf(id, 0, 2, data);
    }

    std::string StepperDriver::getErrorMessage() const
    {
        return _debug_error_message;
    }

    uint8_t StepperDriver::sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout)
    {
        direction = (direction > 0 ) ? 1 : 0;

        uint8_t data[8] = {CAN_CMD_CALIBRATE, static_cast<uint8_t>((offset >> 16) & 0xFF),
                           static_cast<uint8_t>((offset >> 8) & 0xFF), static_cast<uint8_t>(offset & 0XFF),
                           static_cast<uint8_t>((delay >> 8) & 0xFF), static_cast<uint8_t>(delay & 0xFF),
                           static_cast<uint8_t>(direction), static_cast<uint8_t>(timeout)};
        return sendCanMsgBuf(id, 0, 8, data);
    }

    int32_t StepperDriver::getCalibrationResult(uint8_t id) const
    {
        return _calibration_map.at(id).getCalibrationResult();
    }

    EStepperCalibrationStatus StepperDriver::getCalibrationStatus() const
    {
        return _calibration_result;
    }

    void StepperDriver::startCalibration()
    {
        //start read calibration state thread
        if (!isCalibrationInProgress())
        {
            //refresh state map
            this->_refreshMotorTimeout();

            // Join the previous calibration thread (otherwise we cannot reassign the thread)
            if (_calibration_thread.joinable())
                _calibration_thread.join();
            _calibration_thread = std::thread(&StepperDriver::readCalibrationStates, this);
        }
    }

    void StepperDriver::stopCalibration()
    {
        _calibration_result  = EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;

        if (_calibration_thread.joinable())
            _calibration_thread.join();
    }

    /**
     * @brief StepperDriver::readCalibrationStates
     */
    void StepperDriver::readCalibrationStates()
    {
        _calibration_result = EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS;
        double time_thread_begin = ros::Time::now().toSec();

        //this vector keep a list of motor ids to process. When a motors has been processed, it is removed from the list
        std::set<uint8_t> motorsToProcess;
        for(auto const& it: _calibration_map)
            motorsToProcess.insert(it.first);

        while(!motorsToProcess.empty() && ros::Time::now().toSec() - time_thread_begin < _calibration_timeout)
        {
            if (canReadData())
            {
                unsigned long rxId;
                uint8_t len;
                std::array<uint8_t, 8> rxBuf;

                readMsgBuf(&rxId, &len, rxBuf);
                CalibrationStepperData calib_data = {rxId, len, rxBuf};
                uint8_t motor_id = calib_data.getId();

                if(calib_data.isValid() && motorsToProcess.count(motor_id))
                {
                    EStepperCalibrationStatus can_enum_result = calib_data.getStatus();
                    if (EStepperCalibrationStatus::CALIBRATION_OK == can_enum_result)
                    {
                        ROS_INFO("StepperDriver::interpreteCalibrationCommand - Motor %d calibration OK", motor_id);
                        _calibration_map.at(motor_id) = calib_data;

                        ROS_INFO("StepperDriver::interpreteCalibrationCommand - Motor %d - Absolute steps at offset position : %d",
                                 motor_id,  _calibration_map.at(motor_id).getCalibrationResult());

                        motorsToProcess.erase(motor_id);

                        // keep torque ON for axis 1
                        if (1 == motor_id)
                            sendTorqueOnCommand(1, true);
                    }
                    else {
                        switch(can_enum_result)
                        {
                        case EStepperCalibrationStatus::CALIBRATION_TIMEOUT:
                            ROS_ERROR("StepperDriver::interpreteCalibrationCommand - Motor %d had calibration timeout", motor_id);
                            _calibration_result = can_enum_result;
                            return;
                        case EStepperCalibrationStatus::CALIBRATION_BAD_PARAM:
                            ROS_ERROR("StepperDriver::interpreteCalibrationCommand - Bad params given to motor %d", motor_id);
                            _calibration_result = can_enum_result;
                            return;
                        default:
                            break;
                        }
                    }
                }
            } // if (canReadData())
        } // while

        //if still motors to process -> timeout
        if (!motorsToProcess.empty())
        {
            _calibration_result = EStepperCalibrationStatus::CALIBRATION_TIMEOUT;
            ROS_ERROR("StepperDriver::readCalibrationStates - Calibration timeout after: %lf s", ros::Time::now().toSec() - time_thread_begin);

            for (uint8_t m_id : motorsToProcess)
                ROS_ERROR("StepperDriver::readCalibrationStates - Motor %d timeout: may be disconnected", m_id);

        }
        else {
            _calibration_result = EStepperCalibrationStatus::CALIBRATION_OK;
            ROS_DEBUG("StepperDriver::readCalibrationStates - Calibration thread ended with success");
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

    uint8_t StepperDriver::sendConveyorOnCommand(uint8_t id, bool conveyor_on, int conveyor_speed, int8_t direction)
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

    //******************
    //  Getters
    //******************

    /**
     * @brief StepperDriver::getMotorsState
     * @param motor_id
     * @return
     */
    StepperMotorState StepperDriver::getMotorState(uint8_t motor_id) const
    {
        if(!_state_map.count(motor_id))
            throw std::out_of_range("DxlDriver::getMotorsState: Unknown motor id");

        return _state_map.at(motor_id);
    }

    /**
     * @brief StepperDriver::getMotorsStates
     * @return
     */
    std::vector<StepperMotorState> StepperDriver::getMotorsStates() const
    {
        std::vector<common::model::StepperMotorState> states;
        for (auto const& it : _state_map)
            states.push_back(it.second);

        return states;
    }

    /**
     * @brief StepperDriver::getConveyorState
     * @param motor_id
     * @return
     */
    ConveyorState StepperDriver::getConveyorState(uint8_t motor_id) const
    {
        if(!_conveyor_map.count(motor_id))
            throw std::out_of_range("DxlDriver::getMotorsState: Unknown motor id");

        return _conveyor_map.at(motor_id);
    }

    /**
     * @brief StepperDriver::getConveyorsStates
     * @return
     */
    std::vector<ConveyorState> StepperDriver::getConveyorsStates() const
    {
        std::vector<ConveyorState> states;
        for (auto const& it : _conveyor_map)
            states.push_back(it.second);

        return states;
    }

} // namespace StepperDriver
