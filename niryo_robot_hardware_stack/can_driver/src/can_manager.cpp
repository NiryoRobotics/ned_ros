/*
    can_manager.cpp
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

#include "can_driver/can_manager.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/bus_protocol_enum.hpp"

// c++
#include <functional>
#include <string>
#include <vector>
#include <set>
#include <utility>

using ::common::model::EStepperCalibrationStatus;
using ::common::model::ConveyorState;
using ::common::model::StepperMotorState;
using ::common::model::EStepperCommandType;
using ::common::model::EBusProtocol;

namespace can_driver
{

/**
 * @brief CanManager::CanManager
 */
CanManager::CanManager(ros::NodeHandle& nh) :
    _calibration_status(EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED)
{
    ROS_DEBUG("CanManager - ctor");

    init(nh);

    if (CAN_OK == setupCAN(nh))
    {
        scanAndCheck();

        _stepper_timeout_thread = std::thread(&CanManager::_verifyMotorTimeoutLoop, this);
    }
    else
    {
        ROS_WARN("CanManager - Stepper setup Failed");
    }
}

/**
 * @brief CanManager::~CanManager
 */
CanManager::~CanManager()
{
    if (_stepper_timeout_thread.joinable())
        _stepper_timeout_thread.join();
}

/**
 * @brief CanManager::startCalibration
 */
void CanManager::startCalibration()
{
    ROS_DEBUG("CanManager::startCalibration: starting...");

    for (auto const& s : _state_map)
    {
        if (s.second && !s.second->isConveyor())
            s.second->setCalibration(EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS, 0);
    }

    _calibration_status = EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS;
}

/**
 * @brief CanManager::resetCalibration
 */
void CanManager::resetCalibration()
{
    ROS_DEBUG("CanManager::resetCalibration: reseting...");

    _calibration_status = EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
}

/**
 * @brief CanManager::init : initialize the internal data (map, vectors) based on conf
 * @return
 */
bool CanManager::init(ros::NodeHandle& nh)
{
    nh.getParam("/niryo_robot_hardware_interface/joints_interface/calibration_timeout", _calibration_timeout);
    ROS_DEBUG("CanManager::init - Calibration timeout %f", _calibration_timeout);

    return true;
}

/**
 * @brief CanManager::setupCAN
 * @return
 */
int CanManager::setupCAN(ros::NodeHandle& nh)
{
    int result = CAN_FAILINIT;
    int spi_channel = 0;
    int spi_baudrate = 0;
    int gpio_can_interrupt = 0;

    ros::NodeHandle nh_private("~");
    nh.getParam("bus_params/spi_channel", spi_channel);
    nh.getParam("bus_params/spi_baudrate", spi_baudrate);
    nh.getParam("bus_params/gpio_can_interrupt", gpio_can_interrupt);

    ROS_DEBUG("CanManager::CanManager - Can bus parameters: spi_channel : %d", spi_channel);
    ROS_DEBUG("CanManager::CanManager - Can bus parameters: spi_baudrate : %d", spi_baudrate);
    ROS_DEBUG("CanManager::CanManager - Can bus parameters: gpio_can_interrupt : %d", gpio_can_interrupt);

    mcp_can = std::make_unique<mcp_can_rpi::MCP_CAN>(spi_channel, spi_baudrate,
                                                     static_cast<uint8_t>(gpio_can_interrupt));

    if (mcp_can)
    {
        if (mcp_can->setupInterruptGpio())
        {
            ROS_DEBUG("CanManager::setupCAN - setup interrupt GPIO successfull");
            if (mcp_can->setupSpi())
            {
                ROS_DEBUG("CanManager::setupCAN - setup spi successfull");

                // no mask or filter used, receive all messages from CAN bus
                // messages with ids != motor_id will be sent to another ROS interface
                // so we can use many CAN devices with this only driver
                result = mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);

                if (CAN_OK == result)
                {
                    // set mode to normal
                    mcp_can->setMode(MCP_NORMAL);
                    _is_connection_ok = false;
                    ros::Duration(0.05).sleep();
                }
                else
                {
                    ROS_ERROR("CanManager::setupCAN - Failed to init MCP2515 (CAN bus)");
                    _debug_error_message = "Failed to init MCP2515 (CAN bus)";
                }
            }
            else
            {
                ROS_WARN("CanManager::setupCAN - Failed to start spi");
                _debug_error_message = "Failed to start spi";
                result = CAN_SPI_FAILINIT;
            }
        }
        else
        {
            ROS_WARN("CanManager::setupCAN - Failed to start gpio");
            _debug_error_message = "Failed to start gpio";
            result = CAN_GPIO_FAILINIT;
        }
    }

    return result;
}

/**
 * @brief CanManager::isConnectionOk
 * @return
 */
bool CanManager::isConnectionOk() const
{
    return _is_connection_ok;
}

/**
 * @brief CanManager::removeMotor
 * @param id
 */
void CanManager::removeMotor(uint8_t id)
{
    ROS_DEBUG("CanManager::removeMotor - Remove motor id: %d", id);

    if (_state_map.count(id))
    {
        _state_map.erase(id);
    }
}

/**
 * @brief CanManager::getPosition
 * @param motor_id
 * @return
 */
int32_t CanManager::getPosition(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) || !_state_map.at(motor_id))
    {
        throw std::out_of_range("CanManager::getPosition: Unknown motor id");
    }

    return _state_map.at(motor_id)->getPositionState();
}


/**
 * @brief CanManager::readCommand
 * @param cmd
 * @return
 */
int CanManager::writeSingleCommand(std::shared_ptr<common::model::AbstractCanSingleMotorCmd> cmd)
{
    int result = CAN_INVALID_CMD;
    ROS_DEBUG("CanManager::readCommand - Received stepper cmd %s", cmd->str().c_str());

    if (cmd->isValid() && _state_map.count(cmd->getId()) > 0)  // certifies that params is not empty
    {
        switch (EStepperCommandType(cmd->getCmdType()))
        {
            case EStepperCommandType::CMD_TYPE_POSITION:
                result = sendPositionCommand(cmd->getId(),
                                             cmd->getParams().front());
            break;
            case EStepperCommandType::CMD_TYPE_TORQUE:
                result = sendTorqueOnCommand(cmd->getId(),
                                             cmd->getParams().front());
            break;
            case EStepperCommandType::CMD_TYPE_SYNCHRONIZE:
                result = sendSynchronizePositionCommand(cmd->getId(),
                                                        cmd->getParams().front());
            break;
            case EStepperCommandType::CMD_TYPE_RELATIVE_MOVE:
                result = sendRelativeMoveCommand(cmd->getId(),
                                                 cmd->getParams().at(0),
                                                 cmd->getParams().at(1));
            break;
            case EStepperCommandType::CMD_TYPE_MAX_EFFORT:
                result = sendMaxEffortCommand(cmd->getId(),
                                              cmd->getParams().front());
            break;
            case EStepperCommandType::CMD_TYPE_MICRO_STEPS:
                result = sendMicroStepsCommand(cmd->getId(),
                                               cmd->getParams().front());
            break;
            case EStepperCommandType::CMD_TYPE_CALIBRATION:
                result = sendCalibrationCommand(cmd->getId(),
                                                cmd->getParams().at(0),
                                                cmd->getParams().at(1),
                                                cmd->getParams().at(2),
                                                cmd->getParams().at(3));

            break;
            case EStepperCommandType::CMD_TYPE_POSITION_OFFSET:
                    result = sendPositionOffsetCommand(cmd->getId(),
                                                       cmd->getParams().at(0),
                                                       cmd->getParams().at(1));
                break;
            case EStepperCommandType::CMD_TYPE_CONVEYOR:
                    result = sendConveyorOnCommand(cmd->getId(),
                                                  cmd->getParams().at(0),
                                                  static_cast<uint8_t>(cmd->getParams().at(1)),
                                                  static_cast<uint8_t>(cmd->getParams().at(2)));
                break;
            default:
                break;
        }
    }

    if (CAN_OK != result)
    {
        ROS_ERROR_THROTTLE(0.5, "CanManager::readCommand - Failed to read stepper cmd");
        _debug_error_message = "CanManager - Failed to read stepper cmd";
    }

    ROS_DEBUG_THROTTLE(0.5, "CanManager::readCommand - Received stepper cmd finished");
    return result;
}

/**
 * @brief CanManager::executeJointTrajectoryCmd
 * @param cmd_vec : need to be passed by copy, so that we ensure the data will not change in this method
 */
void CanManager::executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, int32_t> > cmd_vec)
{
    for (auto const& cmd : cmd_vec)
    {
        if (sendPositionCommand(cmd.first, cmd.second) != CAN_OK)
        {
            ROS_WARN("Stepper Driver - send positions to motor id %d failed", cmd.first);
        }
    }
}

/**
 * @brief CanManager::readStatus
 */
void CanManager::readStatus()
{
    if (canReadData())
    {
        INT32U rxId;
        uint8_t len;
        std::array<uint8_t, 8> rxBuf;
        readMsgBuf(&rxId, &len, rxBuf);
        uint8_t motor_id = rxId & 0x0F;

        if (_state_map.count(motor_id) && _state_map.at(motor_id))
        {
            // update last time read
            _state_map.at(motor_id)->updateLastTimeRead();

            int control_byte = rxBuf[0];
            switch (control_byte)
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
                case CAN_DATA_CALIBRATION_RESULT:
                    fillCalibrationState(motor_id, len, rxBuf);
                    break;
                default:
                    ROS_ERROR("CanManager::readMotorsState : unknown control byte value");
                break;
            }
        }
        else
        {
            _debug_error_message = "Unknow connected motor : ";
            _debug_error_message += std::to_string(motor_id);
        }
    }
}

/**
 * @brief CanManager::scanAndCheck : to check if all motors in state are accessible
 * @return
 */
int CanManager::scanAndCheck()
{
    std::lock_guard<std::mutex> lck(_stepper_timeout_mutex);

    ROS_DEBUG("CanManager::scanAndCheck");
    int result = CAN_OK;

    _all_motor_connected.clear();

    _debug_error_message = "";

    double time_begin_scan = ros::Time::now().toSec();
    double timeout = 0.5;

    // only for non conveyor motors
    std::set<uint8_t> motors_unfound;
    for (auto const& it : _state_map)
    {
        if (it.second)
            motors_unfound.insert(it.first);
    }

    while ((!motors_unfound.empty()) && (ros::Time::now().toSec() - time_begin_scan < timeout))
    {
        ros::Duration(0.001).sleep();  // check at 1000 Hz
        if (canReadData())
        {
            INT32U rxId;
            uint8_t len;
            std::array<uint8_t, 8> rxBuf;
            readMsgBuf(&rxId, &len, rxBuf);
            uint8_t motor_id = rxId & 0x0F;

            if (motors_unfound.count(motor_id))
            {
                motors_unfound.erase(motor_id);
                _all_motor_connected.emplace_back(motor_id);
                _state_map.at(motor_id)->updateLastTimeRead();
            }
        }
    }

    // if timeout
    if (!motors_unfound.empty())
    {
        ROS_ERROR_THROTTLE(2, "CanManager::scanAndCheck - CAN scan Timeout");
        _debug_error_message = "CAN bus scan failed : motors ";
        for (uint8_t m_id : motors_unfound)
            _debug_error_message += " " + std::to_string(m_id) + ",";
        _debug_error_message.pop_back();  // remove trailing ","
        _debug_error_message += "are not connected";

        _is_connection_ok = false;
        ROS_ERROR_THROTTLE(2, "CanManager::scanAndCheck - %s", _debug_error_message.c_str());

        result = CAN_FAIL;
    }
    else
    {
        ROS_DEBUG("CanManager::scanAndCheck successful");
        _is_connection_ok = true;
    }

    return result;
}

/**
 * @brief CanManager::ping : scan the bus for any motor id. It is used mainly to add an unknown
 * id to the current list of id (using addMotor, for a conveyor for example)
 * @param motor_to_find
 * @return
 */
bool CanManager::ping(uint8_t motor_to_find)
{
    double time_begin_scan = ros::Time::now().toSec();

    while (ros::Time::now().toSec() - time_begin_scan < STEPPER_MOTOR_TIMEOUT_VALUE)
    {
        ros::Duration(0.001).sleep();  // check at 1000 Hz
        if (canReadData())
        {
            INT32U rxId;
            uint8_t len;
            std::array<uint8_t, 8> rxBuf;
            readMsgBuf(&rxId, &len, rxBuf);
            uint8_t motor_id = rxId & 0x0F;

            if (motor_id == motor_to_find)
                return true;
        }
    }


    ROS_ERROR("CanManager::scanMotorId - Motor with id %d not found", static_cast<int>(motor_to_find));
    return false;
}

/**
 * @brief CanManager::fillPositionStatus
 * @param motor_id
 * @param len
 * @param data
 */
void CanManager::fillPositionStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
{
    if (MESSAGE_DIAGNOSTICS_LENGTH == len)
    {
        if (_state_map.count(motor_id) && _state_map.at(motor_id))
        {
            int32_t pos = (data[1] << 16) + (data[2] << 8) + data[3];
            pos = (pos & (1 << 15)) ? -1 * ((~pos + 1) & 0xFFFF) : pos;

            _state_map.at(motor_id)->setPositionState(pos);
        }
        else
        {
            ROS_ERROR("CanManager::fillPositionStatus - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
    {
        ROS_ERROR("CanManager::fillPositionStatus - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanManager::fillTemperatureStatus
 * @param motor_id
 * @param len
 * @param data
 */
void CanManager::fillTemperatureStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
{
    if (MESSAGE_DIAGNOSTICS_LENGTH == len)
    {
        if (_state_map.count(motor_id) && _state_map.at(motor_id))
        {
            int driver_temp_raw = (data[2] << 8) + data[3];
            double a = -0.00316;
            double b = -12.924;
            double c = 2367.7;
            double v_temp = driver_temp_raw * 3.3 / 1024.0 * 1000.0;
            uint32_t driver_temp = static_cast<uint32_t>((-b - std::sqrt(b * b - 4 * a * (c - v_temp))) / (2 * a) + 30);

            // fill data
            _state_map.at(motor_id)->setTemperature(driver_temp);
        }
        else
        {
            ROS_ERROR("CanManager::fillTemperatureStatus - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
    {
        ROS_ERROR("CanManager::fillTemperatureStatus - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanManager::fillFirmwareVersion
 * @param motor_id
 * @param len
 * @param data
 */
void CanManager::fillFirmwareVersion(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
{
    if (MESSAGE_DIAGNOSTICS_LENGTH == len)
    {
        if (_state_map.count(motor_id) && _state_map.at(motor_id))
        {
            int v_major = data[1];
            int v_minor = data[2];
            int v_patch = data[3];
            std::ostringstream ss;
            ss << v_major << "." << v_minor << "." << v_patch;
            std::string version = ss.str();

            // fill data
            _state_map.at(motor_id)->setFirmwareVersion(version);
        }
        else
        {
            ROS_ERROR("CanManager::fillFirmwareVersion - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
{
        ROS_ERROR("CanManager::fillFirmwareVersion - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanManager::fillCalibrationState
 * @param motor_id
 * @param len
 * @param data
 */
void CanManager::fillCalibrationState(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
{
    if (MESSAGE_DIAGNOSTICS_LENGTH == len)
    {
        if (_state_map.count(motor_id) && _state_map.at(motor_id))
        {
            // fill data
            EStepperCalibrationStatus status = static_cast<EStepperCalibrationStatus>(data[1]);
            int32_t value = (data[2] << 8) + data[3];
            _state_map.at(motor_id)->setCalibration(status, value);

            updateCurrentCalibrationStatus();
        }
        else
        {
            ROS_ERROR("CanManager::fillCalibrationState - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
    {
        ROS_ERROR("CanManager::fillCalibrationState - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanManager::fillConveyorState
 * @param motor_id
 * @param data
 */
void CanManager::fillConveyorState(uint8_t motor_id, const std::array<uint8_t, 8> &data)
{
    if (_state_map.count(motor_id) && _state_map.at(motor_id))
    {
        bool state = data[1];
        int16_t speed = data[2];
        int8_t direction = static_cast<int8_t>(data[3]);

        auto cState = std::dynamic_pointer_cast<ConveyorState>(_state_map.at(motor_id));
        cState->setDirection(direction);
        cState->setSpeed(speed);
        cState->setState(state);
    }
}

/**
 * @brief CanManager::_verifyMotorTimeoutLoop : check that motors are still visible in the duration defined by getCurrentTimeout()
 * This timeout changed overtime according to the current state of the driver (in calibration or not)
 */
void CanManager::_verifyMotorTimeoutLoop()
{
    while (ros::ok())
    {
        std::lock_guard<std::mutex> lck(_stepper_timeout_mutex);
        std::vector<uint8_t> timeout_motors;

        for (auto const &mState : _state_map)
        {
            // we locate the motor for the current id in _all_motor_connected
            auto position = std::find(_all_motor_connected.begin(), _all_motor_connected.end(), mState.first);

            // if it has timeout, we remove it from the vector
            if (mState.second && ros::Time::now().toSec() - mState.second->getLastTimeRead() > getCurrentTimeout())
            {
                timeout_motors.emplace_back(mState.first);
                if (position != _all_motor_connected.end())
                    _all_motor_connected.erase(position);
            }  // else, if it is not in the list of connected motors, we add it
            else if (position == _all_motor_connected.end())
            {
                _all_motor_connected.push_back(mState.first);
            }
        }

        // if we detected motors in timeout, we change the state and display error message
        if (!timeout_motors.empty())
        {
            _is_connection_ok = false;

            std::ostringstream ss;
            ss <<  "Disconnected stepper motor(s):";
            for (auto const& m : timeout_motors)
                ss << " " << static_cast<int>(m) << ",";
            _debug_error_message = ss.str();
            _debug_error_message.pop_back();
        }
        else
        {
            _is_connection_ok = true;
        }
        ros::Duration(0.1).sleep();
    }
}

/**
 * @brief CanManager::updateCurrentCalibrationStatus
 */
void CanManager::updateCurrentCalibrationStatus()
{
    // update current state of the calibrationtimeout_motors
    // rule is : if a status in a motor is worse than one previously found, we take it
    // we are not taking "uninitialized" into account as it means a calibration as not been started for this motor
    EStepperCalibrationStatus newStatus = EStepperCalibrationStatus::CALIBRATION_OK;
    for (auto const& s : _state_map)
    {
        if (s.second && newStatus < s.second->getCalibrationState())
            newStatus = s.second->getCalibrationState();
    }

    _calibration_status = newStatus;
}

/**
 * @brief CanManager::getCurrentTimeout
 * used to adapt the timeout according to the state of the can (calibration or not)
 * @return
 */
double CanManager::getCurrentTimeout() const
{
    if (isCalibrationInProgress())
        return _calibration_timeout;
    else
        return STEPPER_MOTOR_TIMEOUT_VALUE;
}

// ***************
//  Private
// ***************

/**
 * @brief CanManager::readMsgBuf
 * @param id
 * @param len
 * @param buf
 * @return
 */
uint8_t CanManager::readMsgBuf(INT32U *id, uint8_t *len, std::array<uint8_t, 8> &buf)
{
    uint8_t status = CAN_FAIL;

    for (auto i = 0; i < 10 && CAN_OK != status; ++i)
    {
        status = mcp_can->readMsgBuf(id, len, buf.data());
        ROS_WARN_COND(CAN_OK != status, "CanManager::readMsgBuf - Reading Stepper message on CAN Bus failed");
    }

    return status;
}

/**
 * @brief CanManager::sendCanMsgBuf
 * @param id
 * @param ext
 * @param len
 * @param buf
 * @return
 */
uint8_t CanManager::sendCanMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    uint8_t status = CAN_FAIL;

    for (auto i = 0; i < 10 && CAN_OK != status; ++i)
    {
        status = mcp_can->sendMsgBuf(id, ext, len, buf);
        ROS_WARN_COND(CAN_OK != status, "CanManager::sendCanMsgBuf - Sending Stepper message on CAN Bus failed");
    }

    return status;
}

// ******************
//      Senders
// *****************

/**
 * @brief CanManager::sendConveyorOnCommand
 * @param id
 * @param conveyor_on
 * @param conveyor_speed
 * @param direction
 * @return
 */
uint8_t CanManager::sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction)
{
    ROS_DEBUG("CanManager::scanMotorId - Send conveyor id %d enabled (%d) at speed %d on direction %d",
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

/**
 * @brief CanManager::sendPositionCommand
 * @param id
 * @param cmd
 * @return
 */
uint8_t CanManager::sendPositionCommand(uint8_t id, int cmd)
{
    uint8_t data[4] = {CAN_CMD_POSITION, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                       static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF)};
    return sendCanMsgBuf(id, 0, 4, data);
}

/**
 * @brief CanManager::sendRelativeMoveCommand
 * @param id
 * @param steps
 * @param delay
 * @return
 */
uint8_t CanManager::sendRelativeMoveCommand(uint8_t id, int steps, int delay)
{
    uint8_t data[7] = {CAN_CMD_MOVE_REL,
                       static_cast<uint8_t>((steps >> 16) & 0xFF),
                       static_cast<uint8_t>((steps >> 8) & 0xFF),
                       static_cast<uint8_t>(steps & 0XFF),
                       static_cast<uint8_t>((delay >> 16) & 0xFF),
                       static_cast<uint8_t>((delay >> 8) & 0xFF),
                       static_cast<uint8_t>(delay & 0XFF)};
    return sendCanMsgBuf(id, 0, 7, data);
}

/**
 * @brief CanManager::sendTorqueOnCommand
 * @param id
 * @param torque_on
 * @return
 */
uint8_t CanManager::sendTorqueOnCommand(uint8_t id, int torque_on)
{
    uint8_t data[2] = {0};
    data[0] = CAN_CMD_MODE;
    data[1] = (torque_on) ? STEPPER_CONTROL_MODE_STANDARD : STEPPER_CONTROL_MODE_RELAX;
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanManager::sendPositionOffsetCommand
 * @param id
 * @param cmd
 * @param absolute_steps_at_offset_position
 * @return
 */
uint8_t CanManager::sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position)
{
    uint8_t data[6] = {CAN_CMD_OFFSET, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                       static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF),
                       static_cast<uint8_t>((absolute_steps_at_offset_position >> 8) & 0xFF),
                       static_cast<uint8_t>(absolute_steps_at_offset_position & 0xFF)};
    return sendCanMsgBuf(id, 0, 6, data);
}

/**
 * @brief CanManager::sendSynchronizePositionCommand
 * @param id
 * @param begin_traj
 * @return
 */
uint8_t CanManager::sendSynchronizePositionCommand(uint8_t id, bool begin_traj)
{
    uint8_t data[2] = {CAN_CMD_SYNCHRONIZE, static_cast<uint8_t>(begin_traj)};
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanManager::sendMicroStepsCommand
 * @param id
 * @param micro_steps
 * @return
 */
uint8_t CanManager::sendMicroStepsCommand(uint8_t id, int micro_steps)
{
    uint8_t data[2] = {CAN_CMD_MICRO_STEPS, static_cast<uint8_t>(micro_steps)};
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanManager::sendMaxEffortCommand
 * @param id
 * @param effort
 * @return
 */
uint8_t CanManager::sendMaxEffortCommand(uint8_t id, int effort)
{
    uint8_t data[2] = {CAN_CMD_MAX_EFFORT, static_cast<uint8_t>(effort)};
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanManager::sendCalibrationCommand
 * @param id
 * @param offset
 * @param delay
 * @param direction
 * @param timeout
 * @return
 */
uint8_t CanManager::sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout)
{
    direction = (direction > 0) ? 1 : 0;

    uint8_t data[8] = {CAN_CMD_CALIBRATE, static_cast<uint8_t>((offset >> 16) & 0xFF),
                       static_cast<uint8_t>((offset >> 8) & 0xFF), static_cast<uint8_t>(offset & 0XFF),
                       static_cast<uint8_t>((delay >> 8) & 0xFF), static_cast<uint8_t>(delay & 0xFF),
                       static_cast<uint8_t>(direction), static_cast<uint8_t>(timeout)};
    return sendCanMsgBuf(id, 0, 8, data);
}

/**
 * @brief CanManager::sendUpdateConveyorId
 * @param old_id
 * @param new_id
 * @return
 */
uint8_t CanManager::sendUpdateConveyorId(uint8_t old_id, uint8_t new_id)
{
    ROS_DEBUG("CanManager::sendUpdateConveyorId - Send update conveyor id from %d to %d", old_id, new_id);
    uint8_t data[3] = {0};
    data[0] = CAN_CMD_MODE;
    data[1] = CAN_UPDATE_CONVEYOR_ID;
    data[2] = new_id;
    return sendCanMsgBuf(old_id, 0, 3, data);
}


// ******************
//  Getters
// ******************

/**
 * @brief CanManager::getErrorMessage
 * @return
 */
std::string CanManager::getErrorMessage() const
{
    return _debug_error_message;
}

/**
 * @brief CanManager::getBusState
 * @param connection_status
 * @param motor_list
 * @param error
 */
void CanManager::getBusState(bool &connection_status, std::vector<uint8_t> &motor_list, std::string &error) const
{
    error = _debug_error_message;
    motor_list = _all_motor_connected;
    connection_status = isConnectionOk();
}

/**
 * @brief CanManager::getCalibrationResult
 * @param motor_id
 * @return
 */
int32_t CanManager::getCalibrationResult(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) && _state_map.at(motor_id))
        throw std::out_of_range("CanManager::getMotorsState: Unknown motor id");

    return _state_map.at(motor_id)->getCalibrationValue();
}

/**
 * @brief CanManager::getMotorsStates
 * @return
 */
std::vector<std::shared_ptr<StepperMotorState> > CanManager::getMotorsStates() const
{
    std::vector<std::shared_ptr<StepperMotorState> > states;
    for (auto const& it : _state_map)
        states.push_back(it.second);

    return states;
}

}  // namespace can_driver
