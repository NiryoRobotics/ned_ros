/*
    can_driver.cpp
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

#include "can_driver/can_driver.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/stepper_command_type_enum.hpp"

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
using ::common::model::StepperMotorCmd;
using ::common::model::SingleMotorCmdI;
namespace can_driver
{

/**
 * @brief CanDriver::CanDriver
 */
CanDriver::CanDriver(ros::NodeHandle& nh) :
    _calibration_status(EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED)
{
    ROS_DEBUG("CanDriver - ctor");

    init(nh);

    if (CAN_OK == setupCAN(nh))
    {
        scanAndCheck();

        _stepper_timeout_thread = std::thread(&CanDriver::_verifyMotorTimeoutLoop, this);
    }
    else
    {
        ROS_WARN("CanDriver - Stepper setup Failed");
    }
}

/**
 * @brief CanDriver::~CanDriver
 */
CanDriver::~CanDriver()
{
    if (_stepper_timeout_thread.joinable())
        _stepper_timeout_thread.join();
}

/**
 * @brief CanDriver::startCalibration
 */
void CanDriver::startCalibration()
{
    ROS_DEBUG("CanDriver::startCalibration: starting...");

    for (auto const& s : _state_map)
    {
        if (s.second)
            s.second->setCalibration(EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS, 0);
    }

    _calibration_status = EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS;
}

/**
 * @brief CanDriver::resetCalibration
 */
void CanDriver::resetCalibration()
{
    ROS_DEBUG("CanDriver::resetCalibration: reseting...");

    _calibration_status = EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
}

/**
 * @brief CanDriver::init : initialize the internal data (map, vectors) based on conf
 * @return
 */
bool CanDriver::init(ros::NodeHandle& nh)
{
    nh.getParam("calibration_timeout", _calibration_timeout);
    ROS_DEBUG("CanDriver::init - Calibration timeout %f", _calibration_timeout);

    std::vector<int> idListRaw;
    std::vector<std::string> typeListRaw;
    std::vector<std::string> typeProtocolList;

    nh.getParam("motors_types/motor_id_list", idListRaw);
    nh.getParam("motors_types/motor_type_list", typeListRaw);
    nh.getParam("motors_types/motor_type_protocol", typeProtocolList);
    
    // check that the two lists have the same size
    if (idListRaw.size() != typeListRaw.size() || idListRaw.size() != typeProtocolList.size())
        ROS_ERROR("CanDriver::init - wrong motors configuration. "
                  "Please check your configuration file motor_id_list, motor_type_list, protocol_type_list");

    std::vector<int> idList;
    std::vector<std::string> typeList;

    for (size_t i = 0; i < typeProtocolList.size(); i++)
    {
        if (typeProtocolList[i] == "can")
        {
            idList.push_back(idListRaw.at(i));
            typeList.push_back(typeListRaw.at(i));
        }
    }

    // debug - display info
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < idList.size(); i++)
        ss << " id " << idList.at(i) << ",";

    std::string motor_string_list = ss.str();
    motor_string_list.pop_back();  // remove last ","
    motor_string_list += "]";

    ROS_INFO("CanDriver::init - Stepper motor list: %s ", motor_string_list.c_str());

    // put everything in maps
    for (size_t i = 0; i < idList.size(); i++)
    {
        uint8_t id = static_cast<uint8_t>(idList.at(i));

        if (!_state_map.count(id))
            addMotor(id);
        else
            ROS_ERROR("CanDriver::init - duplicate id %d. Please check your configuration file "
                      "(niryo_robot_hardware_stack/can_driver/config/motors_config.yaml)", id);
    }

    // display internal data for debug
    for (auto const &s : _state_map)
        ROS_DEBUG("CanDriver::init - State map: %d => %s", s.first, s.second->str().c_str());

    return true;
}

/**
 * @brief CanDriver::setupCAN
 * @return
 */
int CanDriver::setupCAN(ros::NodeHandle& nh)
{
    int result = CAN_FAILINIT;
    int spi_channel = 0;
    int spi_baudrate = 0;
    int gpio_can_interrupt = 0;

    ros::NodeHandle nh_private("~");
    nh.getParam("can_bus/spi_channel", spi_channel);
    nh.getParam("can_bus/spi_baudrate", spi_baudrate);
    nh.getParam("can_bus/gpio_can_interrupt", gpio_can_interrupt);

    ROS_DEBUG("CanDriver::CanDriver - Can bus parameters: spi_channel : %d", spi_channel);
    ROS_DEBUG("CanDriver::CanDriver - Can bus parameters: spi_baudrate : %d", spi_baudrate);
    ROS_DEBUG("CanDriver::CanDriver - Can bus parameters: gpio_can_interrupt : %d", gpio_can_interrupt);

    mcp_can = std::make_unique<mcp_can_rpi::MCP_CAN>(spi_channel, spi_baudrate,
                                                     static_cast<uint8_t>(gpio_can_interrupt));

    if (mcp_can)
    {
        if (mcp_can->setupInterruptGpio())
        {
            ROS_DEBUG("CanDriver::setupCAN - setup interrupt GPIO successfull");
            if (mcp_can->setupSpi())
            {
                ROS_DEBUG("CanDriver::setupCAN - setup spi successfull");

                // no mask or filter used, receive all messages from CAN bus
                // messages with ids != motor_id will be sent to another ROS interface
                // so we can use many CAN devices with this only driver
                result = mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
                ROS_DEBUG("CanDriver::setupCAN - Result begin can : %d", result);

                if (CAN_OK == result)
                {
                    // set mode to normal
                    mcp_can->setMode(MCP_NORMAL);
                    _is_connection_ok = false;
                    ros::Duration(0.05).sleep();
                }
                else
                {
                    ROS_ERROR("CanDriver::setupCAN - Failed to init MCP2515 (CAN bus)");
                    _debug_error_message = "Failed to init MCP2515 (CAN bus)";
                }
            }
            else
            {
                ROS_WARN("CanDriver::setupCAN - Failed to start spi");
                _debug_error_message = "Failed to start spi";
                result = CAN_SPI_FAILINIT;
            }
        }
        else
        {
            ROS_WARN("CanDriver::setupCAN - Failed to start gpio");
            _debug_error_message = "Failed to start gpio";
            result = CAN_GPIO_FAILINIT;
        }
    }

    return result;
}

/**
 * @brief CanDriver::isConnectionOk
 * @return
 */
bool CanDriver::isConnectionOk() const
{
    return _is_connection_ok;
}

/**
 * @brief CanDriver::addMotor
 * @param id
 * @param isConveyor
 */
void CanDriver::addMotor(uint8_t id, bool isConveyor)
{
    ROS_DEBUG("CanDriver::addMotor - Add motor id: %d", id);

    // add id to _state_map
    if (isConveyor)
        _state_map.insert(std::make_pair(id, std::make_shared<ConveyorState>(id)));
    else
        _state_map.insert(std::make_pair(id, std::make_shared<StepperMotorState>(id)));
}

/**
 * @brief CanDriver::removeMotor
 * @param id
 */
void CanDriver::removeMotor(uint8_t id)
{
    ROS_DEBUG("CanDriver::removeMotor - Remove motor id: %d", id);

    if (_state_map.count(id))
    {
        _state_map.erase(id);
    }
}

/**
 * @brief CanDriver::getPosition
 * @param motor_id
 * @return
 */
int32_t CanDriver::getPosition(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) || !_state_map.at(motor_id))
    {
        throw std::out_of_range("CanDriver::getPosition: Unknown motor id");
    }

    return _state_map.at(motor_id)->getPositionState();
}


/**
 * @brief CanDriver::readCommand
 * @param cmd
 * @return
 */
int CanDriver::readSingleCommand(std::shared_ptr<SingleMotorCmdI> cmd)
{
    int result = CAN_INVALID_CMD;
    ROS_DEBUG("CanDriver::readCommand - Received stepper cmd %s", cmd->str().c_str());

    if (cmd->isValid())  // certifies that params is not empty
    {
        switch (EStepperCommandType(cmd->getTypeCmd()))
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
            case EStepperCommandType::CMD_TYPE_UPDATE_CONVEYOR:
                    result = sendUpdateConveyorId(cmd->getId(),
                                                  static_cast<uint8_t>(cmd->getParams().front()));
                    if (result == CAN_OK)
                    {
                        removeMotor(cmd->getId());
                        addMotor(static_cast<uint8_t>(cmd->getParams().front()));
                    }
            break;
            default:
                break;
        }
    }

    if (CAN_OK != result)
    {
        ROS_ERROR_THROTTLE(0.5, "CanDriver::readCommand - Failed to read stepper cmd");
        _debug_error_message = "CanDriver - Failed to read stepper cmd";
    }

    ROS_DEBUG_THROTTLE(0.5, "CanDriver::readCommand - Received stepper cmd finished");
    return result;
}

/**
 * @brief CanDriver::executeJointTrajectoryCmd
 * @param cmd_vec : need to be passed by copy, so that we ensure the data will not change in this method
 */
void CanDriver::executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, uint32_t> > cmd_vec)
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
 * @brief CanDriver::readMotorsState
 */
void CanDriver::readStatus()
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
                    ROS_ERROR("CanDriver::readMotorsState : unknown control byte value");
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
 * @brief CanDriver::scanAndCheck : to check if all motors in state are accessible
 * @return
 */
int CanDriver::scanAndCheck()
{
    std::lock_guard<std::mutex> lck(_stepper_timeout_mutex);

    ROS_DEBUG("CanDriver::scanAndCheck");
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
        ROS_ERROR_THROTTLE(2, "CanDriver::scanAndCheck - CAN scan Timeout");
        _debug_error_message = "CAN bus scan failed : motors ";
        for (uint8_t m_id : motors_unfound)
            _debug_error_message += " " + std::to_string(m_id) + ",";
        _debug_error_message.pop_back();  // remove trailing ","
        _debug_error_message += "are not connected";

        _is_connection_ok = false;
        ROS_ERROR_THROTTLE(2, "CanDriver::scanAndCheck - %s", _debug_error_message.c_str());

        result = CAN_FAIL;
    }
    else
    {
        ROS_DEBUG("CanDriver::scanAndCheck successful");
        _is_connection_ok = true;
    }

    return result;
}

/**
 * @brief CanDriver::ping : scan the bus for any motor id. It is used mainly to add an unknown
 * id to the current list of id (using addMotor, for a conveyor for example)
 * @param motor_to_find
 * @return
 */
bool CanDriver::ping(uint8_t motor_to_find)
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


    ROS_ERROR("CanDriver::scanMotorId - Motor with id %d not found", static_cast<int>(motor_to_find));
    return false;
}

/**
 * @brief CanDriver::fillPositionStatus
 * @param motor_id
 * @param len
 * @param data
 */
void CanDriver::fillPositionStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
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
            ROS_ERROR("CanDriver::fillPositionStatus - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
    {
        ROS_ERROR("CanDriver::fillPositionStatus - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanDriver::fillTemperatureStatus
 * @param motor_id
 * @param len
 * @param data
 */
void CanDriver::fillTemperatureStatus(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
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
            int driver_temp = static_cast<int>((-b - std::sqrt(b * b - 4 * a * (c - v_temp))) / (2 * a) + 30);

            // fill data
            _state_map.at(motor_id)->setTemperatureState(driver_temp);
        }
        else
        {
            ROS_ERROR("CanDriver::fillTemperatureStatus - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
    {
        ROS_ERROR("CanDriver::fillTemperatureStatus - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanDriver::fillFirmwareVersion
 * @param motor_id
 * @param len
 * @param data
 */
void CanDriver::fillFirmwareVersion(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
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
            ROS_ERROR("CanDriver::fillFirmwareVersion - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
{
        ROS_ERROR("CanDriver::fillFirmwareVersion - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanDriver::fillCalibrationState
 * @param motor_id
 * @param len
 * @param data
 */
void CanDriver::fillCalibrationState(uint8_t motor_id, const uint8_t &len, const std::array<uint8_t, 8> &data)
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
            ROS_ERROR("CanDriver::fillCalibrationState - unknown motor id %d", static_cast<int>(motor_id));
        }
    }
    else
    {
        ROS_ERROR("CanDriver::fillCalibrationState - frame should contain 4 data bytes");
    }
}

/**
 * @brief CanDriver::fillConveyorState
 * @param motor_id
 * @param data
 */
void CanDriver::fillConveyorState(uint8_t motor_id, const std::array<uint8_t, 8> &data)
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
 * @brief CanDriver::_verifyMotorTimeoutLoop : check that motors are still visible in the duration defined by getCurrentTimeout()
 * This timeout changed overtime according to the current state of the driver (in calibration or not)
 */
void CanDriver::_verifyMotorTimeoutLoop()
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
 * @brief CanDriver::updateCurrentCalibrationStatus
 */
void CanDriver::updateCurrentCalibrationStatus()
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
 * @brief CanDriver::getCurrentTimeout
 * used to adapt the timeout according to the state of the can (calibration or not)
 * @return
 */
double CanDriver::getCurrentTimeout() const
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
 * @brief CanDriver::readMsgBuf
 * @param id
 * @param len
 * @param buf
 * @return
 */
uint8_t CanDriver::readMsgBuf(INT32U *id, uint8_t *len, std::array<uint8_t, 8> &buf)
{
    uint8_t status = CAN_FAIL;

    for (auto i = 0; i < 10 && CAN_OK != status; ++i)
    {
        status = mcp_can->readMsgBuf(id, len, buf.data());
        ROS_WARN_COND(CAN_OK != status, "CanDriver::readMsgBuf - Reading Stepper message on CAN Bus failed");
    }

    return status;
}

/**
 * @brief CanDriver::sendCanMsgBuf
 * @param id
 * @param ext
 * @param len
 * @param buf
 * @return
 */
uint8_t CanDriver::sendCanMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    uint8_t status = CAN_FAIL;

    for (auto i = 0; i < 10 && CAN_OK != status; ++i)
    {
        status = mcp_can->sendMsgBuf(id, ext, len, buf);
        ROS_WARN_COND(CAN_OK != status, "CanDriver::sendCanMsgBuf - Sending Stepper message on CAN Bus failed");
    }

    return status;
}

// ******************
//      Senders
// *****************

/**
 * @brief CanDriver::sendConveyorOnCommand
 * @param id
 * @param conveyor_on
 * @param conveyor_speed
 * @param direction
 * @return
 */
uint8_t CanDriver::sendConveyorOnCommand(uint8_t id, bool conveyor_on, uint8_t conveyor_speed, uint8_t direction)
{
    ROS_DEBUG("CanDriver::scanMotorId - Send conveyor id %d enabled (%d) at speed %d on direction %d",
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
 * @brief CanDriver::sendPositionCommand
 * @param id
 * @param cmd
 * @return
 */
uint8_t CanDriver::sendPositionCommand(uint8_t id, int cmd)
{
    uint8_t data[4] = {CAN_CMD_POSITION, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                       static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF)};
    return sendCanMsgBuf(id, 0, 4, data);
}

/**
 * @brief CanDriver::sendRelativeMoveCommand
 * @param id
 * @param steps
 * @param delay
 * @return
 */
uint8_t CanDriver::sendRelativeMoveCommand(uint8_t id, int steps, int delay)
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
 * @brief CanDriver::sendTorqueOnCommand
 * @param id
 * @param torque_on
 * @return
 */
uint8_t CanDriver::sendTorqueOnCommand(uint8_t id, int torque_on)
{
    uint8_t data[2] = {0};
    data[0] = CAN_CMD_MODE;
    data[1] = (torque_on) ? STEPPER_CONTROL_MODE_STANDARD : STEPPER_CONTROL_MODE_RELAX;
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanDriver::sendPositionOffsetCommand
 * @param id
 * @param cmd
 * @param absolute_steps_at_offset_position
 * @return
 */
uint8_t CanDriver::sendPositionOffsetCommand(uint8_t id, int cmd, int absolute_steps_at_offset_position)
{
    uint8_t data[6] = {CAN_CMD_OFFSET, static_cast<uint8_t>((cmd >> 16) & 0xFF),
                       static_cast<uint8_t>((cmd >> 8) & 0xFF), static_cast<uint8_t>(cmd & 0XFF),
                       static_cast<uint8_t>((absolute_steps_at_offset_position >> 8) & 0xFF),
                       static_cast<uint8_t>(absolute_steps_at_offset_position & 0xFF)};
    return sendCanMsgBuf(id, 0, 6, data);
}

/**
 * @brief CanDriver::sendSynchronizePositionCommand
 * @param id
 * @param begin_traj
 * @return
 */
uint8_t CanDriver::sendSynchronizePositionCommand(uint8_t id, bool begin_traj)
{
    uint8_t data[2] = {CAN_CMD_SYNCHRONIZE, static_cast<uint8_t>(begin_traj)};
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanDriver::sendMicroStepsCommand
 * @param id
 * @param micro_steps
 * @return
 */
uint8_t CanDriver::sendMicroStepsCommand(uint8_t id, int micro_steps)
{
    uint8_t data[2] = {CAN_CMD_MICRO_STEPS, static_cast<uint8_t>(micro_steps)};
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanDriver::sendMaxEffortCommand
 * @param id
 * @param effort
 * @return
 */
uint8_t CanDriver::sendMaxEffortCommand(uint8_t id, int effort)
{
    uint8_t data[2] = {CAN_CMD_MAX_EFFORT, static_cast<uint8_t>(effort)};
    return sendCanMsgBuf(id, 0, 2, data);
}

/**
 * @brief CanDriver::sendCalibrationCommand
 * @param id
 * @param offset
 * @param delay
 * @param direction
 * @param timeout
 * @return
 */
uint8_t CanDriver::sendCalibrationCommand(uint8_t id, int offset, int delay, int direction, int timeout)
{
    direction = (direction > 0) ? 1 : 0;

    uint8_t data[8] = {CAN_CMD_CALIBRATE, static_cast<uint8_t>((offset >> 16) & 0xFF),
                       static_cast<uint8_t>((offset >> 8) & 0xFF), static_cast<uint8_t>(offset & 0XFF),
                       static_cast<uint8_t>((delay >> 8) & 0xFF), static_cast<uint8_t>(delay & 0xFF),
                       static_cast<uint8_t>(direction), static_cast<uint8_t>(timeout)};
    return sendCanMsgBuf(id, 0, 8, data);
}

/**
 * @brief CanDriver::sendUpdateConveyorId
 * @param old_id
 * @param new_id
 * @return
 */
uint8_t CanDriver::sendUpdateConveyorId(uint8_t old_id, uint8_t new_id)
{
    ROS_DEBUG("CanDriver::sendUpdateConveyorId - Send update conveyor id from %d to %d", old_id, new_id);
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
 * @brief CanDriver::getErrorMessage
 * @return
 */
std::string CanDriver::getErrorMessage() const
{
    return _debug_error_message;
}

/**
 * @brief CanDriver::getBusState
 * @param connection_status
 * @param motor_list
 * @param error
 */
void CanDriver::getBusState(bool &connection_status, std::vector<uint8_t> &motor_list, std::string &error) const
{
    error = _debug_error_message;
    motor_list = _all_motor_connected;
    connection_status = isConnectionOk();
}

/**
 * @brief CanDriver::getCalibrationResult
 * @param motor_id
 * @return
 */
int32_t CanDriver::getCalibrationResult(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) && _state_map.at(motor_id))
        throw std::out_of_range("CanDriver::getMotorsState: Unknown motor id");

    return _state_map.at(motor_id)->getCalibrationValue();
}

/**
 * @brief CanDriver::getMotorsState
 * @param motor_id
 * @return
 */
StepperMotorState CanDriver::getMotorState(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) && _state_map.at(motor_id))
        throw std::out_of_range("CanDriver::getMotorsState: Unknown motor id");

    return *_state_map.at(motor_id).get();
}

/**
 * @brief CanDriver::getMotorsStates
 * @return
 */
std::vector<std::shared_ptr<StepperMotorState> > CanDriver::getMotorsStates() const
{
    std::vector<std::shared_ptr<StepperMotorState> > states;
    for (auto const& it : _state_map)
        states.push_back(it.second);

    return states;
}

}  // namespace can_driver
