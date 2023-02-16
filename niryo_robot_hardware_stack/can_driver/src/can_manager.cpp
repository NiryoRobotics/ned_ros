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
#include "can_driver/mock_stepper_driver.hpp"
#include "can_driver/stepper_driver.hpp"
#include "common/model/bus_protocol_enum.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "niryo_robot_msgs/CommandStatus.h"

// c++
#include <algorithm>
#include <asm-generic/errno.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

using ::common::model::ConveyorState;
using ::common::model::EHardwareType;
using ::common::model::EStepperCalibrationStatus;
using ::common::model::JointState;
using ::common::model::StepperMotorState;

namespace can_driver
{

/**
 * @brief CanManager::CanManager
 */
CanManager::CanManager(ros::NodeHandle &nh) : _nh(nh)
{
    ROS_DEBUG("CanManager - ctor");

    _debug_error_message = "CanManager - No connection with CAN motors has been made yet";

    init(nh);

    if (CAN_OK == setupCommunication())
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
 * @brief CanManager::init : initialize the internal data (map, vectors) based on conf
 * @param nh
 * @return
 */
bool CanManager::init(ros::NodeHandle &nh)
{
    int spi_channel = 0;
    int spi_baudrate = 0;
    int gpio_can_interrupt = 0;

    bool simu_conveyor{false};
    nh.getParam("simulation_mode", _simulation_mode);
    nh.getParam("simu_conveyor", simu_conveyor);
    nh.getParam("bus_params/spi_channel", spi_channel);
    nh.getParam("bus_params/spi_baudrate", spi_baudrate);
    nh.getParam("bus_params/gpio_can_interrupt", gpio_can_interrupt);
    nh.getParam("/niryo_robot_hardware_interface/joints_interface/calibration_timeout", _calibration_timeout);

    ROS_DEBUG("CanManager::init - Can bus parameters: spi_channel : %d", spi_channel);
    ROS_DEBUG("CanManager::init - Can bus parameters: spi_baudrate : %d", spi_baudrate);
    ROS_DEBUG("CanManager::CanManager - Can bus parameters: gpio_can_interrupt : %d", gpio_can_interrupt);
    ROS_DEBUG("CanManager::init - Calibration timeout %f", _calibration_timeout);

    if (!_simulation_mode)
        _mcp_can = std::make_shared<mcp_can_rpi::MCP_CAN>(spi_channel, spi_baudrate, static_cast<uint8_t>(gpio_can_interrupt));
    else
    {
        readFakeConfig(simu_conveyor);
    }
    return true;
}

/**
 * @brief CanManager::setupCAN
 * @return
 */
int CanManager::setupCommunication()
{
    int ret = CAN_FAILINIT;
    ROS_DEBUG("CanManager::setupCommunication - initializing connection...");

    // if _simulation mode, not setup can
    if (_simulation_mode)
    {
        _debug_error_message.clear();
        return CAN_OK;
    }
    // Can bus setup
    if (_mcp_can)
    {
        _debug_error_message.clear();

        if (_mcp_can->setupInterruptGpio())
        {
            ROS_DEBUG("CanManager::setupCommunication - Setup Interrupt GPIO successfull");
            ros::Duration(0.05).sleep();

            if (_mcp_can->setupSpi())
            {
                ROS_DEBUG("CanManager::setupCommunication - Setup SPI successfull");
                ros::Duration(0.05).sleep();
                // no mask or filter used, receive all messages from CAN bus
                // messages with ids != motor_id will be sent to another ROS interface
                // so we can use many CAN devices with this only driver
                ret = _mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);

                if (CAN_OK == ret)
                {
                    ROS_DEBUG("CanManager::setupCommunication - MCP can initialized");

                    // set mode to normal
                    _mcp_can->setMode(MCP_NORMAL);
                    _is_connection_ok = false;
                    ros::Duration(0.05).sleep();
                }
                else
                {
                    ROS_ERROR("CanManager::setupCommunication - Failed to init MCP2515 (CAN bus)");
                    _debug_error_message = "Failed to init MCP2515 (CAN bus)";
                }
            }
            else
            {
                ROS_WARN("CanManager::setupCommunication - Failed to start spi");
                _debug_error_message = "Failed to start spi";
                ret = CAN_SPI_FAILINIT;
            }
        }
        else
        {
            ROS_WARN("CanManager::setupCommunication - Failed to start gpio");
            _debug_error_message = "Failed to start gpio";
            ret = CAN_GPIO_FAILINIT;
        }
    }
    else
        ROS_ERROR("CanManager::setupCommunication - Invalid CAN handler");

    return ret;
}

/**
 * @brief CanManager::addHardwareComponent add the state of a hardware component to can manager
 * It can be a joint, conveyor...
 * @param state
 */
int CanManager::addHardwareComponent(std::shared_ptr<common::model::AbstractHardwareState> &&state)  // NOLINT
{
    int result = niryo_robot_msgs::CommandStatus::FAILURE;

    common::model::EHardwareType hardware_type = state->getHardwareType();
    uint8_t id = state->getId();

    ROS_DEBUG("CanManager::addHardwareComponent : %s", state->str().c_str());

    // if not already instanciated
    if (!_state_map.count(id))
    {
        _state_map.insert(std::make_pair(id, state));
    }

    addHardwareDriver(hardware_type);

    result = niryo_robot_msgs::CommandStatus::SUCCESS;

    return result;
}

/**
 * @brief CanManager::removeHardwareComponent
 * @param id
 */
void CanManager::removeHardwareComponent(uint8_t id)
{
    ROS_DEBUG("CanManager::removeMotor - Remove motor id: %d", id);

    if (_state_map.count(id) && _state_map.at(id))
    {
        _state_map.erase(id);
    }

    _removed_motor_id_list.erase(std::remove(_removed_motor_id_list.begin(), _removed_motor_id_list.end(), id), _removed_motor_id_list.end());
}

// ****************
//  commands
// ****************

/**
 * @brief CanManager::changeId changeId of a hw component. Ex: Using when more than 1 conveyor,
 * we need to change id of conveyor to accept the next one.
 */
int CanManager::changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id)
{
    std::lock_guard<std::mutex> lck(_stepper_timeout_mutex);

    int ret = CAN_FAIL;
    if (old_id == new_id)
    {
        ret = CAN_OK;
    }
    else if (_driver_map.count(motor_type))
    {
        auto driver = std::dynamic_pointer_cast<AbstractStepperDriver>(_driver_map.at(motor_type));

        if (driver)
        {
            ret = driver->sendUpdateConveyorId(old_id, new_id);
            if (CAN_OK == ret)
            {
                // update all maps
                auto i_state = _state_map.find(old_id);
                // update all maps
                if (i_state != _state_map.end())
                {
                    // update all maps
                    std::swap(_state_map[new_id], i_state->second);
                    // update all maps
                    _state_map.erase(i_state);
                }
            }
        }
    }
    return ret;
}

/**
 * @brief CanManager::readStatus
 */
void CanManager::readStatus()
{
    // read from all drivers for all motors
    for (auto const &it : _driver_map)
    {
        auto driver = std::dynamic_pointer_cast<AbstractStepperDriver>(it.second);

        if (driver && driver->canReadData())
        {
            uint8_t motor_id{};
            int control_byte{};
            std::array<uint8_t, AbstractCanDriver::MAX_MESSAGE_LENGTH> rxBuf{};
            std::string error_message;

            if (CAN_OK == driver->readData(motor_id, control_byte, rxBuf, error_message))
            {
                if (_state_map.count(motor_id) && _state_map.at(motor_id))
                {
                    auto stepperState = std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(motor_id));
                    // update last time read
                    stepperState->updateLastTimeRead();
                    _debug_error_message.clear();
                    switch (control_byte)
                    {
                    case AbstractStepperDriver::CAN_DATA_POSITION:
                        stepperState->setPosition(driver->interpretPositionStatus(rxBuf));
                        break;
                    case AbstractStepperDriver::CAN_DATA_DIAGNOSTICS:
                        stepperState->setTemperature(driver->interpretTemperatureStatus(rxBuf));
                        break;
                    case AbstractStepperDriver::CAN_DATA_FIRMWARE_VERSION:
                        stepperState->setFirmwareVersion(driver->interpretFirmwareVersion(rxBuf));
                        break;
                    case AbstractStepperDriver::CAN_DATA_CONVEYOR_STATE:
                    {
                        auto cState = std::dynamic_pointer_cast<ConveyorState>(stepperState);
                        if (cState)
                        {
                            cState->updateData(driver->interpretConveyorData(rxBuf));
                            cState->setGoalDirection(cState->getGoalDirection() * cState->getDirection());
                        }
                        break;
                    }
                    case AbstractStepperDriver::CAN_DATA_CALIBRATION_RESULT:
                    {
                        stepperState->setCalibration(driver->interpretHomingData(rxBuf));
                        updateCurrentCalibrationStatus();
                        break;
                    }
                    default:
                        ROS_ERROR("CanManager::readMotorsState : unknown control byte value");
                        _debug_error_message = "unknown control byte value";
                        break;
                    }
                }
                else
                {
                    _debug_error_message = "Unknown connected motor : ";
                    _debug_error_message += std::to_string(motor_id);
                }
            }
            else
            {
                ROS_ERROR("CanManager::readStatus - %s", error_message.c_str());
            }
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
    int result = CAN_FAIL;

    _all_motor_connected.clear();
    _is_connection_ok = false;

    EHardwareType type;
    if (_driver_map.count(EHardwareType::STEPPER))
        type = EHardwareType::STEPPER;
    else if (_driver_map.count(EHardwareType::FAKE_STEPPER_MOTOR))
        type = EHardwareType::FAKE_STEPPER_MOTOR;
    else
        return result;
    // only stepper for now
    if (_driver_map.at(type))
    {
        // only for valid states (conveyor with default id is not valid)
        std::set<uint8_t> motors_unfound;
        for (auto const &it : _state_map)
        {
            if (it.second && it.second->isValid())
                motors_unfound.insert(it.first);
        }

        if (CAN_OK == _driver_map.at(type)->scan(motors_unfound, _all_motor_connected))
        {
            ROS_DEBUG("CanManager::scanAndCheck successful");
            _is_connection_ok = true;
            result = CAN_OK;
            _removed_motor_id_list.clear();
        }
        else
        {
            ROS_ERROR_THROTTLE(2, "CanManager::scanAndCheck - CAN scan Timeout");
            _debug_error_message = "CAN bus scan failed : motors ";
            _removed_motor_id_list.clear();
            for (uint8_t m_id : motors_unfound)
            {
                _removed_motor_id_list.emplace_back(m_id);
                _debug_error_message += " " + std::to_string(m_id) + ",";
            }
            _debug_error_message.pop_back();  // remove trailing ","
            _debug_error_message += " are not connected";

            ROS_ERROR_THROTTLE(2, "CanManager::scanAndCheck - %s", _debug_error_message.c_str());
        }

        // update last time read on found motors
        for (auto &motor_id : _all_motor_connected)
        {
            if (_state_map.count(motor_id) && _state_map.at(motor_id))
                std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(motor_id))->updateLastTimeRead();
        }
    }

    return result;
}

/**
 * @brief CanManager::ping : scan the bus for any motor id. It is used mainly to add an unknown
 * id to the current list of id (using addMotor, for a conveyor for example)
 * @param id
 * @return
 */
bool CanManager::ping(uint8_t id)
{
    int result = false;

    if (_state_map.find(id) != _state_map.end())
    {
        if (_driver_map.count(_state_map.find(id)->second->getHardwareType()))
        {
            auto it = _driver_map.at(_state_map.find(id)->second->getHardwareType());
            if (it)
            {
                _isPing = true;
                result = (CAN_OK == it->ping(id));
                _isPing = false;
            }
            else
            {
                ROS_ERROR_THROTTLE(1, "CanManager::ping - the can drivers seeems uninitialized");
            }
        }
    }

    return result;
}

/**
 * @brief CanManager::_verifyMotorTimeoutLoop : check that motors are still visible in the duration defined by getCurrentTimeout()
 * This timeout changed overtime according to the current state of the driver (in calibration or not)
 */
void CanManager::_verifyMotorTimeoutLoop()
{
    while (ros::ok())
    {
        std::vector<uint8_t> timeout_motors;

        // using have_motor to notify that the connection is down when no motors recognize yet
        bool have_motor = false;

        if (!_all_motor_connected.empty())
        {
            for (auto const &map_it : _state_map)
            {
                if (map_it.second && map_it.second->getComponentType() != common::model::EComponentType::CONVEYOR)
                {
                    // using mutex in for to protect only state_map if needed
                    std::lock_guard<std::mutex> lck(_stepper_timeout_mutex);
                    have_motor = true;
                    // we locate the motor for the current id in _all_motor_connected
                    auto position = std::find(_all_motor_connected.begin(), _all_motor_connected.end(), map_it.first);

                    // only if valid state (invalid if default id of conveyor for example)
                    if (map_it.second && map_it.second->isValid())
                    {
                        auto state = std::dynamic_pointer_cast<StepperMotorState>(map_it.second);

                        // if it has timeout, we remove it from the vector
                        if (state && ros::Time::now().toSec() - state->getLastTimeRead() > getCurrentTimeout())
                        {
                            timeout_motors.emplace_back(map_it.first);
                            if (position != _all_motor_connected.end())
                                _all_motor_connected.erase(position);
                        }  // else, if it is not in the list of connected motors, we add it (? _all_motor_connected got from scan, why add if id of state not found)
                        else if (position == _all_motor_connected.end())
                        {
                            _all_motor_connected.push_back(map_it.first);
                        }
                    }
                }
            }
        }

        // if we detected motors in timeout, we change the state and display error message
        if (!timeout_motors.empty() || !have_motor)
        {
            _is_connection_ok = false;

            std::ostringstream ss;
            ss << "No motor found or Disconnected stepper motor(s)(";
            for (auto const &m : timeout_motors)
                ss << " " << static_cast<int>(m) << ",";
            ss << ")";
            _debug_error_message = ss.str();
            _debug_error_message.pop_back();
        }
        else
        {
            _is_connection_ok = true;
            _debug_error_message.clear();
        }
        ros::Duration(0.5).sleep();
    }
}

/**
 * @brief CanManager::getCurrentTimeout
 * used to adapt the timeout according to the state of the can (calibration or not)
 * @return
 */
double CanManager::getCurrentTimeout() const
{
    double res = AbstractStepperDriver::STEPPER_MOTOR_TIMEOUT_VALUE;

    if (EStepperCalibrationStatus::IN_PROGRESS == _calibration_status)
        res = _calibration_timeout;
    else if (_isPing)
        res = _ping_timeout;

    return res;
}

// ******************
//  Read operations
// ******************

/**
 * @brief CanManager::getPosition
 * @param motor_state
 * @return
 */
int32_t CanManager::getPosition(const JointState &motor_state) const
{
    uint8_t motor_id = motor_state.getId();

    if (!_state_map.count(motor_id) || !_state_map.at(motor_id))
    {
        throw std::out_of_range("CanManager::getPosition: Unknown motor id");
    }
    auto jState = std::dynamic_pointer_cast<JointState>(_state_map.at(motor_id));
    if (jState)
        return jState->getPosition();

    return 0;
}

// ******************
//  Write operations
// ******************

/**
 * @brief CanManager::writeSingleCommand
 * @param cmd
 * @return
 */
int CanManager::writeSingleCommand(std::unique_ptr<common::model::AbstractCanSingleMotorCmd> &&cmd)  // NOLINT
{
    int result = CAN_INVALID_CMD;
    ROS_DEBUG("CanManager::readCommand - Received stepper cmd %s", cmd->str().c_str());

    uint8_t id = cmd->getId();

    if (cmd->isValid())
    {
        if (_state_map.count(id) != 0)
        {
            auto state = _state_map.at(id);

            common::model::EHardwareType hardware_type = state->getHardwareType();
            result = CAN_FAIL;
            if (_driver_map.count(hardware_type) && _driver_map.at(hardware_type))
            {
                result = _driver_map.at(hardware_type)->writeSingleCmd(cmd);
            }

            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }
    }

    if (result != CAN_OK)
    {
        ROS_WARN("CanManager::writeSingleCommand - Failed to write a single command on motor id : %d", id);
        _debug_error_message = "CanManager - Failed to write a single command";
    }

    return result;
}

/**
 * @brief CanManager::executeJointTrajectoryCmd
 * @param cmd_vec : need to be passed by copy, so that we ensure the data will not change in this method
 */
void CanManager::executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, int32_t>> cmd_vec)
{
    for (auto const &it : _driver_map)
    {
        auto driver = std::dynamic_pointer_cast<AbstractStepperDriver>(it.second);

        for (auto const &cmd : cmd_vec)
        {
            if (_state_map.count(cmd.first) && it.first == _state_map.at(cmd.first)->getHardwareType())
            {
                int err = driver->sendPositionCommand(cmd.first, cmd.second);
                if (err != CAN_OK)
                {
                    ROS_WARN("CanManager::executeJointTrajectoryCmd - Failed to write position");
                    _debug_error_message = "CanManager - Failed to write position";
                }
            }
        }
    }
}

// ******************
//  Calibration
// ******************

/**
 * @brief CanManager::startCalibration
 */
void CanManager::startCalibration()
{
    ROS_DEBUG("CanManager::startCalibration: starting...");

    for (auto const &s : _state_map)
    {
        if (s.second && (EHardwareType::STEPPER == s.second->getHardwareType() || EHardwareType::FAKE_STEPPER_MOTOR == s.second->getHardwareType()) &&
            !std::dynamic_pointer_cast<StepperMotorState>(s.second)->isConveyor())
            std::dynamic_pointer_cast<StepperMotorState>(s.second)->setCalibration(EStepperCalibrationStatus::IN_PROGRESS, 0);
    }

    _calibration_status = EStepperCalibrationStatus::IN_PROGRESS;
}

/**
 * @brief CanManager::resetCalibration
 */
void CanManager::resetCalibration()
{
    ROS_DEBUG_THROTTLE(0.5, "CanManager::resetCalibration: reseting...");

    _calibration_status = EStepperCalibrationStatus::UNINITIALIZED;
    for (auto &s : _state_map)
    {
        if (s.second && (s.second->getHardwareType() == EHardwareType::STEPPER || s.second->getHardwareType() == EHardwareType::FAKE_STEPPER_MOTOR) &&
            !std::dynamic_pointer_cast<StepperMotorState>(s.second)->isConveyor())
            std::dynamic_pointer_cast<StepperMotorState>(s.second)->setCalibration(EStepperCalibrationStatus::UNINITIALIZED, 0);
    }
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

    return std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(motor_id))->getCalibrationValue();
}

/**
 * @brief CanManager::updateCurrentCalibrationStatus
 */
void CanManager::updateCurrentCalibrationStatus()
{
    EStepperCalibrationStatus newStatus = _calibration_status > EStepperCalibrationStatus::OK ? EStepperCalibrationStatus::OK : _calibration_status;
    // update current state of the calibrationtimeout_motors
    // rule is : if a status in a motor is worse than one previously found, we take it
    // we are not taking "uninitialized" into account as it means a calibration as not been started for this motor
    for (auto const &s : _state_map)
    {
        if (s.second && (s.second->getHardwareType() == EHardwareType::STEPPER || s.second->getHardwareType() == EHardwareType::FAKE_STEPPER_MOTOR))
        {
            auto sState = std::dynamic_pointer_cast<StepperMotorState>(s.second);
            if (sState && !sState->isConveyor())
            {
                EStepperCalibrationStatus status = sState->getCalibrationStatus();
                if (newStatus < status)
                    newStatus = status;
            }
        }
    }

    _calibration_status = newStatus;
}

// ******************
//  Getters
// ******************

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
 * @brief CanManager::getMotorsStates
 * @return only the joints states
 */
std::vector<std::shared_ptr<JointState>> CanManager::getMotorsStates() const
{
    std::vector<std::shared_ptr<JointState>> states;
    for (const auto &it : _state_map)
    {
        if (EHardwareType::UNKNOWN != it.second->getHardwareType())
        {
            states.push_back(std::dynamic_pointer_cast<JointState>(it.second));
        }
    }

    return states;
}

/**
 * @brief CanManager::getHardwareState
 * @param motor_id
 * @return
 */
std::shared_ptr<common::model::AbstractHardwareState> CanManager::getHardwareState(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) && _state_map.at(motor_id))
        throw std::out_of_range("CanManager::getMotorsState: Unknown motor id");

    return _state_map.at(motor_id);
}

// ********************
//  Private
// ********************

/**
 * @brief CanManager::addHardwareDriver add driver corresponding to a type of hardware
 * @param hardware_type
 */
void CanManager::addHardwareDriver(common::model::EHardwareType hardware_type)
{
    // if not already instanciated
    if (!_driver_map.count(hardware_type))
    {
        switch (hardware_type)
        {
        case common::model::EHardwareType::STEPPER:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<StepperDriver<StepperReg>>(_mcp_can)));
            break;
        case common::model::EHardwareType::FAKE_STEPPER_MOTOR:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<MockStepperDriver>(_fake_data)));
            break;
        default:
            ROS_ERROR("CanManager - Unable to instanciate driver, unknown type");
            break;
        }
    }
}

/**
 * @brief CanManager::readFakeConfig read the config for fake driver.
 */
void CanManager::readFakeConfig(bool use_simu_conveyor)
{
    _fake_data = std::make_shared<FakeCanData>();

    if (_nh.hasParam("fake_params"))
    {
        if (_nh.hasParam("fake_params/steppers"))
        {
            std::string current_ns = "fake_params/steppers/";
            retrieveFakeMotorData(current_ns, _fake_data->stepper_registers);

            int pos_spam{30};
            _nh.getParam("fake_params/position_spam", pos_spam);
            _fake_data->position_spam = static_cast<uint8_t>(pos_spam);
        }

        if (use_simu_conveyor && _nh.hasParam("fake_params/conveyors/"))
        {
            std::string current_ns = "fake_params/conveyors/";
            retrieveFakeMotorData(current_ns, _fake_data->stepper_registers);
        }

        _fake_data->updateFullIdList();
    }
}

}  // namespace can_driver
