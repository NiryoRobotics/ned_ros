/*
    ttl_driver.cpp
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

#include "ttl_driver/ttl_manager.hpp"

// cpp
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// ros
#include "ros/serialization.h"
#include "ros/time.h"

// niryo
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/end_effector_state.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/stepper_motor_state.hpp"
#include "common/model/tool_state.hpp"


#include "dynamixel_sdk/packet_handler.h"
#include "ttl_driver/end_effector_reg.hpp"
#include "ttl_driver/stepper_reg.hpp"

#include "ttl_driver/dxl_driver.hpp"
#include "ttl_driver/end_effector_driver.hpp"
#include "ttl_driver/ned3pro_end_effector_driver.hpp"
#include "ttl_driver/fake_ttl_data.hpp"
#include "ttl_driver/mock_dxl_driver.hpp"
#include "ttl_driver/mock_end_effector_driver.hpp"
#include "ttl_driver/mock_stepper_driver.hpp"
#include "ttl_driver/stepper_driver.hpp"
#include "ttl_driver/ned3pro_stepper_driver.hpp"
#include "ttl_driver/CalibrationStatus.h"

using ::std::ostringstream;
using ::std::set;
using ::std::shared_ptr;
using ::std::string;
using ::std::to_string;
using ::std::vector;

using ::common::model::EHardwareType;
using ::common::model::EndEffectorState;
using ::common::model::EStepperCalibrationStatus;
using ::common::model::HardwareTypeEnum;
using ::common::model::JointState;
using ::common::model::StepperMotorState;
using ::common::model::ConveyorState;

namespace ttl_driver
{
/**
 * @brief TtlManager::TtlManager
 */
TtlManager::TtlManager(ros::NodeHandle &nh) : _nh(nh), _debug_error_message("TtlManager - No connection with TTL motors has been made yet")
{
    ROS_DEBUG("TtlManager - ctor");

    init(nh);

    if (COMM_SUCCESS != setupCommunication())
        ROS_WARN("TtlManager - TTL Communication Failed");
}

/**
 * @brief TtlManager::~TtlManager
 */
TtlManager::~TtlManager()
{
    if (_portHandler)
    {
        _portHandler->clearPort();
        _portHandler->closePort();
    }
}

/**
 * @brief TtlManager::init
 * @param nh
 * @return
 */
bool TtlManager::init(ros::NodeHandle &nh)
{
    // get params from rosparams
    bool use_simu_gripper{false};
    bool use_simu_conveyor{false};

    nh.getParam("bus_params/uart_device_name", _device_name);
    nh.getParam("bus_params/baudrate", _baudrate);
    nh.getParam("led_motor", _led_motor_type_cfg);

    nh.getParam("simulation_mode", _simulation_mode);
    nh.getParam("simu_gripper", use_simu_gripper);
    nh.getParam("simu_conveyor", use_simu_conveyor);

    ROS_DEBUG("TtlManager::init - Dxl : set port name (%s), baudrate(%d)", _device_name.c_str(), _baudrate);
    ROS_DEBUG("TtlManager::init - led motor type config : %s", _led_motor_type_cfg.c_str());

    ROS_DEBUG("TtlManager::init - Simulation mode: %s, simu_gripper: %s, simu_conveyor: %s", _simulation_mode ? "True" : "False", use_simu_gripper ? "True" : "False",
              use_simu_conveyor ? "True" : "False");

    if (!_simulation_mode)
    {
        _portHandler.reset(dynamixel::PortHandler::getPortHandler(_device_name.c_str()));
        _packetHandler.reset(dynamixel::PacketHandler::getPacketHandler(TTL_BUS_PROTOCOL_VERSION));

        // init default ttl driver for common operations between drivers
        _default_ttl_driver = std::make_shared<StepperDriver<StepperReg>>(_portHandler, _packetHandler);
    }
    else
    {
        readFakeConfig(use_simu_gripper, use_simu_conveyor);
        _default_ttl_driver = std::make_shared<MockStepperDriver>(_fake_data);
    }

    _calibration_status_publisher = nh.advertise<ttl_driver::CalibrationStatus>("calibration_status", 1, true);

    return true;
}

/**
 * @brief TtlManager::setupCommunication
 * @return
 */
int TtlManager::setupCommunication()
{
    int ret = COMM_NOT_AVAILABLE;

    ROS_DEBUG("TtlManager::setupCommunication - initializing connection...");

    // fake drivers will always succeed for the communication
    if (_simulation_mode)
    {
        ret = COMM_SUCCESS;
    }
    else
    {
        // Ttl bus setup
        if (_portHandler)
        {
            _debug_error_message.clear();

            // Open port
            if (_portHandler->openPort())
            {
                // Set baudrate
                if (_portHandler->setBaudRate(_baudrate))
                {
                    // wait a bit to be sure the connection is established
                    ros::Duration(0.1).sleep();

                    // clear port
                    _portHandler->clearPort();

                    ret = COMM_SUCCESS;
                }
                else
                {
                    ROS_ERROR("TtlManager::setupCommunication - Failed to set baudrate for Dynamixel bus");
                    _debug_error_message = "TtlManager - Failed to set baudrate for Dynamixel bus";
                    ret = TTL_FAIL_PORT_SET_BAUDRATE;
                }
            }
            else
            {
                ROS_ERROR("TtlManager::setupCommunication - Failed to open Uart port for Dynamixel bus");
                _debug_error_message = "TtlManager - Failed to open Uart port for Dynamixel bus";
                ret = TTL_FAIL_OPEN_PORT;
            }
        }
        else
            ROS_ERROR("TtlManager::setupCommunication - Invalid port handler");
    }

    return ret;
}

/**
 * @brief TtlManager::addHardwareComponent add hardware component like joint, ee, tool... to ttl manager
 * @param state
 */
int TtlManager::addHardwareComponent(std::shared_ptr<common::model::AbstractHardwareState> &&state)  // NOLINT
{
    if (!state)
    {
        return niryo_robot_msgs::CommandStatus::FAILURE;
    }

    EHardwareType hardware_type = state->getHardwareType();
    uint8_t id = state->getId();

    ROS_DEBUG("TtlManager::addHardwareComponent : %s", state->str().c_str());

    addHardwareDriver(hardware_type);

    auto driver_it = _driver_map.find(hardware_type);
    if (driver_it == _driver_map.end())
    {
        ROS_ERROR("TtlManager::addHardwareComponent - No driver found for hardware type %d", static_cast<int>(hardware_type));
        return niryo_robot_msgs::CommandStatus::FAILURE;
    }
    auto driver = driver_it->second;

    // check if the driver is valid for this hardware
    if (state->getStrictModelNumber() && driver->checkModelNumber(id) != COMM_SUCCESS)
    {
        ROS_WARN("TtlManager::addHardwareComponent - Model number check failed for hardware id %d", id);
        return niryo_robot_msgs::CommandStatus::HARDWARE_NOT_SUPPORTED;
    }

    // update firmware version
    std::string version;
    int res = COMM_RX_FAIL;
    for (int tries = 10; tries > 0; tries--)
    {
        res = driver->readFirmwareVersion(id, version);
        if (COMM_SUCCESS == res)
        {
            state->setFirmwareVersion(version);
            break;
        }
        ros::Duration(0.1).sleep();
    }

    if (COMM_SUCCESS != res)
    {
        ROS_WARN("TtlManager::addHardwareComponent : Unable to retrieve firmware version for "
                    "hardware id %d : result = %d",
                    id, res);
    }

    // add state to state map
    _state_map[id] = state;

    // add id to ids_map
    _ids_map[hardware_type].emplace_back(id);

    // add to global lists
    if (common::model::EComponentType::CONVEYOR == state->getComponentType())
    {
        if (std::find(_conveyor_list.begin(), _conveyor_list.end(), id) == _conveyor_list.end())
            _conveyor_list.emplace_back(id);
    }

    // Reset the calibration if we add a new joint and it is not a simulated one
    if (common::model::EComponentType::JOINT == state->getComponentType())
    {
        if (!(common::model::EHardwareType::FAKE_STEPPER_MOTOR == hardware_type
        || common::model::EHardwareType::FAKE_DXL_MOTOR == hardware_type))
        {
            _calibration_status = EStepperCalibrationStatus::UNINITIALIZED;
        }
    }

    setLeds(_led_state);
    return niryo_robot_msgs::CommandStatus::SUCCESS;
}

/**
 * @brief TtlManager::removeHardwareComponent
 * @param id
 */
void TtlManager::removeHardwareComponent(uint8_t id)
{
    ROS_DEBUG("TtlManager::removeMotor - Remove motor id: %d", id);

    if (_state_map.count(id) && _state_map.at(id))
    {
        EHardwareType type = _state_map.at(id)->getHardwareType();

        // std::remove to remove hypothetic duplicates too
        if (_ids_map.count(type))
        {
            auto &ids = _ids_map.at(type);
            ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
            if (ids.empty())
            {
                _ids_map.erase(type);
            }
        }

        _state_map.erase(id);
    }
    // remove id from conveyor list if they contains id
    _conveyor_list.erase(std::remove(_conveyor_list.begin(), _conveyor_list.end(), id), _conveyor_list.end());

    _removed_motor_id_list.erase(std::remove(_removed_motor_id_list.begin(), _removed_motor_id_list.end(), id), _removed_motor_id_list.end());
}

/**
 * @brief TtlManager::isMotorType
 * @param type
 * @return true
 * @return false
 */
bool TtlManager::isMotorType(EHardwareType type)
{
    // All motors have value under 7 (check in EHardwareType)
    return (static_cast<int>(type) <= 7);
}

// ****************
//  commands
// ****************

/**
 * @brief TtlManager::changeId
 * @param motor_type
 * @param old_id
 * @param new_id
 * @return
 */
int TtlManager::changeId(EHardwareType motor_type, uint8_t old_id, uint8_t new_id)
{
    int ret = COMM_TX_FAIL;

    if (old_id == new_id)
    {
        ret = COMM_SUCCESS;
    }
    else if (_driver_map.count(motor_type))
    {
        auto driver = std::dynamic_pointer_cast<AbstractMotorDriver>(_driver_map.at(motor_type));

        if (driver)
        {
            ret = driver->changeId(old_id, new_id);
            if (COMM_SUCCESS == ret)
            {
                // update all maps
                auto i_state = _state_map.find(old_id);
                // update all maps
                if (i_state != _state_map.end())
                {
                    // insert new_id in map, move i_state->second to its new place
                    std::swap(_state_map[new_id], i_state->second);
                    // update all maps
                    _state_map.erase(i_state);

                    assert(_state_map.at(new_id));

                    // update conveyor list if needed
                    if (common::model::EComponentType::CONVEYOR == _state_map.at(new_id)->getComponentType())
                    {
                        // change old id into new id in vector
                        auto iter = std::find(_conveyor_list.begin(), _conveyor_list.end(), old_id);
                        if (iter != _conveyor_list.end())
                            *iter = new_id;
                    }
                }
                if (_ids_map.count(motor_type))
                {
                    // update all maps
                    _ids_map.at(motor_type).erase(std::remove(_ids_map.at(motor_type).begin(), _ids_map.at(motor_type).end(), old_id), _ids_map.at(motor_type).end());

                    // update all maps
                    _ids_map.at(motor_type).emplace_back(new_id);
                }
            }
        }
    }

    return ret;
}

/**
 * @brief TtlManager::scanAndCheck
 * @return
 */
int TtlManager::scanAndCheck()
{
    ROS_DEBUG("TtlManager::scanAndCheck");
    int result = COMM_PORT_BUSY;

    _is_connection_ok = false;

    // 1. retrieve list of connected motors
    _all_ids_connected.clear();
    for (int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
    {
        result = getAllIdsOnBus(_all_ids_connected);
        ROS_DEBUG_COND(COMM_SUCCESS != result, "TtlManager::scanAndCheck status: %d (counter: %d)", result, counter);
        ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
    }

    if (COMM_SUCCESS == result)
    {
        // 2. update list of removed ids and update corresponding states
        _removed_motor_id_list.clear();
        std::string error_motors_message;
        for (auto &istate : _state_map)
        {
            if (istate.second)
            {
                uint8_t id = istate.first;
                auto it = find(_all_ids_connected.begin(), _all_ids_connected.end(), id);
                // not found
                if (it == _all_ids_connected.end())
                {
                    _removed_motor_id_list.emplace_back(id);
                    error_motors_message += " " + to_string(id);
                    istate.second->setConnectionStatus(true);
                }
                else
                {
                    istate.second->setConnectionStatus(false);
                }
            }
        }

        if (_removed_motor_id_list.empty())
        {
            _is_connection_ok = true;
            _debug_error_message.clear();
            result = TTL_SCAN_OK;
        }
        else
        {
            _debug_error_message = "Motor(s):" + error_motors_message + " do not seem to be connected";
            result = TTL_SCAN_MISSING_MOTOR;
        }
    }
    else
    {
        _debug_error_message = "TtlManager - Failed to scan motors, physical bus is too busy. Will retry...";
        ROS_WARN_THROTTLE(1, "TtlManager::scanAndCheck - Failed to scan motors, physical bus is too busy");
    }

    return result;
}

/**
 * @brief TtlManager::ping
 * @param id
 * @return
 * The ping method is identical to all drivers, we can just use the first one
 * (same behaviour in getAllIdsOnBus with scan method)
 */
bool TtlManager::ping(uint8_t id)
{
    int result = false;

    if (_default_ttl_driver)
    {
        if (COMM_SUCCESS == _default_ttl_driver->ping(id))
            result = true;
    }

    return result;
}

/**
 * @brief TtlManager::rebootHardware
 * @param hw_id
 * @return
 */
int TtlManager::rebootHardware(uint8_t hw_id)
{
    int return_value = COMM_TX_FAIL;

    if (_state_map.count(hw_id) != 0 && _state_map.at(hw_id))
    {
        EHardwareType type = _state_map.at(hw_id)->getHardwareType();
        ROS_DEBUG("TtlManager::rebootHardware - Reboot hardware with ID: %d", hw_id);
        if (_driver_map.count(type) && _driver_map.at(type))
        {
            return_value = _driver_map.at(type)->reboot(hw_id);
            if (COMM_SUCCESS == return_value)
            {
                std::string version;
                int res = COMM_RX_FAIL;
                for (int tries = 10; tries > 0; tries--)
                {
                    res = _driver_map.at(type)->readFirmwareVersion(hw_id, version);
                    if (COMM_SUCCESS == res)
                    {
                        _state_map.at(hw_id)->setFirmwareVersion(version);
                        break;
                    }
                    ros::Duration(0.1).sleep();
                }

                if (COMM_SUCCESS != res)
                {
                    ROS_WARN("TtlManager::addHardwareComponent : Unable to retrieve firmware version for "
                             "hardware id %d : result = %d",
                             hw_id, res);
                }
            }
            ROS_WARN_COND(COMM_SUCCESS != return_value, "TtlManager::rebootHardware - Failed to reboot hardware: %d", return_value);
        }
    }

    return return_value;
}

/**
 * @brief TtlManager::resetTorques
 * @return
 */
void TtlManager::resetTorques()
{
    for (auto const &it : _driver_map)
    {
        auto hw_type = it.first;
        auto driver = std::dynamic_pointer_cast<ttl_driver::AbstractMotorDriver>(it.second);

        if (driver && _ids_map.count(hw_type) && !_ids_map.at(hw_type).empty())
        {
            // we retrieve all the associated id for the type of the current driver
            vector<uint8_t> ids_list = _ids_map.at(hw_type);

            auto jStates = getMotorsStates();

            for (size_t i = 0; i < ids_list.size(); ++i)
            {
                auto jState_it = std::find_if(jStates.begin(), jStates.end(), [i](const std::shared_ptr<JointState> &jState){
                    return i == jState->getId();
                });
                if (jState_it == jStates.end())
                {
                    continue;
                }
                auto jState = *jState_it;
                ROS_DEBUG("TtlManager::resetTorques - Torque ON on stepper ID: %d", static_cast<int>(ids_list.at(i)));
                driver->writeTorquePercentage(ids_list.at(i), jState->getTorquePercentage());
            }  // for ids_list
        }
    }  // for _driver_map
}

// ******************
//  Read operations
// ******************

/**
 * @brief TtlManager::getPosition
 * @param motor_state
 * @return
 */
uint32_t TtlManager::getPosition(const JointState &motor_state)
{
    uint32_t position = 0;
    EHardwareType hardware_type = motor_state.getHardwareType();
    if (_driver_map.count(hardware_type))
    {
        auto driver = std::dynamic_pointer_cast<AbstractMotorDriver>(_driver_map.at(hardware_type));
        if (driver)
        {
            for (_hw_fail_counter_read = 0; _hw_fail_counter_read < MAX_HW_FAILURE; ++_hw_fail_counter_read)
            {
                if (COMM_SUCCESS == driver->readPosition(motor_state.getId(), position))
                {
                    _hw_fail_counter_read = 0;
                    break;
                }
            }
        }

        if (0 < _hw_fail_counter_read)
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::getPosition - motor connection problem - Failed to read from bus");
            _debug_error_message = "TtlManager - Connection problem with Bus.";
            _hw_fail_counter_read = 0;
            _is_connection_ok = false;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::getPosition - Driver not found for requested motor id");
        _debug_error_message = "TtlManager::getPosition - Driver not found for requested motor id";
    }
    return position;
}

bool TtlManager::readHomingAbsPosition()
{
    EHardwareType hw_type(EHardwareType::STEPPER);
    auto driver = std::dynamic_pointer_cast<ttl_driver::StepperDriver<ttl_driver::StepperReg>>(_driver_map[hw_type]);
    if (driver && _ids_map.count(hw_type) && !_ids_map.at(hw_type).empty())
    {
        // we retrieve all the associated id for the type of the current driver
        vector<uint8_t> ids_list = _ids_map.at(hw_type);

        // we retrieve all the associated id for the type of the current driver
        vector<uint32_t> homing_abs_position_list;

        // retrieve joint status
        int res = driver->syncReadHomingAbsPosition(ids_list, homing_abs_position_list);
        if (COMM_SUCCESS == res)
        {
            if (ids_list.size() == homing_abs_position_list.size())
            {
                // set motors states accordingly
                for (size_t i = 0; i < ids_list.size(); ++i)
                {
                    uint8_t id = ids_list.at(i);

                    if (_state_map.count(id))
                    {
                        auto state = std::dynamic_pointer_cast<common::model::StepperMotorState>(_state_map.at(id));
                        if (state)
                        {
                            state->setHomingAbsPosition(static_cast<uint32_t>(homing_abs_position_list.at(i)));
                        }
                        else
                        {
                            ROS_ERROR("TtlManager::readHomingAbsPosition - null pointer");
                            return false;
                        }
                    }
                    else
                    {
                        ROS_ERROR("TtlManager::readHomingAbsPosition - No hardware state assossiated to ID: %d", static_cast<int>(id));
                        return false;
                    }
                }
            }
            else
            {
                ROS_ERROR("TtlManager::readHomingAbsPosition - size of requested id %d mismatch size of retrieved homing position %d", static_cast<int>(ids_list.size()),
                          static_cast<int>(homing_abs_position_list.size()));
                return false;
            }
        }
        else
        {
            ROS_ERROR("TtlManager::readHomingAbsPosition - communication error: %d", static_cast<int>(res));
            return false;
        }
    }
    else
    {
        ROS_ERROR("TtlManager::readHomingAbsPosition - null pointer or no hardware type in map %d or empty vector of ids: %d", static_cast<int>(_ids_map.count(hw_type)),
                  static_cast<int>(_ids_map.at(hw_type).empty()));
        return false;
    }

    return true;
}

/**
 * @brief TtlManager::readJointsStatus
 * @return
 */
bool TtlManager::readJointsStatus()
{
    uint8_t hw_errors_increment = 0;

    // syncread position for all motors.
    // for ned and one -> we need at least one xl430 and one xl320 drivers as they are different

    for (auto const &it : _driver_map)
    {
        auto hw_type = it.first;
        auto driver = std::dynamic_pointer_cast<ttl_driver::AbstractMotorDriver>(it.second);

        if (!driver)
        {
            continue;
        }
        auto id_list_it = _ids_map.find(hw_type);
        if (id_list_it == _ids_map.end())
        {
            continue;
        }
        auto ids_list = id_list_it->second;

        if (ids_list.empty())
        {
            continue;
        }

        // we retrieve all the associated id for the type of the current driver
        _position_list.clear();

        // retrieve joint status
        int res = driver->syncReadPosition(ids_list, _position_list);
        if (COMM_SUCCESS == res)
        {
            if (ids_list.size() == _position_list.size())
            {
                // set motors states accordingly
                for (size_t i = 0; i < ids_list.size(); ++i)
                {
                    uint8_t id = ids_list.at(i);

                    if (_state_map.count(id))
                    {
                        auto state = std::dynamic_pointer_cast<common::model::AbstractMotorState>(_state_map.at(id));
                        if (state)
                        {
                            state->setPosition(static_cast<int>((_position_list.at(i))));
                        }
                    }
                }
            }
            else
            {
                // warn to avoid sound and light error on high level (error on ROS_ERROR)
                ROS_WARN("TtlManager::readJointStatus : Fail to sync read joint state - "
                            "vector mismatch (id_list size %d, position_list size %d)",
                            static_cast<int>(ids_list.size()), static_cast<int>(_position_list.size()));
                hw_errors_increment++;
            }
        }
        else
        {
            // debug to avoid sound and light error on high level (error on ROS_ERROR)
            // also for Ned which has much more errors on XL320 motor
            ROS_DEBUG("TtlManager::readJointStatus : Fail to sync read joint state - "
                        "driver fail to syncReadPosition");
            hw_errors_increment++;
        }
    }  // for driver_map

    // check collision by END_EFFECTOR
    if (_isRealCollision)
    {
        readCollisionStatus();
    }
    else
    {
        _collision_status = false;
        // check collision by END_EFFECTOR
        if (_isRealCollision)
        {
            checkCollision();
        }
        else
        {
            _collision_status = false;
        }
    }

    ROS_DEBUG_THROTTLE(2, "_hw_fail_counter_read, hw_errors_increment: %d, %d", _hw_fail_counter_read, hw_errors_increment);

    // we reset the global error variable only if no errors
    if (0 == hw_errors_increment)
    {
        _hw_fail_counter_read = 0;
    }
    else
    {
        _hw_fail_counter_read += hw_errors_increment;
    }

    return (0 == hw_errors_increment);
}

/**
 * @brief TtlManager::readEndEffectorStatus
 * @return
 */
bool TtlManager::readEndEffectorStatus()
{
    bool res = false;

    // hardware_version is not available within this class to know whether there is a ned2/ned3pro eef
    // however if hardware_type == ned2 _driver_map will contain EHardwareType::END_EFFECTOR
    // and if hardware_type == ned3pro _driver_map will contain EHardwareType::NED3PRO_END_EFFECTOR
    EHardwareType ee_type;

    if (_simulation_mode)
        ee_type = EHardwareType::FAKE_END_EFFECTOR;
    else if (_driver_map.count(EHardwareType::END_EFFECTOR))
        ee_type = EHardwareType::END_EFFECTOR;
    else if (_driver_map.count(EHardwareType::NED3PRO_END_EFFECTOR))
        ee_type = EHardwareType::NED3PRO_END_EFFECTOR;


    if (_driver_map.count(ee_type))
    {
        unsigned int hw_errors_increment = 0;

        auto driver = std::dynamic_pointer_cast<AbstractEndEffectorDriver>(_driver_map.at(ee_type));
        if (driver)
        {
            if (_ids_map.count(ee_type) && !_ids_map.at(ee_type).empty())
            {
                uint8_t id = _ids_map.at(ee_type).front();

                if (_state_map.count(id))
                {
                    // we retrieve the associated id for the end effector
                    auto state = std::dynamic_pointer_cast<EndEffectorState>(_state_map[id]);

                    if (state)
                    {
                        vector<common::model::EActionType> action_list;

                        // **********  buttons
                        // get action of free driver button, save pos button, custom button
                        if (COMM_SUCCESS == driver->syncReadButtonsStatus(id, action_list))
                        {
                            for (uint8_t i = 0; i < action_list.size(); i++)
                            {
                                state->setButtonStatus(i, action_list.at(i));
                                // In case free driver button, it we hold this button, normally, because of the small threshold of collision detection
                                // this action make EE confuse that it is a collision. That's why when we hold buttons, we need to deactivate the detection of collision.
                                if (action_list.at(i) != common::model::EActionType::NO_ACTION)
                                {
                                    _isRealCollision = false;
                                }
                                else if (!_isRealCollision)
                                {
                                    // when previous action is not no_action => need to wait a short period to make sure no collision detected
                                    // Note, we need to read one time the status of collision just after releasing button to reset the status.
                                    _isRealCollision = true;
                                    _isWrongAction = true;
                                    _last_collision_detection_activating = ros::Time::now().toSec();
                                }
                            }
                        }
                        else
                        {
                            hw_errors_increment++;
                        }

                        // **********  digital data
                        bool digital_data{};
                        if (COMM_SUCCESS == driver->readDigitalInput(id, digital_data))
                        {
                            state->setDigitalIn(digital_data);
                        }
                        else
                        {
                            hw_errors_increment++;
                        }
                    }  // if (state)
                }
            }  // if (_ids_map.count(EHardwareType::END_EFFECTOR))
        }      // if (driver)

        // we reset the global error variable only if no errors
        if (0 == hw_errors_increment)
        {
            _end_effector_fail_counter_read = 0;
            _debug_error_message.clear();

            res = true;
        }
        else
        {
            ROS_DEBUG_COND(_end_effector_fail_counter_read > 10, "TtlManager::readEndEffectorStatus: nb error > 10 :  %d", _end_effector_fail_counter_read);
            _end_effector_fail_counter_read += hw_errors_increment;
        }

        if (_end_effector_fail_counter_read > MAX_READ_EE_FAILURE)
        {
            ROS_ERROR("TtlManager::readEndEffectorStatus - motor connection problem - Failed to read from bus (hw_fail_counter_read : %d)", _end_effector_fail_counter_read);
            _end_effector_fail_counter_read = 0;
            _debug_error_message = "TtlManager - Connection problem with physical Bus.";
        }
    }

    return res;
}

/**
 * @brief TtlManager::checkCollision
 * @return false if read failed. Careful, there can be lots of errors as the TTL bus is not perfect
 * better not use the return status
 */
bool TtlManager::checkCollision()
{
    bool res = false;

    // hardware_version is not available within this class to know whether there is a ned2/ned3pro eef
    // however if hardware_type == ned2 _driver_map will contain EHardwareType::END_EFFECTOR
    // and if hardware_type == ned3pro _driver_map will contain EHardwareType::NED3PRO_END_EFFECTOR
    EHardwareType ee_type;

    if (_simulation_mode)
        ee_type = EHardwareType::FAKE_END_EFFECTOR;
    else if (_driver_map.count(EHardwareType::END_EFFECTOR))
        ee_type = EHardwareType::END_EFFECTOR;
    else if (_driver_map.count(EHardwareType::NED3PRO_END_EFFECTOR))
        ee_type = EHardwareType::NED3PRO_END_EFFECTOR;

    if (_driver_map.count(ee_type))
    {
        unsigned int hw_errors_increment = 0;

        auto driver = std::dynamic_pointer_cast<AbstractEndEffectorDriver>(_driver_map.at(ee_type));
        if (driver)
        {
            if (_ids_map.count(ee_type) && !_ids_map.at(ee_type).empty())
            {
                uint8_t id = _ids_map.at(ee_type).front();

                // **********  collision
                // not accept other status of collistion in 1 second if it detected a collision
                if (0.0 == _last_collision_detection_activating)
                {
                    bool last_statut = _collision_status;
                    if (COMM_SUCCESS == driver->readCollisionStatus(id, _collision_status))
                    {
                        if (last_statut == _collision_status && _collision_status)
                        {
                            // if we have a collision, we will publish the statut collision only once
                            // This avoid that the delay in reading statut influent to next movement.
                            _collision_status = false;
                        }
                        else if (_collision_status)
                        {
                            if (_isWrongAction)
                            {
                                // if an action did a wrong detection of collision, we need to read once to reset the status
                                _isWrongAction = false;
                                _collision_status = false;
                            }
                            else
                                _last_collision_detection_activating = ros::Time::now().toSec();
                        }
                    }
                    else
                    {
                        hw_errors_increment++;
                    }
                }
                else if (ros::Time::now().toSec() - _last_collision_detection_activating >= 1.0)
                {
                    _last_collision_detection_activating = 0.0;
                }
            }  // if (_ids_map.count(EHardwareType::END_EFFECTOR))
        }      // if (driver)

        // we reset the global error variable only if no errors
        if (0 == hw_errors_increment)
        {
            _end_effector_fail_counter_read = 0;
            res = true;
        }
        else
        {
            ROS_DEBUG_COND(_end_effector_fail_counter_read > 10, "TtlManager::checkCollision: nb error > 10 :  %d", _end_effector_fail_counter_read);
            _end_effector_fail_counter_read += hw_errors_increment;
        }
    }
    return res;
}

/**
 * @brief TtlManager::readCollisionStatus
 * @return
 */
bool TtlManager::readCollisionStatus()
{
    bool res = false;

    // hardware_version is not available within this class to know whether there is a ned2/ned3pro eef
    // however if hardware_type == ned2 _driver_map will contain EHardwareType::END_EFFECTOR
    // and if hardware_type == ned3pro _driver_map will contain EHardwareType::NED3PRO_END_EFFECTOR
    EHardwareType ee_type;

    if (_simulation_mode)
        ee_type = EHardwareType::FAKE_END_EFFECTOR;
    else if (_driver_map.count(EHardwareType::END_EFFECTOR))
        ee_type = EHardwareType::END_EFFECTOR;
    else if (_driver_map.count(EHardwareType::NED3PRO_END_EFFECTOR))
        ee_type = EHardwareType::NED3PRO_END_EFFECTOR;

    if (_driver_map.count(ee_type))
    {
        auto driver = std::dynamic_pointer_cast<AbstractEndEffectorDriver>(_driver_map.at(ee_type));

        if (driver)
        {
            if (_ids_map.count(ee_type) && !_ids_map.at(ee_type).empty())
            {
                uint8_t id = _ids_map.at(ee_type).front();

                // **********  collision
                // don't accept other status of collistion in 1 second if it detected a collision
                if (0.0 == _last_collision_detection_activating)
                {
                    if (COMM_SUCCESS == driver->readCollisionStatus(id, _collision_status))
                    {
                        res = true;

                        if (_collision_status)
                        {
                            if (_isWrongAction)
                            {
                                // if an action did a wrong detection of collision, we need to read once to reset the status
                                _isWrongAction = false;
                                _collision_status = false;
                            }
                            else
                            {
                                _last_collision_detection_activating = ros::Time::now().toSec();
                            }
                        }
                    }
                    else
                    {
                        _end_effector_fail_counter_read++;
                    }
                }
                else if (ros::Time::now().toSec() - _last_collision_detection_activating >= 1.0)
                {
                    _last_collision_detection_activating = 0.0;
                }
            }
        }
    }

    return res;
}

/**
 * @brief TtlManager::readHardwareStatus
 */
bool TtlManager::readHardwareStatus()
{
    bool res = false;

    unsigned int hw_errors_increment = 0;

    // take all hw status dedicated drivers
    for (auto const &it : _driver_map)
    {
        auto type = it.first;
        auto driver = it.second;

        if (driver && _ids_map.count(type) && !_ids_map.at(type).empty())
        {
            // we retrieve all the associated id for the type of the current driver
            vector<uint8_t> ids_list = _ids_map.at(type);

            // 1. syncread for all motors
            // **********  voltage and Temperature
            vector<std::pair<double, uint8_t>> hw_data_list;

            if (COMM_SUCCESS != driver->syncReadHwStatus(ids_list, hw_data_list))
            {
                // this operation can fail, it is normal, so no error message
                hw_errors_increment++;
            }
            else if (ids_list.size() != hw_data_list.size())
            {
                // however, if we have a mismatch here, it is not normal
                ROS_ERROR("TtlManager::readHardwareStatusOptimized : syncReadHwStatus failed - "
                          "vector mistmatch (id_list size %d, hw_data_list size %d)",
                          static_cast<int>(ids_list.size()), static_cast<int>(hw_data_list.size()));

                hw_errors_increment++;
            }

            // **********  error state
            vector<uint8_t> hw_error_status_list;

            if (COMM_SUCCESS != driver->syncReadHwErrorStatus(ids_list, hw_error_status_list))
            {
                hw_errors_increment++;
            }
            else if (ids_list.size() != hw_error_status_list.size())
            {
                ROS_ERROR("TtlManager::readHardwareStatus : syncReadTemperature failed - "
                          "vector mistmatch (id_list size %d, hw_status_list size %d)",
                          static_cast<int>(ids_list.size()), static_cast<int>(hw_error_status_list.size()));

                hw_errors_increment++;
            }

            // 2. set motors states accordingly
            for (size_t i = 0; i < ids_list.size(); ++i)
            {
                uint8_t id = ids_list.at(i);

                if (_state_map.count(id) && _state_map.at(id))
                {
                    auto state = _state_map.at(id);

                    // **************  temperature and voltage
                    if (hw_data_list.size() > i)
                    {
                        double voltage = (hw_data_list.at(i)).first;
                        uint8_t temperature = (hw_data_list.at(i)).second;

                        state->setTemperature(temperature);
                        state->setRawVoltage(voltage);
                    }

                    // **********  error state
                    if (hw_error_status_list.size() > i)
                    {
                        state->setHardwareError(hw_error_status_list.at(i));
                    }

                    // interpret any error code into message (even if not retrieved now)
                    string hardware_message = driver->interpretErrorState(state->getHardwareError());
                    state->setHardwareError(hardware_message);
                }
            }  // for ids_list
        }      // if driver
    }          // for (auto it : _hw_status_driver_map)

    // **********  steppers related informations (conveyor and calibration)
    hw_errors_increment += readSteppersStatus();

    // we reset the global error variable only if no errors
    if (0 == hw_errors_increment)
    {
        _hw_fail_counter_read = 0;
        _debug_error_message.clear();

        res = true;
    }
    else
    {
        _hw_fail_counter_read += hw_errors_increment;
    }

    // if too much errors, disconnect
    if (_hw_fail_counter_read > MAX_HW_FAILURE)
    {
        ROS_ERROR_THROTTLE(1,
                           "TtlManager::readHardwareStatus - motor connection problem - "
                           "Failed to read from bus (hw_fail_counter_read : %d)",
                           _hw_fail_counter_read);
        _hw_fail_counter_read = 0;
        res = false;
        _is_connection_ok = false;
        _debug_error_message = "TtlManager - Connection problem with physical Bus.";
    }

    return res;
}

/**
 * @brief TtlManager::readCalibrationStatus : reads specific steppers related information (ned2 only)
 * @return
 */
uint8_t TtlManager::readSteppersStatus()
{
    uint8_t hw_errors_increment = 0;

    // take all hw status dedicated drivers
    auto [stepper_joint_driver, hw_type] = getJointsStepperDriver();
    if (stepper_joint_driver)
    {
        if (hw_type == EHardwareType::STEPPER || hw_type == EHardwareType::FAKE_STEPPER_MOTOR )
        {
            // 1. read calibration status if needed

            // we want to check calibration (done at startup and when calibration is started)
            if (CalibrationMachineState::State::IDLE != _calib_machine_state.status() && _ids_map.count(hw_type))
            {
                // When calibration is running, sometimes at an unexpected position, calibration make EE thinks that there is a collision
                // Have to disable the feature collision detection in this period
                _isRealCollision = false;

                vector<uint8_t> id_list = _ids_map.at(hw_type);
                vector<uint8_t> stepper_id_list;
                std::copy_if(id_list.begin(), id_list.end(), std::back_inserter(stepper_id_list),
                            [this](uint8_t id) { return _state_map[id] && _state_map.at(id)->getComponentType() != common::model::EComponentType::CONVEYOR; });

                /* Truth Table
                * still_in_progress | state | new state
                *        0          | IDLE  =  IDLE
                *        0          | START =  START
                *        0          | PROG  >  UPDAT
                *        0          | UPDAT R  IDLE
                *        1          | IDLE  =  IDLE
                *        1          | START >  PROG
                *        1          | PROG  =  PROG
                *        1          | UPDAT =  UPDAT
                */

                // ***********  calibration status, only if initialized
                std::vector<uint8_t> homing_status_list;
                if (COMM_SUCCESS == stepper_joint_driver->syncReadHomingStatus(stepper_id_list, homing_status_list))
                {
                    if (stepper_id_list.size() == homing_status_list.size())
                    {
                        // max status need to be kept not converted into EStepperCalibrationStatus because max status is "in progress" in the enum
                        int max_status = -1;

                        // debug only
                        std::ostringstream ss_debug;
                        ss_debug << "homing status : ";

                        bool still_in_progress = false;

                        // set states accordingly
                        for (size_t i = 0; i < homing_status_list.size(); ++i)
                        {
                            uint8_t id = stepper_id_list.at(i);
                            ss_debug << static_cast<int>(homing_status_list.at(i)) << ", ";

                            if (_state_map.count(id))
                            {
                                EStepperCalibrationStatus status = stepper_joint_driver->interpretHomingData(homing_status_list.at(i));

                                // set status in state
                                auto stepperState = std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(id));
                                if (stepperState && !stepperState->isConveyor())
                                {
                                    stepperState->setCalibration(status, 1);

                                    // get max status of all motors (to retrieve potential errors)
                                    // carefull to those possible cases :
                                    // 1, 1, 1
                                    // 1, 2, 1
                                    // 0, 2, 2
                                    // 2, 0, 2

                                    // if 0, uninitialized, else, take max
                                    // we need to keep the status unconverted to have the correct order
                                    if (0 != max_status && homing_status_list.at(i) > max_status)
                                        max_status = homing_status_list.at(i);

                                    // if one status is in progress or uinitialized, we are really in progress
                                    if ((0 == homing_status_list.at(i)) || EStepperCalibrationStatus::IN_PROGRESS == status)
                                    {
                                        still_in_progress = true;
                                    }
                                }
                            }
                        }  // for homing_status_list

                        ss_debug << " => max_status: " << static_cast<int>(max_status);

                        ROS_DEBUG_THROTTLE(2.0, "TtlManager::readCalibrationStatus : %s", ss_debug.str().c_str());

                        // see truth table above
                        // timeout is here to prevent being stuck here if retrying calibration when already at the butee (then the system has no time to switch to "in progress"
                        // before "ok" or "error"
                        if ((!still_in_progress && CalibrationMachineState::State::IN_PROGRESS == _calib_machine_state.status()) ||
                            (still_in_progress && CalibrationMachineState::State::STARTING == _calib_machine_state.status()) ||
                            (!still_in_progress && _calib_machine_state.isTimeout()))
                        {
                            _calib_machine_state.next();
                        }

                        // see truth table above
                        if (CalibrationMachineState::State::UPDATING == _calib_machine_state.status())
                        {
                            _calibration_status = stepper_joint_driver->interpretHomingData(static_cast<uint8_t>(max_status));
                            _calib_machine_state.reset();

                            // In this CalibrationMachineState, the calibration is considered as finished => can activate collision detection here
                            // we need to wait a short period to make sure no collision detected
                            // calibration make a wrong collision, so we have to read the collision status once time to reset it.
                            _isWrongAction = true;
                            _isRealCollision = true;
                            _last_collision_detection_activating = ros::Time::now().toSec();
                        }
                    }
                    else
                    {
                        ROS_ERROR("TtlManager::readCalibrationStatus : syncReadHomingStatus failed - "
                                "vector mistmatch (id_list size %d, homing_status_list size %d)",
                                static_cast<int>(stepper_id_list.size()), static_cast<int>(homing_status_list.size()));

                        hw_errors_increment++;
                    }
                }
                else
                {
                    hw_errors_increment++;
                }
            }  // if (_driver_map.count(hw_type) && _driver_map.at(hw_type))
        }
        else if (hw_type == EHardwareType::NED3PRO_STEPPER)
        {
            hw_errors_increment = readNed3ProSteppersStatus();
        }

         // 2. read conveyors states if has
        for (auto &conveyor_state : getConveyorsStates())
        {
            auto conveyor_id = conveyor_state->getId();
            auto conveyor_hw_type = conveyor_state->getHardwareType();
            auto conveyor_driver = std::dynamic_pointer_cast<ttl_driver::AbstractStepperDriver>(_driver_map[conveyor_hw_type]);
            int32_t conveyor_speed_percent = 0;
            int32_t direction = 0;
            if (COMM_SUCCESS == conveyor_driver->readConveyorVelocity(conveyor_id, conveyor_speed_percent, direction))
            {
                conveyor_state->setGoalDirection(direction);
                conveyor_state->setSpeed(conveyor_speed_percent);
                conveyor_state->setState(conveyor_speed_percent);
            }
            else
            {
                hw_errors_increment++;
            }
        }
    }

    ROS_DEBUG_THROTTLE(2.0, "TtlManager::readCalibrationStatus: _calibration_status: %s", common::model::StepperCalibrationStatusEnum(_calibration_status).toString().c_str());
    ROS_DEBUG_THROTTLE(2.0, "TtlManager::readCalibrationStatus: _calib_machine_state: %d", static_cast<int>(_calib_machine_state.status()));

    return hw_errors_increment;
}

uint8_t TtlManager::readNed3ProSteppersStatus()
{
  uint8_t hw_errors_increment = 0;

  EHardwareType hw_type = EHardwareType::NED3PRO_STEPPER;

  // 1. read calibration status if needed

  // we want to check calibration (done at startup and when calibration is started)
  if (CalibrationMachineState::State::IDLE != _calib_machine_state.status() && _ids_map.count(hw_type))
  {
    // When calibration is running, sometimes at an unexpected position, calibration make EE thinks that
    // there is a collision Have to disable the feature collision detection in this period
    _isRealCollision = false;

    vector<uint8_t> id_list = _ids_map.at(hw_type);
    vector<uint8_t> stepper_id_list;
    std::copy_if(id_list.begin(), id_list.end(), std::back_inserter(stepper_id_list), [this](uint8_t id) {
      return _state_map[id] &&
             _state_map.at(id)->getComponentType() != common::model::EComponentType::CONVEYOR;
    });

    // ***********  calibration status, only if initialized
    auto [stepper_joint_driver, hw_type] = getJointsStepperDriver();
    std::vector<uint8_t> homing_status_list;
    if (stepper_joint_driver &&
        COMM_SUCCESS == stepper_joint_driver->syncReadHomingStatus(stepper_id_list, homing_status_list))
    {
      if (stepper_id_list.size() == homing_status_list.size())
      {
        // TODO(i.ambit) publish calibration message
        ttl_driver::CalibrationStatus calibration_status_msg;

        EStepperCalibrationStatus highest_priority_status = EStepperCalibrationStatus::UNINITIALIZED;
        // set states accordingly
        for (size_t i = 0; i < homing_status_list.size(); ++i)
        {
          uint8_t id = stepper_id_list.at(i);

          if (_state_map.count(id))
          {
            // set status in state
            auto stepperState = std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(id));
            if (stepperState && !stepperState->isConveyor())
            {
              auto status = stepper_joint_driver->interpretHomingData(homing_status_list.at(i));
              stepperState->setCalibration(status, 1);

              if (status > highest_priority_status)
                highest_priority_status = status;

              calibration_status_msg.ids.push_back(id);

              switch (status)
              {
              case EStepperCalibrationStatus::IN_PROGRESS:
                calibration_status_msg.status.push_back(ttl_driver::CalibrationStatus::CALIBRATING);
                break;
              case EStepperCalibrationStatus::OK:
                calibration_status_msg.status.push_back(ttl_driver::CalibrationStatus::CALIBRATED);
                break;
              case EStepperCalibrationStatus::FAIL:
                calibration_status_msg.status.push_back(ttl_driver::CalibrationStatus::CALIBRATION_ERROR);
                break;
              default:
                calibration_status_msg.status.push_back(ttl_driver::CalibrationStatus::CALIBRATION_ERROR);
                break;
              }

              _calibration_status_publisher.publish(calibration_status_msg);
            }
          }
        }  // for homing_status_list
        _calibration_status = highest_priority_status;
      }
      else
      {
        ROS_ERROR("TtlManager::readCalibrationStatus : syncReadHomingStatus failed - "
                  "vector mistmatch (id_list size %d, homing_status_list size %d)",
                  static_cast<int>(stepper_id_list.size()), static_cast<int>(homing_status_list.size()));

        hw_errors_increment++;
      }
    }
    else
    {
      hw_errors_increment++;
    }
  }  // if (_driver_map.count(hw_type) && _driver_map.at(hw_type))

  return hw_errors_increment;
}

/**
 * @brief TtlManager::getAllIdsOnDxlBus
 * @param id_list
 * @return
 * The scan method is identical to all drivers, we can just use the first one
 * (same behaviour in ping with ping method)
 */
int TtlManager::getAllIdsOnBus(vector<uint8_t> &id_list)
{
    int result = COMM_RX_FAIL;

    // 1. Get all ids from ttl bus. We can use any driver for that
    if (_default_ttl_driver)
    {
        vector<uint8_t> l_idList;
        result = _default_ttl_driver->scan(l_idList);
        id_list.insert(id_list.end(), l_idList.begin(), l_idList.end());

        string ids_str;
        for (auto const &id : l_idList)
            ids_str += to_string(id) + " ";

        ROS_DEBUG_THROTTLE(1, "TtlManager::getAllIdsOnTtlBus - Found ids (%s) on bus using default driver", ids_str.c_str());

        if (COMM_SUCCESS != result)
        {
            if (COMM_RX_TIMEOUT != result)
            {  // -3001
                _debug_error_message = "TtlManager - No motor found. "
                                       "Make sure that motors are correctly connected and powered on.";
            }
            else
            {  // -3002 or other
                _debug_error_message = "TtlManager - Failed to scan bus.";
            }
            ROS_WARN_THROTTLE(1,
                              "TtlManager::getAllIdsOnTtlBus - Broadcast ping failed, "
                              "result : %d (-3001: timeout, -3002: corrupted packet)",
                              result);
        }
    }
    else
    {
        // if no driver, no motors on bus, it is not a failure of scan
        result = COMM_SUCCESS;
    }

    return result;
}

// ******************
//  Write operations
// ******************

/**
 * @brief TtlManager::setLeds
 * @param led
 * @return
 */
int TtlManager::setLeds(int led)
{
    _led_state = led;
    int ret = niryo_robot_msgs::CommandStatus::TTL_WRITE_ERROR;

    EHardwareType mType = HardwareTypeEnum(_led_motor_type_cfg.c_str());

    if (mType == EHardwareType::FAKE_DXL_MOTOR)
        return niryo_robot_msgs::CommandStatus::SUCCESS;

    // get list of motors of the given type
    vector<uint8_t> id_list;
    if (_ids_map.count(mType) && _driver_map.count(mType))
    {
        id_list = _ids_map.at(mType);

        auto driver = std::dynamic_pointer_cast<AbstractDxlDriver>(_driver_map.at(mType));
        if (driver)
        {
            // sync write led state
            vector<uint8_t> command_led_value(id_list.size(), static_cast<uint8_t>(led));
            if (0 <= led && 7 >= led)
            {
                int result = COMM_TX_FAIL;
                for (int error_counter = 0; result != COMM_SUCCESS && error_counter < 5; ++error_counter)
                {
                    ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                    result = driver->syncWriteLed(id_list, command_led_value);
                }

                if (COMM_SUCCESS == result)
                    ret = niryo_robot_msgs::CommandStatus::SUCCESS;
                else
                    ROS_WARN("TtlManager::setLeds - Failed to write LED");
            }
        }
        else
        {
            ROS_DEBUG("Set leds failed. Driver is not compatible, check the driver's implementation ");
            return niryo_robot_msgs::CommandStatus::FAILURE;
        }
    }
    else
    {
        ROS_DEBUG("Set leds failed. It is maybe that this service is not support for this product");
        ret = niryo_robot_msgs::CommandStatus::SUCCESS;
    }

    return ret;
}

/**
 * @brief TtlManager::sendCustomCommand
 * @param id
 * @param reg_address
 * @param value
 * @param byte_number
 * @return
 */
int TtlManager::sendCustomCommand(uint8_t id, int reg_address, int value, int byte_number)
{
    int result = COMM_TX_FAIL;
    ROS_DEBUG("TtlManager::sendCustomCommand:\n"
              "\t\t ID: %d, Value: %d, Address: %d, Size: %d",
              static_cast<int>(id), value, reg_address, byte_number);

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type) && _driver_map.at(motor_type))
        {
            int32_t value_conv = value;
            result = _driver_map.at(motor_type)->writeCustom(static_cast<uint16_t>(reg_address), static_cast<uint8_t>(byte_number), id, static_cast<uint32_t>(value_conv));
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("TtlManager::sendCustomCommand - Failed to write custom command: %d", result);
                // TODO(Thuc): change TTL_WRITE_ERROR -> WRITE_ERROR
                result = niryo_robot_msgs::CommandStatus::TTL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::sendCustomCommand - driver for motor %s not available", HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::sendCustomCommand - driver for motor id %d unknown", static_cast<int>(id));
        result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
    }

    ros::Duration(0.005).sleep();
    return result;
}

/**
 * @brief TtlManager::readCustomCommand
 * @param id
 * @param reg_address
 * @param value
 * @param byte_number
 * @return
 */
int TtlManager::readCustomCommand(uint8_t id, int32_t reg_address, int &value, int byte_number)
{
    int result = COMM_RX_FAIL;
    ROS_DEBUG("TtlManager::readCustomCommand: ID: %d, Address: %d, Size: %d", static_cast<int>(id), static_cast<int>(reg_address), byte_number);

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type) && _driver_map.at(motor_type))
        {
            uint32_t data = 0;
            result = _driver_map.at(motor_type)->readCustom(static_cast<uint16_t>(reg_address), static_cast<uint8_t>(byte_number), id, data);
            auto data_conv = static_cast<int32_t>(data);
            value = data_conv;

            if (result != COMM_SUCCESS)
            {
                ROS_WARN("TtlManager::readCustomCommand - Failed to read custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readCustomCommand - driver for motor %s not available", HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::readCustomCommand - driver for motor id %d unknown", static_cast<int>(id));
        result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
    }

    ros::Duration(0.005).sleep();
    return result;
}

/**
 * @brief TtlManager::readMotorPID
 * @param id
 * @param pos_p_gain
 * @param pos_i_gain
 * @param pos_d_gain
 * @param vel_p_gain
 * @param vel_i_gain
 * @param ff1_gain
 * @param ff2_gain
 * @return
 */
int TtlManager::readMotorPID(uint8_t id, uint16_t &pos_p_gain, uint16_t &pos_i_gain, uint16_t &pos_d_gain, uint16_t &vel_p_gain, uint16_t &vel_i_gain, uint16_t &ff1_gain,
                             uint16_t &ff2_gain)
{
    int result = COMM_RX_FAIL;

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type))
        {
            auto driver = std::dynamic_pointer_cast<AbstractDxlDriver>(_driver_map.at(motor_type));
            if (driver)
            {
                std::vector<uint16_t> data;
                result = driver->readPID(id, data);

                if (COMM_SUCCESS == result)
                {
                    pos_p_gain = data.at(0);
                    pos_i_gain = data.at(1);
                    pos_d_gain = data.at(2);
                    vel_p_gain = data.at(3);
                    vel_i_gain = data.at(4);
                    ff1_gain = data.at(5);
                    ff2_gain = data.at(6);
                }
                else
                {
                    ROS_WARN("TtlManager::readMotorPID - Failed to read PID: %d", result);
                    result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;
                    return result;
                }
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readMotorPID - driver for motor %s not available", HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::readMotorPID - driver for motor id %d unknown", static_cast<int>(id));
        result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
    }

    ros::Duration(0.005).sleep();
    return result;
}

/**
 * @brief TtlManager::readVelocityProfile
 * @param id
 * @param v_start
 * @param a_1
 * @param v_1
 * @param a_max
 * @param v_max
 * @param d_max
 * @param d_1
 * @param v_stop
 * @return
 */
int TtlManager::readVelocityProfile(uint8_t id, uint32_t &v_start, uint32_t &a_1, uint32_t &v_1, uint32_t &a_max, uint32_t &v_max, uint32_t &d_max, uint32_t &d_1, uint32_t &v_stop)
{
    int result = COMM_RX_FAIL;

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();
        auto [joint_stepper_driver, hw_type] = getJointsStepperDriver();
        if (joint_stepper_driver)
        {
            std::vector<uint32_t> data;
            result = joint_stepper_driver->readVelocityProfile(id, data);

            if (COMM_SUCCESS == result)
            {
                v_start = data.at(0);
                a_1 = data.at(1);
                v_1 = data.at(2);
                a_max = data.at(3);
                v_max = data.at(4);
                d_max = data.at(5);
                d_1 = data.at(6);
                v_stop = data.at(7);
            }
            else
            {
                ROS_WARN("TtlManager::readVelocityProfile - Failed to read velocity profile: %d", result);
                result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;
                return result;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readVelocityProfile - driver for motor %s not available", HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::readVelocityProfile - driver for motor id %d unknown", static_cast<int>(id));
        result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
    }

    ros::Duration(0.005).sleep();
    return result;
}

/**
 * @brief TtlManager::readMoving
 * @param id
 * @param status
 * @return
 */
int TtlManager::readMoving(uint8_t id, uint8_t &status)
{
    int result = COMM_RX_FAIL;

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        // Only used for xl330 and xl320 for now
        if (_driver_map.count(motor_type) && (motor_type == EHardwareType::XL330 || motor_type == EHardwareType::XL320|| motor_type == EHardwareType::FAKE_DXL_MOTOR))
        {
            auto driver = std::dynamic_pointer_cast<AbstractDxlDriver>(_driver_map.at(motor_type));
            if (driver)
            {
                result = driver->readMoving(id, status);
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readMoving - register MOVING for motor %s not available", HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::readMoving - driver for motor id %d unknown", static_cast<int>(id));
        result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
    }

    ros::Duration(0.005).sleep();
    return result;
}

/**
 * @brief TtlManager::readControlMode
 * @param id
 * @param control_mode
 * @return
 */
int TtlManager::readControlMode(uint8_t id, uint8_t &control_mode)
{
    int result = COMM_RX_FAIL;

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type))
        {
            auto driver = std::dynamic_pointer_cast<AbstractDxlDriver>(_driver_map.at(motor_type));
            if (driver)
            {
                result = driver->readControlMode(id, control_mode);
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readControlMode - driver for motor %s not available", HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::readControlMode - driver for motor id %d unknown", static_cast<int>(id));
        result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
    }

    ros::Duration(0.005).sleep();
    return result;
}

/**
 * @brief TtlManager::writeSynchronizeCommand
 * @param cmd
 * @return
 */
int TtlManager::writeSynchronizeCommand(std::unique_ptr<common::model::AbstractTtlSynchronizeMotorCmd> &&cmd)  // NOLINT
{
    int result = COMM_TX_ERROR;
    ROS_DEBUG_THROTTLE(0.5, "TtlManager::writeSynchronizeCommand:  %s", cmd->str().c_str());

    if (cmd->isValid())
    {
        std::set<EHardwareType> typesToProcess = cmd->getMotorTypes();

        // process all the motors using each successive drivers
        for (uint32_t counter = 0; counter < MAX_HW_FAILURE; ++counter)
        {
            ROS_DEBUG_THROTTLE(0.5, "TtlManager::writeSynchronizeCommand: try to sync write (counter %d)", counter);

            for (auto const &it : _driver_map)
            {
                if (typesToProcess.count(it.first) != 0)
                {
                    result = COMM_TX_ERROR;

                    // syncwrite for this driver. The driver is responsible for sync write only to its associated motors
                    auto driver = std::dynamic_pointer_cast<AbstractMotorDriver>(it.second);
                    if (driver)
                    {
                        result = driver->writeSyncCmd(cmd->getCmdType(), cmd->getMotorsId(it.first), cmd->getParams(it.first));

                        ros::Duration(0.05).sleep();
                    }

                    // if successful, don't process this driver in the next loop
                    if (COMM_SUCCESS == result)
                    {
                        typesToProcess.erase(typesToProcess.find(it.first));
                    }
                    else
                    {
                        ROS_ERROR("TtlManager::writeSynchronizeCommand : unable to sync write function : %d", result);
                    }
                }
            }

            // if all drivers are processed, go out of for loop
            if (typesToProcess.empty())
            {
                result = COMM_SUCCESS;
                break;
            }

            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }
    }
    else
    {
        ROS_ERROR("TtlManager::writeSynchronizeCommand - Invalid command");
    }

    if (COMM_SUCCESS != result)
    {
        ROS_ERROR_THROTTLE(0.5, "TtlManager::writeSynchronizeCommand - Failed to write synchronize position");
        _debug_error_message = "TtlManager - Failed to write synchronize position";
    }

    return result;
}

/**
 * @brief TtlManager::writeSingleCommand
 * @param cmd
 * @return
 */
int TtlManager::writeSingleCommand(std::unique_ptr<common::model::AbstractTtlSingleMotorCmd> &&cmd)  // NOLINT
{
    int result = COMM_TX_ERROR;

    uint8_t id = cmd->getId();

    if (cmd->isValid())
    {
        int counter = 0;

        ROS_DEBUG("TtlManager::writeSingleCommand:  %s", cmd->str().c_str());

        if (_state_map.count(id) && _state_map.at(id))
        {
            auto state = _state_map.at(id);
            while ((COMM_SUCCESS != result) && (counter < 50))
            {
                EHardwareType hardware_type = state->getHardwareType();
                result = COMM_TX_ERROR;
                if (_driver_map.count(hardware_type) && _driver_map.at(hardware_type))
                {
                    // writeSingleCmd is in a for loop, we cannot infer that this command will succeed. Thus we cannot move cmd in parameter
                    result = _driver_map.at(hardware_type)->writeSingleCmd(cmd);
                }

                counter += 1;

                ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
            }
        }
        else
        {
            ROS_DEBUG("TtlManager::writeSingleCommand: command is sent to a removed hardware component. Skipped or write to a unknow device");
            result = COMM_TX_ERROR;
        }
    }

    if (result != COMM_SUCCESS)
    {
        ROS_WARN("TtlManager::writeSingleCommand - Fail to write single command : %s", cmd->str().c_str());
        _debug_error_message = "TtlManager - Failed to write a single command: " + cmd->str();
    }

    return result;
}

/**
 * @brief TtlManager::executeJointTrajectoryCmd
 * @param cmd_vec
 */
void TtlManager::executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, uint32_t>> cmd_vec)
{
    for (auto const &it : _driver_map)
    {
        // build list of ids and params for this motor
        _position_goal_ids.clear();
        _position_goal_params.clear();
        for (auto const &cmd : cmd_vec)
        {
            if (_state_map.count(cmd.first) && it.first == _state_map.at(cmd.first)->getHardwareType())
            {
                _position_goal_ids.emplace_back(cmd.first);
                _position_goal_params.emplace_back(cmd.second);
            }
        }

        // syncwrite for this driver. The driver is responsible for sync write only to its associated motors
        auto driver = std::dynamic_pointer_cast<AbstractMotorDriver>(it.second);

        if (driver)
        {
            int err = driver->syncWritePositionGoal(_position_goal_ids, _position_goal_params);
            if (err != COMM_SUCCESS)
            {
                ROS_WARN("TtlManager::executeJointTrajectoryCmd - Failed to write position");
                _debug_error_message = "TtlManager - Failed to write position";
            }
        }
    }
}

// ******************
//  Calibration
// ******************

/**
 * @brief TtlManager::startCalibration
 */
void TtlManager::startCalibration()
{
    ROS_DEBUG("TtlManager::startCalibration: starting...");

    std::vector<uint8_t> stepper_list;
    if (_ids_map.count(EHardwareType::STEPPER))
    {
        _calibration_status = EStepperCalibrationStatus::IN_PROGRESS;
        _calib_machine_state.start();
        stepper_list = _ids_map.at(EHardwareType::STEPPER);
    }
    else if (_ids_map.count(EHardwareType::NED3PRO_STEPPER))
    {
        stepper_list = _ids_map.at(EHardwareType::NED3PRO_STEPPER);
    }
    else if (_ids_map.count(EHardwareType::FAKE_STEPPER_MOTOR))
    {
        _calibration_status = EStepperCalibrationStatus::IN_PROGRESS;
        _calib_machine_state.start();
        stepper_list = _ids_map.at(EHardwareType::FAKE_STEPPER_MOTOR);
    }

    for (auto const &id : stepper_list)
    {
        if (_state_map.count(id))
        {
            auto stepperState = std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(id));

            if (stepperState && !stepperState->isConveyor())
            {
                stepperState->setCalibration(EStepperCalibrationStatus::IN_PROGRESS, 1);
            }
        }
    }  // for steppers_list
}

/**
 * @brief TtlManager::resetCalibration
 */
void TtlManager::resetCalibration()
{
    ROS_INFO("TtlManager::resetCalibration: reseting...");

    std::vector<uint8_t> stepper_list;
    if (_ids_map.count(EHardwareType::STEPPER))
    {
        _calibration_status = EStepperCalibrationStatus::UNINITIALIZED;
        _calib_machine_state.reset();
        stepper_list = _ids_map.at(EHardwareType::STEPPER);
    }
    else if (_ids_map.count(EHardwareType::NED3PRO_STEPPER))
        stepper_list = _ids_map.at(EHardwareType::NED3PRO_STEPPER);

    for (auto const id : stepper_list)
    {
        if (_state_map.count(id))
        {
            auto stepperState = std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(id));

            if (stepperState && !stepperState->isConveyor())
            {
                stepperState->setCalibration(EStepperCalibrationStatus::UNINITIALIZED, 1);
            }
        }
    }  // for steppers_list
}

/**
 * @brief TtlManager::getCalibrationResult
 * @param motor_id
 * @return
 */
int32_t TtlManager::getCalibrationResult(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) && _state_map.at(motor_id))
        throw std::out_of_range("TtlManager::getMotorsState: Unknown motor id");

    return std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(motor_id))->getCalibrationValue();
}

// ******************
//  Getters
// ******************

/**
 * @brief TtlManager::getBusState
 * @param connection_state
 * @param motor_id
 * @param debug_msg
 */
void TtlManager::getBusState(bool &connection_state, std::vector<uint8_t> &motor_id, std::string &debug_msg) const
{
    debug_msg = _debug_error_message;
    motor_id = _all_ids_connected;
    connection_state = isConnectionOk();
}

/**
 * @brief TtlManager::getMotorsStates
 * @return only the joints states
 */
std::vector<std::shared_ptr<JointState>> TtlManager::getMotorsStates() const
{
    std::vector<std::shared_ptr<JointState>> states;

    for (const auto &it : _state_map)
    {
        if (it.second && it.second->getComponentType() == common::model::EComponentType::JOINT)
        {
            states.emplace_back(std::dynamic_pointer_cast<JointState>(it.second));
        }
    }

    return states;
}

std::vector<std::shared_ptr<ConveyorState>> TtlManager::getConveyorsStates() const
{
    std::vector<std::shared_ptr<ConveyorState>> conveyor_states;
    for (const auto &it : _state_map)
    {
        if (it.second && it.second->getComponentType() == common::model::EComponentType::CONVEYOR)
        {
            conveyor_states.emplace_back(std::dynamic_pointer_cast<ConveyorState>(it.second));
        }
    }

    return conveyor_states;
}

/**
 * @brief TtlManager::getHardwareState
 * @param motor_id
 * @return
 */
std::shared_ptr<common::model::AbstractHardwareState> TtlManager::getHardwareState(uint8_t motor_id) const
{
    if (!_state_map.count(motor_id) && _state_map.at(motor_id))
        throw std::out_of_range("TtlManager::getMotorsState: Unknown motor id");

    return _state_map.at(motor_id);
}

// ********************
//  Private
// ********************

/**
 * @brief TtlManager::addHardwareDriver
 * @param hardware_type
 */
void TtlManager::addHardwareDriver(EHardwareType hardware_type)
{
    // if not already instanciated
    if (!_driver_map.count(hardware_type))
    {
        switch (hardware_type)
        {
        case EHardwareType::STEPPER:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<StepperDriver<StepperReg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::NED3PRO_STEPPER:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<Ned3ProStepperDriver<Ned3ProStepperReg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::FAKE_STEPPER_MOTOR:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<MockStepperDriver>(_fake_data)));
            break;
        case EHardwareType::XL430:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XL430Reg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::XC430:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XC430Reg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::XM430:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XM430Reg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::XL320:
            _driver_map.insert(make_pair(hardware_type, std::make_shared<DxlDriver<XL320Reg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::XL330:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XL330Reg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::XH430:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XH430Reg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::FAKE_DXL_MOTOR:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<MockDxlDriver>(_fake_data)));
            break;
        case EHardwareType::END_EFFECTOR:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<EndEffectorDriver<EndEffectorReg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::NED3PRO_END_EFFECTOR:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<Ned3ProEndEffectorDriver<Ned3ProEndEffectorReg>>(_portHandler, _packetHandler)));
            break;
        case EHardwareType::FAKE_END_EFFECTOR:
            _driver_map.insert(std::make_pair(hardware_type, std::make_shared<MockEndEffectorDriver>(_fake_data)));
            break;
        default:
            ROS_ERROR("TtlManager - Unable to instanciate driver, unknown type");
            break;
        }
    }
}

/**
 * @brief TtlManager::readFakeConfig
 */
void TtlManager::readFakeConfig(bool use_simu_gripper, bool use_simu_conveyor)
{
    _fake_data = std::make_shared<FakeTtlData>();

    if (_nh.hasParam("fake_params"))
    {
        std::vector<int> full_id_list;
        if (_nh.hasParam("fake_params/id_list"))
            _nh.getParam("fake_params/id_list", full_id_list);
        for (auto id : full_id_list)
            _fake_data->full_id_list.emplace_back(static_cast<uint8_t>(id));

        if (_nh.hasParam("fake_params/steppers"))
        {
            std::string current_ns = "fake_params/steppers/";
            retrieveFakeMotorData(current_ns, _fake_data->stepper_registers);
        }

        if (_nh.hasParam("fake_params/dynamixels/"))
        {
            std::string current_ns = "fake_params/dynamixels/";
            retrieveFakeMotorData(current_ns, _fake_data->dxl_registers);
        }

        if (_nh.hasParam("fake_params/end_effector"))
        {
            string current_ns = "fake_params/end_effector/";
            vector<int> id_list, temperature_list, voltage_list;
            vector<string> firmware_list;
            _nh.getParam(current_ns + "id", id_list);
            _nh.getParam(current_ns + "temperature", temperature_list);
            _nh.getParam(current_ns + "voltage", voltage_list);
            _nh.getParam(current_ns + "firmware", firmware_list);

            assert(!id_list.empty());
            assert(!temperature_list.empty());
            assert(!voltage_list.empty());
            assert(!firmware_list.empty());

            _fake_data->end_effector.id = static_cast<uint8_t>(id_list.at(0));
            _fake_data->end_effector.temperature = static_cast<uint8_t>(temperature_list.at(0));
            _fake_data->end_effector.voltage = static_cast<double>(voltage_list.at(0));

            _fake_data->end_effector.firmware = firmware_list.at(0);
        }

        if (use_simu_gripper && _nh.hasParam("fake_params/tool/"))
        {
            std::string current_ns = "fake_params/tool/";
            retrieveFakeMotorData(current_ns, _fake_data->dxl_registers);
        }

        if (use_simu_conveyor && _nh.hasParam("fake_params/conveyors/"))
        {
            std::string current_ns = "fake_params/conveyors/";
            retrieveFakeMotorData(current_ns, _fake_data->stepper_registers);
        }

        _fake_data->updateFullIdList();
    }
}

/**
 * @brief: Return the driver of the joints
 * 
 * All stepper joints should have the same type of driver
*/
TtlManager::GetJointsStepperDriverResult TtlManager::getJointsStepperDriver(){
    auto joint_states = getMotorsStates();
    auto some_joint_state_it = std::find_if(joint_states.begin(), joint_states.end(), [](const std::shared_ptr<JointState>& joint_state){
        return joint_state->isStepper();
    });
    if (some_joint_state_it == joint_states.end()){
        return {};
    }
    auto some_joint_state = *some_joint_state_it;
    auto hardware_type = some_joint_state->getHardwareType();
    return {
        .driver = std::dynamic_pointer_cast<AbstractStepperDriver>(_driver_map.at(hardware_type)),
        .hardware_type = hardware_type
        };
}


}  // namespace ttl_driver
