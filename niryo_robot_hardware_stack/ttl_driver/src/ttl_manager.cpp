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
#include <cmath>
#include <utility>
#include <vector>
#include <set>
#include <string>
#include <algorithm>
#include <sstream>
#include <unordered_map>
#include <cstdlib>
#include <cassert>

// ros
#include "ros/serialization.h"
#include "ros/time.h"

// niryo
#include "common/model/hardware_type_enum.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/tool_state.hpp"
#include "common/model/stepper_motor_state.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/end_effector_state.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"

#include "dynamixel_sdk/packet_handler.h"
#include "ttl_driver/stepper_reg.hpp"
#include "ttl_driver/end_effector_reg.hpp"

#include "ttl_driver/dxl_driver.hpp"
#include "ttl_driver/mock_dxl_driver.hpp"
#include "ttl_driver/stepper_driver.hpp"
#include "ttl_driver/mock_stepper_driver.hpp"
#include "ttl_driver/end_effector_driver.hpp"
#include "ttl_driver/mock_end_effector_driver.hpp"
#include "ttl_driver/fake_ttl_data.hpp"

using ::std::shared_ptr;
using ::std::vector;
using ::std::string;
using ::std::ostringstream;
using ::std::to_string;
using ::std::set;

using ::common::model::EStepperCalibrationStatus;
using ::common::model::StepperMotorState;
using ::common::model::EndEffectorState;
using ::common::model::JointState;
using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;

namespace ttl_driver
{
/**
 * @brief TtlManager::TtlManager
 */
TtlManager::TtlManager(ros::NodeHandle& nh) :
    _nh(nh),
    _debug_error_message("TtlManager - No connection with TTL motors has been made yet")
{
    ROS_DEBUG("TtlManager - ctor");

    init(nh);

    if (COMM_SUCCESS != setupCommunication())
        ROS_WARN("TtlManager - TTL Communication Failed");
}

/**
 * @brief TtlManager::init
 * @param nh
 * @return
 */
bool TtlManager::init(ros::NodeHandle& nh)
{
    // get params from rosparams
    nh.getParam("simulation_mode", _simulation_mode);
    nh.getParam("bus_params/uart_device_name", _device_name);
    nh.getParam("bus_params/baudrate", _baudrate);
    nh.getParam("led_motor", _led_motor_type_cfg);
    nh.getParam("simu_gripper", _use_simu_gripper);
    nh.getParam("conveyor/direction", _conveyor_direction);

    if (!_simulation_mode)
    {
        _portHandler.reset(dynamixel::PortHandler::getPortHandler(_device_name.c_str()));
        _packetHandler.reset(dynamixel::PacketHandler::getPacketHandler(TTL_BUS_PROTOCOL_VERSION));
    }
    else {
        readFakeConfig();
    }

    ROS_DEBUG("TtlManager::init - Dxl : set port name (%s), baudrate(%d)", _device_name.c_str(), _baudrate);
    ROS_DEBUG("TtlManager::init - led motor type config : %s", _led_motor_type_cfg.c_str());

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
void TtlManager::addHardwareComponent(std::shared_ptr<common::model::AbstractHardwareState> && state)
{
    common::model::EHardwareType hardware_type = state->getHardwareType();
    uint8_t id = state->getId();

    ROS_DEBUG("TtlManager::addHardwareComponent : %s", state->str().c_str());

    // if not already instanciated
    if (!_state_map.count(id))
    {
        _state_map.insert(std::make_pair(id, state));
    }

    // if not already instanciated
    if (!_ids_map.count(hardware_type))
    {
        _ids_map.insert(std::make_pair(hardware_type, std::vector<uint8_t>({id})));
    }
    else
    {
        _ids_map.at(hardware_type).emplace_back(id);
    }

    addHardwareDriver(hardware_type);
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
            auto& ids = _ids_map.at(type);
            ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
            if (ids.empty())
            {
                _ids_map.erase(type);
            }
        }

        _state_map.erase(id);
    }

    _removed_motor_id_list.erase(std::remove(_removed_motor_id_list.begin(),
                                             _removed_motor_id_list.end(), id),
                                             _removed_motor_id_list.end());
}

/**
 * @brief TtlManager::isMotorType
 * @param type 
 * @return true 
 * @return false 
 */
bool TtlManager::isMotorType(common::model::EHardwareType type)
{
    // All motors have value under 7 (check in common::model::EHardwareType)
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
int TtlManager::changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id)
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
                auto i_state  = _state_map.find(old_id);
                // update all maps
                if (i_state != _state_map.end())
                {
                    // update all maps
                    std::swap(_state_map[new_id], i_state->second);
                    // update all maps
                    _state_map.erase(i_state);
                }
                if (_ids_map.count(motor_type))
                {
                    // update all maps
                    _ids_map.at(motor_type).erase(std::remove(_ids_map.at(motor_type).begin(), _ids_map.at(motor_type).end(), old_id),
                                                _ids_map.at(motor_type).end());

                    // update all maps
                    _ids_map.at(motor_type).emplace_back(new_id);
                }
                // change id in conveyor list
                _conveyor_list.erase(std::remove(_conveyor_list.begin(), _conveyor_list.end(), old_id), _conveyor_list.end());
                _conveyor_list.push_back(new_id);
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

    _all_motor_connected.clear();
    _is_connection_ok = false;

    for (int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
    {
        result = getAllIdsOnBus(_all_motor_connected);
        ROS_DEBUG_COND(COMM_SUCCESS != result, "TtlManager::scanAndCheck status: %d (counter: %d)", result, counter);
        ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
    }

    if (COMM_SUCCESS == result)
    {
        checkRemovedMotors();

        if (_removed_motor_id_list.empty())
        {
            _is_connection_ok = true;
            _debug_error_message.clear();
            result = TTL_SCAN_OK;
        }
        else
        {
            _debug_error_message = "Motor(s):";
            for (auto const &id : _removed_motor_id_list)
            {
                _debug_error_message += " " + to_string(id);
            }
            _debug_error_message += " do not seem to be connected";
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

    auto state = _state_map.find(id);
    if (state != _state_map.end() && state->second && _driver_map.find(state->second->getHardwareType()) != _driver_map.end())
    {
        auto it = _driver_map.at(state->second->getHardwareType());
        if (it)
        {
            if (COMM_SUCCESS == it->ping(id))
                result = true;
        }
    }

    ROS_DEBUG_THROTTLE(1, "TtlManager::ping with result %d", result);

    return result;
}

/**
 * @brief TtlManager::rebootMotors
 * @return
 */
int TtlManager::rebootMotors()
{
    int return_value = niryo_robot_msgs::CommandStatus::FAILURE;

    for (auto const &it : _state_map)
    {
        EHardwareType type = it.second->getHardwareType();
        ROS_DEBUG("TtlManager::rebootMotors - Reboot TTL motor with ID: %d", it.first);
        if (_driver_map.count(type))
        {
            int result = _driver_map.at(type)->reboot(it.first);
            if (COMM_SUCCESS == result)
            {
                ROS_DEBUG("TtlManager::rebootMotors - Reboot motor successfull");
                return_value = niryo_robot_msgs::CommandStatus::SUCCESS;
            }
            else
            {
                ROS_WARN("TtlManager::rebootMotors - Failed to reboot motor: %d", result);
                return_value = result;
            }
        }
    }

    return return_value;
}

/**
 * @brief TtlManager::rebootMotor
 * @param motor_id
 * @return
 */
int TtlManager::rebootMotor(uint8_t motor_id)
{
    int return_value = COMM_TX_FAIL;

    if (_state_map.count(motor_id) != 0 && _state_map.at(motor_id))
    {
        EHardwareType type = _state_map.at(motor_id)->getHardwareType();
        ROS_DEBUG("TtlManager::rebootMotors - Reboot motor with ID: %d", motor_id);
        if (_driver_map.count(type))
        {
            return_value = _driver_map.at(type)->reboot(motor_id);
            if (COMM_SUCCESS == return_value)
            {
                ros::Time start_time = ros::Time::now();
                uint32_t tmp = 0;
                int wait_result = _driver_map.at(type)->readTemperature(motor_id, tmp);
                while (COMM_SUCCESS != wait_result || !tmp)
                {
                    if ((ros::Time::now() - start_time).toSec() > 1)
                        break;
                    ros::Duration(0.1).sleep();
                    wait_result = _driver_map.at(type)->readTemperature(motor_id, tmp);
                }
            }
            ROS_WARN_COND(COMM_SUCCESS != return_value,
                          "TtlManager::rebootMotors - Failed to reboot motor: %d",
                          return_value);
        }
    }

    return return_value;
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
    if (_driver_map.count(hardware_type) && _driver_map.at(hardware_type))
    {
        for (_hw_fail_counter_read = 0; _hw_fail_counter_read < MAX_HW_FAILURE; ++_hw_fail_counter_read)
        {
            auto driver = std::dynamic_pointer_cast<AbstractMotorDriver>(_driver_map.at(hardware_type));

            if (driver && COMM_SUCCESS == driver->readPosition(motor_state.getId(), position))
            {
                _hw_fail_counter_read = 0;
                break;
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

/**
 * @brief TtlManager::readJointStatus : reads the position and velocity of each joint and updates the states accordingly
 * TODO(CC) add syncreadload
 *
 */
bool TtlManager::readPositionsStatus()
{
    bool res = false;
    unsigned int hw_errors_increment = 0;

    // syncread position for all motors. Using only one driver for all motors to avoid loop.
    // All addresses for position are the same
    if (!_driver_map.empty())
    {
        std::shared_ptr<ttl_driver::AbstractMotorDriver> driver;
        try {
            // get driver of a motor
            auto ttl_driver = _driver_map.at(_state_map.at(_motor_list.at(0))->getHardwareType());
            driver = std::dynamic_pointer_cast<AbstractMotorDriver>(ttl_driver);
        }
        catch (const std::exception& e) {}

        if (driver)  // && _ids_map.count(type))
        {
            std::lock_guard<std::mutex> lock(_sync_mutex);

            // we retrieve all the associated id for the type of the current driver
            vector<uint32_t> position_list;

            // retrieve positions
            if (COMM_SUCCESS == driver->syncReadPosition(_motor_list, position_list))
            {
                if (_motor_list.size() == position_list.size())
                {
                    // set motors states accordingly
                    for (size_t i = 0; i < _motor_list.size(); ++i)
                    {
                        uint8_t id = _motor_list.at(i);
                        int position = static_cast<int>(position_list.at(i));

                        if (_state_map.count(id))
                        {
                            auto state = std::dynamic_pointer_cast<common::model::AbstractMotorState>(_state_map.at(id));
                            if (state)
                            {
                                state->setPosition(position);
                            }
                        }
                    }
                }
                else
                {
                    ROS_ERROR("TtlManager::readJointStatus : Fail to sync read joint state - "
                                "vector mismatch (id_list size %d, position_list size %d)",
                                static_cast<int>(_motor_list.size()),
                                static_cast<int>(position_list.size()));
                    hw_errors_increment++;
                }
            }
            else
            {
                hw_errors_increment++;
            }
        }
    }  // for driver_map

    // we reset the global error variable only if no errors
    if (0 == hw_errors_increment)
    {
        _hw_fail_counter_read = 0;
        res = true;
    }
    else
    {
        _hw_fail_counter_read += hw_errors_increment;
    }

    if (_hw_fail_counter_read > MAX_HW_FAILURE)
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::readJointStatus - motor connection problem - "
                                "Failed to read from bus (hw_fail_counter_read : %d)", _hw_fail_counter_read);
        _hw_fail_counter_read = 0;
        _is_connection_ok = false;
        _debug_error_message = "TtlManager - Connection problem with physical Bus.";
    }

    return res;
}

/**
 * @brief TtlManager::readEndEffectorStatus
 */
bool TtlManager::readEndEffectorStatus()
{
    bool res = false;

    // if has end effector driver
    if (_driver_map.count(EHardwareType::END_EFFECTOR))
    {
        unsigned int hw_errors_increment = 0;
        try
        {
            shared_ptr<EndEffectorDriver<EndEffectorReg> > driver = std::dynamic_pointer_cast<EndEffectorDriver<EndEffectorReg> >(_driver_map.at(EHardwareType::END_EFFECTOR));
            if (_ids_map.count(EHardwareType::END_EFFECTOR))
            {
                // we retrieve the associated id for the end effector
                uint8_t id = _ids_map.at(EHardwareType::END_EFFECTOR).front();
                common::model::EActionType action;

                // read buttons status if state map have id of EE
                auto state = std::dynamic_pointer_cast<EndEffectorState>(_state_map.at(id));
                // free drive button
                if (COMM_SUCCESS == driver->readButton0Status(id, action))
                    state->setButtonStatus(0, action);
                else
                    hw_errors_increment++;

                // save pos button
                if (COMM_SUCCESS == driver->readButton1Status(id, action))
                    state->setButtonStatus(1, action);
                else
                    hw_errors_increment++;

                // custom button
                if (COMM_SUCCESS == driver->readButton2Status(id, action))
                    state->setButtonStatus(2, action);
                else
                    hw_errors_increment++;

                bool digital_data;
                if (COMM_SUCCESS == driver->readDigitalInput(id, digital_data))
                    state->setDigitalIn(digital_data);
                else
                    hw_errors_increment++;
            }  // for driver_map
        }
        catch(const std::exception& e) {}

        // we reset the global error variable only if no errors
        if (0 == hw_errors_increment)
        {
            _hw_fail_counter_read = 0;
            res = true;
        }
        else
        {
            _hw_fail_counter_read += hw_errors_increment;
        }

        if (_hw_fail_counter_read > MAX_HW_FAILURE)
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readEndEffectorStatus - motor connection problem - "
                                  "Failed to read from bus (hw_fail_counter_read : %d)", _hw_fail_counter_read);
            _hw_fail_counter_read = 0;
            _debug_error_message = "TtlManager - Connection problem with physical Bus.";
        }
    }
    else
    {
        ROS_DEBUG_THROTTLE(2, "TtlManager::readEndEffectorStatus - No end effector found");
    }

    return res;
}

/**
 * @brief TtlManager::readHwStatus
 * TODO(CC) refacto
 */
bool TtlManager::readHardwareStatus()
{
    bool res = false;

    unsigned int hw_errors_increment = 0;

    // **************  read Velocity
    if (!_driver_map.empty())
    {
        std::shared_ptr<ttl_driver::AbstractMotorDriver> driver;
        try {
            // get driver of a motor
            auto ttl_driver = _driver_map.at(_state_map.at(_motor_list.at(0))->getHardwareType());
            driver = std::dynamic_pointer_cast<AbstractMotorDriver>(ttl_driver);
        }
        catch (const std::exception& e) {}

        if (driver)
        {
            vector<uint32_t> velocity_list;

            // retrieve velocity
            if (COMM_SUCCESS == driver->syncReadVelocity(_motor_list, velocity_list))
            {
                if (_motor_list.size() == velocity_list.size())
                {
                    // set motors states accordingly
                    for (size_t i = 0; i < _motor_list.size(); ++i)
                    {
                        uint8_t id = _motor_list.at(i);
                        int velocity = static_cast<int>(velocity_list.at(i));

                        if (_state_map.count(id))
                        {
                            auto state = std::dynamic_pointer_cast<common::model::AbstractMotorState>(_state_map.at(id));
                            if (state)
                            {
                                state->setVelocity(velocity);
                            }
                        }
                    }
                }
                else
                {
                    hw_errors_increment++;
                }
            }
            else
            {
                hw_errors_increment++;
            }
        }
    }  // for driver_map

    // syncread from all drivers for all motors
    for (auto const& it : _driver_map)
    {
        EHardwareType type = it.first;
        shared_ptr<AbstractTtlDriver> driver = it.second;

        if (driver && _ids_map.count(type))
        {
            // we retrieve all the associated id for the type of the current driver
            vector<uint8_t> id_list = _ids_map.at(type);
            size_t id_list_size = id_list.size();

            // **************  firmware version
            vector<std::string> firmware_version_list;

            if (COMM_SUCCESS != driver->syncReadFirmwareVersion(id_list, firmware_version_list))
            {
                // this operation can fail, it is normal, so no error message
                hw_errors_increment++;
            }
            else if (id_list_size != firmware_version_list.size())
            {
                // however, if we have a mismatch here, it is not normal

                ROS_ERROR("TtlManager::readHwStatus : syncReadFirmwareVersion failed - "
                            "vector mistmatch (id_list size %d, temperature_list size %d)",
                            static_cast<int>(id_list_size), static_cast<int>(firmware_version_list.size()));

                hw_errors_increment++;
            }

            // **************  temperature
            vector<uint32_t> temperature_list;

            if (COMM_SUCCESS != driver->syncReadTemperature(id_list, temperature_list))
            {
                // this operation can fail, it is normal, so no error message
                hw_errors_increment++;
            }
            else if (id_list_size != temperature_list.size())
            {
                // however, if we have a mismatch here, it is not normal

                ROS_ERROR("TtlManager::readHwStatus : syncReadTemperature failed - "
                            "vector mistmatch (id_list size %d, temperature_list size %d)",
                            static_cast<int>(id_list_size), static_cast<int>(temperature_list.size()));

                hw_errors_increment++;
            }

            // **********  voltage
            vector<double> voltage_list;

            if (COMM_SUCCESS != driver->syncReadVoltage(id_list, voltage_list))
            {
                hw_errors_increment++;
            }
            else if (id_list_size != voltage_list.size())
            {
                ROS_ERROR("TtlManager::readHwStatus : syncReadTemperature failed - "
                            "vector mistmatch (id_list size %d, voltage_list size %d)",
                            static_cast<int>(id_list_size), static_cast<int>(voltage_list.size()));

                hw_errors_increment++;
            }

            // **********  error state
            vector<uint32_t> hw_status_list;

            if (COMM_SUCCESS != driver->syncReadHwErrorStatus(id_list, hw_status_list))
            {
                hw_errors_increment++;
            }
            else if (id_list_size != hw_status_list.size())
            {
                ROS_ERROR("TtlManager::readHwStatus : syncReadTemperature failed - "
                            "vector mistmatch (id_list size %d, hw_status_list size %d)",
                            static_cast<int>(id_list_size), static_cast<int>(hw_status_list.size()));

                hw_errors_increment++;
            }

            // ***********  calibration status
            if ( (EHardwareType::FAKE_STEPPER_MOTOR == type || EHardwareType::STEPPER == type))
            {
                for (auto id : _ids_map.at(type))
                {
                    if (_state_map.find(id) != _state_map.end())
                    {
                        uint32_t status{0};     // not in calibration status table
                        shared_ptr<ttl_driver::AbstractStepperDriver> stepper_driver = std::dynamic_pointer_cast<ttl_driver::AbstractStepperDriver>(driver);
                        if (_calibration_status != EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED &&
                            COMM_SUCCESS == stepper_driver->readHomingStatus(id, status))
                        {
                            if (_map_calibration_status.count(status))
                            {
                                std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(id))->setCalibration(_map_calibration_status.at(status), 1);
                                updateCurrentCalibrationStatus();
                            }
                        }
                    }
                }
            }

            // **********  conveyor state
            if (!_conveyor_list.empty() && (EHardwareType::FAKE_STEPPER_MOTOR == type || EHardwareType::STEPPER == type))
            {
                try
                {
                    std::shared_ptr<ttl_driver::AbstractStepperDriver> stepper_driver;
                    stepper_driver = std::dynamic_pointer_cast<ttl_driver::AbstractStepperDriver>(driver);

                    for (auto id : _conveyor_list)
                    {
                        std::shared_ptr<StepperMotorState> state;
                        state = std::dynamic_pointer_cast<StepperMotorState>(_state_map.at(id));
                        if (state && state->isConveyor())
                        {
                            uint32_t velocity;
                            if (COMM_SUCCESS != stepper_driver->readVelocity(id, velocity))
                            {
                                hw_errors_increment++;
                            }

                            auto speed = static_cast<int16_t>(velocity);
                            auto cState = std::dynamic_pointer_cast<common::model::ConveyorState>(state);
                            cState->setGoalDirection(cState->getDirection() * (speed > 0 ? 1 : -1));
                            cState->setSpeed(static_cast<int16_t>(std::abs(speed)));
                            cState->setState(speed);
                        }
                    }
                }
                catch(const std::exception& e) {}
            }
            // set motors states accordingly
            for (size_t i = 0; i < id_list_size; ++i)
            {
                uint8_t id = id_list.at(i);

                if (_state_map.count(id))
                {
                    auto state = _state_map.at(id);

                    // **************  firmware versions
                    if (firmware_version_list.size() > i)
                    {
                        state->setFirmwareVersion(firmware_version_list.at(i));
                    }

                    // **************  temperature
                    if (temperature_list.size() > i)
                    {
                        state->setTemperature(temperature_list.at(i));
                    }

                    // **********  voltage
                    if (voltage_list.size() > i)
                    {
                        state->setVoltage(voltage_list.at(i));
                    }

                    // **********  error state
                    if (hw_status_list.size() > i)
                    {
                        state->setHardwareError(hw_status_list.at(i));
                        string hardware_message = driver->interpreteErrorState(hw_status_list.at(i));
                        state->setHardwareError(hardware_message);
                    }
                }
            }  // for id_list
        }
    }  // for driver_map

    // we reset the global error variable only if no errors
    if (0 == hw_errors_increment)
    {
        _hw_fail_counter_read = 0;
        res = true;
    }
    else
    {
        _hw_fail_counter_read += hw_errors_increment;
    }

    // if too much errors, disconnect
    if (_hw_fail_counter_read > MAX_HW_FAILURE )
    {
        ROS_ERROR_THROTTLE(1, "TtlManager::readHwStatus - motor connection problem - "
                              "Failed to read from bus (hw_fail_counter_read : %d)", _hw_fail_counter_read);
        _hw_fail_counter_read = 0;
        res = false;
        _is_connection_ok = false;
        _debug_error_message = "TtlManager - Connection problem with physical Bus.";
    }


    return res;
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

    // remove tool in fake driver
    if (!_use_simu_gripper && _driver_map.count(EHardwareType::FAKE_DXL_MOTOR) && _driver_map.at(EHardwareType::FAKE_DXL_MOTOR))
    {
        std::dynamic_pointer_cast<MockDxlDriver>(_driver_map.at(EHardwareType::FAKE_DXL_MOTOR))->removeGripper();
    }

    // 1. Get all ids from ttl bus. We can use any driver for that
    auto it = _driver_map.begin();

    if  (it != _driver_map.end())
    {
      if (it->second)
      {
          vector<uint8_t> l_idList;
          result = it->second->scan(l_idList);
          id_list.insert(id_list.end(), l_idList.begin(), l_idList.end());

          string ids_str;
          for (auto const &id : l_idList)
              ids_str += to_string(id) + " ";

          ROS_DEBUG_THROTTLE(1, "TtlManager::getAllIdsOnTtlBus - Found ids (%s) on bus using first driver (type: %s)",
                              ids_str.c_str(),
                              HardwareTypeEnum(it->first).toString().c_str());

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
              ROS_WARN_THROTTLE(1, "TtlManager::getAllIdsOnTtlBus - Broadcast ping failed, "
                              "result : %d (-3001: timeout, -3002: corrupted packet)",
                              result);
          }
      }
    }
    else {
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
    int ret = niryo_robot_msgs::CommandStatus::TTL_WRITE_ERROR;
    _led_state = led;

    EHardwareType mType = HardwareTypeEnum(_led_motor_type_cfg.c_str());

    if (mType == EHardwareType::FAKE_DXL_MOTOR)
        return niryo_robot_msgs::CommandStatus::SUCCESS;

    // get list of motors of the given type
    vector<uint8_t> id_list;
    if (_ids_map.count(mType) && _driver_map.count(mType))
    {
        id_list = _ids_map.at(mType);

        auto driver = std::dynamic_pointer_cast<AbstractDxlDriver>(_driver_map.at(mType));

        // sync write led state
        vector<uint32_t> command_led_id(id_list.size(), static_cast<uint32_t>(led));
        if (0 <= led && 7 >= led)
        {
            int result = COMM_TX_FAIL;
            for (int error_counter = 0; result != COMM_SUCCESS && error_counter < 5; ++error_counter)
            {
                ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                result = driver->syncWriteLed(id_list, command_led_id);
            }

            if (COMM_SUCCESS == result)
                ret = niryo_robot_msgs::CommandStatus::SUCCESS;
            else
                ROS_WARN("TtlManager::setLeds - Failed to write LED");
        }
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
int TtlManager::sendCustomCommand(uint8_t id, int reg_address, int value,  int byte_number)
{
    int result = COMM_TX_FAIL;
    ROS_DEBUG("TtlManager::sendCustomCommand:\n"
              "\t\t ID: %d, Value: %d, Address: %d, Size: %d",
              static_cast<int>(id), value,
              reg_address, byte_number);

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type) && _driver_map.at(motor_type))
        {
            result = _driver_map.at(motor_type)->writeCustom(static_cast<uint8_t>(reg_address),
                                                             static_cast<uint8_t>(byte_number),
                                                             id,
                                                             static_cast<uint32_t>(value));
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("TtlManager::sendCustomCommand - Failed to write custom command: %d", result);
                // TODO(Thuc): change TTL_WRITE_ERROR -> WRITE_ERROR
                result = niryo_robot_msgs::CommandStatus::TTL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::sendCustomCommand - driver for motor %s not available",
                               HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
      ROS_ERROR_THROTTLE(1, "TtlManager::sendCustomCommand - driver for motor id %d unknown",
                         static_cast<int>(id));
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
int TtlManager::readCustomCommand(uint8_t id, int32_t reg_address, int& value, int byte_number)
{
    int result = COMM_RX_FAIL;
    ROS_DEBUG("TtlManager::readCustomCommand: ID: %d, Address: %d, Size: %d",
              static_cast<int>(id),
              static_cast<int>(reg_address), byte_number);

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type) && _driver_map.at(motor_type))
        {
            uint32_t data = 0;
            result = _driver_map.at(motor_type)->readCustom(static_cast<uint8_t>(reg_address),
                                                            static_cast<uint8_t>(byte_number),
                                                            id,
                                                            data);
            value = static_cast<int>(data);

            if (result != COMM_SUCCESS)
            {
                ROS_WARN("TtlManager::readCustomCommand - Failed to read custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readCustomCommand - driver for motor %s not available",
                               HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
      ROS_ERROR_THROTTLE(1, "TtlManager::readCustomCommand - driver for motor id %d unknown",
                         static_cast<int>(id));
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
int TtlManager::readMotorPID(uint8_t id,
                             uint32_t& pos_p_gain, uint32_t& pos_i_gain, uint32_t& pos_d_gain,
                             uint32_t& vel_p_gain, uint32_t& vel_i_gain,
                             uint32_t& ff1_gain, uint32_t& ff2_gain)
{
    int result = COMM_RX_FAIL;

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type) && _driver_map.at(motor_type))
        {
            auto driver = std::dynamic_pointer_cast<AbstractDxlDriver>(_driver_map.at(motor_type));
            if (driver)
            {
                std::vector<uint32_t> data;
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
            ROS_ERROR_THROTTLE(1, "TtlManager::readMotorPID - driver for motor %s not available",
                               HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
      ROS_ERROR_THROTTLE(1, "TtlManager::readMotorPID - driver for motor id %d unknown",
                         static_cast<int>(id));
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
int TtlManager::readVelocityProfile(uint8_t id, uint32_t &v_start, uint32_t &a_1, uint32_t &v_1,
                                    uint32_t &a_max, uint32_t &v_max, uint32_t &d_max,
                                    uint32_t &d_1, uint32_t &v_stop)
{
    int result = COMM_RX_FAIL;

    if (_state_map.count(id) != 0 && _state_map.at(id))
    {
        EHardwareType motor_type = _state_map.at(id)->getHardwareType();

        if (_driver_map.count(motor_type) && _driver_map.at(motor_type))
        {
            auto driver = std::dynamic_pointer_cast<AbstractStepperDriver>(_driver_map.at(motor_type));
            if (driver)
            {
                std::vector<uint32_t> data;
                result = driver->readVelocityProfile(id, data);

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
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "TtlManager::readVelocityProfile - driver for motor %s not available",
                               HardwareTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
    }
    else
    {
      ROS_ERROR_THROTTLE(1, "TtlManager::readMotorPID - driver for motor id %d unknown",
                         static_cast<int>(id));
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
int TtlManager::writeSynchronizeCommand(std::unique_ptr<common::model::AbstractTtlSynchronizeMotorCmd> && cmd)
{
    int result = COMM_TX_ERROR;
    ROS_DEBUG_THROTTLE(0.5, "TtlManager::writeSynchronizeCommand:  %s", cmd->str().c_str());

    if (cmd->isValid())
    {
        std::set<common::model::EHardwareType> typesToProcess = cmd->getMotorTypes();

        // process all the motors using each successive drivers
        for (uint32_t counter = 0; counter < MAX_HW_FAILURE; ++counter)
        {
            ROS_DEBUG_THROTTLE(0.5, "TtlManager::writeSynchronizeCommand: try to sync write (counter %d)", counter);

            for (auto const& it : _driver_map)
            {
                if (typesToProcess.count(it.first) != 0)
                {
                    result = COMM_TX_ERROR;

                    // syncwrite for this driver. The driver is responsible for sync write only to its associated motors
                    auto driver = std::dynamic_pointer_cast<AbstractMotorDriver>(it.second);
                    if (driver)
                    {
                      std::lock_guard<std::mutex> lock(_sync_mutex);

                      result = driver->writeSyncCmd(cmd->getCmdType(),
                                                    cmd->getMotorsId(it.first),
                                                    cmd->getParams(it.first));

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
int TtlManager::writeSingleCommand(std::unique_ptr<common::model::AbstractTtlSingleMotorCmd> && cmd)
{
    int result = COMM_TX_ERROR;

    uint8_t id = cmd->getId();

    if (cmd->isValid())
    {
        int counter = 0;

        ROS_DEBUG("TtlManager::writeSingleCommand:  %s", cmd->str().c_str());

        if (_state_map.count(id) != 0)
        {
            auto state = _state_map.at(id);
            while ((COMM_SUCCESS != result) && (counter < 50))
            {
                common::model::EHardwareType hardware_type = state->getHardwareType();
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
    }

    if (result != COMM_SUCCESS)
    {
        ROS_WARN("TtlManager::writeSingleCommand - Failed to write a single command on motor id : %d", id);
        _debug_error_message = "TtlManager - Failed to write a single command";
    }

    return result;
}

/**
 * @brief TtlManager::executeJointTrajectoryCmd
 * @param cmd_vec
 */
void TtlManager::executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, uint32_t> > cmd_vec)
{
    // using only one driver for all motors to write faster. (all addresses of goal position are the same)
    if (!_driver_map.empty())
    {
        std::shared_ptr<ttl_driver::AbstractMotorDriver> driver;
        try {
            auto ttl_driver = _driver_map.at(_state_map.at(_motor_list.at(0))->getHardwareType());
            driver = std::dynamic_pointer_cast<AbstractMotorDriver>(ttl_driver);
        }
        catch (const std::exception& e)
        {
            std::stringstream msg;
            std::cout << "TtlManager::executeJointTrajectoryCmd: access to an element not exist in map with error " << e.what() << std::endl;
        }
        if (driver)
        {
            std::vector<uint8_t> ids;
            std::vector<uint32_t> params;
            for (auto cmd : cmd_vec)
            {
                ids.push_back(cmd.first);
                params.push_back(cmd.second);
            }
            int err = driver->syncWritePositionGoal(ids, params);
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

    for (auto const& s : _state_map)
    {
        if (s.second && EHardwareType::STEPPER == s.second->getHardwareType() && !std::dynamic_pointer_cast<StepperMotorState>(s.second)->isConveyor())
            std::dynamic_pointer_cast<StepperMotorState>(s.second)->setCalibration(EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS, 0);
    }

    _calibration_status = EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS;
}

/**
 * @brief TtlManager::resetCalibration
 */
void TtlManager::resetCalibration()
{
    ROS_DEBUG("TtlManager::resetCalibration: reseting...");

    _calibration_status = EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
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

/**
 * @brief TtlManager::updateCurrentCalibrationStatus
 */
void TtlManager::updateCurrentCalibrationStatus()
{
    EStepperCalibrationStatus newStatus = EStepperCalibrationStatus::CALIBRATION_OK;
    // update current state of the calibrationtimeout_motors
    // rule is : if a status in a motor is worse than one previously found, we take it
    // we are not taking "uninitialized" into account as it means a calibration as not been started for this motor
    for (auto const& s : _state_map)
    {
        if (s.second && (s.second->getHardwareType() == EHardwareType::STEPPER
                         || s.second->getHardwareType() == EHardwareType::FAKE_STEPPER_MOTOR))
        {
            auto sState = std::dynamic_pointer_cast<StepperMotorState>(s.second);
            if (sState && !sState->isConveyor())
            {
              EStepperCalibrationStatus status = std::dynamic_pointer_cast<StepperMotorState>(s.second)->getCalibrationState();
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
 * @brief TtlManager::getBusState
 * @param connection_state
 * @param motor_id
 * @param debug_msg
 */
void TtlManager::getBusState(bool &connection_state,
                             std::vector<uint8_t> &motor_id,
                             std::string &debug_msg) const
{
  debug_msg = _debug_error_message;
  motor_id = _all_motor_connected;
  connection_state = isConnectionOk();
}

/**
 * @brief TtlManager::getMotorsStates
 * @return only the joints states
 */
std::vector<std::shared_ptr<JointState> >
TtlManager::getMotorsStates() const
{
    std::vector<std::shared_ptr<JointState> > states;
    for (const auto& it : _state_map)
    {
        if (EHardwareType::UNKNOWN != it.second->getHardwareType()
            && EHardwareType::END_EFFECTOR != it.second->getHardwareType())
        {
            states.emplace_back(std::dynamic_pointer_cast<JointState>(it.second));
        }
    }

    return states;
}

/**
 * @brief TtlManager::getHardwareState
 * @param motor_id
 * @return
 */
std::shared_ptr<common::model::AbstractHardwareState>
TtlManager::getHardwareState(uint8_t motor_id) const
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
void TtlManager::addHardwareDriver(common::model::EHardwareType hardware_type)
{
    // if not already instanciated
    if (!_driver_map.count(hardware_type))
    {
        switch (hardware_type)
        {
            case common::model::EHardwareType::STEPPER:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<StepperDriver<StepperReg> >(_portHandler, _packetHandler)));
            break;
            case common::model::EHardwareType::XL430:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XL430Reg> >(_portHandler, _packetHandler)));
            break;
            case common::model::EHardwareType::XC430:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XC430Reg> >(_portHandler, _packetHandler)));
            break;
            case common::model::EHardwareType::XL320:
                _driver_map.insert(make_pair(hardware_type, std::make_shared<DxlDriver<XL320Reg> >(_portHandler, _packetHandler)));
            break;
            case common::model::EHardwareType::XL330:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<DxlDriver<XL330Reg> >(_portHandler, _packetHandler)));
            break;
            case common::model::EHardwareType::END_EFFECTOR:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<EndEffectorDriver<EndEffectorReg> >(_portHandler, _packetHandler)));
            break;
            case common::model::EHardwareType::FAKE_DXL_MOTOR:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<MockDxlDriver>(_fake_data)));
            break;
            case common::model::EHardwareType::FAKE_STEPPER_MOTOR:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<MockStepperDriver>(_fake_data)));
            break;
            case common::model::EHardwareType::FAKE_END_EFFECTOR:
                _driver_map.insert(std::make_pair(hardware_type, std::make_shared<MockEndEffectorDriver>(_fake_data)));
            break;
            default:
                ROS_ERROR("TtlManager - Unable to instanciate driver, unknown type");
            break;
        }
    }
}
/**
 * @brief TtlManager::checkRemovedMotors
 */
void TtlManager::checkRemovedMotors()
{
    // get list of ids
    std::vector<uint8_t> motor_list;
    for (auto const& istate : _state_map)
    {
        auto it = find(_all_motor_connected.begin(), _all_motor_connected.end(), istate.first);
        if (it == _all_motor_connected.end())
            motor_list.emplace_back(istate.first);
    }
    _removed_motor_id_list = motor_list;
}

 /**
 * @brief TtlManager::readFakeConfig
 */
void TtlManager::readFakeConfig()
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
            std::string current_ns = "fake_params/end_effector/";
            int id, temperature, voltage;
            _nh.getParam(current_ns + "id", id);
            _fake_data->end_effector.id = static_cast<uint8_t>(id);
            _nh.getParam(current_ns + "temperature", temperature);
            _fake_data->end_effector.temperature = static_cast<uint32_t>(temperature);
            _nh.getParam(current_ns + "voltage", voltage);
            _fake_data->end_effector.voltage = static_cast<double>(voltage);

            std::string firmware;
            _nh.getParam(current_ns + "firmware", firmware);
            _fake_data->end_effector.firmware = firmware;
        }
    }
}

}  // namespace ttl_driver
