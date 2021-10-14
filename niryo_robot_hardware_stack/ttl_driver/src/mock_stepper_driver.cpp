/*
mock_dxl_driver.cpp
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

#include <cstdint>
#include <cstdio>
#include "dynamixel_sdk/packet_handler.h"
#include "ttl_driver/mock_stepper_driver.hpp"
#include <type_traits>
#include <utility>
#include <map>
#include <string>
#include <vector>

namespace ttl_driver
{
// definition of methods

/**
 * @brief MockStepperDriver::MockStepperDriver
 * @param data
 */
MockStepperDriver::MockStepperDriver(std::shared_ptr<FakeTtlData>  data) :
  _fake_data(std::move(data))
{
  // retrieve list of ids
  for (auto const& imap : _fake_data->stepper_registers)
      _id_list.emplace_back(imap.first);
}

/**
 * @brief MockStepperDriver::str
 * @return
 */
std::string MockStepperDriver::str() const
{
  return "Mock Stepper Driver (OK)";
}

//*****************************
// AbstractTtlDriver interface
//*****************************
/**
 * @brief MockStepperDriver::ping
 * @param id
 * @return
 */
int MockStepperDriver::ping(uint8_t id)
{
    if (std::find(_id_list.begin(), _id_list.end(), id) != _id_list.end())
        return COMM_SUCCESS;
    return COMM_TX_FAIL;
}

/**
 * @brief MockStepperDriver::getModelNumber
 * @param id
 * @param model_number
 * @return
 */
int MockStepperDriver::getModelNumber(uint8_t id, uint16_t& model_number)
{
    if (_fake_data->stepper_registers.count(id))
        model_number = _fake_data->stepper_registers.at(id).model_number;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::scan
 * @param id_list
 * @return
 */
int MockStepperDriver::scan(std::vector<uint8_t>& id_list)
{
    id_list = _fake_data->full_id_list;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::reboot
 * @param id
 * @return
 */
int MockStepperDriver::reboot(uint8_t id)
{
    if (std::find(_id_list.begin(), _id_list.end(), id) == _id_list.end())
        return COMM_TX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::interpreteErrorState
 * @return
 */
std::string MockStepperDriver::interpreteErrorState(uint32_t /*hw_state*/) const
{
    return "";
}

/**
 * @brief MockStepperDriver::changeId
 * @param id
 * @param new_id
 * @return
 */
int MockStepperDriver::changeId(uint8_t id, uint8_t new_id)
{
    if (std::find(_id_list.begin(), _id_list.end(), id) != _id_list.end() &&
        std::find(_fake_data->full_id_list.begin(), _fake_data->full_id_list.end(), id) != _fake_data->full_id_list.end())
    {
        _id_list.erase(std::remove(_id_list.begin(), _id_list.end(), id), _id_list.end());
        _fake_data->full_id_list.erase(std::remove(_fake_data->full_id_list.begin(), _fake_data->full_id_list.end(), id), _fake_data->full_id_list.end());
        _id_list.emplace_back(new_id);
        _fake_data->full_id_list.emplace_back(new_id);

        return COMM_SUCCESS;
    }

    return COMM_TX_FAIL;
}

/**
 * @brief MockStepperDriver::checkModelNumber
 * @param id
 * @return
 */
int MockStepperDriver::checkModelNumber(uint8_t id)
{
    uint16_t model_number = 0;
    int ping_result = getModelNumber(id, model_number);

    if (ping_result == COMM_SUCCESS)
    {
        if (model_number)
        {
            return PING_WRONG_MODEL_NUMBER;
        }
    }

    return ping_result;
}

/**
 * @brief MockStepperDriver::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
int MockStepperDriver::readFirmwareVersion(uint8_t id, std::string &version)
{
    if (_fake_data->stepper_registers.count(id))
        version = _fake_data->stepper_registers.at(id).firmware;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readMinPosition
 * @param id
 * @param pos
 * @return
 */
int MockStepperDriver::readMinPosition(uint8_t id, uint32_t &pos)
{
    if (_fake_data->stepper_registers.count(id))
        pos = _fake_data->stepper_registers.at(id).min_position;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readMaxPosition
 * @param id
 * @param pos
 * @return
 */
int MockStepperDriver::readMaxPosition(uint8_t id, uint32_t &pos)
{
    if (_fake_data->stepper_registers.count(id))
        pos = _fake_data->stepper_registers.at(id).max_position;
    return COMM_SUCCESS;
}

// ram write

/**
 * @brief MockStepperDriver::setTorqueEnable
 * @param id
 * @param torque_enable
 * @return
 */
int MockStepperDriver::setTorqueEnable(uint8_t id, uint32_t /*torque_enable*/)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::setGoalPosition
 * @param id
 * @param position
 * @return
 */
int MockStepperDriver::setGoalPosition(uint8_t id, uint32_t position)
{
    if (_fake_data->stepper_registers.count(id))
        _fake_data->stepper_registers.at(id).position = position;
    return COMM_SUCCESS;
}

// according to the registers, the data should be an int32_t ?
/**
 * @brief MockStepperDriver::setGoalVelocity
 * @param id
 * @param velocity
 * @return
 */
int MockStepperDriver::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    if (_fake_data->stepper_registers.count(id))
        _fake_data->stepper_registers.at(id).velocity = velocity;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncWriteTorqueEnable
 * @param id_list
 * @param torque_enable_list
 * @return
 */
int MockStepperDriver::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &/*torque_enable_list*/)
{
    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;
    // Iterate over the vector and store the frequency of each element in map
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncWritePositionGoal
 * @param id_list
 * @param position_list
 * @return
 */
int MockStepperDriver::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    if (id_list.size() != position_list.size())
        return LEN_ID_DATA_NOT_SAME;

    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;
    for (size_t i = 0; i < id_list.size(); ++i)
    {
        if (!_fake_data->stepper_registers.count(id_list.at(i)))
            return COMM_TX_FAIL;
        _fake_data->stepper_registers.at(id_list.at(i)).position = position_list.at(i);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id_list.at(i), 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncWriteVelocityGoal
 * @param id_list
 * @param velocity_list
 * @return
 */
int MockStepperDriver::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &/*velocity_list*/)
{
    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;
    // Iterate over the vector and store the frequency of each element in map
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

// ram read

/**
 * @brief MockStepperDriver::readPosition
 * @param id
 * @param present_position
 * @return
 */
int MockStepperDriver::readPosition(uint8_t id, uint32_t& present_position)
{
    if (_fake_data->stepper_registers.count(id))
        present_position = _fake_data->stepper_registers.at(id).position;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readTemperature
 * @param id
 * @param temperature
 * @return
 */
int MockStepperDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    if (_fake_data->stepper_registers.count(id))
        temperature = _fake_data->stepper_registers.at(id).temperature;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readVoltage
 * @param id
 * @param voltage
 * @return
 */
int MockStepperDriver::readVoltage(uint8_t id, double &voltage)
{
    if (_fake_data->stepper_registers.count(id))
        voltage = _fake_data->stepper_registers.at(id).voltage;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readHwErrorStatus
 * @param id
 * @param hardware_status
 * @return
 */
int MockStepperDriver::readHwErrorStatus(uint8_t /*id*/, uint32_t& hardware_status)
{
    hardware_status = 0;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncReadPosition
 * @param id_list
 * @param position_list
 * @return
 */
int MockStepperDriver::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    std::map<uint8_t, uint8_t> countMap;
    position_list.clear();
    for (auto & id : id_list)
    {
        if (_fake_data->stepper_registers.count(id))
        {
            position_list.emplace_back(_fake_data->stepper_registers.at(id).position);
        }
        else
            return COMM_RX_FAIL;
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncReadFirmwareVersion
 * @param id_list
 * @param firmware_list
 * @return
 */
int MockStepperDriver::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    std::map<uint8_t, uint8_t> countMap;
    firmware_list.clear();
    for (auto & id : id_list)
    {
        if (_fake_data->stepper_registers.count(id))
            firmware_list.emplace_back(_fake_data->stepper_registers.at(id).firmware);
        else
            return COMM_RX_FAIL;
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
int MockStepperDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    std::map<uint8_t, uint8_t> countMap;
    temperature_list.clear();
    for (auto & id : id_list)
    {
        if (_fake_data->stepper_registers.count(id))
            temperature_list.emplace_back(_fake_data->stepper_registers.at(id).temperature);
        else
            return COMM_RX_FAIL;
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncReadVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
int MockStepperDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    std::map<uint8_t, uint8_t> countMap;
    voltage_list.clear();
    for (auto & id : id_list)
    {
        if (_fake_data->stepper_registers.count(id))
            voltage_list.emplace_back(_fake_data->stepper_registers.at(id).voltage);
        else
            return COMM_RX_FAIL;
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
int MockStepperDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    std::map<uint8_t, uint8_t> countMap;
    hw_error_list.clear();
    for (auto & id : id_list)
    {
        hw_error_list.emplace_back(0);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

//*****************************
// AbstractStepperDriver interface
//*****************************

/**
 * @brief MockStepperDriver::writeVelocityProfile
 * @param id
 * @param data
 * @return
 */
int MockStepperDriver::writeVelocityProfile(uint8_t id, const std::vector<uint32_t>& /*data*/)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::startHoming
 * @param id
 * @return
 */
int MockStepperDriver::startHoming(uint8_t id)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    _calibration_status = CALIBRATION_IN_PROGRESS;
    fake_time = 3;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::writeHomingDirection
 * @param id
 * @param direction
 * @return
 */
int MockStepperDriver::writeHomingDirection(uint8_t id, uint8_t /*direction*/)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readHomingStatus
 * @param id
 * @param status
 * @return
 */
int MockStepperDriver::readHomingStatus(uint8_t id, uint32_t &status)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    if (fake_time)
    {
        fake_time--;
    }
    else
        _calibration_status = CALIBRATION_SUCCESS;

    status = _calibration_status;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readGoalVelocity
 * @param id
 * @param present_velocity
 * @return
 */
int MockStepperDriver::readGoalVelocity(uint8_t id, uint32_t& present_velocity)
{
    if (_fake_data->stepper_registers.count(id))
        present_velocity = _fake_data->stepper_registers.at(id).velocity;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readFirmwareRunning
 * @param id
 * @param is_running
 * @return
 */
int MockStepperDriver::readFirmwareRunning(uint8_t id, bool &is_running)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    is_running = true;
    return COMM_SUCCESS;
}

}  // namespace ttl_driver
