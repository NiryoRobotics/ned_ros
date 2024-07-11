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

#include "ttl_driver/mock_dxl_driver.hpp"
#include "dynamixel_sdk/packet_handler.h"
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ttl_driver
{

/**
 * @brief MockDxlDriver::MockDxlDriver
 */
MockDxlDriver::MockDxlDriver(std::shared_ptr<FakeTtlData> data) : _fake_data(std::move(data))
{
    // retrieve list of ids
    for (auto const &imap : _fake_data->dxl_registers)
        _id_list.emplace_back(imap.first);
}

/**
 * @brief MockDxlDriver::str
 * @return
 */
std::string MockDxlDriver::str() const { return "Mock Dynamixel Driver (OK)"; }

//*****************************
// AbstractTtlDriver interface
//*****************************

/**
 * @brief MockDxlDriver::ping
 * @param id
 * @return
 */
int MockDxlDriver::ping(uint8_t id)
{
    if (std::find(_fake_data->full_id_list.begin(), _fake_data->full_id_list.end(), id) != _fake_data->full_id_list.end())
        return COMM_SUCCESS;
    return COMM_TX_FAIL;
}

/**
 * @brief MockDxlDriver::getModelNumber
 * @param id
 * @param model_number
 * @return
 */
int MockDxlDriver::getModelNumber(uint8_t id, uint16_t &model_number)
{
    if (_fake_data->dxl_registers.count(id))
        model_number = _fake_data->dxl_registers.at(id).model_number;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::checkModelNumber
 * @param id
 * @return
 */
int MockDxlDriver::checkModelNumber(uint8_t id)
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
 * @brief MockDxlDriver::scan
 * @param id_list
 * @return
 */
int MockDxlDriver::scan(std::vector<uint8_t> &id_list)
{
    id_list = _fake_data->full_id_list;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::reboot
 * @param id
 * @return
 */
int MockDxlDriver::reboot(uint8_t id) { return ping(id); }

/**
 * @brief MockDxlDriver::interpretErrorState
 * @return
 */
std::string MockDxlDriver::interpretErrorState(uint32_t /*hw_state*/) const { return ""; }

/**
 * @brief MockDxlDriver::readCustom
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int MockDxlDriver::readCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t &data)
{
    (void)address;   // unused
    (void)data_len;  // unused
    (void)id;        // unused
    (void)data;      // unused

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::writeCustom
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int MockDxlDriver::writeCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t data)
{
    (void)address;   // unused
    (void)data_len;  // unused
    (void)id;        // unused
    (void)data;      // unused

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::changeId
 * @param id
 * @param new_id
 * @return
 */
int MockDxlDriver::changeId(uint8_t id, uint8_t new_id)
{
    (void)id;      // unused
    (void)new_id;  // unused

    return COMM_TX_FAIL;
}

/**
 * @brief MockDxlDriver::writeStartupConfiguration
 * @param id
 * @param value
 * @return
 */
int MockDxlDriver::writeStartupConfiguration(uint8_t id, uint8_t value)
{
    (void)id;     // unused
    (void)value;  // unused

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::writeTemperatureLimit
 * @param id
 * @param temperature_limit
 * @return
 */
int MockDxlDriver::writeTemperatureLimit(uint8_t id, uint8_t temperature_limit)
{
    (void)id;                 // unused
    (void)temperature_limit;  // unused

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::writeShutdownConfiguration
 * @param id
 * @param configuration
 * @return
 */
int MockDxlDriver::writeShutdownConfiguration(uint8_t id, uint8_t configuration)
{
    (void)id;             // unused
    (void)configuration;  // unused

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
int MockDxlDriver::readFirmwareVersion(uint8_t id, std::string &version)
{
    if (_fake_data->dxl_registers.count(id))
        version = _fake_data->dxl_registers.at(id).firmware;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readMinPosition
 * @param id
 * @param pos
 * @return
 */
int MockDxlDriver::readMinPosition(uint8_t id, uint32_t &pos)
{
    if (_fake_data->dxl_registers.count(id))
        pos = _fake_data->dxl_registers.at(id).min_position;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readMaxPosition
 * @param id
 * @param pos
 * @return
 */
int MockDxlDriver::readMaxPosition(uint8_t id, uint32_t &pos)
{
    if (_fake_data->dxl_registers.count(id))
        pos = _fake_data->dxl_registers.at(id).max_position;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

// ram write

/**
 * @brief MockDxlDriver::writeTorquePercentage
 * @param id
 * @param torque_enable
 * @return
 */
int MockDxlDriver::writeTorquePercentage(uint8_t id, uint8_t torque_enable)
{
    if (_fake_data->dxl_registers.count(id))
        _fake_data->dxl_registers.at(id).torque = torque_enable;
    else if (_fake_data->stepper_registers.count(id))
        _fake_data->stepper_registers.at(id).torque = torque_enable;
    else
        return COMM_TX_ERROR;

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::writePositionGoal
 * @param id
 * @param position
 * @return
 */
int MockDxlDriver::writePositionGoal(uint8_t id, uint32_t position)
{
    if (_fake_data->dxl_registers.count(id))
        _fake_data->dxl_registers.at(id).position = position;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::writeVelocityGoal
 * @param id
 * @param velocity
 * @return
 */
int MockDxlDriver::writeVelocityGoal(uint8_t id, uint32_t velocity)
{
    if (_fake_data->dxl_registers.count(id))
        _fake_data->dxl_registers.at(id).velocity = velocity;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::writeVelocityProfile
 * @param id
 * @param data_list
 * @return
 */
int MockDxlDriver::writeVelocityProfile(uint8_t id, const std::vector<uint32_t> &data_list)
{
    (void)data_list;
    int res = COMM_RX_FAIL;
    if (_fake_data->dxl_registers.count(id))
    {
        res = COMM_SUCCESS;
    }

    return res;
}

/**
 * @brief MockDxlDriver::syncWriteTorquePercentage
 * @param id_list
 * @param torque_percentage_list
 * @return
 */
int MockDxlDriver::syncWriteTorquePercentage(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &torque_percentage_list)
{
    // Create a map to store the frequency of each element in vector
    std::set<uint8_t> countSet;
    // Iterate over the vector and store the frequency of each element in map
    for (size_t i = 0; i < id_list.size(); i++)
    {
        auto result = countSet.insert(id_list.at(i));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
        writeTorquePercentage(id_list.at(i), torque_percentage_list.at(i));
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWritePositionGoal get position goal and write it as the current position of each joint
 * @param id_list
 * @param position_list
 * @return
 */
int MockDxlDriver::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    if (id_list.size() != position_list.size())
        return LEN_ID_DATA_NOT_SAME;

    // Create a map to store the frequency of each element in id_list. It helps find out which ID is redondant
    std::set<uint8_t> countSet;
    for (size_t i = 0; i < id_list.size(); i++)
    {
        if (_fake_data->dxl_registers.count(id_list.at(i)))
            _fake_data->dxl_registers.at(id_list.at(i)).position = position_list.at(i);
        else if (_fake_data->stepper_registers.count(id_list.at(i)))
            _fake_data->stepper_registers.at(id_list.at(i)).position = position_list.at(i);
        else
            return COMM_TX_ERROR;
        // write goal position as the current position

        auto result = countSet.insert(id_list[i]);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWriteVelocityGoal
 * @param id_list
 * @param velocity_list
 * @return
 */
int MockDxlDriver::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
{
    if (id_list.size() != velocity_list.size())
        return LEN_ID_DATA_NOT_SAME;

    // Create a map to store the frequency of each element in id_list. It helps find out which ID is redondant
    std::set<uint8_t> countSet;
    for (size_t i = 0; i < id_list.size(); i++)
    {
        if (!_fake_data->dxl_registers.count(id_list.at(i)))
            return COMM_TX_ERROR;
        // write goal position as the current position
        _fake_data->dxl_registers.at(id_list.at(i)).velocity = velocity_list.at(i);

        auto result = countSet.insert(id_list[i]);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::readVelocityProfile(uint8_t id, std::vector<uint32_t> &data_list)
{
    data_list.clear();
    if (_fake_data->dxl_registers.count(id))
        return COMM_RX_FAIL;

    data_list.emplace_back(0);
    data_list.emplace_back(0);

    return COMM_SUCCESS;
}

// ram read

/**
 * @brief MockDxlDriver::readPosition
 * @param id
 * @param present_position
 * @return
 */
int MockDxlDriver::readPosition(uint8_t id, uint32_t &present_position)
{
    if (_fake_data->dxl_registers.count(id))
        present_position = _fake_data->dxl_registers.at(id).position;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readVelocity
 * @param id
 * @param present_velocity
 * @return
 */
int MockDxlDriver::readVelocity(uint8_t id, uint32_t &present_velocity)
{
    if (_fake_data->dxl_registers.count(id))
        present_velocity = _fake_data->dxl_registers.at(id).velocity;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readTemperature
 * @param id
 * @param temperature
 * @return
 */
int MockDxlDriver::readTemperature(uint8_t id, uint8_t &temperature)
{
    if (_fake_data->dxl_registers.count(id))
        temperature = _fake_data->dxl_registers.at(id).temperature;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readVoltage
 * @param id
 * @param voltage
 * @return
 */
int MockDxlDriver::readVoltage(uint8_t id, double &voltage)
{
    if (_fake_data->dxl_registers.count(id))
        voltage = _fake_data->dxl_registers.at(id).voltage;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readHwErrorStatus
 * @param id
 * @param hardware_error_status
 * @return
 */
int MockDxlDriver::readHwErrorStatus(uint8_t id, uint8_t &hardware_error_status)
{
    if (_fake_data->dxl_registers.count(id))
        return COMM_RX_FAIL;

    hardware_error_status = 0;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadPosition
 * @param id_list
 * @param position_list
 * @return
 */
int MockDxlDriver::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
            position_list.emplace_back(_fake_data->dxl_registers.at(id).position);
        else if (_fake_data->stepper_registers.count(id))
            position_list.emplace_back(_fake_data->stepper_registers.at(id).position);
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadVelocity
 * @param id_list
 * @param velocity_list
 * @return
 */
int MockDxlDriver::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
            velocity_list.emplace_back(_fake_data->dxl_registers.at(id).velocity);
        else if (_fake_data->stepper_registers.count(id))
            velocity_list.emplace_back(_fake_data->stepper_registers.at(id).velocity);
        else
            return COMM_TX_ERROR;
        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadJointStatus
 * @param id_list
 * @param data_array_list
 * @return
 */
int MockDxlDriver::syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2>> &data_array_list)
{
    std::set<uint8_t> countSet;
    data_array_list.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
        {
            std::array<uint32_t, 2> blocks{};

            blocks.at(0) = _fake_data->dxl_registers.at(id).velocity;
            blocks.at(1) = _fake_data->dxl_registers.at(id).position;

            data_array_list.emplace_back(blocks);
        }
        else if (_fake_data->stepper_registers.count(id))
        {
            std::array<uint32_t, 2> blocks{};

            blocks.at(0) = _fake_data->stepper_registers.at(id).velocity;
            blocks.at(1) = _fake_data->stepper_registers.at(id).position;

            data_array_list.emplace_back(blocks);
        }
        else
            return COMM_RX_FAIL;
        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadFirmwareVersion
 * @param id_list
 * @param firmware_list
 * @return
 */
int MockDxlDriver::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        if (!_fake_data->dxl_registers.count(id))
            return COMM_TX_ERROR;

        firmware_list.emplace_back(_fake_data->dxl_registers.at(id).firmware);

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
int MockDxlDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &temperature_list)
{
    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        if (!_fake_data->dxl_registers.count(id))
            return COMM_TX_ERROR;

        temperature_list.emplace_back(_fake_data->dxl_registers.at(id).temperature);

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
int MockDxlDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        if (!_fake_data->dxl_registers.count(id))
            return COMM_TX_ERROR;

        voltage_list.emplace_back(_fake_data->dxl_registers.at(id).voltage);

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadRawVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
int MockDxlDriver::syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) { return syncReadVoltage(id_list, voltage_list); }

/**
 * @brief MockDxlDriver::syncReadHwStatus
 * @param id_list
 * @param data_list
 * @return
 */
int MockDxlDriver::syncReadHwStatus(const std::vector<uint8_t> &id_list, std::vector<std::pair<double, uint8_t>> &data_list)
{
    data_list.clear();

    std::set<uint8_t> countSet;

    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
        {
            double voltage = _fake_data->dxl_registers.at(id).voltage;
            uint8_t temperature = _fake_data->dxl_registers.at(id).temperature;
            data_list.emplace_back(std::make_pair(voltage, temperature));
        }
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
int MockDxlDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list)
{
    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        hw_error_list.emplace_back(0);
        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readPID
 * @param id
 * @param data
 * @return
 */
int MockDxlDriver::readPID(uint8_t id, std::vector<uint16_t> &data)
{
    int result = COMM_RX_FAIL;

    data.clear();
    if (_fake_data->dxl_registers.count(id))
    {
        data.emplace_back(_fake_data->dxl_registers.at(id).position_p_gain);
        data.emplace_back(_fake_data->dxl_registers.at(id).position_i_gain);
        data.emplace_back(_fake_data->dxl_registers.at(id).position_d_gain);
        data.emplace_back(_fake_data->dxl_registers.at(id).velocity_p_gain);
        data.emplace_back(_fake_data->dxl_registers.at(id).velocity_i_gain);
        data.emplace_back(_fake_data->dxl_registers.at(id).ff1_gain);
        data.emplace_back(_fake_data->dxl_registers.at(id).ff2_gain);

        result = COMM_SUCCESS;
    }

    return result;
}

/**
 * @brief MockDxlDriver::writePID
 * @param id
 * @param data
 * @return
 */
int MockDxlDriver::writePID(uint8_t id, const std::vector<uint16_t> &data)
{
    int result = COMM_RX_FAIL;

    if (_fake_data->dxl_registers.count(id))
    {
        _fake_data->dxl_registers.at(id).position_p_gain = data.at(0);
        _fake_data->dxl_registers.at(id).position_i_gain = data.at(1);
        _fake_data->dxl_registers.at(id).position_d_gain = data.at(2);
        _fake_data->dxl_registers.at(id).velocity_p_gain = data.at(3);
        _fake_data->dxl_registers.at(id).velocity_i_gain = data.at(4);
        _fake_data->dxl_registers.at(id).ff1_gain = data.at(5);
        _fake_data->dxl_registers.at(id).ff2_gain = data.at(6);

        result = COMM_SUCCESS;
    }

    return result;
}

/**
 * @brief MockDxlDriver::writeControlMode
 * @param id
 * @param data
 * @return
 */
int MockDxlDriver::writeControlMode(uint8_t id, uint8_t data)
{
    (void)data;  // unused

    if (!_fake_data->dxl_registers.count(id))
        return COMM_TX_ERROR;

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readControlMode
 * @param id
 * @param data
 * @return
 */
int MockDxlDriver::readControlMode(uint8_t id, uint8_t &data)
{
    (void)data;  // unused

    if (!_fake_data->dxl_registers.count(id))
        return COMM_TX_ERROR;

    return COMM_SUCCESS;
}

//*****************************
// AbstractDxlDriver interface
//*****************************
/**
 * @brief MockDxlDriver::writeLed
 * @param id
 * @param led_value
 * @return
 */
int MockDxlDriver::writeLed(uint8_t id, uint8_t led_value)
{
    (void)led_value;  // unused

    if (!_fake_data->dxl_registers.count(id))
        return COMM_TX_ERROR;

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWriteLed
 * @param id_list
 * @param led_list
 * @return
 */
int MockDxlDriver::syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &led_list)
{
    (void)led_list;  // unused

    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        if (!_fake_data->dxl_registers.count(id))
            return COMM_TX_ERROR;
        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::writeTorqueGoal
 * @param id
 * @param torque
 * @return
 */
int MockDxlDriver::writeTorqueGoal(uint8_t id, uint16_t torque)
{
    (void)torque;  // unused

    if (!_fake_data->dxl_registers.count(id))
        return COMM_TX_ERROR;

    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWriteTorqueGoal
 * @param id_list
 * @param torque_list
 * @return
 */
int MockDxlDriver::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint16_t> &torque_list)
{
    (void)torque_list;  // unused

    std::set<uint8_t> countSet;
    for (auto &id : id_list)
    {
        if (!_fake_data->dxl_registers.count(id))
            return COMM_TX_ERROR;

        auto result = countSet.insert(id);

        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readLoad
 * @param id
 * @param present_load
 * @return
 */
int MockDxlDriver::readLoad(uint8_t id, uint16_t &present_load)
{
    (void)present_load;  // unused

    if (_fake_data->dxl_registers.count(id))
        return COMM_SUCCESS;
    return COMM_RX_FAIL;
}

/**
 * @brief MockDxlDriver::readMoving
 * @param id
 * @param status
 * @return
 */
int MockDxlDriver::readMoving(uint8_t id, uint8_t &status)
{
    (void)status;  // unused

    if (_fake_data->dxl_registers.count(id))
        return COMM_SUCCESS;
    return COMM_RX_FAIL;
}

/**
 * @brief MockDxlDriver::syncReadLoad
 * @param id_list
 * @param load_list
 * @return
 */
int MockDxlDriver::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint16_t> &load_list)
{
    load_list = {};
    for (size_t i = 0; i < id_list.size(); i++)
        load_list.emplace_back(0);
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::interpretFirmwareVersion
 * @param fw_version
 * @return
 */
std::string MockDxlDriver::interpretFirmwareVersion(uint32_t fw_version) const { return std::to_string(fw_version); }

}  // namespace ttl_driver
