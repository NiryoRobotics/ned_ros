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

#include "ttl_driver/mock_stepper_driver.hpp"
#include "dynamixel_sdk/packet_handler.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ttl_driver
{
// definition of methods

/**
 * @brief MockStepperDriver::MockStepperDriver
 * @param data
 */
MockStepperDriver::MockStepperDriver(std::shared_ptr<FakeTtlData> data) : _fake_data(std::move(data)) { init(); }

/**
 * @brief MockStepperDriver::init
 * @return
 */
bool MockStepperDriver::init()
{
    bool res = false;

    if (_fake_data)
    {
        // retrieve list of ids
        for (auto const &imap : _fake_data->stepper_registers)
            _id_list.emplace_back(imap.first);

        res = true;
    }
    else
    {
        std::cout << "ERROR : Fake data not initialized" << std::endl;
    }

    return res;
}
/**
 * @brief MockStepperDriver::str
 * @return
 */
std::string MockStepperDriver::str() const { return "Mock Stepper Driver (OK)"; }

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
    if (std::find(_fake_data->full_id_list.begin(), _fake_data->full_id_list.end(), id) != _fake_data->full_id_list.end())
        return COMM_SUCCESS;
    return COMM_TX_FAIL;
}

/**
 * @brief MockStepperDriver::getModelNumber
 * @param id
 * @param model_number
 * @return
 */
int MockStepperDriver::getModelNumber(uint8_t id, uint16_t &model_number)
{
    if (_fake_data->stepper_registers.count(id))
        model_number = _fake_data->stepper_registers.at(id).model_number;
    return COMM_SUCCESS;
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
 * @brief MockStepperDriver::scan
 * @param id_list
 * @return
 */
int MockStepperDriver::scan(std::vector<uint8_t> &id_list)
{
    // full id list using only for scan
    id_list = _fake_data->full_id_list;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::reboot
 * @param id
 * @return
 */
int MockStepperDriver::reboot(uint8_t id) { return ping(id); }

/**
 * @brief MockStepperDriver::changeId
 * @param id
 * @param new_id
 * @return
 */
int MockStepperDriver::changeId(uint8_t id, uint8_t new_id)
{
    int result = COMM_TX_FAIL;
    if (std::find(_id_list.begin(), _id_list.end(), id) != _id_list.end() &&
        std::find(_fake_data->full_id_list.begin(), _fake_data->full_id_list.end(), id) != _fake_data->full_id_list.end())
    {
        _id_list.erase(std::remove(_id_list.begin(), _id_list.end(), id), _id_list.end());
        _fake_data->full_id_list.erase(std::remove(_fake_data->full_id_list.begin(), _fake_data->full_id_list.end(), id), _fake_data->full_id_list.end());
        _id_list.emplace_back(new_id);
        _fake_data->full_id_list.emplace_back(new_id);

        result = COMM_SUCCESS;
    }

    const auto it = _fake_data->stepper_registers.find(id);
    if (it != _fake_data->stepper_registers.end())
    {
        std::swap(_fake_data->stepper_registers[new_id], it->second);
        result = COMM_SUCCESS;
    }
    else
        result = COMM_TX_FAIL;

    return result;
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
    else
        return COMM_RX_FAIL;
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
    else
        return COMM_RX_FAIL;
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
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

// ram write

/**
 * @brief MockStepperDriver::writeTorquePercentage
 * @param id
 * @param torque_enable
 * @return
 */
int MockStepperDriver::writeTorquePercentage(uint8_t id, uint8_t /*torque_percentage*/)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::writePositionGoal
 * @param id
 * @param position
 * @return
 */
int MockStepperDriver::writePositionGoal(uint8_t id, uint32_t position)
{
    if (_fake_data->stepper_registers.count(id))
        _fake_data->stepper_registers.at(id).position = position;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

// according to the registers, the data should be an int32_t ?
/**
 * @brief MockStepperDriver::writeVelocityGoal
 * @param id
 * @param velocity
 * @return
 */
int MockStepperDriver::writeVelocityGoal(uint8_t id, uint32_t velocity)
{
    if (_fake_data->stepper_registers.count(id))
        _fake_data->stepper_registers.at(id).velocity = velocity;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncWriteTorquePercentage
 * @param id_list
 * @param torque_percentage_list
 * @return
 */
int MockStepperDriver::syncWriteTorquePercentage(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> & /*torque_percentage_list*/)
{
    // Create a map to store the frequency of each element in vector
    std::set<uint8_t> countSet;

    // Iterate over the vector and store the frequency of each element in map
    for (auto &id : id_list)
    {
        auto result = countSet.insert(id);
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
    std::set<uint8_t> countSet;

    for (size_t i = 0; i < id_list.size(); ++i)
    {
        if (_fake_data->dxl_registers.count(id_list.at(i)))
            _fake_data->dxl_registers.at(id_list.at(i)).position = position_list.at(i);
        else if (_fake_data->stepper_registers.count(id_list.at(i)))
            _fake_data->stepper_registers.at(id_list.at(i)).position = position_list.at(i);
        else
            return COMM_TX_ERROR;

        auto result = countSet.insert(id_list.at(i));
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
int MockStepperDriver::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> & /*velocity_list*/)
{
    // Create a map to store the frequency of each element in vector
    std::set<uint8_t> countSet;

    // Iterate over the vector and store the frequency of each element in map
    for (auto &id : id_list)
    {
        auto result = countSet.insert(id);
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
int MockStepperDriver::readPosition(uint8_t id, uint32_t &present_position)
{
    if (_fake_data->stepper_registers.count(id))
        present_position = _fake_data->stepper_registers.at(id).position;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readVelocity
 * @param id
 * @param present_velocity
 * @return
 */
int MockStepperDriver::readVelocity(uint8_t id, uint32_t &present_velocity)
{
    if (_fake_data->stepper_registers.count(id))
        present_velocity = _fake_data->stepper_registers.at(id).velocity;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readTemperature
 * @param id
 * @param temperature
 * @return
 */
int MockStepperDriver::readTemperature(uint8_t id, uint8_t &temperature)
{
    if (_fake_data->stepper_registers.count(id))
        temperature = _fake_data->stepper_registers.at(id).temperature;
    else
        return COMM_RX_FAIL;
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
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readHwErrorStatus
 * @param id
 * @param hardware_error_status
 * @return
 */
int MockStepperDriver::readHwErrorStatus(uint8_t /*id*/, uint8_t &hardware_error_status)
{
    hardware_error_status = 0;
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
    std::set<uint8_t> countSet;

    position_list.clear();
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
 * @brief MockStepperDriver::syncReadVelocity
 * @param id_list
 * @param velocity_list
 * @return
 */
int MockStepperDriver::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    std::set<uint8_t> countSet;

    velocity_list.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
            velocity_list.emplace_back(_fake_data->dxl_registers.at(id).velocity);
        else if (_fake_data->stepper_registers.count(id))
            velocity_list.emplace_back(_fake_data->stepper_registers.at(id).velocity);
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief StepperDriver::syncReadJointStatus
 * @param id_list
 * @param data_array_list
 * @return
 */
int MockStepperDriver::syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2>> &data_array_list)
{
    std::set<uint8_t> countSet;

    data_array_list.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->stepper_registers.count(id))
        {
            std::array<uint32_t, 2> blocks{};

            blocks.at(0) = _fake_data->stepper_registers.at(id).velocity;
            blocks.at(1) = _fake_data->stepper_registers.at(id).position;

            data_array_list.emplace_back(blocks);
        }
        else if (_fake_data->dxl_registers.count(id))
        {
            std::array<uint32_t, 2> blocks{};

            blocks.at(0) = _fake_data->dxl_registers.at(id).velocity;
            blocks.at(1) = _fake_data->dxl_registers.at(id).position;

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
 * @brief MockStepperDriver::syncReadFirmwareVersion
 * @param id_list
 * @param firmware_list
 * @return
 */
int MockStepperDriver::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    std::set<uint8_t> countSet;

    firmware_list.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
            firmware_list.emplace_back(_fake_data->dxl_registers.at(id).firmware);
        else if (_fake_data->stepper_registers.count(id))
            firmware_list.emplace_back(_fake_data->stepper_registers.at(id).firmware);
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
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
int MockStepperDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &temperature_list)
{
    std::set<uint8_t> countSet;

    temperature_list.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
            temperature_list.emplace_back(_fake_data->dxl_registers.at(id).temperature);
        else if (_fake_data->stepper_registers.count(id))
            temperature_list.emplace_back(_fake_data->stepper_registers.at(id).temperature);
        else if (_fake_data->end_effector.id == id)
            temperature_list.emplace_back(_fake_data->end_effector.temperature);
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
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
    std::set<uint8_t> countSet;

    voltage_list.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
            voltage_list.emplace_back(_fake_data->dxl_registers.at(id).voltage / 10);
        else if (_fake_data->stepper_registers.count(id))
            voltage_list.emplace_back(_fake_data->stepper_registers.at(id).voltage / 1000);
        else if (_fake_data->end_effector.id == id)
            voltage_list.emplace_back(_fake_data->end_effector.voltage / 1000);
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
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
int MockStepperDriver::syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    std::set<uint8_t> countSet;

    voltage_list.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->dxl_registers.count(id))
            voltage_list.emplace_back(_fake_data->dxl_registers.at(id).voltage);
        else if (_fake_data->stepper_registers.count(id))
            voltage_list.emplace_back(_fake_data->stepper_registers.at(id).voltage);
        else if (_fake_data->end_effector.id == id)
            voltage_list.emplace_back(_fake_data->end_effector.voltage);
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncReadHwStatus
 * @param id_list
 * @param data_list
 * @return
 */
int MockStepperDriver::syncReadHwStatus(const std::vector<uint8_t> &id_list, std::vector<std::pair<double, uint8_t>> &data_list)
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
        else if (_fake_data->stepper_registers.count(id))
        {
            double voltage = _fake_data->stepper_registers.at(id).voltage;
            uint8_t temperature = _fake_data->stepper_registers.at(id).temperature;
            data_list.emplace_back(std::make_pair(voltage, temperature));
        }
        else if (_fake_data->end_effector.id == id)
        {
            double voltage = _fake_data->end_effector.voltage;
            uint8_t temperature = _fake_data->end_effector.temperature;
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
 * @brief MockStepperDriver::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
int MockStepperDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list)
{
    std::set<uint8_t> countSet;

    hw_error_list.clear();
    for (auto &id : id_list)
    {
        hw_error_list.emplace_back(0);
        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

//*****************************
// AbstractStepperDriver interface
//*****************************

int MockStepperDriver::readControlMode(uint8_t id, uint8_t &mode)
{
    if (_fake_data->stepper_registers.count(id))
        mode = _fake_data->stepper_registers.at(id).operating_mode;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

int MockStepperDriver::writeControlMode(uint8_t id, uint8_t mode)
{
    if (_fake_data->stepper_registers.count(id))
        _fake_data->stepper_registers.at(id).operating_mode = mode;
    else
        return COMM_RX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::readVelocityProfile
 * @param id
 * @param data
 * @return
 */
int MockStepperDriver::readVelocityProfile(uint8_t id, std::vector<uint32_t> &data)
{
    int result = COMM_RX_FAIL;

    data.clear();
    if (_fake_data->stepper_registers.count(id))
    {
        data.emplace_back(_fake_data->stepper_registers.at(id).v_start);
        data.emplace_back(_fake_data->stepper_registers.at(id).a_1);
        data.emplace_back(_fake_data->stepper_registers.at(id).v_1);
        data.emplace_back(_fake_data->stepper_registers.at(id).a_max);
        data.emplace_back(_fake_data->stepper_registers.at(id).v_max);
        data.emplace_back(_fake_data->stepper_registers.at(id).d_max);
        data.emplace_back(_fake_data->stepper_registers.at(id).d_1);
        data.emplace_back(_fake_data->stepper_registers.at(id).v_stop);

        result = COMM_SUCCESS;
    }

    return result;
}

/**
 * @brief MockStepperDriver::writeVelocityProfile
 * @param id
 * @param data
 * @return
 */
int MockStepperDriver::writeVelocityProfile(uint8_t id, const std::vector<uint32_t> &data)
{
    int result = COMM_RX_FAIL;

    if (_fake_data->stepper_registers.count(id))
    {
        _fake_data->stepper_registers.at(id).v_start = data.at(0);
        _fake_data->stepper_registers.at(id).a_1 = data.at(1);
        _fake_data->stepper_registers.at(id).v_1 = data.at(2);
        _fake_data->stepper_registers.at(id).a_max = data.at(3);
        _fake_data->stepper_registers.at(id).v_max = data.at(4);
        _fake_data->stepper_registers.at(id).d_max = data.at(5);
        _fake_data->stepper_registers.at(id).d_1 = data.at(6);
        _fake_data->stepper_registers.at(id).v_stop = data.at(7);

        result = COMM_SUCCESS;
    }

    return result;
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
    _fake_time = 2;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::writeHomingDirection
 * @param id
 * @param direction
 * @param stall_threshold
 * @return
 */
int MockStepperDriver::writeHomingSetup(uint8_t id, uint8_t /*direction*/, uint8_t /*stall_threshold*/) { return ping(id); }

/**
 * @brief MockStepperDriver::readHomingStatus
 * @param id
 * @param status
 * @return
 */
int MockStepperDriver::readHomingStatus(uint8_t id, uint8_t &status)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    if (_fake_time)
    {
        _fake_time--;
    }
    else
        _calibration_status = CALIBRATION_SUCCESS;

    status = _calibration_status;
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncReadHomingStatus
 * @param id_list
 * @param status_list
 * @return
 */
int MockStepperDriver::syncReadHomingStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &status_list)
{
    if (_fake_time)
    {
        _fake_time--;
    }
    else  // when calibration finished or at startup
        _calibration_status = CALIBRATION_SUCCESS;

    std::set<uint8_t> countSet;

    status_list.clear();
    for (auto &id : id_list)
    {
        status_list.emplace_back(_calibration_status);
        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }

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

/**
 * @brief MockStepperDriver::syncReadHomingAbsPosition
 * @param id_list
 * @param abs_position
 * @return
 */
int MockStepperDriver::syncReadHomingAbsPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &abs_position)
{
    std::set<uint8_t> countSet;

    abs_position.clear();
    for (auto &id : id_list)
    {
        if (_fake_data->stepper_registers.count(id))
            abs_position.emplace_back(_fake_data->stepper_registers.at(id).homing_abs_position);
        else
            return COMM_RX_FAIL;

        auto result = countSet.insert(id);
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockStepperDriver::syncWriteHomingAbsPosition
 * @param id_list
 * @param abs_position
 * @return
 */
int MockStepperDriver::syncWriteHomingAbsPosition(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &abs_position)
{
    if (id_list.size() != abs_position.size())
        return LEN_ID_DATA_NOT_SAME;

    std::set<uint8_t> countSet;

    for (size_t i = 0; i < id_list.size(); ++i)
    {
        if (_fake_data->stepper_registers.count(id_list.at(i)))
            _fake_data->stepper_registers.at(id_list.at(i)).homing_abs_position = abs_position.at(i);
        else
            return COMM_TX_ERROR;

        auto result = countSet.insert(id_list.at(i));
        if (!result.second)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}


float MockStepperDriver::velocityUnit() const { return 1.0; }


}  // namespace ttl_driver
