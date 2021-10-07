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
#include "ttl_driver/mock_dxl_driver.hpp"
#include <type_traits>
#include <utility>
#include <map>
#include <string>
#include <vector>

namespace ttl_driver
{

/**
 * @brief MockDxlDriver::MockDxlDriver
 */
MockDxlDriver::MockDxlDriver(FakeTtlData data)
{
    initializeFakeData(data);
    // retrieve list of ids
    for (auto const& imap : _map_fake_registers)
        _id_list.emplace_back(imap.first);
}

/**
 * @brief MockDxlDriver::~MockDxlDriver
 */
MockDxlDriver::~MockDxlDriver()
{
}

/**
 * @brief MockDxlDriver::str
 * @return
 */
std::string MockDxlDriver::str() const
{
  return "Mock Dynamixel Driver (OK)";
}

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
    return _map_fake_registers.count(id) ? COMM_SUCCESS : COMM_TX_FAIL;
}

/**
 * @brief MockDxlDriver::getModelNumber
 * @param id
 * @param model_number
 * @return
 */
int MockDxlDriver::getModelNumber(uint8_t id, uint16_t& model_number)
{
    if (_map_fake_registers.count(id))
        model_number = _map_fake_registers.at(id).model_number;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::scan
 * @param id_list
 * @return
 */
int MockDxlDriver::scan(std::vector<uint8_t>& id_list)
{
    id_list = _full_id_list;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::reboot
 * @param id
 * @return
 */
int MockDxlDriver::reboot(uint8_t id)
{
    if (std::find(_id_list.begin(), _id_list.end(), id) == _id_list.end())
        return COMM_TX_FAIL;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::interpreteErrorState
 * @return
 */
std::string MockDxlDriver::interpreteErrorState(uint32_t /*hw_state*/) const
{
  return "";
}

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
    return COMM_TX_FAIL;
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
        if (model_number && model_number != 0)
        {
            return PING_WRONG_MODEL_NUMBER;
        }
    }

    return ping_result;
}

/**
 * @brief MockDxlDriver::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
int MockDxlDriver::readFirmwareVersion(uint8_t id, std::string &version)
{
    if (_map_fake_registers.count(id))
        version = _map_fake_registers.at(id).firmware;
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
    if (_map_fake_registers.count(id))
        pos = _map_fake_registers.at(id).min_position;
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
    if (_map_fake_registers.count(id))
        pos = _map_fake_registers.at(id).max_position;
    return COMM_SUCCESS;
}

// ram write

/**
 * @brief MockDxlDriver::setTorqueEnable
 * @param id
 * @param torque_enable
 * @return
 */
int MockDxlDriver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setGoalPosition
 * @param id
 * @param position
 * @return
 */
int MockDxlDriver::setGoalPosition(uint8_t id, uint32_t position)
{
    if (_map_fake_registers.count(id))
        _map_fake_registers.at(id).position = position;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setGoalVelocity
 * @param id
 * @param velocity
 * @return
 */
int MockDxlDriver::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    // in mode control Position Control Mode, velocity profile in datasheet is used to set velocity (except xl320)
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWriteTorqueEnable
 * @param id_list
 * @param torque_enable_list
 * @return
 */
int MockDxlDriver::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)
{
    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;
    // Iterate over the vector and store the frequency of each element in map
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWritePositionGoal get position goal and set it as the current position of each joint
 * @param id_list
 * @param position_list
 * @return
 */
int MockDxlDriver::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    if (id_list.size() != position_list.size())
        return LEN_ID_DATA_NOT_SAME;

    // Create a map to store the frequency of each element in id_list. It helps find out which ID is redondant
    std::map<uint8_t, uint8_t> countMap;
    for (size_t i = 0; i < id_list.size(); i++)
    {
        if (!_map_fake_registers.count(id_list.at(i)))
            return COMM_TX_ERROR;
        // set goal position as the current position
        _map_fake_registers.at(id_list.at(i)).position = position_list.at(i);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id_list[i], 1));
        if (result.second == false)
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
    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;
    // Iterate over the vector and store the frequency of each element in map
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

// ram read

/**
 * @brief MockDxlDriver::readPosition
 * @param id
 * @param present_position
 * @return
 */
int MockDxlDriver::readPosition(uint8_t id, uint32_t& present_position)
{
    if (!_map_fake_registers.count(id))
        present_position = _map_fake_registers.at(id).position;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readTemperature
 * @param id
 * @param temperature
 * @return
 */
int MockDxlDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    if (_map_fake_registers.count(id))
        temperature = _map_fake_registers.at(id).temperature;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readVoltage
 * @param id
 * @param voltage
 * @return
 */
int MockDxlDriver::readVoltage(uint8_t id, double& voltage)
{
    if (_map_fake_registers.count(id))
        voltage = _map_fake_registers.at(id).voltage;
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readHwErrorStatus
 * @param id
 * @param hardware_status
 * @return
 */
int MockDxlDriver::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    hardware_status = 0;
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
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        if (!_map_fake_registers.count(id))
            return COMM_TX_ERROR;
        position_list.emplace_back(_map_fake_registers.at(id).position);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
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
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        if (!_map_fake_registers.count(id))
            return COMM_TX_ERROR;
        firmware_list.emplace_back(_map_fake_registers.at(id).firmware);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
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
int MockDxlDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        if (!_map_fake_registers.count(id))
            return COMM_TX_ERROR;
        temperature_list.emplace_back(_map_fake_registers.at(id).temperature);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
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
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        if (!_map_fake_registers.count(id))
            return COMM_TX_ERROR;
        voltage_list.emplace_back(_map_fake_registers.at(id).voltage);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
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
int MockDxlDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list,
                                         std::vector<uint32_t> &hw_error_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        hw_error_list.push_back(0);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

//*****************************
// AbstractDxlDriver interface
//*****************************
/**
 * @brief MockDxlDriver::setLed
 * @param id
 * @param led_value
 * @return
 */
int MockDxlDriver::setLed(uint8_t id, uint32_t led_value)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWriteLed
 * @param id_list
 * @param led_list
 * @return
 */
int MockDxlDriver::syncWriteLed(const std::vector<uint8_t> &id_list,
                                const std::vector<uint32_t> &led_list)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setGoalTorque
 * @param id
 * @param torque
 * @return
 */
int MockDxlDriver::setGoalTorque(uint8_t id, uint32_t torque)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncWriteTorqueGoal
 * @param id_list
 * @param torque_list
 * @return
 */
int MockDxlDriver::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list,
                                       const std::vector<uint32_t> &torque_list)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setPositionPGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::setPositionPGain(uint8_t id, uint32_t gain)
{
  if (_map_fake_registers.count(id))
      _map_fake_registers.at(id).position_p_gain = gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setPositionIGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::setPositionIGain(uint8_t id, uint32_t gain)
{
  if (_map_fake_registers.count(id))
      _map_fake_registers.at(id).position_i_gain = gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setPositionDGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::setPositionDGain(uint8_t id, uint32_t gain)
{
  if (_map_fake_registers.count(id))
      _map_fake_registers.at(id).position_d_gain = gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setVelocityPGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::setVelocityPGain(uint8_t id, uint32_t gain)
{
  if (_map_fake_registers.count(id))
      _map_fake_registers.at(id).velocity_p_gain = gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setVelocityIGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::setVelocityIGain(uint8_t id, uint32_t gain)
{
  if (_map_fake_registers.count(id))
      _map_fake_registers.at(id).velocity_i_gain = gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setff1Gain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::setff1Gain(uint8_t id, uint32_t gain)
{
  if (_map_fake_registers.count(id))
      _map_fake_registers.at(id).ff1_gain = gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::setff2Gain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::setff2Gain(uint8_t id, uint32_t gain)
{
  if (_map_fake_registers.count(id))
      _map_fake_registers.at(id).ff2_gain = gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readPositionPGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::readPositionPGain(uint8_t id, uint32_t& gain)
{
  if (_map_fake_registers.count(id))
      gain = _map_fake_registers.at(id).position_p_gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readPositionIGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::readPositionIGain(uint8_t id, uint32_t& gain)
{
  if (_map_fake_registers.count(id))
      gain = _map_fake_registers.at(id).position_i_gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readPositionDGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::readPositionDGain(uint8_t id, uint32_t& gain)
{
  if (_map_fake_registers.count(id))
      gain = _map_fake_registers.at(id).position_d_gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readVelocityPGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::readVelocityPGain(uint8_t id, uint32_t& gain)
{
  if (_map_fake_registers.count(id))
      gain = _map_fake_registers.at(id).velocity_p_gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readVelocityIGain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::readVelocityIGain(uint8_t id, uint32_t& gain)
{
  if (_map_fake_registers.count(id))
      gain = _map_fake_registers.at(id).velocity_i_gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readFF1Gain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::readFF1Gain(uint8_t id, uint32_t& gain)
{
  if (_map_fake_registers.count(id))
      gain = _map_fake_registers.at(id).ff1_gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readff2Gain
 * @param id
 * @param gain
 * @return
 */
int MockDxlDriver::readFF2Gain(uint8_t id, uint32_t& gain)
{
  if (_map_fake_registers.count(id))
      gain = _map_fake_registers.at(id).ff2_gain;
  return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readLoad
 * @param id
 * @param present_load
 * @return
 */
int MockDxlDriver::readLoad(uint8_t id, uint32_t& present_load)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::syncReadLoad
 * @param id_list
 * @param load_list
 * @return
 */
int MockDxlDriver::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    load_list = {};
    for (size_t i = 0; i < id_list.size(); i++)
        load_list.push_back(0);
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::readVelocity
 * @param id
 * @param present_velocity
 * @return
 */
int MockDxlDriver::readVelocity(uint8_t id, uint32_t& present_velocity)
{
    present_velocity = 0;
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
    velocity_list = {};
    for (size_t i = 0; i < id_list.size(); i++)
        velocity_list.push_back(0);
    return COMM_SUCCESS;
}

/**
 * @brief MockDxlDriver::interpreteFirmwareVersion
 * @param fw_version
 * @return
 */
std::string MockDxlDriver::interpreteFirmwareVersion(uint32_t fw_version) const
{
    return "";
}

/**
 * @brief MockDxlDriver::removeGripper
 */
void MockDxlDriver::removeGripper()
{
    _full_id_list.erase(std::remove(_full_id_list.begin(), _full_id_list.end(), 11), _full_id_list.end());
    _map_fake_registers.erase(11);
}

/**
 * @brief MockDxlDriver::initializeFakeData
 * @param data
 */
void MockDxlDriver::initializeFakeData(FakeTtlData data)
{
    _full_id_list = data.full_id_list;
    for (auto fdata : data.dxl_registers)
    {
        if (!_map_fake_registers.count(fdata.id))
        {
            _map_fake_registers.insert(std::pair<uint8_t, FakeTtlData::FakeDxlRegister>(fdata.id, fdata));
        }
    }
}
}  // namespace ttl_driver
