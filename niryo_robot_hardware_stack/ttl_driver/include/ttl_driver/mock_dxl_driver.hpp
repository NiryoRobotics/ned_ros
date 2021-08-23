/*
mock_dxl_driver.hpp
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

#ifndef MOCK_DXL_DRIVER_HPP
#define MOCK_DXL_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "abstract_dxl_driver.hpp"
#include "common/common_defs.hpp"

namespace ttl_driver
{


/**
 * @brief The DxlDriver class
 */
class MockDxlDriver : public AbstractDxlDriver
{
    public:
        MockDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                  std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        virtual ~MockDxlDriver() override;

        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver
    public:
        virtual int ping(uint8_t id);
        virtual int getModelNumber(uint8_t id,
                            uint16_t& dxl_model_number);
        virtual int scan(std::vector<uint8_t>& id_list);
        virtual int reboot(uint8_t id);

        virtual std::string interpreteErrorState(uint32_t hw_state) override;

        // eeprom write
        virtual int changeId(uint8_t id, uint8_t new_id) override;

        // eeprom read
        virtual int checkModelNumber(uint8_t id) override;
        virtual int getFirmwareVersion(uint8_t id, uint32_t &version) override;
        virtual int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        virtual int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        // ram write
        virtual int setTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        virtual int setGoalPosition(uint8_t id, uint32_t position) override;
        virtual int setGoalVelocity(uint8_t id, uint32_t velocity) override;

        virtual int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        virtual int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        virtual int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        // ram read
        virtual int readPosition(uint8_t id, uint32_t &present_position) override;
        virtual int readTemperature(uint8_t id, uint32_t &temperature) override;
        virtual int readVoltage(uint8_t id, uint32_t &voltage) override;
        virtual int readHwErrorStatus(uint8_t id, uint32_t &hardware_status) override;

        virtual int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) override;
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

        virtual int read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t& data) override;
        virtual int write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data) override;
        // AbstractDxlDriver interface
    public:
        virtual int setLed(uint8_t id, uint32_t led_value) override;
        virtual int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) override;
        virtual int setGoalTorque(uint8_t id, uint32_t torque) override;
        virtual int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) override;
        virtual int setPositionPGain(uint8_t id, uint32_t gain) override;
        virtual int setPositionIGain(uint8_t id, uint32_t gain) override;
        virtual int setPositionDGain(uint8_t id, uint32_t gain) override;
        virtual int setVelocityPGain(uint8_t id, uint32_t gain) override;
        virtual int setVelocityIGain(uint8_t id, uint32_t gain) override;
        virtual int setff1Gain(uint8_t id, uint32_t gain) override;
        virtual int setff2Gain(uint8_t id, uint32_t gain) override;
        virtual int readLoad(uint8_t id, uint32_t &present_load) override;
        virtual int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) override;
        virtual int readVelocity(uint8_t id, uint32_t &present_velocity) override;
        virtual int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
    private:
        std::map<uint8_t, uint32_t> _map_fake_pos;

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int LEN_ID_DATA_NOT_SAME    = 20;
};

// definition of methods

/**
 * @brief MockDxlDriver::MockDxlDriver
 */
MockDxlDriver::MockDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                               std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractDxlDriver(portHandler, packetHandler)
{
}

MockDxlDriver::~MockDxlDriver()
{
}

//*****************************
// AbstractTtlDriver interface
//*****************************

int MockDxlDriver::ping(uint8_t id)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::getModelNumber(uint8_t id, uint16_t& dxl_model_number)
{
    dxl_model_number = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::scan(std::vector<uint8_t>& id_list)
{
    id_list = {2, 3, 6};
    return COMM_SUCCESS;
}

int MockDxlDriver::reboot(uint8_t id)
{
    return COMM_SUCCESS;
}

std::string MockDxlDriver::interpreteErrorState(uint32_t /*hw_state*/)
{
    return "no error table";
}

int MockDxlDriver::changeId(uint8_t id, uint8_t new_id)
{
    return COMM_SUCCESS;
}

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

int MockDxlDriver::getFirmwareVersion(uint8_t id, uint32_t &version)
{
    version = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::readMinPosition(uint8_t id, uint32_t &pos)
{
    pos = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::readMaxPosition(uint8_t id, uint32_t &pos)
{
    pos = 0;
    return COMM_SUCCESS;
}

// ram write

int MockDxlDriver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setGoalPosition(uint8_t id, uint32_t position)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    // in mode control Position Control Mode, velocity profile in datasheet is used to set velocity (except xl320)
    return COMM_SUCCESS;
}

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

int MockDxlDriver::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    if (id_list.size() != position_list.size())
        return LEN_ID_DATA_NOT_SAME;

    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;    
    for (size_t i = 0; i < id_list.size(); i++)
    {
        _map_fake_pos.at(id_list[i]) = position_list[i];
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id_list[i], 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

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

int MockDxlDriver::readPosition(uint8_t id, uint32_t& present_position)
{
    present_position = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    temperature = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::readVoltage(uint8_t id, uint32_t& voltage)
{
    voltage = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    hardware_status = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    std::map<uint8_t, uint8_t> countMap;
    if (_map_fake_pos.empty())
        for (size_t i = 0; i < id_list.size(); i++)
        {
            position_list.push_back(0);
            auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id_list[i], 1));
            if (result.second == false)
                return GROUP_SYNC_REDONDANT_ID;  // redondant id
        }
    else
    {
        for (size_t i = 0; i < id_list.size(); i++)
        {
            position_list.emplace_back(_map_fake_pos.at(id_list[i]));
            auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id_list[i], 1));
            if (result.second == false)
                return GROUP_SYNC_REDONDANT_ID;  // redondant id
        }
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        temperature_list.push_back(0);
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        voltage_list.push_back(0);
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        hw_error_list.push_back(0);
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t& data)
{
    data = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data)
{
    return COMM_SUCCESS;
}

//*****************************
// AbstractDxlDriver interface
//*****************************


int MockDxlDriver::setLed(uint8_t id, uint32_t led_value)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setGoalTorque(uint8_t id, uint32_t torque)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setPositionPGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setPositionIGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setPositionDGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setVelocityPGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setVelocityIGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setff1Gain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setff2Gain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::readLoad(uint8_t id, uint32_t& present_load)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    load_list = {};
    for (size_t i = 0; i < id_list.size(); i++)
        load_list.push_back(0);
    return COMM_SUCCESS;
}

int MockDxlDriver::readVelocity(uint8_t id, uint32_t& present_velocity)
{
    present_velocity = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    velocity_list = {};
    for (size_t i = 0; i < id_list.size(); i++)
        velocity_list.push_back(0);
    return COMM_SUCCESS;
}

} // DynamixelDriver

#endif // MOCK_DXL_DRIVER
