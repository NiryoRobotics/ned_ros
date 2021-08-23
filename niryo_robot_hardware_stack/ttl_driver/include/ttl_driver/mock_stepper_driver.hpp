/*
mock_stepper_driver.hpp
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

#ifndef MOCK_STEPPER_DRIVER_HPP
#define MOCK_STEPPER_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "abstract_stepper_driver.hpp"

namespace ttl_driver
{

/**
 * @brief The StepperDriver class
 */
class MockStepperDriver : public AbstractStepperDriver
{
    public:
        MockStepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                      std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        virtual ~MockStepperDriver() override;

        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of DxlDriver
    public:
        virtual int ping(uint8_t id);
        virtual int getModelNumber(uint8_t id,
                            uint16_t& stepper_model_number);
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
        // AbstractStepperDriver interface
    public:
        virtual int startHoming(uint8_t id) override;
        virtual int getHomingStatus(uint8_t id, uint32_t &status) override;
        // conveyor control
        virtual int setGoalConveyorDirection(uint8_t id, int8_t direction) override;
        virtual int setConveyorState(uint8_t id, bool state) override;
        virtual int getConveyorSpeed(uint8_t id, uint32_t &velocity) override;
        virtual int getConveyorDirection(uint8_t id, int8_t &direction) override;
        virtual int getConveyorState(uint8_t id, bool &state) override;
    
    private:
        std::map<uint8_t, uint32_t> _map_fake_pos;

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int LEN_ID_DATA_NOT_SAME    = 20;
};

// definition of methods

/**
 * @brief DxlDriver<reg_type>::DxlDriver
 */
MockStepperDriver::MockStepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                       std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractStepperDriver(portHandler, packetHandler)
{
}

/**
 * @brief DxlDriver<reg_type>::~DxlDriver
 */
MockStepperDriver::~MockStepperDriver()
{
}

//*****************************
// AbstractTtlDriver interface
//*****************************
int MockStepperDriver::ping(uint8_t id)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::getModelNumber(uint8_t id, uint16_t& dxl_model_number)
{
    dxl_model_number = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::scan(std::vector<uint8_t>& id_list)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::reboot(uint8_t id)
{
    return COMM_SUCCESS;
}

std::string MockStepperDriver::interpreteErrorState(uint32_t /*hw_state*/)
{
    return "no error table";
}

int MockStepperDriver::changeId(uint8_t id, uint8_t new_id)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::checkModelNumber(uint8_t id)
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

int MockStepperDriver::getFirmwareVersion(uint8_t id, uint32_t &version)
{
    version = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::readMinPosition(uint8_t id, uint32_t &pos)
{
    pos = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::readMaxPosition(uint8_t id, uint32_t &pos)
{
    pos = 0;
    return COMM_SUCCESS;
}

// ram write

int MockStepperDriver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::setGoalPosition(uint8_t id, uint32_t position)
{
    return COMM_SUCCESS;
}

// according to the registers, the data should be an int32_t ?
int MockStepperDriver::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)
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

int MockStepperDriver::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
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

int MockStepperDriver::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
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

int MockStepperDriver::readPosition(uint8_t id, uint32_t& present_position)
{
    present_position = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    temperature = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::readVoltage(uint8_t id, uint32_t& voltage)
{
    voltage = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::readHwErrorStatus(uint8_t /*id*/, uint32_t& hardware_status)
{
    hardware_status = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
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

int MockStepperDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
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

int MockStepperDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
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

int MockStepperDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
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

int MockStepperDriver::read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t& data)
{
    data = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data)
{
    return COMM_SUCCESS;
}
//*****************************
// AbstractStepperDriver interface
//*****************************

int MockStepperDriver::startHoming(uint8_t id)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::getHomingStatus(uint8_t id, uint32_t &status)
{
    status = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::setGoalConveyorDirection(uint8_t id, int8_t direction)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::setConveyorState(uint8_t id, bool state)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::getConveyorSpeed(uint8_t id, uint32_t &velocity)
{
    velocity = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::getConveyorDirection(uint8_t id, int8_t &direction)
{
    direction = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::getConveyorState(uint8_t id, bool &state)
{
    state = 0;
    return COMM_SUCCESS;
}

}
#endif // MOCK_STEPPER_DRIVER_HPP
