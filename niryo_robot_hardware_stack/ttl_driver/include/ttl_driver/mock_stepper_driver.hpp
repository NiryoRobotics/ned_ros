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
#include "common/model/stepper_calibration_status_enum.hpp"

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
        virtual int ping(uint8_t id) override;
        virtual int getModelNumber(uint8_t id,
                            uint16_t& model_number) override;
        virtual int scan(std::vector<uint8_t>& id_list) override;
        virtual int reboot(uint8_t id) override;

        virtual std::string interpreteErrorState(uint32_t hw_state) const override;

        // eeprom write
        virtual int changeId(uint8_t id, uint8_t new_id) override;

        // eeprom read
        virtual int checkModelNumber(uint8_t id) override;
        virtual int readFirmwareVersion(uint8_t id, std::string &version) override;
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

        virtual int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) override;
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

        // AbstractStepperDriver interface
    public:
        virtual int startHoming(uint8_t id) override;
        virtual int setHomingDirection(uint8_t id, uint8_t direction) override;
        virtual int readHomingStatus(uint8_t id, uint32_t &status) override;
        // conveyor control
        virtual int setGoalConveyorDirection(uint8_t id, int8_t direction) override;
        virtual int setConveyorState(uint8_t id, bool state) override;
        virtual int readConveyorSpeed(uint8_t id, uint32_t &velocity) override;
        virtual int readConveyorDirection(uint8_t id, int8_t &direction) override;
        virtual int readConveyorState(uint8_t id, bool &state) override;
    
    private:
        struct FakeRegister
        {
          uint32_t       position{};
          uint32_t       temperature{};
          uint32_t       voltage{};
          uint32_t       min_position{};
          uint32_t       max_position{};
          uint16_t       model_number{};
          std::string    firmware{};
        };

        std::map<uint8_t, FakeRegister> _map_fake_registers{ {2, {1900, 50, 12, 0, 4096, 1, "0.0.1"}},
                                                             {3, {0, 52, 12, 0, 4096, 1, "0.0.1"}},
                                                             {4, {0, 54, 12, 0, 4096, 1, "0.0.1"}}};

        std::vector<uint8_t> _full_id_list{2,3,4,5,6,7,11};
        std::vector<uint8_t> _id_list;

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int LEN_ID_DATA_NOT_SAME    = 20;

        common::model::EStepperCalibrationStatus _calibration_status = common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
        uint8_t fake_time = 0;
};

// definition of methods

/**
 * @brief DxlDriver<reg_type>::DxlDriver
 */
MockStepperDriver::MockStepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                       std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractStepperDriver(portHandler, packetHandler)
{
    // retrieve list of ids
    for(auto const& imap: _map_fake_registers)
        _id_list.emplace_back(imap.first);
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
  return _map_fake_registers.count(id) ? COMM_SUCCESS : COMM_TX_FAIL;
}

int MockStepperDriver::getModelNumber(uint8_t id, uint16_t& model_number)
{
    model_number = _map_fake_registers.at(id).model_number;
    return COMM_SUCCESS;
}

int MockStepperDriver::scan(std::vector<uint8_t>& id_list)
{
    id_list = _full_id_list;
    return COMM_SUCCESS;
}

int MockStepperDriver::reboot(uint8_t id)
{
    if (std::find(_id_list.begin(), _id_list.end(), id) == _id_list.end())
        return COMM_TX_FAIL;
    return COMM_SUCCESS;
}

std::string MockStepperDriver::interpreteErrorState(uint32_t /*hw_state*/) const
{
    return "";
}

int MockStepperDriver::changeId(uint8_t id, uint8_t new_id)
{
    return COMM_TX_FAIL;
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

int MockStepperDriver::readFirmwareVersion(uint8_t id, std::string &version)
{
    version = _map_fake_registers.at(id).firmware;
    return COMM_SUCCESS;
}

int MockStepperDriver::readMinPosition(uint8_t id, uint32_t &pos)
{
    pos = _map_fake_registers.at(id).min_position;
    return COMM_SUCCESS;
}

int MockStepperDriver::readMaxPosition(uint8_t id, uint32_t &pos)
{
    pos = _map_fake_registers.at(id).max_position;
    return COMM_SUCCESS;
}

// ram write

int MockStepperDriver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::setGoalPosition(uint8_t id, uint32_t position)
{
    _map_fake_registers.at(id).position = position;
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
    for (size_t i = 0; i < id_list.size(); ++i)
    {
        _map_fake_registers.at(id_list.at(i)).position = position_list.at(i);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id_list.at(i), 1));
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
    present_position = _map_fake_registers.at(id).position;
    return COMM_SUCCESS;
}

int MockStepperDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    temperature = _map_fake_registers.at(id).temperature;
    return COMM_SUCCESS;
}

int MockStepperDriver::readVoltage(uint8_t id, uint32_t& voltage)
{
    voltage = _map_fake_registers.at(id).voltage;
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
    position_list.clear();
    for (auto & id : id_list)
    {
        position_list.emplace_back(_map_fake_registers.at(id).position);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockStepperDriver::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    std::map<uint8_t, uint8_t> countMap;
    firmware_list.clear();
    for (auto & id : id_list)
    {
        firmware_list.emplace_back(_map_fake_registers.at(id).firmware);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockStepperDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    std::map<uint8_t, uint8_t> countMap;
    temperature_list.clear();
    for (auto & id : id_list)
    {
        temperature_list.emplace_back(_map_fake_registers.at(id).temperature);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockStepperDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
{
    std::map<uint8_t, uint8_t> countMap;
    voltage_list.clear();
    for (auto & id : id_list)
    {
        voltage_list.emplace_back(_map_fake_registers.at(id).voltage);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockStepperDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    std::map<uint8_t, uint8_t> countMap;
    hw_error_list.clear();
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
// AbstractStepperDriver interface
//*****************************

int MockStepperDriver::startHoming(uint8_t id)
{
    _calibration_status = common::model::EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS;
    fake_time = 5;
    return COMM_SUCCESS;
}

int MockStepperDriver::setHomingDirection(uint8_t id, uint8_t direction)
{
    return COMM_SUCCESS;
}

int MockStepperDriver::readHomingStatus(uint8_t id, uint32_t &status)
{
    if (fake_time)
    {
        fake_time--;
    }
    else
        _calibration_status = common::model::EStepperCalibrationStatus::CALIBRATION_OK;

    status = static_cast<uint32_t>(_calibration_status);
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

int MockStepperDriver::readConveyorSpeed(uint8_t id, uint32_t &velocity)
{
    velocity = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::readConveyorDirection(uint8_t id, int8_t &direction)
{
    direction = 0;
    return COMM_SUCCESS;
}

int MockStepperDriver::readConveyorState(uint8_t id, bool &state)
{
    state = 0;
    return COMM_SUCCESS;
}

}
#endif // MOCK_STEPPER_DRIVER_HPP
