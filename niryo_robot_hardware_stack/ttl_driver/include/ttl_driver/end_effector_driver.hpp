/*
end_effector_driver.hpp
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

#ifndef END_EFFECTOR_DRIVER_HPP
#define END_EFFECTOR_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>
#include <ros/ros.h>

#include "abstract_end_effector_driver.hpp"

#include "end_effector_reg.hpp"
#include "common/model/end_effector_command_type_enum.hpp"
#include "common/model/end_effector_state.hpp"

using ::common::model::EEndEffectorCommandType;

namespace ttl_driver
{

/**
 * @brief The EndEffectorDriver class
 */
template<typename reg_type = EndEffectorReg>
class EndEffectorDriver : public AbstractEndEffectorDriver
{
    public:
        EndEffectorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                          std::shared_ptr<dynamixel::PacketHandler> packetHandler);

    public:
        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver
        std::string str() const override;

        int checkModelNumber(uint8_t id) override;
        int readFirmwareVersion(uint8_t id, std::string &version) override;
        
        int readTemperature(uint8_t id, uint32_t &temperature) override;
        int readVoltage(uint8_t id, double &voltage) override;
        int readHwErrorStatus(uint8_t id, uint32_t &hardware_status) override;

        int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

    public:
        // AbstractEndEffectorDriver

        int readButton0Status(uint8_t id, common::model::EActionType& action) override;
        int readButton1Status(uint8_t id, common::model::EActionType& action) override;
        int readButton2Status(uint8_t id, common::model::EActionType& action) override;

        int readAccelerometerXValue(uint8_t id, uint32_t& x_value) override;
        int readAccelerometerYValue(uint8_t id, uint32_t& y_value) override;
        int readAccelerometerZValue(uint8_t id, uint32_t& z_value) override;

        int readCollisionStatus(uint8_t id, bool& status) override;

        int readDigitalInput(uint8_t id, bool& in) override;
        int writeDigitalOutput(uint8_t id, bool out) override;      
};

// definition of methods

/**
 * @brief EndEffectorDriver<reg_type>::EndEffectorDriver
 */
template<typename reg_type>
EndEffectorDriver<reg_type>::EndEffectorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                               std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractEndEffectorDriver(std::move(portHandler),
                              std::move(packetHandler))
{}

//*****************************
// AbstractTtlDriver interface
//*****************************

/**
 * @brief EndEffectorDriver<reg_type>::str
 * @return
 */
template<typename reg_type>
std::string EndEffectorDriver<reg_type>::str() const
{
    return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + ttl_driver::AbstractEndEffectorDriver::str();
}

/**
 * @brief EndEffectorDriver<reg_type>::checkModelNumber
 * @param id
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::checkModelNumber(uint8_t id)
{
    uint16_t model_number = 0;
    int ping_result = getModelNumber(id, model_number);

    if (ping_result == COMM_SUCCESS)
    {
        if (model_number && model_number != reg_type::MODEL_NUMBER)
        {
            return PING_WRONG_MODEL_NUMBER;
        }
    }

    return ping_result;
}

/**
 * @brief EndEffectorDriver<reg_type>::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readFirmwareVersion(uint8_t id, std::string &version)
{
    int res = COMM_RX_FAIL;
    uint32_t data{};
    res = read(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id, data);
    version = interpreteFirmwareVersion(data);
    return res;
}

// ram read

/**
 * @brief EndEffectorDriver<reg_type>::readTemperature
 * @param id
 * @param temperature
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readTemperature(uint8_t id, uint32_t& temperature)
{
    return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
}

/**
 * @brief EndEffectorDriver<reg_type>::readVoltage
 * @param id
 * @param voltage
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readVoltage(uint8_t id, double& voltage)
{
  uint32_t voltage_mV = 0;
  int res = read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage_mV);
  voltage = static_cast<double>(voltage_mV) / reg_type::VOLTAGE_CONVERSION;
  return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::readHwErrorStatus
 * @param id
 * @param hardware_status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    hardware_status = 0;
    return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadFirmwareVersion
 * @param id_list
 * @param firmware_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    int res = 0;
    firmware_list.clear();
    std::vector<uint32_t> data_list{};
    res = syncRead(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id_list, data_list);
    for(auto const& data : data_list)
      firmware_list.emplace_back(interpreteFirmwareVersion(data));
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    voltage_list.clear();
    std::vector<uint32_t> v_read;
    int res = syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, v_read);
    for(auto const& v : v_read)
        voltage_list.emplace_back(static_cast<double>(v) / reg_type::VOLTAGE_CONVERSION);
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
}

// buttons status

/**
 * @brief EndEffectorDriver<reg_type>::readButton0Status
 * @param id
 * @param action
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton0Status(uint8_t id,
                                                   common::model::EActionType& action)
{
    uint32_t status;
    int res = read(reg_type::ADDR_BUTTON_0_STATUS, reg_type::SIZE_BUTTON_0_STATUS, id, status);
    action = interpreteActionValue(status);
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::readButton1Status
 * @param id
 * @param action
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton1Status(uint8_t id, common::model::EActionType& action)
{
    uint32_t status;
    int res = read(reg_type::ADDR_BUTTON_1_STATUS, reg_type::SIZE_BUTTON_1_STATUS, id, status);
    action = interpreteActionValue(status);
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::readButton2Status
 * @param id
 * @param action
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton2Status(uint8_t id, common::model::EActionType& action)
{
    uint32_t status;
    int res = read(reg_type::ADDR_BUTTON_2_STATUS, reg_type::SIZE_BUTTON_2_STATUS, id, status);
    action = interpreteActionValue(status);
    return res;
}

// accelerometers and collision

/**
 * @brief EndEffectorDriver<reg_type>::readAccelerometerXValue
 * @param id
 * @param x_value
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerXValue(uint8_t id, uint32_t& x_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_X, reg_type::SIZE_ACCELERO_VALUE_X, id, x_value);
}

/**
 * @brief EndEffectorDriver<reg_type>::readAccelerometerYValue
 * @param id
 * @param y_value
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerYValue(uint8_t id, uint32_t& y_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_Y, reg_type::SIZE_ACCELERO_VALUE_Y, id, y_value);
}

/**
 * @brief EndEffectorDriver<reg_type>::readAccelerometerZValue
 * @param id
 * @param z_value
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerZValue(uint8_t id, uint32_t& z_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_Z, reg_type::SIZE_ACCELERO_VALUE_Z, id, z_value);
}

/**
 * @brief EndEffectorDriver<reg_type>::readCollisionStatus
 * @param id
 * @param status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readCollisionStatus(uint8_t id, bool& status)
{
    uint32_t value;

    int res = read(reg_type::ADDR_COLLISION_STATUS, reg_type::SIZE_COLLISION_STATUS, id, value);
    status = (value > 0) ? true : false;
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::readDigitalInput
 * @param id
 * @param in
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readDigitalInput(uint8_t id, bool& in)
{
    uint32_t value;
    int res = read(reg_type::ADDR_DIGITAL_IN, reg_type::SIZE_DIGITAL_IN, id, value);
    in = (value > 0) ? true : false;
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::setDigitalOutput
 * @param id
 * @param out
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::writeDigitalOutput(uint8_t id, bool out)
{
    return write(reg_type::ADDR_DIGITAL_OUT, reg_type::SIZE_DIGITAL_OUT, id, (out > 0) ? 1 : 0);
}

} // ttl_driver

#endif // EndEffectorDriver
