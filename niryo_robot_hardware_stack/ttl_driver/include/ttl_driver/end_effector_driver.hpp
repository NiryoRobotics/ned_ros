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
        
        int readTemperature(uint8_t id, uint8_t& temperature) override;
        int readVoltage(uint8_t id, double &voltage) override;
        int readHwErrorStatus(uint8_t id, uint8_t& hardware_error_status) override;

        int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t>& temperature_list) override;
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadHwStatus(const std::vector<uint8_t> &id_list, std::vector<std::pair<double, uint8_t> >& data_list) override;

        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list) override;

    public:
        // AbstractEndEffectorDriver

        int readButton0Status(uint8_t id, common::model::EActionType& action) override;
        int readButton1Status(uint8_t id, common::model::EActionType& action) override;
        int readButton2Status(uint8_t id, common::model::EActionType& action) override;
        int syncReadButtonsStatus(const uint8_t& id, std::vector<common::model::EActionType>& action_list) override;

        int readAccelerometerXValue(uint8_t id, uint32_t& x_value) override;
        int readAccelerometerYValue(uint8_t id, uint32_t& y_value) override;
        int readAccelerometerZValue(uint8_t id, uint32_t& z_value) override;

        int readCollisionStatus(uint8_t id, bool& status) override;

        int readDigitalInput(uint8_t id, bool& in) override;
        int writeDigitalOutput(uint8_t id, bool out) override;

        int writeCollisionThresh(uint8_t id, int thresh) override;
        int writeCollisionThreshAlgo2(uint8_t id, int thresh) override;
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
    res = read<typename reg_type::TYPE_FIRMWARE_VERSION>(reg_type::ADDR_FIRMWARE_VERSION, id, data);
    version = interpretFirmwareVersion(data);
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
int EndEffectorDriver<reg_type>::readTemperature(uint8_t id, uint8_t& temperature)
{
    return read<typename reg_type::TYPE_PRESENT_TEMPERATURE>(reg_type::ADDR_PRESENT_TEMPERATURE, id, temperature);
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
  uint16_t voltage_mV = 0;
  int res = read<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id, voltage_mV);
  voltage = static_cast<double>(voltage_mV) / reg_type::VOLTAGE_CONVERSION;
  return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::readHwErrorStatus
 * @param id
 * @param hardware_error_status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readHwErrorStatus(uint8_t id, uint8_t& hardware_error_status)
{
    hardware_error_status = 0;
    return read<typename reg_type::TYPE_HW_ERROR_STATUS>(reg_type::ADDR_HW_ERROR_STATUS, id, hardware_error_status);
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
    res = syncRead<typename reg_type::TYPE_FIRMWARE_VERSION>(reg_type::ADDR_FIRMWARE_VERSION, id_list, data_list);
    for(auto const& data : data_list)
      firmware_list.emplace_back(interpretFirmwareVersion(data));
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t>& temperature_list)
{
    return syncRead<typename reg_type::TYPE_PRESENT_TEMPERATURE>(reg_type::ADDR_PRESENT_TEMPERATURE, id_list, temperature_list);
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
    std::vector<uint16_t> v_read;
    int res = syncRead<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, v_read);
    for(auto const& v : v_read)
        voltage_list.emplace_back(static_cast<double>(v) / reg_type::VOLTAGE_CONVERSION);
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadRawVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    voltage_list.clear();
    std::vector<uint16_t> v_read;
    int res = syncRead<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, v_read);
    for(auto const& v : v_read)
        voltage_list.emplace_back(static_cast<double>(v));
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadHwStatus
 * @param id_list
 * @param data_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadHwStatus(const std::vector<uint8_t> &id_list,
                                                  std::vector<std::pair<double, uint8_t> > &data_list)
{
    data_list.clear();

    std::vector<std::array<uint8_t, 3> > raw_data;
    int res = syncReadConsecutiveBytes<uint8_t, 3>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, raw_data);

    for (auto const& data : raw_data)
    {
        // Voltage is first reg, uint16
        auto voltage = static_cast<double>((static_cast<uint16_t>(data.at(1)) << 8) | data.at(0));

        // Temperature is second reg, uint8
        uint8_t temperature = data.at(2);

        data_list.emplace_back(std::make_pair(voltage, temperature));
    }

    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list)
{
    return syncRead<typename reg_type::TYPE_HW_ERROR_STATUS>(reg_type::ADDR_HW_ERROR_STATUS, id_list, hw_error_list);
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
    uint8_t status;
    int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_BUTTON_0_STATUS, id, status);
    action = interpretActionValue(status);
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
    uint8_t status;
    int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_BUTTON_1_STATUS, id, status);
    action = interpretActionValue(status);
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
    uint8_t status;
    int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_BUTTON_2_STATUS, id, status);
    action = interpretActionValue(status);
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadButtonsStatus
 * @param id
 * @param action_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadButtonsStatus(const uint8_t& id,
                                                        std::vector<common::model::EActionType>& action_list)
{
    std::vector<std::array<typename reg_type::TYPE_BUTTON_STATUS, 3> > data_array_list;
    int res;
    res = syncReadConsecutiveBytes<typename reg_type::TYPE_BUTTON_STATUS, 3>(reg_type::ADDR_BUTTON_0_STATUS, {id}, data_array_list);
    if (res == COMM_SUCCESS && data_array_list.size() == 1)
    {
        for (auto data : data_array_list.at(0))
        {
            action_list.push_back(interpretActionValue(data));
        }
    }
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
    return read<typename reg_type::TYPE_ACCELERO_VALUE_X>(reg_type::ADDR_ACCELERO_VALUE_X, id, x_value);
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
    return read<typename reg_type::TYPE_ACCELERO_VALUE_Y>(reg_type::ADDR_ACCELERO_VALUE_Y, id, y_value);
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
    return read<typename reg_type::TYPE_ACCELERO_VALUE_Z>(reg_type::ADDR_ACCELERO_VALUE_Z, id, z_value);
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
    uint8_t value = 0;
    status = false;
    int res = read<typename reg_type::TYPE_COLLISION_STATUS>(reg_type::ADDR_COLLISION_STATUS, id, value);
    if (COMM_SUCCESS == res)
    {
        status = (value > 0) ? true : false;
    }
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
    uint8_t value;
    int res = read<typename reg_type::TYPE_DIGITAL_IN>(reg_type::ADDR_DIGITAL_IN, id, value);
    in = (value > 0) ? true : false;
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::setDigitalOutput
 * @param id
 * @param out  1 : true, 0: false
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::writeDigitalOutput(uint8_t id, bool out)
{
    return write<typename reg_type::TYPE_DIGITAL_OUT>(reg_type::ADDR_DIGITAL_OUT, id, static_cast<uint8_t>(out));
}

/**
 * @brief EndEffectorDriver<reg_type>::writeCollisionThresh
 * @param id
 * @param thresh
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::writeCollisionThresh(uint8_t id, int thresh)
{
    return write<typename reg_type::TYPE_COLLISION_THRESHOLD>(reg_type::ADDR_COLLISION_THRESHOLD, id, static_cast<uint8_t>(thresh));
}

/**
 * @brief EndEffectorDriver<reg_type>::writeCollisionThreshAlgo2
 * @param id
 * @param thresh
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::writeCollisionThreshAlgo2(uint8_t id, int thresh)
{
    std::cout << "writeCollisionThreshAlgo2 not implemented for end effector" << std::endl;

    return 0;
}

} // ttl_driver

#endif // EndEffectorDriver
