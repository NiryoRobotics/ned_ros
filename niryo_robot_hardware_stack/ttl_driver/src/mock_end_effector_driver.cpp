/*
mock_end_effector_driver.cpp
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

#include "ttl_driver/mock_end_effector_driver.hpp"

#include "ttl_driver/end_effector_reg.hpp"
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace ttl_driver
{
// definition of methods

/**
 * @brief MockEndEffectorDriver::EndEffectorDriver
 */
MockEndEffectorDriver::MockEndEffectorDriver(std::shared_ptr<FakeTtlData> data) :
  _fake_data(std::move(data))
{}

//*****************************
// AbstractTtlDriver interface
//*****************************

/**
 * @brief MockEndEffectorDriver::str
 * @return
 */
std::string MockEndEffectorDriver::str() const
{
    return common::model::HardwareTypeEnum(EndEffectorReg::motor_type).toString() + " : " + ttl_driver::AbstractEndEffectorDriver::str();
}

/**
 * @brief MockEndEffectorDriver::ping
 * @param id
 * @return
 */
int MockEndEffectorDriver::ping(uint8_t id)
{
    if (id == _fake_data->end_effector.id)
        return COMM_SUCCESS;
    return COMM_TX_FAIL;
}

/**
 * @brief MockEndEffectorDriver::checkModelNumber
 * @param id
 * @return
 */
int MockEndEffectorDriver::checkModelNumber(uint8_t id)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
int MockEndEffectorDriver::readFirmwareVersion(uint8_t id, std::string &version)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    version = _fake_data->end_effector.firmware;
    return COMM_SUCCESS;
}

// ram read

/**
 * @brief MockEndEffectorDriver::readTemperature
 * @param id
 * @param temperature
 * @return
 */
int MockEndEffectorDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    temperature = _fake_data->end_effector.temperature;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readVoltage
 * @param id
 * @param voltage
 * @return
 */
int MockEndEffectorDriver::readVoltage(uint8_t id, double& voltage)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    voltage = static_cast<double>(voltage) / EndEffectorReg::VOLTAGE_CONVERSION;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readHwErrorStatus
 * @param id
 * @param hardware_status
 * @return
 */
int MockEndEffectorDriver::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    hardware_status = 0;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::syncReadFirmwareVersion
 * @param id_list
 * @param firmware_list
 * @return
 */
int MockEndEffectorDriver::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    int res = 0;
    firmware_list.clear();
    for (size_t i = 0; i < id_list.size(); i++)
        firmware_list.emplace_back(_fake_data->end_effector.firmware);
    return res;
}

/**
 * @brief MockEndEffectorDriver::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
int MockEndEffectorDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    temperature_list.clear();
    for (size_t i = 0; i < id_list.size(); i++)
        temperature_list.emplace_back(_fake_data->end_effector.temperature);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::syncReadVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
int MockEndEffectorDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    voltage_list.clear();
    for (size_t i = 0; i < id_list.size(); i++)
        voltage_list.emplace_back(static_cast<double>(_fake_data->end_effector.voltage) / EndEffectorReg::VOLTAGE_CONVERSION);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
int MockEndEffectorDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &/*id_list*/, std::vector<uint32_t> &hw_error_list)
{
    // return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
    hw_error_list.clear();
    hw_error_list.emplace_back(0);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::scan
 * @param id_list
 * @return
 */
int MockEndEffectorDriver::scan(std::vector<uint8_t> &id_list)
{
    id_list = _fake_data->full_id_list;
    return COMM_SUCCESS;
}
// buttons status

/**
 * @brief MockEndEffectorDriver::readButton1Status
 * @param id
 * @param action
 * @return
 */
int MockEndEffectorDriver::readButton1Status(uint8_t id, common::model::EActionType& action)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    action = interpreteActionValue(_fake_data->end_effector.button1_action);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readButton2Status
 * @param id
 * @param action
 * @return
 */
int MockEndEffectorDriver::readButton2Status(uint8_t id, common::model::EActionType& action)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    action = interpreteActionValue(_fake_data->end_effector.button2_action);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readButton3Status
 * @param id
 * @param action
 * @return
 */
int MockEndEffectorDriver::readButton3Status(uint8_t id, common::model::EActionType& action)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    action = interpreteActionValue(_fake_data->end_effector.button3_action);
    return COMM_SUCCESS;
}

// accelerometers and collision

/**
 * @brief MockEndEffectorDriver::readAccelerometerXValue
 * @param id
 * @param x_value
 * @return
 */
int MockEndEffectorDriver::readAccelerometerXValue(uint8_t id, uint32_t& x_value)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    x_value = _fake_data->end_effector.x_value;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readAccelerometerYValue
 * @param id
 * @param y_value
 * @return
 */
int MockEndEffectorDriver::readAccelerometerYValue(uint8_t id, uint32_t& y_value)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    y_value = _fake_data->end_effector.y_value;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readAccelerometerZValue
 * @param id
 * @param z_value
 * @return
 */
int MockEndEffectorDriver::readAccelerometerZValue(uint8_t id, uint32_t& z_value)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    z_value = _fake_data->end_effector.z_value;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readCollisionStatus
 * @param id
 * @param status
 * @return
 */
int MockEndEffectorDriver::readCollisionStatus(uint8_t id, bool& status)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    status = false;
    std::cout << "MockEndEffectorDriver::readCollisionStatus: need to be implemented!" << std::endl;
    return 0;
}

/**
 * @brief MockEndEffectorDriver::readDigitalInput
 * @param id
 * @param in
 * @return
 */
int MockEndEffectorDriver::readDigitalInput(uint8_t id, bool& in)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    in = _fake_data->end_effector.digitalInput;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::setDigitalOutput
 * @param id
 * @param out
 * @return
 */
int MockEndEffectorDriver::writeDigitalOutput(uint8_t id, bool out)
{
    if (COMM_SUCCESS != ping(id))
        return COMM_RX_FAIL;

    _fake_data->end_effector.DigitalOutput = out;
    return COMM_SUCCESS;
}



}  // namespace ttl_driver
