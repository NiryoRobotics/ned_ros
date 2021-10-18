/*
dxl_driver.hpp
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

#ifndef DXL_DRIVER_HPP
#define DXL_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "abstract_dxl_driver.hpp"
#include "common/common_defs.hpp"

#include "xc430_reg.hpp"
#include "xl430_reg.hpp"
#include "xl330_reg.hpp"
#include "xl320_reg.hpp"

namespace ttl_driver
{


/**
 * @brief The DxlDriver class
 */
template<typename reg_type>
class DxlDriver : public AbstractDxlDriver
{
    public:
        DxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                  std::shared_ptr<dynamixel::PacketHandler> packetHandler);

        std::string str() const override;
        std::string interpreteErrorState(uint32_t hw_state) const override;

    public:
        int checkModelNumber(uint8_t id) override;
        int readFirmwareVersion(uint8_t id, std::string &version) override;

        int readTemperature(uint8_t id, uint32_t &temperature) override;
        int readVoltage(uint8_t id, double &voltage) override;
        int readHwErrorStatus(uint8_t id, uint32_t &hardware_status) override;

        int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

    protected:
        // AbstractTtlDriver interface
        std::string interpreteFirmwareVersion(uint32_t fw_version) const override;

    public:
        // AbstractMotorDriver interface : we cannot define them globally in AbstractMotorDriver
        // as it is needed here for polymorphism (AbstractMotorDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver

        int changeId(uint8_t id, uint8_t new_id) override;

        int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        int writeTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        int writeGoalPosition(uint8_t id, uint32_t position) override;
        int writeGoalVelocity(uint8_t id, uint32_t velocity) override;

        int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        int readPosition(uint8_t id, uint32_t &present_position) override;
        
        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        
    public:
        // AbstractDxlDriver interface
        int writePID(uint8_t id, const std::vector<uint32_t>& data) override;
        int readPID(uint8_t id, std::vector<uint32_t>& data_list) override;

        int writeLed(uint8_t id, uint32_t led_value) override;
        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) override;

        int writeGoalTorque(uint8_t id, uint32_t torque) override;
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) override;

        int readLoad(uint8_t id, uint32_t &present_load) override;
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) override;

        int readVelocity(uint8_t id, uint32_t &present_velocity) override;
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;

private:
        int writePositionPGain(uint8_t id, uint32_t gain);
        int writePositionIGain(uint8_t id, uint32_t gain);
        int writePositionDGain(uint8_t id, uint32_t gain);
        int writeVelocityPGain(uint8_t id, uint32_t gain);
        int writeVelocityIGain(uint8_t id, uint32_t gain);
        int writeFF1Gain(uint8_t id, uint32_t gain);
        int writeFF2Gain(uint8_t id, uint32_t gain);

        int readPositionPGain(uint8_t id, uint32_t& gain);
        int readPositionIGain(uint8_t id, uint32_t& gain);
        int readPositionDGain(uint8_t id, uint32_t& gain);
        int readVelocityPGain(uint8_t id, uint32_t& gain);
        int readVelocityIGain(uint8_t id, uint32_t& gain);
        int readFF1Gain(uint8_t id, uint32_t& gain);
        int readFF2Gain(uint8_t id, uint32_t& gain);
};

// definition of methods

/**
 * @brief DxlDriver<reg_type>::DxlDriver
 */
template<typename reg_type>
DxlDriver<reg_type>::DxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                               std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractDxlDriver(std::move(portHandler),
                      std::move(packetHandler))
{}

//*****************************
// AbstractMotorDriver interface
//*****************************

/**
 * @brief DxlDriver<reg_type>::str
 * @return
 */
template<typename reg_type>
std::string DxlDriver<reg_type>::str() const
{
    return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + AbstractDxlDriver::str();
}

/**
 * @brief DxlDriver<reg_type>::interpreteErrorState
 * @return
 */
template<typename reg_type>
std::string DxlDriver<reg_type>::interpreteErrorState(uint32_t /*hw_state*/) const
{
    return "";
}

/**
 * @brief DxlDriver<reg_type>::interpreteFirmwareVersion
 * @param fw_version
 * @return
 */
template<typename reg_type>
std::string DxlDriver<reg_type>::interpreteFirmwareVersion(uint32_t fw_version) const
{
    std::string version = std::to_string(static_cast<uint8_t>(fw_version));

    return version;
}

/**
 * @brief DxlDriver<reg_type>::changeId
 * @param id
 * @param new_id
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::changeId(uint8_t id, uint8_t new_id)
{
    return write(reg_type::ADDR_ID, reg_type::SIZE_ID, id, new_id);
}

/**
 * @brief DxlDriver<reg_type>::checkModelNumber
 * @param id
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::checkModelNumber(uint8_t id)
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
 * @brief DxlDriver<reg_type>::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readFirmwareVersion(uint8_t id, std::string &version)
{
    int res = COMM_RX_FAIL;
    uint32_t data{};
    res = read(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id, data);
    version = interpreteFirmwareVersion(data);
    return res;
}

/**
 * @brief DxlDriver<reg_type>::readMinPosition
 * @param id
 * @param pos
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readMinPosition(uint8_t id, uint32_t &pos)
{
    return read(reg_type::ADDR_MIN_POSITION_LIMIT, reg_type::SIZE_MIN_POSITION_LIMIT, id, pos);
}

/**
 * @brief DxlDriver<reg_type>::readMaxPosition
 * @param id
 * @param pos
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readMaxPosition(uint8_t id, uint32_t &pos)
{
    return read(reg_type::ADDR_MAX_POSITION_LIMIT, reg_type::SIZE_MAX_POSITION_LIMIT, id, pos);
}

// ram write

/**
 * @brief DxlDriver<reg_type>::writeTorqueEnable
 * @param id
 * @param torque_enable
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return write(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id, torque_enable);
}

/**
 * @brief DxlDriver<reg_type>::writeGoalPosition
 * @param id
 * @param position
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeGoalPosition(uint8_t id, uint32_t position)
{
    return write(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id, position);
}

/**
 * @brief DxlDriver<reg_type>::writeGoalVelocity
 * @param id
 * @param velocity
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeGoalVelocity(uint8_t id, uint32_t velocity)
{
    // in mode control Position Control Mode, velocity profile in datasheet is used to write velocity (except xl320)
    return write(reg_type::ADDR_PROFILE_VELOCITY, reg_type::SIZE_PROFILE_VELOCITY, id, velocity);
}

/**
 * @brief DxlDriver<reg_type>::syncWriteTorqueEnable
 * @param id_list
 * @param torque_enable_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)
{
    return syncWrite(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id_list, torque_enable_list);
}

/**
 * @brief DxlDriver<reg_type>::syncWritePositionGoal
 * @param id_list
 * @param position_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    return syncWrite(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id_list, position_list);
}

/**
 * @brief DxlDriver<reg_type>::syncWriteVelocityGoal
 * @param id_list
 * @param velocity_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
{
    // in mode control Position Control Mode, velocity profile in datasheet is used to write velocity (except xl320)
    return syncWrite(reg_type::ADDR_PROFILE_VELOCITY, reg_type::SIZE_PROFILE_VELOCITY, id_list, velocity_list);
}

// ram read

/**
 * @brief DxlDriver<reg_type>::readPosition
 * @param id
 * @param present_position
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readPosition(uint8_t id, uint32_t& present_position)
{
    return read(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id, present_position);
}

/**
 * @brief DxlDriver<reg_type>::readTemperature
 * @param id
 * @param temperature
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readTemperature(uint8_t id, uint32_t& temperature)
{
    return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
}

/**
 * @brief DxlDriver<reg_type>::readVoltage
 * @param id
 * @param voltage
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readVoltage(uint8_t id, double& voltage)
{
  uint32_t voltage_mV = 0;
  int res = read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage_mV);
  voltage = static_cast<double>(voltage_mV) / reg_type::VOLTAGE_CONVERSION;
  return res;
}

/**
 * @brief DxlDriver<reg_type>::readHwErrorStatus
 * @param id
 * @param hardware_status
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
}

/**
 * @brief DxlDriver<reg_type>::syncReadPosition
 * @param id_list
 * @param position_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
  return syncRead(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id_list, position_list);
}

/**
 * @brief DxlDriver<reg_type>::writePID
 * @param id
 * @param data
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writePID(uint8_t id, const std::vector<uint32_t> &data)
{
    int res = 0;

    if (COMM_SUCCESS != writePositionPGain(id, data.at(0)))
        res++;
    if (COMM_SUCCESS != writePositionIGain(id, data.at(1)))
        res++;
    if (COMM_SUCCESS != writePositionDGain(id, data.at(2)))
        res++;
    if (COMM_SUCCESS != writeVelocityPGain(id, data.at(3)))
        res++;
    if (COMM_SUCCESS != writeVelocityIGain(id, data.at(4)))
        res++;
    if (COMM_SUCCESS != writeFF1Gain(id, data.at(5)))
        res++;
    if (COMM_SUCCESS != writeFF2Gain(id, data.at(6)))
        res++;

    if(res > 0)
    {
        std::cout << "Failures during writeVelocityProfile : " << res << std::endl;
        return COMM_TX_FAIL;
    }

    return COMM_SUCCESS;
}

/**
 * @brief DxlDriver<reg_type>::syncReadFirmwareVersion
 * @param id_list
 * @param firmware_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    int res = COMM_RX_FAIL;
    std::vector<uint32_t> data_list;
    res = syncRead(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id_list, data_list);
    for(auto const& data : data_list)
      firmware_list.emplace_back(interpreteFirmwareVersion(data));
    return res;
}

/**
 * @brief DxlDriver<reg_type>::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
}

/**
 * @brief DxlDriver<reg_type>::syncReadVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
  voltage_list.clear();
  std::vector<uint32_t> v_read;
  int res = syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, v_read);
  for(auto const& v : v_read)
      voltage_list.emplace_back(static_cast<double>(v) / reg_type::VOLTAGE_CONVERSION);
  return res;
}

/**
 * @brief DxlDriver<reg_type>::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
}

//*****************************
// AbstractDxlDriver interface
//*****************************


/**
 * @brief DxlDriver<reg_type>::writeLed
 * @param id
 * @param led_value
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeLed(uint8_t id, uint32_t led_value)
{
    return write(reg_type::ADDR_LED, reg_type::SIZE_LED, id, led_value);
}

/**
 * @brief DxlDriver<reg_type>::syncWriteLed
 * @param id_list
 * @param led_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list)
{
    return syncWrite(reg_type::ADDR_LED, reg_type::SIZE_LED, id_list, led_list);
}

/**
 * @brief DxlDriver<reg_type>::writeGoalTorque
 * @param id
 * @param torque
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeGoalTorque(uint8_t id, uint32_t torque)
{
    return write(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id, torque);
}

/**
 * @brief DxlDriver<reg_type>::syncWriteTorqueGoal
 * @param id_list
 * @param torque_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)
{
    return syncWrite(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id_list, torque_list);
}

// read

/**
 * @brief DxlDriver<reg_type>::readPID
 * @param id
 * @param data_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readPID(uint8_t id, std::vector<uint32_t>& data_list)
{
    int res = 0;
    data_list.clear();

    uint32_t pos_p_gain{0};
    if (COMM_SUCCESS != readPositionPGain(id, pos_p_gain))
        res++;

    data_list.emplace_back(pos_p_gain);

    uint32_t pos_i_gain{0};
    if (COMM_SUCCESS != readPositionIGain(id, pos_i_gain))
        res++;

    data_list.emplace_back(pos_i_gain);

    uint32_t pos_d_gain{0};
    if (COMM_SUCCESS != readPositionDGain(id, pos_d_gain))
        res++;

    data_list.emplace_back(pos_d_gain);

    uint32_t vel_p_gain{0};
    if (COMM_SUCCESS != readVelocityPGain(id, vel_p_gain))
        res++;

    data_list.emplace_back(vel_p_gain);

    uint32_t vel_i_gain{0};
    if (COMM_SUCCESS != readVelocityIGain(id, vel_i_gain))
        res++;

    data_list.emplace_back(vel_i_gain);

    uint32_t ff1_gain{0};
    if (COMM_SUCCESS != readFF1Gain(id, ff1_gain))
        res++;

    data_list.emplace_back(ff1_gain);

    uint32_t ff2_gain{0};
    if (COMM_SUCCESS != readFF2Gain(id, ff2_gain))
        res++;

    data_list.emplace_back(ff2_gain);

    if(res > 0)
    {
        std::cout << "Failures during read PID gains: " << res << std::endl;
        return COMM_TX_FAIL;
    }

    return COMM_SUCCESS;
}

//other

/**
 * @brief DxlDriver<reg_type>::readLoad
 * @param id
 * @param present_load
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readLoad(uint8_t id, uint32_t& present_load)
{
    return read(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id, present_load);
}

/**
 * @brief DxlDriver<reg_type>::syncReadLoad
 * @param id_list
 * @param load_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    return syncRead(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id_list, load_list);
}

/**
 * @brief DxlDriver<reg_type>::readVelocity
 * @param id
 * @param present_velocity
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readVelocity(uint8_t id, uint32_t& present_velocity)
{
    return read(reg_type::ADDR_PRESENT_VELOCITY, reg_type::SIZE_PRESENT_VELOCITY, id, present_velocity);
}

/**
 * @brief DxlDriver<reg_type>::syncReadVelocity
 * @param id_list
 * @param velocity_list
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    return syncRead(reg_type::ADDR_PRESENT_VELOCITY, reg_type::SIZE_PRESENT_VELOCITY, id_list, velocity_list);
}

// private
// read
/**
 * @brief DxlDriver<reg_type>::readPositionPGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readPositionPGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_POSITION_P_GAIN, reg_type::SIZE_POSITION_P_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::readPositionIGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readPositionIGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_POSITION_I_GAIN, reg_type::SIZE_POSITION_I_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::readPositionDGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readPositionDGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_POSITION_D_GAIN, reg_type::SIZE_POSITION_D_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::readVelocityPGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readVelocityPGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_VELOCITY_P_GAIN, reg_type::SIZE_VELOCITY_P_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::readVelocityIGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readVelocityIGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_VELOCITY_I_GAIN, reg_type::SIZE_VELOCITY_I_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::readFF1Gain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readFF1Gain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_FF1_GAIN, reg_type::SIZE_FF1_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::readFF2Gain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::readFF2Gain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_FF2_GAIN, reg_type::SIZE_FF2_GAIN, id, gain);
}

// write
/**
 * @brief DxlDriver<reg_type>::writePositionPGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writePositionPGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_POSITION_P_GAIN, reg_type::SIZE_POSITION_P_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::writePositionIGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writePositionIGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_POSITION_I_GAIN, reg_type::SIZE_POSITION_I_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::writePositionDGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writePositionDGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_POSITION_D_GAIN, reg_type::SIZE_POSITION_D_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::writeVelocityPGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeVelocityPGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_VELOCITY_P_GAIN, reg_type::SIZE_VELOCITY_P_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::writeVelocityIGain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeVelocityIGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_VELOCITY_I_GAIN, reg_type::SIZE_VELOCITY_I_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::writeFF1Gain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeFF1Gain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_FF1_GAIN, reg_type::SIZE_FF1_GAIN, id, gain);
}

/**
 * @brief DxlDriver<reg_type>::writeFF2Gain
 * @param id
 * @param gain
 * @return
 */
template<typename reg_type>
int DxlDriver<reg_type>::writeFF2Gain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_FF2_GAIN, reg_type::SIZE_FF2_GAIN, id, gain);
}

/*
 *  -----------------   specializations   --------------------
 */

// XL320

template<>
inline int DxlDriver<XL320Reg>::readMinPosition(uint8_t /*id*/, uint32_t &pos)
{
    pos = 0;
    std::cout << "min position hardcoded for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::readMaxPosition(uint8_t /*id*/, uint32_t &pos)
{
    pos = 1023;
    std::cout << "max position hardcoded for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline std::string DxlDriver<XL320Reg>::interpreteErrorState(uint32_t hw_state) const
{
    std::string hardware_message;

    if (hw_state & 1<<0)    // 0b00000001
    {
        hardware_message += "Overload";
    }
    if (hw_state & 1<<1)    // 0b00000010
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "OverHeating";
    }
    if (hw_state & 1<<2)    // 0b00000100
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Input voltage out of range";
    }

    return hardware_message;
}

template<>
inline int DxlDriver<XL320Reg>::writeVelocityPGain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "writeVelocityPGain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::writeVelocityIGain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "writeVelocityIGain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::writeFF1Gain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "writeff1Gain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::writeFF2Gain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "writeff2Gain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

// read PID
template<>
inline int DxlDriver<XL320Reg>::readVelocityPGain(uint8_t /*id*/, uint32_t& /*gain*/)
{
    std::cout << "readVelocityPGain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::readVelocityIGain(uint8_t /*id*/, uint32_t& /*gain*/)
{
    std::cout << "readVelocityIGain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::readFF1Gain(uint8_t /*id*/, uint32_t& /*gain*/)
{
    std::cout << "readFF1Gain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::readFF2Gain(uint8_t /*id*/, uint32_t& /*gain*/)
{
    std::cout << "readFF2Gain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::writeGoalVelocity(uint8_t id, uint32_t velocity)
{
    return write(XL320Reg::ADDR_GOAL_VELOCITY, XL320Reg::SIZE_GOAL_VELOCITY, id, velocity);
}

template<>
inline int DxlDriver<XL320Reg>::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
{
    return syncWrite(XL320Reg::ADDR_GOAL_VELOCITY, XL320Reg::SIZE_GOAL_VELOCITY, id_list, velocity_list);
}

// XL430

template<>
inline std::string DxlDriver<XL430Reg>::interpreteErrorState(uint32_t hw_state) const
{
    std::string hardware_message;

    if (hw_state & 1<<0)    // 0b00000001
    {
        hardware_message += "Input Voltage";
    }
    if (hw_state & 1<<2)    // 0b00000100
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "OverHeating";
    }
    if (hw_state & 1<<3)    // 0b00001000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Motor Encoder";
    }
    if (hw_state & 1<<4)    // 0b00010000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Electrical Shock";
    }
    if (hw_state & 1<<5)    // 0b00100000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Overload";
    }
    if (!hardware_message.empty())
        hardware_message += " Error";

    return hardware_message;
}

template<>
inline int DxlDriver<XL430Reg>::writeGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
{
    std::cout << "writeGoalTorque not available for motor XL430" << std::endl;
    return COMM_TX_ERROR;
}

template<>
inline int DxlDriver<XL430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &/*id_list*/, const std::vector<uint32_t> &/*torque_list*/)
{
    std::cout << "syncWriteTorqueGoal not available for motor XL430" << std::endl;
    return COMM_TX_ERROR;
}

// XC430

template<>
inline std::string DxlDriver<XC430Reg>::interpreteErrorState(uint32_t hw_state) const
{
    std::string hardware_message;

    if (hw_state & 1<<0)    // 0b00000001
    {
        hardware_message += "Input Voltage";
    }
    if (hw_state & 1<<2)    // 0b00000100
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "OverHeating";
    }
    if (hw_state & 1<<3)    // 0b00001000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Motor Encoder";
    }
    if (hw_state & 1<<4)    // 0b00010000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Electrical Shock";
    }
    if (hw_state & 1<<5)    // 0b00100000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Overload";
    }
    if (!hardware_message.empty())
        hardware_message += " Error";

    return hardware_message;
}

template<>
inline int DxlDriver<XC430Reg>::writeGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
{
    std::cout << "writeGoalTorque not available for motor XC430" << std::endl;
    return COMM_TX_ERROR;
}

template<>
inline int DxlDriver<XC430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &/*id_list*/, const std::vector<uint32_t> &/*torque_list*/)
{
    std::cout << "syncWriteTorqueGoal not available for motor XC430" << std::endl;
    return COMM_TX_ERROR;
}

// XL330

template<>
inline std::string DxlDriver<XL330Reg>::interpreteErrorState(uint32_t hw_state) const
{
    std::string hardware_message;

    if (hw_state & 1<<0)    // 0b00000001
    {
        hardware_message += "Input Voltage";
    }
    if (hw_state & 1<<2)    // 0b00000100
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "OverHeating";
    }
    if (hw_state & 1<<3)    // 0b00001000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Motor Encoder";
    }
    if (hw_state & 1<<4)    // 0b00010000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Electrical Shock";
    }
    if (hw_state & 1<<5)    // 0b00100000
    {
        if (!hardware_message.empty())
            hardware_message += ", ";
        hardware_message += "Overload";
    }
    if (!hardware_message.empty())
        hardware_message += " Error";

    return hardware_message;
}

// works with current instead of load

template<>
inline int DxlDriver<XL330Reg>::writeGoalTorque(uint8_t id, uint32_t torque)
{
    return write(XL330Reg::ADDR_GOAL_CURRENT, XL330Reg::SIZE_GOAL_CURRENT, id, torque);
}

template<>
inline int DxlDriver<XL330Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)
{
    return syncWrite(XL330Reg::ADDR_GOAL_CURRENT, XL330Reg::SIZE_GOAL_CURRENT, id_list, torque_list);
}

template<>
inline int DxlDriver<XL330Reg>::readLoad(uint8_t id, uint32_t& present_load)
{
    return read(XL330Reg::ADDR_PRESENT_CURRENT, XL330Reg::SIZE_PRESENT_CURRENT, id, present_load);
}

template<>
inline int DxlDriver<XL330Reg>::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    return syncRead(XL330Reg::ADDR_PRESENT_CURRENT, XL330Reg::SIZE_PRESENT_CURRENT, id_list, load_list);
}

} // ttl_driver

#endif // DxlDriver
