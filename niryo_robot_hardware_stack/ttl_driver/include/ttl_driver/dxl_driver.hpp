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

        int setTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        int setGoalPosition(uint8_t id, uint32_t position) override;
        int setGoalVelocity(uint8_t id, uint32_t velocity) override;

        int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        int readPosition(uint8_t id, uint32_t &present_position) override;
        
        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        
    public:
        // AbstractDxlDriver interface
        int setLed(uint8_t id, uint32_t led_value) override;
        int setGoalTorque(uint8_t id, uint32_t torque) override;
        int setPositionPGain(uint8_t id, uint32_t gain) override;
        int setPositionIGain(uint8_t id, uint32_t gain) override;
        int setPositionDGain(uint8_t id, uint32_t gain) override;
        int setVelocityPGain(uint8_t id, uint32_t gain) override;
        int setVelocityIGain(uint8_t id, uint32_t gain) override;
        int setff1Gain(uint8_t id, uint32_t gain) override;
        int setff2Gain(uint8_t id, uint32_t gain) override;
        
        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) override;
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) override;
        
        int readLoad(uint8_t id, uint32_t &present_load) override;
        int readVelocity(uint8_t id, uint32_t &present_velocity) override;

        int readPositionPGain(uint8_t id, uint32_t& gain) override;
        int readPositionIGain(uint8_t id, uint32_t& gain) override;
        int readPositionDGain(uint8_t id, uint32_t& gain) override;
        int readVelocityPGain(uint8_t id, uint32_t& gain) override;
        int readVelocityIGain(uint8_t id, uint32_t& gain) override;
        int readFF1Gain(uint8_t id, uint32_t& gain) override;
        int readFF2Gain(uint8_t id, uint32_t& gain) override;
        
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) override;
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
};

// definition of methods

/**
 * @brief DxlDriver<reg_type>::DxlDriver
 */
template<typename reg_type>
DxlDriver<reg_type>::DxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                               std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractDxlDriver(std::move(portHandler), std::move(packetHandler))
{}

//*****************************
// AbstractMotorDriver interface
//*****************************

template<typename reg_type>
std::string DxlDriver<reg_type>::str() const
{
    return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + AbstractDxlDriver::str();
}

template<typename reg_type>
std::string DxlDriver<reg_type>::interpreteErrorState(uint32_t /*hw_state*/) const
{
    return "";
}

template<typename reg_type>
std::string DxlDriver<reg_type>::interpreteFirmwareVersion(uint32_t fw_version) const
{
    std::string version = std::to_string(static_cast<uint8_t>(fw_version));

    return version;
}

template<typename reg_type>
int DxlDriver<reg_type>::changeId(uint8_t id, uint8_t new_id)
{
    return write(reg_type::ADDR_ID, reg_type::SIZE_ID, id, new_id);
}

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

template<typename reg_type>
int DxlDriver<reg_type>::readFirmwareVersion(uint8_t id, std::string &version)
{
    int res = COMM_RX_FAIL;
    uint32_t data{};
    res = read(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id, data);
    version = interpreteFirmwareVersion(data);
    return res;
}

template<typename reg_type>
int DxlDriver<reg_type>::readMinPosition(uint8_t id, uint32_t &pos)
{
    return read(reg_type::ADDR_MIN_POSITION_LIMIT, reg_type::SIZE_MIN_POSITION_LIMIT, id, pos);
}

template<typename reg_type>
int DxlDriver<reg_type>::readMaxPosition(uint8_t id, uint32_t &pos)
{
    return read(reg_type::ADDR_MAX_POSITION_LIMIT, reg_type::SIZE_MAX_POSITION_LIMIT, id, pos);
}

// ram write

template<typename reg_type>
int DxlDriver<reg_type>::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return write(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id, torque_enable);
}

template<typename reg_type>
int DxlDriver<reg_type>::setGoalPosition(uint8_t id, uint32_t position)
{
    return write(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id, position);
}

template<typename reg_type>
int DxlDriver<reg_type>::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    // in mode control Position Control Mode, velocity profile in datasheet is used to set velocity (except xl320)
    return write(reg_type::ADDR_PROFILE_VELOCITY, reg_type::SIZE_PROFILE_VELOCITY, id, velocity);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)
{
    return syncWrite(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id_list, torque_enable_list);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    return syncWrite(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id_list, position_list);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
{
    // in mode control Position Control Mode, velocity profile in datasheet is used to set velocity (except xl320)
    return syncWrite(reg_type::ADDR_PROFILE_VELOCITY, reg_type::SIZE_PROFILE_VELOCITY, id_list, velocity_list);
}

// ram read

template<typename reg_type>
int DxlDriver<reg_type>::readPosition(uint8_t id, uint32_t& present_position)
{
    return read(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id, present_position);
}

template<typename reg_type>
int DxlDriver<reg_type>::readTemperature(uint8_t id, uint32_t& temperature)
{
    return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
}

template<typename reg_type>
int DxlDriver<reg_type>::readVoltage(uint8_t id, double& voltage)
{
  uint32_t voltage_mV = 0;
  int res = read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage_mV);
  voltage = static_cast<double>(voltage_mV) / reg_type::VOLTAGE_CONVERSION;
  return res;
}

template<typename reg_type>
int DxlDriver<reg_type>::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    return syncRead(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id_list, position_list);
}

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

template<typename reg_type>
int DxlDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
}

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

template<typename reg_type>
int DxlDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
}

//*****************************
// AbstractDxlDriver interface
//*****************************


template<typename reg_type>
int DxlDriver<reg_type>::setLed(uint8_t id, uint32_t led_value)
{
    return write(reg_type::ADDR_LED, reg_type::SIZE_LED, id, led_value);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list)
{
    return syncWrite(reg_type::ADDR_LED, reg_type::SIZE_LED, id_list, led_list);
}

template<typename reg_type>
int DxlDriver<reg_type>::setGoalTorque(uint8_t id, uint32_t torque)
{
    return write(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id, torque);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)
{
    return syncWrite(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id_list, torque_list);
}

//PID

template<typename reg_type>
int DxlDriver<reg_type>::setPositionPGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_POSITION_P_GAIN, reg_type::SIZE_POSITION_P_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::setPositionIGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_POSITION_I_GAIN, reg_type::SIZE_POSITION_I_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::setPositionDGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_POSITION_D_GAIN, reg_type::SIZE_POSITION_D_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::setVelocityPGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_VELOCITY_P_GAIN, reg_type::SIZE_VELOCITY_P_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::setVelocityIGain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_VELOCITY_I_GAIN, reg_type::SIZE_VELOCITY_I_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::setff1Gain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_FF1_GAIN, reg_type::SIZE_FF1_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::setff2Gain(uint8_t id, uint32_t gain)
{
    return write(reg_type::ADDR_FF2_GAIN, reg_type::SIZE_FF2_GAIN, id, gain);
}

// read
template<typename reg_type>
int DxlDriver<reg_type>::readPositionPGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_POSITION_P_GAIN, reg_type::SIZE_POSITION_P_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::readPositionIGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_POSITION_I_GAIN, reg_type::SIZE_POSITION_I_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::readPositionDGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_POSITION_D_GAIN, reg_type::SIZE_POSITION_D_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::readVelocityPGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_VELOCITY_P_GAIN, reg_type::SIZE_VELOCITY_P_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::readVelocityIGain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_VELOCITY_I_GAIN, reg_type::SIZE_VELOCITY_I_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::readFF1Gain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_FF1_GAIN, reg_type::SIZE_FF1_GAIN, id, gain);
}

template<typename reg_type>
int DxlDriver<reg_type>::readFF2Gain(uint8_t id, uint32_t& gain)
{
    return read(reg_type::ADDR_FF2_GAIN, reg_type::SIZE_FF2_GAIN, id, gain);
}

//other

template<typename reg_type>
int DxlDriver<reg_type>::readLoad(uint8_t id, uint32_t& present_load)
{
    return read(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id, present_load);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    return syncRead(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id_list, load_list);
}

template<typename reg_type>
int DxlDriver<reg_type>::readVelocity(uint8_t id, uint32_t& present_velocity)
{
    return read(reg_type::ADDR_PRESENT_VELOCITY, reg_type::SIZE_PRESENT_VELOCITY, id, present_velocity);
}

template<typename reg_type>
int DxlDriver<reg_type>::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    return syncRead(reg_type::ADDR_PRESENT_VELOCITY, reg_type::SIZE_PRESENT_VELOCITY, id_list, velocity_list);
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
inline int DxlDriver<XL320Reg>::setVelocityPGain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "setVelocityPGain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::setVelocityIGain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "setVelocityIGain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::setff1Gain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "setff1Gain not available for motor XL320" << std::endl;
    return COMM_SUCCESS;
}

template<>
inline int DxlDriver<XL320Reg>::setff2Gain(uint8_t /*id*/, uint32_t /*gain*/)
{
    std::cout << "setff2Gain not available for motor XL320" << std::endl;
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
inline int DxlDriver<XL320Reg>::setGoalVelocity(uint8_t id, uint32_t velocity)
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
inline int DxlDriver<XL430Reg>::setGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
{
    std::cout << "setGoalTorque not available for motor XL430" << std::endl;
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
inline int DxlDriver<XC430Reg>::setGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
{
    std::cout << "setGoalTorque not available for motor XC430" << std::endl;
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
inline int DxlDriver<XL330Reg>::setGoalTorque(uint8_t id, uint32_t torque)
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
