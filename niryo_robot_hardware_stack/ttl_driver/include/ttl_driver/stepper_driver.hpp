/*
stepper_driver.hpp
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

#ifndef STEPPER_DRIVER_HPP
#define STEPPER_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include "ros/duration.h"

#include "abstract_stepper_driver.hpp"

#include "stepper_reg.hpp"

namespace ttl_driver
{

/**
 * @brief The StepperDriver class
 */
template<typename reg_type = StepperReg>
class StepperDriver : public AbstractStepperDriver
{
    public:
        StepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                      std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        ~StepperDriver() override;


    public:
        // AbstractTtlDriver interface
        std::string str() const override;

        std::string interpreteErrorState(uint32_t hw_state) const override;

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
        // AbstractMotorDriver interface : we cannot define them globally in AbstractMotorDriver
        // as it is needed here for polymorphism (AbstractMotorDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of DxlDriver

        int changeId(uint8_t id, uint8_t new_id) override;

        int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        int setTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        int setGoalPosition(uint8_t id, uint32_t position) override;
        int setGoalVelocity(uint8_t id, uint32_t velocity) override;
        
        int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        // ram read
        int readPosition(uint8_t id, uint32_t &present_position) override;

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;

        // AbstractStepperDriver interface
    public:
        int writeVelocityProfile(uint8_t id, const std::vector<uint32_t>& data) override;

        int startHoming(uint8_t id) override;
        int writeHomingDirection(uint8_t id, uint8_t direction) override;
        int readHomingStatus(uint8_t id, uint32_t &status) override;
        int readGoalVelocity(uint8_t id, uint32_t& present_velocity) override;

        int readFirmwareRunning(uint8_t id, bool &is_running) override;
};

// definition of methods

/**
 * @brief DxlDriver<reg_type>::DxlDriver
 */
template<typename reg_type>
StepperDriver<reg_type>::StepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                       std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractStepperDriver(portHandler, packetHandler)
{
}

/**
 * @brief DxlDriver<reg_type>::~DxlDriver
 */
template<typename reg_type>
StepperDriver<reg_type>::~StepperDriver()
= default;

//*****************************
// AbstractMotorDriver interface
//*****************************

template<typename reg_type>
std::string StepperDriver<reg_type>::str() const
{
    return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + AbstractStepperDriver::str();
}

template<typename reg_type>
std::string StepperDriver<reg_type>::interpreteErrorState(uint32_t hw_state) const
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

    if (!hardware_message.empty())
        hardware_message += " Error";

    return hardware_message;
}

template<typename reg_type>
int StepperDriver<reg_type>::changeId(uint8_t id, uint8_t new_id)
{
    return write(reg_type::ADDR_ID, reg_type::SIZE_ID, id, new_id);
}

template<typename reg_type>
int StepperDriver<reg_type>::checkModelNumber(uint8_t id)
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
int StepperDriver<reg_type>::readFirmwareVersion(uint8_t id, std::string &version)
{
    int res = 0;
    uint32_t data{};
    res = read(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id, data);
    version = interpreteFirmwareVersion(data);
    return res;
}

template<typename reg_type>
int StepperDriver<reg_type>::readMinPosition(uint8_t id, uint32_t &pos)
{
    return read(reg_type::ADDR_MIN_POSITION_LIMIT, reg_type::SIZE_MIN_POSITION_LIMIT, id, pos);
}

template<typename reg_type>
int StepperDriver<reg_type>::readMaxPosition(uint8_t id, uint32_t &pos)
{
    return read(reg_type::ADDR_MAX_POSITION_LIMIT, reg_type::SIZE_MAX_POSITION_LIMIT, id, pos);
}

// ram write

template<typename reg_type>
int StepperDriver<reg_type>::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return write(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id, torque_enable);
}

template<typename reg_type>
int StepperDriver<reg_type>::setGoalPosition(uint8_t id, uint32_t position)
{
    return write(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id, position);
}

// according to the registers, the data should be an int32_t ?
template<typename reg_type>
int StepperDriver<reg_type>::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    return write(reg_type::ADDR_GOAL_VELOCITY, reg_type::SIZE_GOAL_VELOCITY, id, velocity);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)
{
    return syncWrite(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id_list, torque_enable_list);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    return syncWrite(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id_list, position_list);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
{
    return syncWrite(reg_type::ADDR_GOAL_VELOCITY, reg_type::SIZE_GOAL_VELOCITY, id_list, velocity_list);
}

// ram read

template<typename reg_type>
int StepperDriver<reg_type>::readPosition(uint8_t id, uint32_t& present_position)
{
    return read(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id, present_position);
}

template<typename reg_type>
int StepperDriver<reg_type>::readTemperature(uint8_t id, uint32_t& temperature)
{
    return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
}

template<typename reg_type>
int StepperDriver<reg_type>::readVoltage(uint8_t id, double& voltage)
{
    uint32_t voltage_mV = 0;
    int res = read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage_mV);
    voltage = static_cast<double>(voltage_mV)  / reg_type::VOLTAGE_CONVERSION;
    return res;
}

template<typename reg_type>
int StepperDriver<reg_type>::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
  return syncRead(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id_list, position_list);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    int res = 0;
    firmware_list.clear();
    std::vector<uint32_t> data_list{};
    res = syncRead(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id_list, data_list);
    for(auto const& data : data_list)
      firmware_list.emplace_back(interpreteFirmwareVersion(data));
    return res;
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    voltage_list.clear();
    std::vector<uint32_t> v_read;
    int res = syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, v_read);
    for(auto const& v : v_read)
        voltage_list.emplace_back(static_cast<double>(v)  / reg_type::VOLTAGE_CONVERSION);
    return res;
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
   return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
}

//*****************************
// AbstractStepperDriver interface
//*****************************

template<typename reg_type>
int StepperDriver<reg_type>::writeVelocityProfile(uint8_t id, const std::vector<uint32_t>& data)
{
  int res = 0;
  double wait_duration = 0.5;

  setTorqueEnable(id, true);
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_VSTART, reg_type::SIZE_VSTART, id, data.at(0)))
    res++;
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_A1, reg_type::SIZE_A1, id, data.at(1)))
    res++;
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_V1, reg_type::SIZE_V1, id, data.at(2)))
    res++;
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_AMAX, reg_type::SIZE_AMAX, id, data.at(3)))
    res++;
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_VMAX, reg_type::SIZE_VMAX, id, data.at(4)))
    res++;
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_DMAX, reg_type::SIZE_DMAX, id, data.at(5)))
    res++;
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_D1, reg_type::SIZE_D1, id, data.at(6)))
    res++;
  ros::Duration(wait_duration).sleep();

  if (COMM_SUCCESS != write(reg_type::ADDR_VSTOP, reg_type::SIZE_VSTOP, id, data.at(7)))
    res++;
  ros::Duration(wait_duration).sleep();

  if(res > 0)
  {
      std::cout << "Failures during writeVelocityProfile : " << res << std::endl;
      return COMM_TX_FAIL;
  }
  
    return COMM_SUCCESS;
  

}

template<typename reg_type>
int StepperDriver<reg_type>::startHoming(uint8_t id)
{
    return write(reg_type::ADDR_COMMAND, reg_type::SIZE_COMMAND, id, 0);
}

template<typename reg_type>
int StepperDriver<reg_type>::writeHomingDirection(uint8_t id, uint8_t direction)
{
    return write(reg_type::ADDR_HOMING_DIRECTION, reg_type::SIZE_HOMING_DIRECTION, id, direction);
}

template<typename reg_type>
int StepperDriver<reg_type>::readHomingStatus(uint8_t id, uint32_t &status)
{
    return read(reg_type::ADDR_HOMING_STATUS, reg_type::SIZE_HOMING_STATUS, id, status);
}

template<typename reg_type>
int StepperDriver<reg_type>::readGoalVelocity(uint8_t id, uint32_t& present_velocity)
{
    return read(reg_type::ADDR_GOAL_VELOCITY, reg_type::SIZE_GOAL_VELOCITY, id, present_velocity);
}

template<typename reg_type>
int StepperDriver<reg_type>::readFirmwareRunning(uint8_t id, bool &is_running)
{
  uint32_t data{};
  int res = read(reg_type::ADDR_FIRMWARE_RUNNING, reg_type::SIZE_FIRMWARE_RUNNING, id, data);
  is_running = data;
  return res;
}

} // ttl_driver

#endif // STEPPER_DRIVER_HPP
