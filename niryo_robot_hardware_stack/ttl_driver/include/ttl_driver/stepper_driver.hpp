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
        virtual ~StepperDriver() override;


    public:
        // AbstractTtlDriver interface
        virtual std::string str() const override;

        virtual std::string interpreteErrorState(uint32_t hw_state) override;
        virtual int checkModelNumber(uint8_t id) override;
        virtual int readFirmwareVersion(uint8_t id, uint32_t &version) override;

        virtual int readTemperature(uint8_t id, uint32_t &temperature) override;
        virtual int readVoltage(uint8_t id, uint32_t &voltage) override;
        virtual int readHwErrorStatus(uint8_t id, uint32_t &hardware_status) override;

        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) override;
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

    public:
        // AbstractMotorDriver interface : we cannot define them globally in AbstractMotorDriver
        // as it is needed here for polymorphism (AbstractMotorDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of DxlDriver

        virtual int changeId(uint8_t id, uint8_t new_id) override;

        virtual int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        virtual int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        virtual int setTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        virtual int setGoalPosition(uint8_t id, uint32_t position) override;
        virtual int setGoalVelocity(uint8_t id, uint32_t velocity) override;
        
        virtual int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        virtual int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        virtual int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        // ram read
        virtual int readPosition(uint8_t id, uint32_t &present_position) override;
       
        virtual int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
       
        // AbstractStepperDriver interface
    public:
        virtual int startHoming(uint8_t id) override;
        virtual int readHomingStatus(uint8_t id, uint32_t &status) override;
        // conveyor control
        virtual int setGoalConveyorDirection(uint8_t id, int8_t direction) override;
        virtual int setConveyorState(uint8_t id, bool state) override;
        virtual int readConveyorSpeed(uint8_t id, uint32_t &velocity) override;
        virtual int readConveyorDirection(uint8_t id, int8_t &direction) override;
        virtual int readConveyorState(uint8_t id, bool &state) override;
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
{
}

//*****************************
// AbstractMotorDriver interface
//*****************************

template<typename reg_type>
std::string StepperDriver<reg_type>::str() const
{
    return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + AbstractStepperDriver::str();
}

template<typename reg_type>
std::string StepperDriver<reg_type>::interpreteErrorState(uint32_t /*hw_state*/)
{
    return "no error table";
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
int StepperDriver<reg_type>::readFirmwareVersion(uint8_t id, uint32_t &version)
{
    return read(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id, version);
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
int StepperDriver<reg_type>::readVoltage(uint8_t id, uint32_t& voltage)
{
    return read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage);
}

template<typename reg_type>
int StepperDriver<reg_type>::readHwErrorStatus(uint8_t /*id*/, uint32_t& hardware_status)
{
    hardware_status = 0;
    std::cout << "readHwErrorStatus not yet implemented" << std::endl;
    //return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
    return COMM_RX_FAIL;
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    return syncRead(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id_list, position_list);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
{
    return syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, voltage_list);
}

template<typename reg_type>
int StepperDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    std::cout << "readHwErrorStatus not yet implemented" << std::endl;

   // return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
    return COMM_RX_FAIL;
}

//*****************************
// AbstractStepperDriver interface
//*****************************

template<typename reg_type>
int StepperDriver<reg_type>::startHoming(uint8_t id)
{
    return write(reg_type::ADDR_COMMAND, reg_type::SIZE_COMMAND, id, 0);
}

template<typename reg_type>
int StepperDriver<reg_type>::readHomingStatus(uint8_t id, uint32_t &status)
{
    return read(reg_type::ADDR_HOMING_STATUS, reg_type::SIZE_HOMING_STATUS, id, status);
}

template<typename reg_type>
int StepperDriver<reg_type>::setGoalConveyorDirection(uint8_t id, int8_t direction)
{
    ROS_INFO("StepperDriver<reg_type>::setGoalConveyorDirection: need to be implemented!");
    return 0;
}

template<typename reg_type>
int StepperDriver<reg_type>::setConveyorState(uint8_t id, bool state)
{
    ROS_INFO("StepperDriver<reg_type>::setConveyorState: need to be implemented!");
    return 0;
}

template<typename reg_type>
int StepperDriver<reg_type>::readConveyorSpeed(uint8_t id, uint32_t &velocity)
{
    ROS_INFO("StepperDriver<reg_type>::readConveyorSpeed: need to be implemented!");
    return 0;
}

template<typename reg_type>
int StepperDriver<reg_type>::readConveyorDirection(uint8_t id, int8_t &direction)
{
    ROS_INFO("StepperDriver<reg_type>::readConveyorDirection: need to be implemented!");
    return 0;
}

template<typename reg_type>
int StepperDriver<reg_type>::readConveyorState(uint8_t id, bool &state)
{
    ROS_INFO("StepperDriver<reg_type>::getConveyorStatus: need to be implemented!");
    return 0;
}

} // ttl_driver

#endif // STEPPER_DRIVER_HPP
