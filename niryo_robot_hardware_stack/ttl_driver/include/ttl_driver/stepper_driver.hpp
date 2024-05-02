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
    template <typename reg_type = StepperReg>
    class StepperDriver : public AbstractStepperDriver
    {
    public:
        StepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                      std::shared_ptr<dynamixel::PacketHandler> packetHandler);

    public:
        // AbstractTtlDriver interface
        std::string str() const override;

        int checkModelNumber(uint8_t id) override;
        int readFirmwareVersion(uint8_t id, std::string &version) override;

        int readTemperature(uint8_t id, uint8_t &temperature) override;
        int readVoltage(uint8_t id, double &voltage) override;
        int readHwErrorStatus(uint8_t id, uint8_t &hardware_error_status) override;

        int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &temperature_list) override;
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadHwStatus(const std::vector<uint8_t> &id_list, std::vector<std::pair<double, uint8_t>> &data_list) override;

        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list) override;

    public:
        // AbstractMotorDriver interface : we cannot define them globally in AbstractMotorDriver
        // as it is needed here for polymorphism (AbstractMotorDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of DxlDriver

        int changeId(uint8_t id, uint8_t new_id) override;

        int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        int writeTorquePercentage(uint8_t id, uint8_t torque_enable) override;
        int writePositionGoal(uint8_t id, uint32_t position) override;
        int writeVelocityGoal(uint8_t id, uint32_t velocity) override;

        int syncWriteTorquePercentage(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &torque_percentage_list) override;
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        // ram read
        int readPosition(uint8_t id, uint32_t &present_position) override;
        int readVelocity(uint8_t id, uint32_t &present_velocity) override;

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
        int syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2>> &data_array_list) override;

        // AbstractStepperDriver interface
    public:
        int readControlMode(uint8_t id, uint8_t &operating_mode) override;
        int writeControlMode(uint8_t id, uint8_t operating_mode);

        int readVelocityProfile(uint8_t id, std::vector<uint32_t> &data_list) override;
        int writeVelocityProfile(uint8_t id, const std::vector<uint32_t> &data_list) override;

        int startHoming(uint8_t id) override;
        int writeHomingSetup(uint8_t id, uint8_t direction, uint8_t stall_threshold) override;

        int readHomingStatus(uint8_t id, uint8_t &status) override;
        int syncReadHomingStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &status_list) override;

        int readFirmwareRunning(uint8_t id, bool &is_running) override;

        int syncReadHomingAbsPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &abs_position) override;
        int syncWriteHomingAbsPosition(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &abs_position) override;

        float velocityUnit() const;

    private:
        int writeVStart(uint8_t id, uint32_t v_start);
        int writeA1(uint8_t id, uint32_t a_1);
        int writeV1(uint8_t id, uint32_t v_1);
        int writeAMax(uint8_t id, uint32_t a_max);
        int writeVMax(uint8_t id, uint32_t v_max);
        int writeDMax(uint8_t id, uint32_t d_max);
        int writeD1(uint8_t id, uint32_t d_1);
        int writeVStop(uint8_t id, uint32_t v_stop);
    };

    // definition of methods

    /**
     * @brief DxlDriver<reg_type>::DxlDriver
     */
    template <typename reg_type>
    StepperDriver<reg_type>::StepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                           std::shared_ptr<dynamixel::PacketHandler> packetHandler) : AbstractStepperDriver(std::move(portHandler),
                                                                                                                            std::move(packetHandler))
    {
    }

    //*****************************
    // AbstractMotorDriver interface
    //*****************************

    /**
     * @brief StepperDriver<reg_type>::str
     * @return
     */
    template <typename reg_type>
    std::string StepperDriver<reg_type>::str() const
    {
        return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + AbstractStepperDriver::str();
    }

    /**
     * @brief StepperDriver<reg_type>::changeId
     * @param id
     * @param new_id
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::changeId(uint8_t id, uint8_t new_id)
    {
        // TODO(THUC) verify that COMM_RX_TIMEOUT error code do not impact on data sent to motor
        // when COMM_RX_TIMEOUT error, id changed also, so we consider that change id successfully
        int res = write<typename reg_type::TYPE_ID>(reg_type::ADDR_ID, id, new_id);
        if (res == COMM_RX_TIMEOUT)
            res = COMM_SUCCESS;
        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::checkModelNumber
     * @param id
     * @return
     */
    template <typename reg_type>
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

    /**
     * @brief StepperDriver<reg_type>::readFirmwareVersion
     * @param id
     * @param version
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readFirmwareVersion(uint8_t id, std::string &version)
    {
        int res = 0;
        uint32_t data{};
        res = read<typename reg_type::TYPE_FIRMWARE_VERSION>(reg_type::ADDR_FIRMWARE_VERSION, id, data);
        version = interpretFirmwareVersion(data);
        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::readMinPosition
     * @param id
     * @param pos
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readMinPosition(uint8_t id, uint32_t &pos)
    {
        return read<typename reg_type::TYPE_MIN_POSITION_LIMIT>(reg_type::ADDR_MIN_POSITION_LIMIT, id, pos);
    }

    /**
     * @brief StepperDriver<reg_type>::readMaxPosition
     * @param id
     * @param pos
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readMaxPosition(uint8_t id, uint32_t &pos)
    {
        return read<typename reg_type::TYPE_MAX_POSITION_LIMIT>(reg_type::ADDR_MAX_POSITION_LIMIT, id, pos);
    }

    // ram write

    /**
     * @brief StepperDriver<reg_type>::writeTorquePercentage
     * @param id
     * @param torque_percentage
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeTorquePercentage(uint8_t id, uint8_t torque_percentage)
    {
        auto torque_enable = torque_percentage > 0 ? 1 : 0;
        return write<typename reg_type::TYPE_TORQUE_ENABLE>(reg_type::ADDR_TORQUE_ENABLE, id, torque_enable);
    }

    /**
     * @brief StepperDriver<reg_type>::writePositionGoal
     * @param id
     * @param position
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writePositionGoal(uint8_t id, uint32_t position)
    {
        return write<typename reg_type::TYPE_GOAL_POSITION>(reg_type::ADDR_GOAL_POSITION, id, position);
    }

    // according to the registers, the data should be an int32_t ?
    /**
     * @brief StepperDriver<reg_type>::writeVelocityGoal
     * @param id
     * @param velocity
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeVelocityGoal(uint8_t id, uint32_t velocity)
    {
        return write<typename reg_type::TYPE_GOAL_VELOCITY>(reg_type::ADDR_GOAL_VELOCITY, id, velocity);
    }

    /**
     * @brief StepperDriver<reg_type>::syncWriteTorquePercentage
     * @param id_list
     * @param torque_percentage_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncWriteTorquePercentage(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &torque_percentage_list)
    {
        std::vector<uint8_t> torque_enable_list;
        for(const auto &torque_percentage : torque_percentage_list)
        {
            auto torque_enable = torque_percentage > 0 ? 1 : 0;
            torque_enable_list.push_back(torque_enable);
        }
        return syncWrite<typename reg_type::TYPE_TORQUE_ENABLE>(reg_type::ADDR_TORQUE_ENABLE, id_list, torque_enable_list);
    }

    /**
     * @brief StepperDriver<reg_type>::syncWritePositionGoal
     * @param id_list
     * @param position_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
    {
        return syncWrite<typename reg_type::TYPE_GOAL_POSITION>(reg_type::ADDR_GOAL_POSITION, id_list, position_list);
    }

    /**
     * @brief StepperDriver<reg_type>::syncWriteVelocityGoal
     * @param id_list
     * @param velocity_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
    {
        return syncWrite<typename reg_type::TYPE_GOAL_VELOCITY>(reg_type::ADDR_GOAL_VELOCITY, id_list, velocity_list);
    }

    // ram read

    /**
     * @brief StepperDriver<reg_type>::readPosition
     * @param id
     * @param present_position
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readPosition(uint8_t id, uint32_t &present_position)
    {
        return read<typename reg_type::TYPE_PRESENT_POSITION>(reg_type::ADDR_PRESENT_POSITION, id, present_position);
    }

    /**
     * @brief StepperDriver<reg_type>::readVelocity
     * @param id
     * @param present_velocity
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readVelocity(uint8_t id, uint32_t &present_velocity)
    {
        return read<typename reg_type::TYPE_PRESENT_VELOCITY>(reg_type::ADDR_PRESENT_VELOCITY, id, present_velocity);
    }

    /**
     * @brief StepperDriver<reg_type>::readTemperature
     * @param id
     * @param temperature
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readTemperature(uint8_t id, uint8_t &temperature)
    {
        return read<typename reg_type::TYPE_PRESENT_TEMPERATURE>(reg_type::ADDR_PRESENT_TEMPERATURE, id, temperature);
    }

    /**
     * @brief StepperDriver<reg_type>::readVoltage
     * @param id
     * @param voltage
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readVoltage(uint8_t id, double &voltage)
    {
        uint16_t voltage_mV = 0;
        int res = read<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id, voltage_mV);
        voltage = static_cast<double>(voltage_mV) / reg_type::VOLTAGE_CONVERSION;
        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::readHwErrorStatus
     * @param id
     * @param hardware_error_status
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readHwErrorStatus(uint8_t id, uint8_t &hardware_error_status)
    {
        return read<typename reg_type::TYPE_HW_ERROR_STATUS>(reg_type::ADDR_HW_ERROR_STATUS, id, hardware_error_status);
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadPosition
     * @param id_list
     * @param position_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
    {
        return syncRead<typename reg_type::TYPE_PRESENT_POSITION>(reg_type::ADDR_PRESENT_POSITION, id_list, position_list);
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadVelocity
     * @param id_list
     * @param velocity_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
    {
        return syncRead<typename reg_type::TYPE_PRESENT_VELOCITY>(reg_type::ADDR_PRESENT_VELOCITY, id_list, velocity_list);
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadJointStatus
     * @param id_list
     * @param data_array_list
     * @return
     * reads both position and velocity if torque is enabled. Reads only position otherwise
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadJointStatus(const std::vector<uint8_t> &id_list,
                                                     std::vector<std::array<uint32_t, 2>> &data_array_list)
    {
        int res = COMM_TX_FAIL;

        if (id_list.empty())
            return res;

        data_array_list.clear();

        // read torque enable on first id
        typename reg_type::TYPE_TORQUE_ENABLE torque{1};
        res = read<typename reg_type::TYPE_TORQUE_ENABLE>(reg_type::ADDR_TORQUE_ENABLE, id_list.at(0), torque);
        if (COMM_SUCCESS != res)
            std::cout << "#############"
                         " ERROR reading stepper torque in syncReadJointStatus (error "
                      << res << ")" << std::endl;

        // if torque on, read position and velocity
        if (torque)
        {
            res = syncReadConsecutiveBytes<uint32_t, 2>(reg_type::ADDR_PRESENT_VELOCITY, id_list, data_array_list);
        }
        else // else read position only
        {
            std::vector<uint32_t> position_list;
            res = syncReadPosition(id_list, position_list);
            for (auto p : position_list)
                data_array_list.emplace_back(std::array<uint32_t, 2>{1, p});
        }

        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadFirmwareVersion
     * @param id_list
     * @param firmware_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
    {
        int res = 0;
        firmware_list.clear();
        std::vector<uint32_t> data_list{};
        res = syncRead<typename reg_type::TYPE_FIRMWARE_VERSION>(reg_type::ADDR_FIRMWARE_VERSION, id_list, data_list);
        for (auto const &data : data_list)
            firmware_list.emplace_back(interpretFirmwareVersion(data));
        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadTemperature
     * @param id_list
     * @param temperature_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &temperature_list)
    {
        return syncRead<typename reg_type::TYPE_PRESENT_TEMPERATURE>(reg_type::ADDR_PRESENT_TEMPERATURE, id_list, temperature_list);
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadVoltage
     * @param id_list
     * @param voltage_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
    {
        voltage_list.clear();
        std::vector<uint16_t> v_read;
        int res = syncRead<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, v_read);
        for (auto const &v : v_read)
            voltage_list.emplace_back(static_cast<double>(v) / reg_type::VOLTAGE_CONVERSION);
        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadRawVoltage
     * @param id_list
     * @param voltage_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
    {
        voltage_list.clear();
        std::vector<uint16_t> v_read;
        int res = syncRead<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, v_read);
        for (auto const &v : v_read)
            voltage_list.emplace_back(static_cast<double>(v));
        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadHwStatus
     * @param id_list
     * @param data_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadHwStatus(const std::vector<uint8_t> &id_list,
                                                  std::vector<std::pair<double, uint8_t>> &data_list)
    {
        data_list.clear();

        std::vector<std::array<uint8_t, 3>> raw_data;
        int res = syncReadConsecutiveBytes<uint8_t, 3>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, raw_data);

        for (auto const &data : raw_data)
        {
            // Voltage is first reg, uint16
            auto v = static_cast<uint16_t>((static_cast<uint16_t>(data.at(1)) << 8) | data.at(0)); // concatenate 2 bytes
            auto voltage = static_cast<double>(v);

            // Temperature is second reg, uint8
            uint8_t temperature = data.at(2);

            data_list.emplace_back(std::make_pair(voltage, temperature));
        }

        return res;
    }
    /**
     * @brief StepperDriver<reg_type>::syncReadHwErrorStatus
     * @param id_list
     * @param hw_error_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list)
    {
        return syncRead<typename reg_type::TYPE_HW_ERROR_STATUS>(reg_type::ADDR_HW_ERROR_STATUS, id_list, hw_error_list);
    }

    //*****************************
    // AbstractStepperDriver interface
    //*****************************

    template <typename reg_type>
    int StepperDriver<reg_type>::readControlMode(uint8_t id, uint8_t &operating_mode)
    {
        return read<typename reg_type::TYPE_OPERATING_MODE>(reg_type::ADDR_OPERATING_MODE, id, operating_mode);
    }

    template <typename reg_type>
    int StepperDriver<reg_type>::writeControlMode(uint8_t id, const uint8_t operating_mode)
    {
        return write<typename reg_type::TYPE_OPERATING_MODE>(reg_type::ADDR_OPERATING_MODE, id, operating_mode);
    }

    /**
     * @brief StepperDriver::readVelocityProfile
     * @param id
     * @param data_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readVelocityProfile(uint8_t id, std::vector<uint32_t> &data_list)
    {
        int res = COMM_RX_FAIL;
        data_list.clear();

        std::vector<std::array<typename reg_type::TYPE_PROFILE, 8>> data_array_list;
        if (COMM_SUCCESS == syncReadConsecutiveBytes<typename reg_type::TYPE_PROFILE, 8>(reg_type::ADDR_VSTART, {id}, data_array_list))
        {
            if (!data_array_list.empty())
            {
                auto block = data_array_list.at(0);

                for (auto data : block)
                {
                    data_list.emplace_back(data);
                }
            }

            res = COMM_SUCCESS;
        }
        else
        {
            std::cout << "Failures during read Velocity Profile" << std::endl;
        }

        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::writeVelocityProfile
     * @param id
     * @param data_list
     * @return
     * TODO(CC) remove sleeps when fw will be updated
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeVelocityProfile(uint8_t id, const std::vector<uint32_t> &data_list)
    {
        int tries = 10;
        int res = COMM_RX_FAIL;
        double wait_duration = 0.05;

        // Random positive value to activate the torque, we need this to write on other registers
        constexpr auto ACTIVE_TORQUE_PERCENT = 1;  
        writeTorquePercentage(id, ACTIVE_TORQUE_PERCENT);

        // only rewrite the params which is not success
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeVStart(id, data_list.at(0));
            if (res == COMM_SUCCESS)
                break;
        }
        if (res != COMM_SUCCESS)
            return res;

        tries = 10;
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeA1(id, data_list.at(1));
            if (res == COMM_SUCCESS)
                break;
        }
        if (res != COMM_SUCCESS)
            return res;

        tries = 10;
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeV1(id, data_list.at(2));
            if (res == COMM_SUCCESS)
                break;
        }
        if (res != COMM_SUCCESS)
            return res;

        tries = 10;
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeAMax(id, data_list.at(3));
            if (res == COMM_SUCCESS)
                break;
        }
        if (res != COMM_SUCCESS)
            return res;

        tries = 10;
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeVMax(id, data_list.at(4));
            if (res == COMM_SUCCESS)
                break;
        }
        if (res != COMM_SUCCESS)
            return res;

        tries = 10;
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeDMax(id, data_list.at(5));
            if (res == COMM_SUCCESS)
                break;
        }
        if (res != COMM_SUCCESS)
            return res;

        tries = 10;
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeD1(id, data_list.at(6));
            if (res == COMM_SUCCESS)
                break;
        }
        if (res != COMM_SUCCESS)
            return res;

        tries = 10;
        while (tries > 0) // try 10 times
        {
            tries--;
            ros::Duration(wait_duration).sleep();
            res = writeVStop(id, data_list.at(7));
            if (res == COMM_SUCCESS)
                break;
        }

        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::startHoming
     * @param id
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::startHoming(uint8_t id)
    {
        return write<typename reg_type::TYPE_COMMAND>(reg_type::ADDR_COMMAND, id, 0);
    }

    /**
     * @brief StepperDriver<reg_type>::writeHomingDirection
     * @param id
     * @param direction
     * @param stall_threshold
     * @return
     * TODO(CC) remove sleeps when fw will be updated
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeHomingSetup(uint8_t id, uint8_t direction, uint8_t stall_threshold)
    {
        int res = 0;
        double wait_duration = 0.05;

        // Random positive value to activate the torque, we need this to write on other registers
        constexpr auto ACTIVE_TORQUE_PERCENT = 1;  
        writeTorquePercentage(id, ACTIVE_TORQUE_PERCENT);
        ros::Duration(wait_duration).sleep();

        if (COMM_SUCCESS != write<typename reg_type::TYPE_HOMING_DIRECTION>(reg_type::ADDR_HOMING_DIRECTION, id, direction))
            res++;
        ros::Duration(wait_duration).sleep();

        // if (COMM_SUCCESS != write<typename reg_type::TYPE_HOMING_STALL_THRESHOLD>(reg_type::ADDR_HOMING_STALL_THRESHOLD, id, stall_threshold))
        //     res++;
        // ros::Duration(wait_duration).sleep();

        if (res > 0)
        {
            std::cout << "Failures during writeVelocityProfile : " << res << std::endl;
            return COMM_TX_FAIL;
        }

        return COMM_SUCCESS;
    }

    /**
     * @brief StepperDriver<reg_type>::readHomingStatus
     * @param id
     * @param status
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readHomingStatus(uint8_t id, uint8_t &status)
    {
        return read<typename reg_type::TYPE_HOMING_STATUS>(reg_type::ADDR_HOMING_STATUS, id, status);
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadHomingStatus
     * @param id_list
     * @param status_list
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadHomingStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &status_list)
    {
        return syncRead<typename reg_type::TYPE_HOMING_STATUS>(reg_type::ADDR_HOMING_STATUS, id_list, status_list);
    }

    /**
     * @brief StepperDriver<reg_type>::readFirmwareRunning
     * @param id
     * @param is_running
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::readFirmwareRunning(uint8_t id, bool &is_running)
    {
        typename reg_type::TYPE_FIRMWARE_RUNNING data{};
        int res = read<typename reg_type::TYPE_FIRMWARE_RUNNING>(reg_type::ADDR_FIRMWARE_RUNNING, id, data);
        is_running = data;
        return res;
    }

    /**
     * @brief StepperDriver<reg_type>::syncWriteHomingAbsPosition
     * @param id
     * @param abs_position
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncWriteHomingAbsPosition(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &abs_position)
    {
        return syncWrite<typename reg_type::TYPE_HOMING_ABS_POSITION>(reg_type::ADDR_HOMING_ABS_POSITION, id_list, abs_position);
    }

    /**
     * @brief StepperDriver<reg_type>::syncReadHomingAbsPosition
     * @param id
     * @param abs_position
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::syncReadHomingAbsPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &abs_position)
    {
        return syncRead<typename reg_type::TYPE_HOMING_ABS_POSITION>(reg_type::ADDR_HOMING_ABS_POSITION, id_list, abs_position);
    }

    // private

    /**
     * @brief StepperDriver<reg_type>::writeVStart
     * @param id
     * @param v_start
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeVStart(uint8_t id, uint32_t v_start)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_VSTART, id, v_start);
    }

    /**
     * @brief StepperDriver<reg_type>::writeA1
     * @param id
     * @param a_1
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeA1(uint8_t id, uint32_t a_1)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_A1, id, a_1);
    }

    /**
     * @brief StepperDriver<reg_type>::writeV1
     * @param id
     * @param v_1
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeV1(uint8_t id, uint32_t v_1)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_V1, id, v_1);
    }

    /**
     * @brief StepperDriver<reg_type>::writeAMax
     * @param id
     * @param a_max
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeAMax(uint8_t id, uint32_t a_max)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_AMAX, id, a_max);
    }

    /**
     * @brief StepperDriver<reg_type>::writeVMax
     * @param id
     * @param v_max
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeVMax(uint8_t id, uint32_t v_max)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_VMAX, id, v_max);
    }

    /**
     * @brief StepperDriver<reg_type>::writeDMax
     * @param id
     * @param d_max
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeDMax(uint8_t id, uint32_t d_max)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_DMAX, id, d_max);
    }

    /**
     * @brief StepperDriver<reg_type>::writeD1
     * @param id
     * @param d_1
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeD1(uint8_t id, uint32_t d_1)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_D1, id, d_1);
    }

    /**
     * @brief StepperDriver<reg_type>::writeVStop
     * @param id
     * @param v_stop
     * @return
     */
    template <typename reg_type>
    int StepperDriver<reg_type>::writeVStop(uint8_t id, uint32_t v_stop)
    {
        return write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_VSTOP, id, v_stop);
    }

    template <typename reg_type>
    float StepperDriver<reg_type>::velocityUnit() const
    {
        return reg_type::VELOCITY_UNIT;
    }

} // ttl_driver

#endif // STEPPER_DRIVER_HPP
