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
#include "xm430_reg.hpp"
#include "xl330_reg.hpp"
#include "xl320_reg.hpp"
#include "xh430_reg.hpp"

namespace ttl_driver
{

    /**
     * @brief The DxlDriver class
     */
    template <typename reg_type>
    class DxlDriver : public AbstractDxlDriver
    {
    public:
        DxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                  std::shared_ptr<dynamixel::PacketHandler> packetHandler);

        std::string str() const override;
        std::string interpretErrorState(uint32_t hw_state) const override;

    public:
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

    protected:
        // AbstractTtlDriver interface
        std::string interpretFirmwareVersion(uint32_t fw_version) const override;

    public:
        // AbstractMotorDriver interface : we cannot define them globally in AbstractMotorDriver
        // as it is needed here for polymorphism (AbstractMotorDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver

        // eeprom write
        int changeId(uint8_t id, uint8_t new_id) override;

        // eeprom read
        int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        // ram write
        int writeVelocityProfile(uint8_t id, const std::vector<uint32_t> &data_list) override;

        int writeTorquePercentage(uint8_t id, uint8_t torque_percentage) override;

        int writePositionGoal(uint8_t id, uint32_t position) override;
        int writeVelocityGoal(uint8_t id, uint32_t velocity) override;

        int syncWriteTorquePercentage(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &torque_percentage_list) override;
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        // ram read
        int readVelocityProfile(uint8_t id, std::vector<uint32_t> &data_list) override;

        int readPosition(uint8_t id, uint32_t &present_position) override;
        int readVelocity(uint8_t id, uint32_t &present_velocity) override;

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
        int syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2>> &data_array_list) override;

    public:
        // AbstractDxlDriver interface

        // eeprom read

        // eeprom write
        int writeStartupConfiguration(uint8_t id, uint8_t config) override;
        int writeTemperatureLimit(uint8_t id, uint8_t temperature_limit) override;
        int writeShutdownConfiguration(uint8_t id, uint8_t configuration) override;

        // ram read
        int readLoad(uint8_t id, uint16_t &present_load) override;
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint16_t> &load_list) override;

        int readPID(uint8_t id, std::vector<uint16_t> &data_list) override;
        int readControlMode(uint8_t id, uint8_t &control_mode) override;

        int readMoving(uint8_t id, uint8_t &status) override;

        // ram write
        int writePID(uint8_t id, const std::vector<uint16_t> &data) override;
        int writeControlMode(uint8_t id, uint8_t control_mode) override;

        int writeLed(uint8_t id, uint8_t led_value) override;
        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &led_list) override;

        int writeTorqueGoal(uint8_t id, uint16_t torque) override;
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint16_t> &torque_list) override;
    };

    // definition of methods

    /**
     * @brief DxlDriver<reg_type>::DxlDriver
     */
    template <typename reg_type>
    DxlDriver<reg_type>::DxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                   std::shared_ptr<dynamixel::PacketHandler> packetHandler) : AbstractDxlDriver(std::move(portHandler),
                                                                                                                std::move(packetHandler))
    {
    }

    //*****************************
    // AbstractMotorDriver interface
    //*****************************

    /**
     * @brief DxlDriver<reg_type>::str
     * @return
     */
    template <typename reg_type>
    std::string DxlDriver<reg_type>::str() const
    {
        return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + AbstractDxlDriver::str();
    }

    /**
     * @brief DxlDriver<reg_type>::interpretErrorState
     * @return
     */
    template <typename reg_type>
    std::string DxlDriver<reg_type>::interpretErrorState(uint32_t /*hw_state*/) const
    {
        return "";
    }

    /**
     * @brief DxlDriver<reg_type>::interpretFirmwareVersion
     * @param fw_version
     * @return
     */
    template <typename reg_type>
    std::string DxlDriver<reg_type>::interpretFirmwareVersion(uint32_t fw_version) const
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
    template <typename reg_type>
    int DxlDriver<reg_type>::changeId(uint8_t id, uint8_t new_id)
    {
        return write<typename reg_type::TYPE_ID>(reg_type::ADDR_ID, id, new_id);
    }

    /**
     * @brief DxlDriver<reg_type>::checkModelNumber
     * @param id
     * @return
     */
    template <typename reg_type>
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
    template <typename reg_type>
    int DxlDriver<reg_type>::readFirmwareVersion(uint8_t id, std::string &version)
    {
        int res = COMM_RX_FAIL;
        uint8_t data{};
        res = read<typename reg_type::TYPE_FIRMWARE_VERSION>(reg_type::ADDR_FIRMWARE_VERSION, id, data);
        version = interpretFirmwareVersion(data);
        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::writeStartupConfiguration
     * @param id
     * @param config
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeStartupConfiguration(uint8_t id, uint8_t config)
    {
        std::string version;
        int res = readFirmwareVersion(id, version);

        // only for version above certain version (see registers)
        if (COMM_SUCCESS == res && std::stoi(version) >= reg_type::VERSION_STARTUP_CONFIGURATION)
        {
            res = write<typename reg_type::TYPE_STARTUP_CONFIGURATION>(reg_type::ADDR_STARTUP_CONFIGURATION, id, config);
        }
        else
        {
            std::cout << "Startup configuration available only for version > " << static_cast<int>(reg_type::VERSION_STARTUP_CONFIGURATION) << std::endl;
            res = COMM_SUCCESS;
        }

        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::writeTemperatureLimit
     * @param id
     * @param temperature_limit
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeTemperatureLimit(uint8_t id, uint8_t temperature_limit)
    {
        std::cout << "temperature limit not available for this motor type" << std::endl;
        return COMM_SUCCESS;
    }

    /**
     * @brief DxlDriver<reg_type>::writeShutdownConfiguration
     * @param id
     * @param configuration
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeShutdownConfiguration(uint8_t id, uint8_t configuration)
    {
        std::cout << "shutdown configuration not available for this motor type" << std::endl;
        return COMM_SUCCESS;
    }

    /**
     * @brief DxlDriver<reg_type>::readMinPosition
     * @param id
     * @param pos
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readMinPosition(uint8_t id, uint32_t &pos)
    {
        return read<typename reg_type::TYPE_MIN_POSITION_LIMIT>(reg_type::ADDR_MIN_POSITION_LIMIT, id, pos);
    }

    /**
     * @brief DxlDriver<reg_type>::readMaxPosition
     * @param id
     * @param pos
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readMaxPosition(uint8_t id, uint32_t &pos)
    {
        return read<typename reg_type::TYPE_MAX_POSITION_LIMIT>(reg_type::ADDR_MAX_POSITION_LIMIT, id, pos);
    }

    // ram write

    /**
     * @brief DxlDriver<reg_type>::writeVelocityProfile
     * Write velocity profile for dxl
     * @param id
     * @param data_list [ velocity, acceleration]
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeVelocityProfile(uint8_t id, const std::vector<uint32_t> &data_list)
    {
        // in mode control Position Control Mode, velocity profile in datasheet is used to write velocity (except xl320)
        int res = COMM_RX_FAIL;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_PROFILE_VELOCITY, id, data_list.at(0));
            if (COMM_SUCCESS == res)
                break;
        }

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_PROFILE_ACCELERATION, id, data_list.at(1));
            if (COMM_SUCCESS == res)
                break;
        }

        if (COMM_SUCCESS != res)
            return res;

        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::writeTorquePercentage
     * @param id
     * @param torque_percentage
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeTorquePercentage(uint8_t id, uint8_t torque_percentage)
    {
        auto torque_enable = torque_percentage > 0 ? 1 : 0;
        return write<typename reg_type::TYPE_TORQUE_ENABLE>(reg_type::ADDR_TORQUE_ENABLE, id, torque_enable);
    }

    /**
     * @brief DxlDriver<reg_type>::writePositionGoal
     * @param id
     * @param position
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writePositionGoal(uint8_t id, uint32_t position)
    {
        return write<typename reg_type::TYPE_GOAL_POSITION>(reg_type::ADDR_GOAL_POSITION, id,
                                                            static_cast<typename reg_type::TYPE_GOAL_POSITION>(position));
    }

    /**
     * @brief DxlDriver<reg_type>::writeVelocityGoal
     * @param id
     * @param velocity
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeVelocityGoal(uint8_t id, uint32_t velocity)
    {
        return write<typename reg_type::TYPE_GOAL_VELOCITY>(reg_type::ADDR_GOAL_VELOCITY, id,
                                                            static_cast<typename reg_type::TYPE_GOAL_VELOCITY>(velocity));
    }

    /**
     * @brief DxlDriver<reg_type>::syncWriteTorquePercentage
     * @param id_list
     * @param torque_percentage_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncWriteTorquePercentage(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &torque_percentage_list)
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
     * @brief DxlDriver<reg_type>::syncWritePositionGoal
     * @param id_list
     * @param position_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
    {
        return syncWrite<typename reg_type::TYPE_GOAL_POSITION>(reg_type::ADDR_GOAL_POSITION, id_list, position_list);
    }

    /**
     * @brief DxlDriver<reg_type>::syncWriteVelocityGoal
     * @param id_list
     * @param velocity_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
    {
        return syncWrite<typename reg_type::TYPE_GOAL_VELOCITY>(reg_type::ADDR_GOAL_VELOCITY, id_list, velocity_list);
    }

    // ram read
    template <typename reg_type>
    int DxlDriver<reg_type>::readVelocityProfile(uint8_t id, std::vector<uint32_t> &data_list)
    {
        data_list.clear();
        typename reg_type::TYPE_PROFILE velocity_profile{};
        typename reg_type::TYPE_PROFILE acceleration_profile{};

        int res = read<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_PROFILE_VELOCITY, id, velocity_profile);
        if (COMM_SUCCESS == res)
            res = read<typename reg_type::TYPE_PROFILE>(reg_type::ADDR_PROFILE_ACCELERATION, id, acceleration_profile);

        data_list.emplace_back(velocity_profile);
        data_list.emplace_back(acceleration_profile);

        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::readPosition
     * @param id
     * @param present_position
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readPosition(uint8_t id, uint32_t &present_position)
    {
        return read<typename reg_type::TYPE_PRESENT_POSITION>(reg_type::ADDR_PRESENT_POSITION, id, present_position);
    }

    /**
     * @brief DxlDriver<reg_type>::readTemperature
     * @param id
     * @param temperature
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readTemperature(uint8_t id, uint8_t &temperature)
    {
        return read<typename reg_type::TYPE_PRESENT_TEMPERATURE>(reg_type::ADDR_PRESENT_TEMPERATURE, id, temperature);
    }

    /**
     * @brief DxlDriver<reg_type>::readVoltage
     * @param id
     * @param voltage
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readVoltage(uint8_t id, double &voltage)
    {
        typename reg_type::TYPE_PRESENT_VOLTAGE voltage_mV{};
        int res = read<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id, voltage_mV);
        voltage = static_cast<double>(voltage_mV) / reg_type::VOLTAGE_CONVERSION;
        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::readHwErrorStatus
     * @param id
     * @param hardware_error_status
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readHwErrorStatus(uint8_t id, uint8_t &hardware_error_status)
    {
        return read<typename reg_type::TYPE_HW_ERROR_STATUS>(reg_type::ADDR_HW_ERROR_STATUS, id, hardware_error_status);
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadPosition
     * @param id_list
     * @param position_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
    {
        return syncRead<typename reg_type::TYPE_PRESENT_POSITION>(reg_type::ADDR_PRESENT_POSITION, id_list, position_list);
    }

    /**
     * @brief DxlDriver<reg_type>::writePID
     * @param id
     * @param data
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writePID(uint8_t id, const std::vector<uint16_t> &data)
    {
        int res = COMM_TX_FAIL;

        // only rewrite params which is not success
        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PID_GAIN>(reg_type::ADDR_POSITION_P_GAIN, id, data.at(0));
            if (COMM_SUCCESS == res)
                break;
        }

        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PID_GAIN>(reg_type::ADDR_POSITION_I_GAIN, id, data.at(1));
            if (COMM_SUCCESS == res)
                break;
        }
        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PID_GAIN>(reg_type::ADDR_POSITION_D_GAIN, id, data.at(2));
            if (COMM_SUCCESS == res)
                break;
        }
        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PID_GAIN>(reg_type::ADDR_VELOCITY_P_GAIN, id, data.at(3));
            if (COMM_SUCCESS == res)
                break;
        }
        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PID_GAIN>(reg_type::ADDR_VELOCITY_I_GAIN, id, data.at(4));
            if (COMM_SUCCESS == res)
                break;
        }
        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PID_GAIN>(reg_type::ADDR_FF1_GAIN, id, data.at(5));
            if (COMM_SUCCESS == res)
                break;
        }
        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename reg_type::TYPE_PID_GAIN>(reg_type::ADDR_FF2_GAIN, id, data.at(6));
            if (res == COMM_SUCCESS)
                break;
        }

        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadFirmwareVersion
     * @param id_list
     * @param firmware_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
    {
        int res = COMM_RX_FAIL;
        std::vector<uint8_t> data_list;
        res = syncRead<typename reg_type::TYPE_FIRMWARE_VERSION>(reg_type::ADDR_FIRMWARE_VERSION, id_list, data_list);
        for (auto const &data : data_list)
            firmware_list.emplace_back(interpretFirmwareVersion(data));
        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadTemperature
     * @param id_list
     * @param temperature_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &temperature_list)
    {
        return syncRead<typename reg_type::TYPE_PRESENT_TEMPERATURE>(reg_type::ADDR_PRESENT_TEMPERATURE, id_list, temperature_list);
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadVoltage
     * @param id_list
     * @param voltage_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
    {
        voltage_list.clear();
        std::vector<typename reg_type::TYPE_PRESENT_VOLTAGE> v_read;
        int res = syncRead<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, v_read);
        for (auto const &v : v_read)
            voltage_list.emplace_back(static_cast<double>(v) / reg_type::VOLTAGE_CONVERSION);
        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadRawVoltage
     * @param id_list
     * @param voltage_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
    {
        voltage_list.clear();
        std::vector<typename reg_type::TYPE_PRESENT_VOLTAGE> v_read;
        int res = syncRead<typename reg_type::TYPE_PRESENT_VOLTAGE>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, v_read);
        for (auto const &v : v_read)
            voltage_list.emplace_back(static_cast<double>(v));
        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadHwStatus
     * @param id_list
     * @param data_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadHwStatus(const std::vector<uint8_t> &id_list,
                                              std::vector<std::pair<double, uint8_t>> &data_list)
    {
        data_list.clear();

        std::vector<std::array<uint8_t, 3>> raw_data;
        int res = syncReadConsecutiveBytes<uint8_t, 3>(reg_type::ADDR_PRESENT_VOLTAGE, id_list, raw_data);

        for (auto const &data : raw_data)
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
     * @brief DxlDriver<reg_type>::syncReadHwErrorStatus
     * @param id_list
     * @param hw_error_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list)
    {
        return syncRead<typename reg_type::TYPE_HW_ERROR_STATUS>(reg_type::ADDR_HW_ERROR_STATUS, id_list, hw_error_list);
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
    template <typename reg_type>
    int DxlDriver<reg_type>::writeLed(uint8_t id, uint8_t led_value)
    {
        return write<typename reg_type::TYPE_LED>(reg_type::ADDR_LED, id, led_value);
    }

    /**
     * @brief DxlDriver<reg_type>::syncWriteLed
     * @param id_list
     * @param led_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &led_list)
    {
        return syncWrite<typename reg_type::TYPE_LED>(reg_type::ADDR_LED, id_list, led_list);
    }

    /**
     * @brief DxlDriver<reg_type>::writeTorqueGoal
     * @param id
     * @param torque
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeTorqueGoal(uint8_t id, uint16_t torque)
    {
        return write<typename reg_type::TYPE_GOAL_TORQUE>(reg_type::ADDR_GOAL_TORQUE, id, torque);
    }

    /**
     * @brief DxlDriver<reg_type>::syncWriteTorqueGoal
     * @param id_list
     * @param torque_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint16_t> &torque_list)
    {
        return syncWrite<typename reg_type::TYPE_GOAL_TORQUE>(reg_type::ADDR_GOAL_TORQUE, id_list, torque_list);
    }

    // read

    /**
     * @brief DxlDriver<reg_type>::readPID
     * @param id
     * @param data_list : [pos_p, pos_i, pos_d, vel_p, vel_i, ff1, ff2]
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readPID(uint8_t id, std::vector<uint16_t> &data_list)
    {
        int res = 0;
        data_list.clear();
        std::vector<std::array<typename reg_type::TYPE_PID_GAIN, 8>> raw_data;

        // only rewrite params which is not success
        for (int tries = 10; tries > 0; --tries)
        {
            res = syncReadConsecutiveBytes<typename reg_type::TYPE_PID_GAIN, 8>(reg_type::ADDR_VELOCITY_I_GAIN, {id}, raw_data);
            if (COMM_SUCCESS == res)
                break;
        }

        // we need to reorder the data in its correct place in data_list
        if (COMM_SUCCESS == res && raw_data.size() == 1)
        {
            data_list.emplace_back(raw_data.at(0).at(4)); // pos p is 5th
            data_list.emplace_back(raw_data.at(0).at(3)); // pos i is 4th
            data_list.emplace_back(raw_data.at(0).at(2)); // pos d is 3rd
            data_list.emplace_back(raw_data.at(0).at(1)); // vel p is 2nd
            data_list.emplace_back(raw_data.at(0).at(0)); // vel i is 1st
            data_list.emplace_back(raw_data.at(0).at(7)); // ff1 is 8th
            data_list.emplace_back(raw_data.at(0).at(6)); // ff2 is 7th
                                                          // nothing at 6th
        }
        else
        {
            std::cout << "Failures during read PID gains: " << res << std::endl;
            return COMM_TX_FAIL;
        }

        return COMM_SUCCESS;
    }

    /**
     * @brief DxlDriver<reg_type>::readMoving
     * @param id
     * @param status
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readMoving(uint8_t id, uint8_t &status)
    {
        return read<typename reg_type::TYPE_MOVING>(reg_type::ADDR_MOVING, id, status);
    }

    /**
     * @brief DxlDriver<reg_type>::writeControlMode
     * @param id
     * @param control_mode
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::writeControlMode(uint8_t id, uint8_t control_mode)
    {
        int res = COMM_TX_ERROR;
        uint8_t torque{0};
        res = read<typename reg_type::TYPE_TORQUE_ENABLE>(reg_type::ADDR_TORQUE_ENABLE, id, torque);
        if (res == COMM_SUCCESS)
        {
            if (torque == 0)
            {
                return write<typename reg_type::TYPE_OPERATING_MODE>(reg_type::ADDR_OPERATING_MODE, id, control_mode);
            }
        }
        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::readControlMode
     * @param id
     * @param control_mode
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readControlMode(uint8_t id, uint8_t &control_mode)
    {
        int res = 0;
        typename reg_type::TYPE_OPERATING_MODE raw{};
        res = read<typename reg_type::TYPE_OPERATING_MODE>(reg_type::ADDR_OPERATING_MODE, id, raw);
        control_mode = static_cast<uint8_t>(raw);
        return res;
    }
    // other

    /**
     * @brief DxlDriver<reg_type>::readLoad
     * @param id
     * @param present_load
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readLoad(uint8_t id, uint16_t &present_load)
    {
        return read<typename reg_type::TYPE_PRESENT_LOAD>(reg_type::ADDR_PRESENT_LOAD, id, present_load);
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadLoad
     * @param id_list
     * @param load_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint16_t> &load_list)
    {
        return syncRead<typename reg_type::TYPE_PRESENT_LOAD>(reg_type::ADDR_PRESENT_LOAD, id_list, load_list);
    }

    /**
     * @brief DxlDriver<reg_type>::readVelocity
     * @param id
     * @param present_velocity
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::readVelocity(uint8_t id, uint32_t &present_velocity)
    {
        return read<typename reg_type::TYPE_PRESENT_VELOCITY>(reg_type::ADDR_PRESENT_VELOCITY, id, present_velocity);
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadVelocity
     * @param id_list
     * @param velocity_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
    {
        return syncRead<typename reg_type::TYPE_PRESENT_VELOCITY>(reg_type::ADDR_PRESENT_VELOCITY, id_list, velocity_list);
    }

    /**
     * @brief DxlDriver::syncReadJointStatus
     * @param id_list
     * @param data_array_list
     * @return
     */
    template <typename reg_type>
    int DxlDriver<reg_type>::syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2>> &data_array_list)
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
                         " ERROR reading dxl torque in syncReadJointStatus (error "
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
                data_array_list.emplace_back(std::array<uint32_t, 2>{0, p});
        }

        return res;
    }

    /*
     *  -----------------   specializations   --------------------
     */

    // XL320

    template <>
    inline int DxlDriver<XL320Reg>::writeStartupConfiguration(uint8_t /*id*/, uint8_t /*config*/)
    {
        std::cout << "startup configuration for XL320 not available" << std::endl;
        return COMM_SUCCESS;
    }

    template <>
    inline int DxlDriver<XL320Reg>::readMinPosition(uint8_t /*id*/, uint32_t &pos)
    {
        pos = 0;
        std::cout << "min position hardcoded for motor XL320" << std::endl;
        return COMM_SUCCESS;
    }

    template <>
    inline int DxlDriver<XL320Reg>::readMaxPosition(uint8_t /*id*/, uint32_t &pos)
    {
        pos = 1023;
        std::cout << "max position hardcoded for motor XL320" << std::endl;
        return COMM_SUCCESS;
    }

    template <>
    inline std::string DxlDriver<XL320Reg>::interpretErrorState(uint32_t hw_state) const
    {
        std::string hardware_message;

        if (hw_state & 1 << 0) // 0b00000001
        {
            hardware_message += "Overload";
        }
        if (hw_state & 1 << 1) // 0b00000010
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1 << 2) // 0b00000100
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Input voltage out of range";
        }
        if (hw_state & 1 << 7) // 0b10000000 => added by us : disconnected error
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Disconnection";
        }

        return hardware_message;
    }

    template <>
    inline int DxlDriver<XL320Reg>::readVelocityProfile(uint8_t /*id*/, std::vector<uint32_t> & /*data_list*/)
    {
        std::cout << "readVelocityProfile not available for XL320" << std::endl;
        return COMM_SUCCESS;
    }

    template <>
    inline int DxlDriver<XL320Reg>::writeVelocityProfile(uint8_t /*id*/, const std::vector<uint32_t> & /*data_list*/)
    {
        std::cout << "writeVelocityProfile not available for XL320" << std::endl;
        return COMM_SUCCESS;
    }

    template <>
    inline int DxlDriver<XL320Reg>::readPosition(uint8_t id, uint32_t &present_position)
    {
        typename XL320Reg::TYPE_PRESENT_POSITION raw_data{};
        int res = read<typename XL320Reg::TYPE_PRESENT_POSITION>(XL320Reg::ADDR_PRESENT_POSITION, id, raw_data);
        present_position = raw_data;
        return res;
    }

    template <>
    inline int DxlDriver<XL320Reg>::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
    {
        position_list.clear();
        std::vector<typename XL320Reg::TYPE_PRESENT_POSITION> raw_data_list{};

        int res = syncRead<typename XL320Reg::TYPE_PRESENT_POSITION>(XL320Reg::ADDR_PRESENT_POSITION, id_list, raw_data_list);
        for (auto p : raw_data_list)
            position_list.emplace_back(p);

        return res;
    }

    template <>
    inline int DxlDriver<XL320Reg>::readVelocity(uint8_t id, uint32_t &present_velocity)
    {
        typename XL320Reg::TYPE_PRESENT_VELOCITY raw_data{};
        int res = read<typename XL320Reg::TYPE_PRESENT_VELOCITY>(XL320Reg::ADDR_PRESENT_VELOCITY, id, raw_data);
        present_velocity = raw_data;

        return res;
    }

    template <>
    inline int DxlDriver<XL320Reg>::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
    {
        std::vector<typename XL320Reg::TYPE_PRESENT_VELOCITY> raw_data_list;
        int res = syncRead<typename XL320Reg::TYPE_PRESENT_VELOCITY>(XL320Reg::ADDR_PRESENT_VELOCITY, id_list, raw_data_list);
        for (auto v : raw_data_list)
            velocity_list.emplace_back(v);
        return res;
    }

    /**
     * @brief DxlDriver::syncReadJointStatus
     * @param id_list
     * @param data_array_list
     * @return
     */
    template <>
    inline int DxlDriver<XL320Reg>::syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2>> &data_array_list)
    {
        int res = COMM_TX_FAIL;

        if (id_list.empty())
            return res;

        data_array_list.clear();

        std::vector<uint32_t> position_list;
        res = syncReadPosition(id_list, position_list);
        if (res == COMM_SUCCESS)
        {
            for (auto p : position_list)
                data_array_list.emplace_back(std::array<uint32_t, 2>{0, p});
        }

        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::syncReadHwStatus
     * @param id_list
     * @param data_list
     * @return
     */
    template <>
    inline int DxlDriver<XL320Reg>::syncReadHwStatus(const std::vector<uint8_t> &id_list,
                                                     std::vector<std::pair<double, uint8_t>> &data_list)
    {
        data_list.clear();

        std::vector<std::array<uint8_t, 2>> raw_data;
        int res = syncReadConsecutiveBytes<uint8_t, 2>(XL320Reg::ADDR_PRESENT_VOLTAGE, id_list, raw_data);

        for (auto const &data : raw_data)
        {
            // Voltage is first reg, uint16
            auto voltage = static_cast<double>(data.at(0));

            // Temperature is second reg, uint8
            uint8_t temperature = data.at(1);

            data_list.emplace_back(std::make_pair(voltage, temperature));
        }

        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::readPID : only position PID for XL320
     * @param id
     * @param data_list : [pos_p, pos_i, pos_d, vel_p, vel_i, ff1, ff2]
     * @return
     */
    template <>
    inline int DxlDriver<XL320Reg>::readPID(uint8_t id, std::vector<uint16_t> &data_list)
    {
        int res = COMM_RX_FAIL;
        data_list.clear();
        std::vector<std::array<typename XL320Reg::TYPE_PID_GAIN, 3>> raw_data;

        // only rewrite params which is not success
        for (int tries = 10; tries > 0; --tries)
        {
            res = syncReadConsecutiveBytes<typename XL320Reg::TYPE_PID_GAIN, 3>(XL320Reg::ADDR_POSITION_D_GAIN, {id}, raw_data);
            if (COMM_SUCCESS == res)
                break;
        }

        // we need to reorder the data in its correct place in data_list
        if (COMM_SUCCESS == res && raw_data.size() == 1)
        {
            data_list.emplace_back(static_cast<uint16_t>(raw_data.at(0).at(2))); // pos p is 3rd
            data_list.emplace_back(static_cast<uint16_t>(raw_data.at(0).at(1))); // pos i is 2nd
            data_list.emplace_back(static_cast<uint16_t>(raw_data.at(0).at(0))); // pos d is 1st
            data_list.emplace_back(0);                                           // no vel p
            data_list.emplace_back(0);                                           // no vel i
            data_list.emplace_back(0);                                           // no ff1
            data_list.emplace_back(0);                                           // no ff2
        }
        else
        {
            std::cout << "Failures during read PID gains: " << res << std::endl;
            return COMM_TX_FAIL;
        }

        return COMM_SUCCESS;
    }

    /**
     * @brief DxlDriver<reg_type>::writePID : only position PID for XL320
     * @param id
     * @param data_list
     * @return
     *  TODO(cc) bulk write all in one shot
     */
    template <>
    inline int DxlDriver<XL320Reg>::writePID(uint8_t id, const std::vector<uint16_t> &data_list)
    {
        int res = COMM_TX_FAIL;

        // only rewrite params which is not success
        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename XL320Reg::TYPE_PID_GAIN>(XL320Reg::ADDR_POSITION_P_GAIN, id,
                                                          static_cast<typename XL320Reg::TYPE_PID_GAIN>(data_list.at(0)));
            if (COMM_SUCCESS == res)
                break;
        }
        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename XL320Reg::TYPE_PID_GAIN>(XL320Reg::ADDR_POSITION_I_GAIN, id,
                                                          static_cast<typename XL320Reg::TYPE_PID_GAIN>(data_list.at(1)));
            if (COMM_SUCCESS == res)
                break;
        }
        if (COMM_SUCCESS != res)
            return res;

        for (int tries = 10; tries > 0; --tries)
        {
            res = write<typename XL320Reg::TYPE_PID_GAIN>(XL320Reg::ADDR_POSITION_D_GAIN, id,
                                                          static_cast<typename XL320Reg::TYPE_PID_GAIN>(data_list.at(2)));
            if (COMM_SUCCESS == res)
                break;
        }

        return res;
    }

    /**
     * @brief DxlDriver<reg_type>::syncWritePositionGoal
     * @param id_list
     * @param position_list
     * @return
     */
    template <>
    inline int DxlDriver<XL320Reg>::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
    {
        std::vector<typename XL320Reg::TYPE_GOAL_POSITION> casted_list;
        casted_list.reserve(position_list.size());
        for (auto const &p : position_list)
        {
            casted_list.emplace_back(static_cast<typename XL320Reg::TYPE_GOAL_POSITION>(p));
        }
        return syncWrite<typename XL320Reg::TYPE_GOAL_POSITION>(XL320Reg::ADDR_GOAL_POSITION, id_list, casted_list);
    }

    /**
     * @brief DxlDriver<reg_type>::syncWriteVelocityGoal
     * @param id_list
     * @param velocity_list
     * @return
     */
    template <>
    inline int DxlDriver<XL320Reg>::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
    {
        std::vector<typename XL320Reg::TYPE_GOAL_VELOCITY> casted_list;
        casted_list.reserve(velocity_list.size());
        for (auto const &v : velocity_list)
        {
            casted_list.emplace_back(static_cast<typename XL320Reg::TYPE_GOAL_VELOCITY>(v));
        }
        return syncWrite<typename XL320Reg::TYPE_GOAL_VELOCITY>(XL320Reg::ADDR_GOAL_VELOCITY, id_list, casted_list);
    }

    template <>
    inline int DxlDriver<XL320Reg>::readControlMode(uint8_t /*id*/, uint8_t & /*data*/)
    {
        std::cout << "readControlMode not available for motor XL320" << std::endl;
        return COMM_SUCCESS;
    }

    // XL430

    template <>
    inline std::string DxlDriver<XL430Reg>::interpretErrorState(uint32_t hw_state) const
    {
        std::string hardware_message;

        if (hw_state & 1 << 0) // 0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1 << 2) // 0b00000100
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1 << 3) // 0b00001000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1 << 4) // 0b00010000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1 << 5) // 0b00100000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hw_state & 1 << 7) // 0b10000000 => added by us : disconnected error
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Disconnection";
        }
        if (!hardware_message.empty())
            hardware_message += " Error";

        return hardware_message;
    }

    template <>
    inline int DxlDriver<XL430Reg>::writeTorqueGoal(uint8_t /*id*/, uint16_t /*torque*/)
    {
        std::cout << "writeTorqueGoal not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    template <>
    inline int DxlDriver<XL430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> & /*id_list*/, const std::vector<uint16_t> & /*torque_list*/)
    {
        std::cout << "syncWriteTorqueGoal not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    // XC430

    template <>
    inline std::string DxlDriver<XC430Reg>::interpretErrorState(uint32_t hw_state) const
    {
        std::string hardware_message;

        if (hw_state & 1 << 0) // 0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1 << 2) // 0b00000100
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1 << 3) // 0b00001000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1 << 4) // 0b00010000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1 << 5) // 0b00100000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hw_state & 1 << 7) // 0b10000000 => added by us : disconnected error
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Disconnection";
        }
        if (!hardware_message.empty())
            hardware_message += " Error";

        return hardware_message;
    }

    template <>
    inline int DxlDriver<XC430Reg>::writeTorqueGoal(uint8_t /*id*/, uint16_t /*torque*/)
    {
        std::cout << "writeTorqueGoal not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    template <>
    inline int DxlDriver<XC430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> & /*id_list*/, const std::vector<uint16_t> & /*torque_list*/)
    {
        std::cout << "syncWriteTorqueGoal not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    // XM430

    template <>
    inline std::string DxlDriver<XM430Reg>::interpretErrorState(uint32_t hw_state) const
    {
        std::string hardware_message;

        if (hw_state & 1 << 0) // 0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1 << 2) // 0b00000100
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1 << 3) // 0b00001000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1 << 4) // 0b00010000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1 << 5) // 0b00100000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hw_state & 1 << 7) // 0b10000000 => added by us : disconnected error
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Disconnection";
        }
        if (!hardware_message.empty())
            hardware_message += " Error";

        return hardware_message;
    }

    // works with current instead of load

    template <>
    inline int DxlDriver<XM430Reg>::writeTorqueGoal(uint8_t id, uint16_t torque)
    {
        return write<typename XM430Reg::TYPE_GOAL_CURRENT>(XM430Reg::ADDR_GOAL_CURRENT, id, torque);
    }

    template <>
    inline int DxlDriver<XM430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint16_t> &torque_list)
    {
        return syncWrite<typename XM430Reg::TYPE_GOAL_CURRENT>(XM430Reg::ADDR_GOAL_CURRENT, id_list, torque_list);
    }

    template <>
    inline int DxlDriver<XM430Reg>::readLoad(uint8_t id, uint16_t &present_load)
    {
        return read<typename XM430Reg::TYPE_PRESENT_CURRENT>(XM430Reg::ADDR_PRESENT_CURRENT, id, present_load);
    }

    template <>
    inline int DxlDriver<XM430Reg>::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint16_t> &load_list)
    {
        return syncRead<typename XM430Reg::TYPE_PRESENT_CURRENT>(XM430Reg::ADDR_PRESENT_CURRENT, id_list, load_list);
    }

    // XL330

    template <>
    inline std::string DxlDriver<XL330Reg>::interpretErrorState(uint32_t hw_state) const
    {
        std::string hardware_message;

        if (hw_state & 1 << 0) // 0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1 << 2) // 0b00000100
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1 << 3) // 0b00001000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1 << 4) // 0b00010000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1 << 5) // 0b00100000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hw_state & 1 << 7) // 0b10000000 => added by us : disconnected error
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Disconnection";
        }
        if (!hardware_message.empty())
            hardware_message += " Error";

        return hardware_message;
    }

    // works with current instead of load

    template <>
    inline int DxlDriver<XL330Reg>::writeTorqueGoal(uint8_t id, uint16_t torque)
    {
        return write<typename XL330Reg::TYPE_GOAL_CURRENT>(XL330Reg::ADDR_GOAL_CURRENT, id, torque);
    }

    template <>
    inline int DxlDriver<XL330Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint16_t> &torque_list)
    {
        return syncWrite<typename XL330Reg::TYPE_GOAL_CURRENT>(XL330Reg::ADDR_GOAL_CURRENT, id_list, torque_list);
    }

    template <>
    inline int DxlDriver<XL330Reg>::readLoad(uint8_t id, uint16_t &present_load)
    {
        return read<typename XL330Reg::TYPE_PRESENT_CURRENT>(XL330Reg::ADDR_PRESENT_CURRENT, id, present_load);
    }

    template <>
    inline int DxlDriver<XL330Reg>::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint16_t> &load_list)
    {
        return syncRead<typename XL330Reg::TYPE_PRESENT_CURRENT>(XL330Reg::ADDR_PRESENT_CURRENT, id_list, load_list);
    }

    template <>
    inline int DxlDriver<XL330Reg>::writeTemperatureLimit(uint8_t id, uint8_t temperature_limit)
    {
        return write<typename XL330Reg::TYPE_TEMPERATURE_LIMIT>(XL330Reg::ADDR_TEMPERATURE_LIMIT, id, temperature_limit);
    }

    template <>
    inline int DxlDriver<XL330Reg>::writeShutdownConfiguration(uint8_t id, uint8_t configuration)
    {
        return write<typename XL330Reg::TYPE_ALARM_SHUTDOWN>(XL330Reg::ADDR_ALARM_SHUTDOWN, id, configuration);
    }

    // XH430

    template <>
    inline std::string DxlDriver<XH430Reg>::interpretErrorState(uint32_t hw_state) const
    {
        std::string hardware_message;

        if (hw_state & 1 << 0) // 0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1 << 2) // 0b00000100
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1 << 3) // 0b00001000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1 << 4) // 0b00010000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1 << 5) // 0b00100000
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hw_state & 1 << 7) // 0b10000000 => added by us : disconnected error
        {
            if (!hardware_message.empty())
                hardware_message += ", ";
            hardware_message += "Disconnection";
        }
        if (!hardware_message.empty())
            hardware_message += " Error";

        return hardware_message;
    }

    template <>
    inline int DxlDriver<XH430Reg>::writeTorqueGoal(uint8_t id, uint16_t torque)
    {
        return write<typename XL330Reg::TYPE_GOAL_CURRENT>(XL330Reg::ADDR_GOAL_CURRENT, id, torque);
    }

    template <>
    inline int DxlDriver<XH430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> & id_list, const std::vector<uint16_t> & torque_list)
    {
        return syncWrite<typename XL330Reg::TYPE_GOAL_CURRENT>(XL330Reg::ADDR_GOAL_CURRENT, id_list, torque_list);
    }

} // ttl_driver

#endif // DxlDriver
