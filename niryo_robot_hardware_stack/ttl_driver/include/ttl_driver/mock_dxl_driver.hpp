/*
mock_dxl_driver.hpp
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

#ifndef MOCK_DXL_DRIVER_HPP
#define MOCK_DXL_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "abstract_dxl_driver.hpp"
#include "common/common_defs.hpp"
#include "ttl_driver/fake_ttl_data.hpp"

namespace ttl_driver
{

    /**
     * @brief The DxlDriver class
     */
    class MockDxlDriver : public AbstractDxlDriver
    {
    public:
        MockDxlDriver(std::shared_ptr<FakeTtlData> data);

        std::string str() const override;

        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver
    public:
        int ping(uint8_t id) override;
        int getModelNumber(uint8_t id,
                           uint16_t &model_number) override;
        int scan(std::vector<uint8_t> &id_list) override;
        int reboot(uint8_t id) override;

        std::string interpretErrorState(uint32_t hw_state) const override;

        int readCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t &data) override;
        int writeCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t data) override;

        // eeprom write
        int changeId(uint8_t id, uint8_t new_id) override;
        int writeStartupConfiguration(uint8_t id, uint8_t value) override;
        int writeTemperatureLimit(uint8_t id, uint8_t temperature_limit) override;
        int writeShutdownConfiguration(uint8_t id, uint8_t configuration) override;

        // eeprom read
        int checkModelNumber(uint8_t id) override;
        int readFirmwareVersion(uint8_t id, std::string &version) override;
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
        int readTemperature(uint8_t id, uint8_t &temperature) override;
        int readVoltage(uint8_t id, double &voltage) override;
        int readHwErrorStatus(uint8_t id, uint8_t &hardware_error_status) override;

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
        int syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2>> &data_array_list) override;

        int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &temperature_list) override;
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadHwStatus(const std::vector<uint8_t> &id_list, std::vector<std::pair<double, uint8_t>> &data_list) override;

        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list) override;

        // AbstractDxlDriver interface
    public:
        int readPID(uint8_t id, std::vector<uint16_t> &data) override;
        int writePID(uint8_t id, const std::vector<uint16_t> &data) override;

        int writeControlMode(uint8_t id, uint8_t data) override;
        int readControlMode(uint8_t id, uint8_t &data) override;

        int writeLed(uint8_t id, uint8_t led_value) override;
        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &led_list) override;
        int writeTorqueGoal(uint8_t id, uint16_t torque) override;
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint16_t> &torque_list) override;

        int readLoad(uint8_t id, uint16_t &present_load) override;
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint16_t> &load_list) override;

        int readMoving(uint8_t id, uint8_t &status) override;

    private:
        std::shared_ptr<FakeTtlData> _fake_data;
        std::vector<uint8_t> _id_list;

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int LEN_ID_DATA_NOT_SAME = 20;

        // AbstractTtlDriver interface
    protected:
        std::string interpretFirmwareVersion(uint32_t fw_version) const override;
    };

} // DynamixelDriver

#endif // MOCK_DXL_DRIVER
