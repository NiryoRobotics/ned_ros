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
        MockDxlDriver(FakeTtlData data);
        virtual ~MockDxlDriver() override;

        virtual std::string str() const override;

        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver
    public:
        virtual int ping(uint8_t id) override;
        virtual int getModelNumber(uint8_t id,
                            uint16_t& model_number) override;
        virtual int scan(std::vector<uint8_t>& id_list) override;
        virtual int reboot(uint8_t id) override;

        virtual std::string interpreteErrorState(uint32_t hw_state) const override;

        // eeprom write
        virtual int changeId(uint8_t id, uint8_t new_id) override;

        // eeprom read
        virtual int checkModelNumber(uint8_t id) override;
        virtual int readFirmwareVersion(uint8_t id, std::string &version) override;
        virtual int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        virtual int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        // ram write
        virtual int setTorqueEnable(uint8_t id, uint32_t torque_enable) override;
        virtual int setGoalPosition(uint8_t id, uint32_t position) override;
        virtual int setGoalVelocity(uint8_t id, uint32_t velocity) override;

        virtual int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override;
        virtual int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        virtual int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        // ram read
        virtual int readPosition(uint8_t id, uint32_t &present_position) override;
        virtual int readTemperature(uint8_t id, uint32_t &temperature) override;
        virtual int readVoltage(uint8_t id, double &voltage) override;
        virtual int readHwErrorStatus(uint8_t id, uint32_t &hardware_status) override;

        virtual int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;

        virtual int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

        // AbstractDxlDriver interface
    public:
        virtual int setLed(uint8_t id, uint32_t led_value) override;
        virtual int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) override;
        virtual int setGoalTorque(uint8_t id, uint32_t torque) override;
        virtual int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) override;
        virtual int setPositionPGain(uint8_t id, uint32_t gain) override;
        virtual int setPositionIGain(uint8_t id, uint32_t gain) override;
        virtual int setPositionDGain(uint8_t id, uint32_t gain) override;
        virtual int setVelocityPGain(uint8_t id, uint32_t gain) override;
        virtual int setVelocityIGain(uint8_t id, uint32_t gain) override;
        virtual int setff1Gain(uint8_t id, uint32_t gain) override;
        virtual int setff2Gain(uint8_t id, uint32_t gain) override;
        virtual int readLoad(uint8_t id, uint32_t &present_load) override;
        virtual int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) override;
        virtual int readVelocity(uint8_t id, uint32_t &present_velocity) override;
        virtual int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
    
        void removeGripper();
    private:
        void initializeFakeData(FakeTtlData data);
    private:

        /*struct FakeRegister
        {
          uint32_t       position{};
          uint32_t       temperature{};
          double         voltage{};
          uint32_t       min_position{};
          uint32_t       max_position{};
          uint16_t       model_number{};
          std::string    firmware{};
        };*/

        std::map<uint8_t, FakeTtlData::FakeRegister> _map_fake_registers;
                                                             /*{ {5,  {2048, 50, 50, 0, 4096, 1, "0.0.2"}},
                                                             {6,  {2048, 52, 50, 0, 4096, 1, "0.0.2"}},
                                                             {7,  {2048, 54, 50, 0, 4096, 1, "0.0.2"}},
                                                             {11, { 370, 56, 50, 0, 4096, 1, "0.0.2"}} };*/
        std::vector<uint8_t> _full_id_list; //{2,3,4,5,6,7,11};
        std::vector<uint8_t> _id_list;

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int LEN_ID_DATA_NOT_SAME    = 20;

        // AbstractTtlDriver interface
    protected:
        virtual std::string interpreteFirmwareVersion(uint32_t fw_version) const override;
};

} // DynamixelDriver

#endif // MOCK_DXL_DRIVER
