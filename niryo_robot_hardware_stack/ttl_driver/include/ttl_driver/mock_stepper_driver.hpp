/*
mock_stepper_driver.hpp
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

#ifndef MOCK_STEPPER_DRIVER_HPP
#define MOCK_STEPPER_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "abstract_stepper_driver.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "ttl_driver/fake_ttl_data.hpp"

namespace ttl_driver
{

/**
 * @brief The StepperDriver class
 */
class MockStepperDriver : public AbstractStepperDriver
{
    public:
        MockStepperDriver(FakeTtlData data);
        virtual ~MockStepperDriver() override;

        virtual std::string str() const override;

        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of DxlDriver
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

        // AbstractStepperDriver interface
    public:
        virtual int writeVelocityProfile(uint8_t id, const std::vector<uint32_t>& data) override;

        virtual int startHoming(uint8_t id) override;
        virtual int writeHomingDirection(uint8_t id, uint8_t direction) override;
        virtual int readHomingStatus(uint8_t id, uint32_t &status) override;
        virtual int readGoalVelocity(uint8_t id, uint32_t& present_velocity) override;

        virtual int readFirmwareRunning(uint8_t id, bool &is_running) override;

    private:
/*        struct FakeRegister
        {
          uint32_t       position{};
          uint32_t       temperature{};
          double       voltage{};
          uint32_t       min_position{};
          uint32_t       max_position{};
          uint16_t       model_number{};
          std::string    firmware{};
        };

        struct FakeConveyor
        {
            uint8_t         id = 8;  
            int8_t          direction = 1;
            int16_t         speed = 0;
            bool            state = false;
        };*/
        
        std::map<uint8_t, FakeTtlData::FakeRegister> _map_fake_registers;
                                                             /*{ {2, {0, 50, 12.1, 0, 4096, 1, "0.0.1"}},
                                                             {3, {0, 52, 12.2, 0, 4096, 1, "0.0.1"}},
                                                             {4, {0, 54, 12.3, 0, 4096, 1, "0.0.1"}}};*/

        FakeTtlData::FakeConveyor _fake_conveyor;
        std::vector<uint8_t> _full_id_list; //{2,3,4,5,6,7,11};
        std::vector<uint8_t> _id_list;

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int LEN_ID_DATA_NOT_SAME    = 20;

        static constexpr int CALIBRATION_IDLE = 0;
        static constexpr int CALIBRATION_IN_PROGRESS = 1;
        static constexpr int CALIBRATION_SUCCESS = 2;
        static constexpr int CALIBRATION_ERROR = 3;
        uint32_t _calibration_status = CALIBRATION_IDLE;
        uint8_t fake_time = 0;
    private:
        void initializeFakeData(FakeTtlData data);
};

}
#endif // MOCK_STEPPER_DRIVER_HPP
