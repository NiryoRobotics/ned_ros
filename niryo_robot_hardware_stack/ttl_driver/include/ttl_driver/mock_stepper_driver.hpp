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
        MockStepperDriver(std::shared_ptr<FakeTtlData> data);

        std::string str() const override;

        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of DxlDriver
    public:
        int ping(uint8_t id) override;
        int getModelNumber(uint8_t id,
                            uint16_t& model_number) override;
        int scan(std::vector<uint8_t>& id_list) override;
        int reboot(uint8_t id) override;

        // eeprom write
        int changeId(uint8_t id, uint8_t new_id) override;

        // eeprom read
        int checkModelNumber(uint8_t id) override;
        int readFirmwareVersion(uint8_t id, std::string &version) override;
        int readMinPosition(uint8_t id, uint32_t &min_pos) override;
        int readMaxPosition(uint8_t id, uint32_t &max_pos) override;

        // ram write
        int writeTorquePercentage(uint8_t id, uint8_t torque_percentage) override;
        int writePositionGoal(uint8_t id, uint32_t position) override;
        int writeVelocityGoal(uint8_t id, uint32_t velocity) override;

        int syncWriteTorquePercentage(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &torque_percentage_list) override;
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override;
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override;

        // ram read

        int readTemperature(uint8_t id, uint8_t& temperature) override;
        int readVoltage(uint8_t id, double &voltage) override;
        int readHwErrorStatus(uint8_t id, uint8_t& hardware_error_status) override;

        int readPosition(uint8_t id, uint32_t &present_position) override;
        int readVelocity(uint8_t id, uint32_t &present_velocity) override;

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override;
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override;
        int syncReadJointStatus(const std::vector<uint8_t> &id_list, std::vector<std::array<uint32_t, 2> >& data_array) override;

        int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint8_t>& temperature_list) override;
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadRawVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        int syncReadHwStatus(const std::vector<uint8_t> &id_list, std::vector<std::pair<double, uint8_t> >& data_list) override;

        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &hw_error_list) override;

        // AbstractStepperDriver interface
    public:
        int readControlMode(uint8_t id, uint8_t &mode) override;
        int writeControlMode(uint8_t id, uint8_t mode) override;

        int readVelocityProfile(uint8_t id, std::vector<uint32_t>& data_list) override;
        int writeVelocityProfile(uint8_t id, const std::vector<uint32_t>& data) override;

        int startHoming(uint8_t id) override;
        int writeHomingSetup(uint8_t id, uint8_t direction, uint8_t stall_threshold) override;

        int readHomingStatus(uint8_t id, uint8_t &status) override;
        int syncReadHomingStatus(const std::vector<uint8_t> &id_list, std::vector<uint8_t> &status_list) override;

        int readFirmwareRunning(uint8_t id, bool &is_running) override;

        int syncReadHomingAbsPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &abs_position) override;
        int syncWriteHomingAbsPosition(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &abs_position) override;

        float velocityUnit() const override;

    private:
        bool init();

        std::shared_ptr<FakeTtlData>  _fake_data;
        std::vector<uint8_t> _id_list;

        uint8_t _calibration_status{CALIBRATION_IDLE};
        // fake time for calibration
        int _fake_time{0};

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int LEN_ID_DATA_NOT_SAME    = 20;

        static constexpr int CALIBRATION_IDLE = 0;
        static constexpr int CALIBRATION_IN_PROGRESS = 1;
        static constexpr int CALIBRATION_SUCCESS = 2;
        static constexpr int CALIBRATION_ERROR = 3;


};

}
#endif // MOCK_STEPPER_DRIVER_HPP
