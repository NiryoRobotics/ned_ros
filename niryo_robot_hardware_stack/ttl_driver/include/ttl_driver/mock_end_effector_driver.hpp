/*
mock_end_effector_driver.hpp
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

#ifndef MOCK_END_EFFECTOR_DRIVER_HPP
#define MOCK_END_EFFECTOR_DRIVER_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "abstract_motor_driver.hpp"

#include "common/model/action_type_enum.hpp"
#include "end_effector_reg.hpp"
#include "common/model/end_effector_command_type_enum.hpp"
#include "common/model/end_effector_state.hpp"

namespace ttl_driver
{

/**
 * @brief The EndEffectorDriver class
 */
class MockEndEffectorDriver : public AbstractTtlDriver
{
    public:
        MockEndEffectorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                          std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        virtual ~MockEndEffectorDriver() override;

    public:
        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver
        virtual std::string str() const override;

        virtual int checkModelNumber(uint8_t id) override;
        virtual int readFirmwareVersion(uint8_t id, std::string &version) override;
        
        virtual int readTemperature(uint8_t id, uint32_t &_temperature) override;
        virtual int readVoltage(uint8_t id, double &_voltage) override;
        virtual int readHwErrorStatus(uint8_t id, uint32_t &hardware_status) override;

        virtual int syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list) override;
        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list) override;
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

        virtual int writeSingleCmd(const std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd) override;
        virtual int writeSyncCmd(int type, const std::vector<uint8_t>& ids, const std::vector<uint32_t>& params) override;

    public:
        int readButton1Status(uint8_t id, common::model::EActionType& action);
        int readButton2Status(uint8_t id, common::model::EActionType& action);
        int readButton3Status(uint8_t id, common::model::EActionType& action);

        int readAccelerometerXValue(uint8_t id, uint32_t& x_value);
        int readAccelerometerYValue(uint8_t id, uint32_t& y_value);
        int readAccelerometerZValue(uint8_t id, uint32_t& z_value);

        int readCollisionStatus(uint8_t id, bool& status);

        int readDigitalInput(uint8_t id, bool& in);
        int writeDigitalOutput(uint8_t id, bool out);

        common::model::EActionType interpreteActionValue(uint32_t value);
        virtual std::string interpreteErrorState(uint32_t hw_state) const override;

        virtual int ping(uint8_t id) override;
    protected:
        virtual std::string interpreteFirmwareVersion(uint32_t fw_version) const override;

    private:
        struct SEndEffectorInfo {
            uint32_t button1_action{0};
            uint32_t button2_action{0};
            uint32_t button3_action{0};
            
            uint32_t x_value{1};
            uint32_t y_value{1};
            uint32_t z_value{1};
            
            bool digitalInput = true;
            bool DigitalOutput = true;
        };

        SEndEffectorInfo _ee_info;

        uint8_t _id{0};
        uint32_t _temperature{32};
        uint32_t _voltage{5000};
        std::string _firmware_version{"v1.0.0"};

};

} // ttl_driver

#endif // MockEndEffectorDriver
