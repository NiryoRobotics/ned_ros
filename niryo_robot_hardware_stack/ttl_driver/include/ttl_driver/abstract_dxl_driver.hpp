/*
abstract_dxl_driver.hpp
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

#ifndef ABSTRACT_DXL_DRIVER_HPP
#define ABSTRACT_DXL_DRIVER_HPP

// ttl
#include "abstract_motor_driver.hpp"

// std
#include <memory>

// common
#include "common/model/single_motor_cmd.hpp"
#include "common/model/synchronize_motor_cmd.hpp"

namespace ttl_driver
{

    class AbstractDxlDriver : public AbstractMotorDriver
    {
    public:
        AbstractDxlDriver() = default;
        AbstractDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                          std::shared_ptr<dynamixel::PacketHandler> packetHandler);

    public:
        // AbstractMotorDriver interface
        std::string str() const override;

        int writeSingleCmd(const std::unique_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd) override;
        int writeSyncCmd(int type, const std::vector<uint8_t> &ids, const std::vector<uint32_t> &params) override;

    public:
        // specific DXL commands

        // eeprom write
        virtual int writeStartupConfiguration(uint8_t id, uint8_t config) = 0;
        virtual int writeTemperatureLimit(uint8_t id, uint8_t temperature_limit) = 0;
        virtual int writeShutdownConfiguration(uint8_t id, uint8_t configuration) = 0;

        // ram read
        virtual int readLoad(uint8_t id, uint16_t &present_load) = 0;
        virtual int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint16_t> &load_list) = 0;

        virtual int readPID(uint8_t id, std::vector<uint16_t> &data) = 0;

        virtual int readMoving(uint8_t id, uint8_t &status) = 0;

        // ram write
        virtual int writePID(uint8_t id, const std::vector<uint16_t> &data) = 0;

        virtual int writeLed(uint8_t id, uint8_t led_value) = 0;
        virtual int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint8_t> &led_list) = 0;

        virtual int writeTorqueGoal(uint8_t id, uint16_t torque) = 0;
        virtual int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint16_t> &torque_list) = 0;
    };

} // ttl_driver

#endif // ABSTRACT_DXL_DRIVER_HPP
