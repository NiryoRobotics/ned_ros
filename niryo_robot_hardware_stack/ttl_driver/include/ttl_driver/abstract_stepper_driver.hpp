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

#ifndef ABSTRACT_STEPPER_DRIVER_HPP
#define ABSTRACT_STEPPER_DRIVER_HPP

// ttl
#include "abstract_motor_driver.hpp"

//std
#include <memory>

// common
#include "common/model/single_motor_cmd.hpp"
#include "common/model/synchronize_motor_cmd.hpp"

namespace ttl_driver
{

class AbstractStepperDriver : public AbstractMotorDriver
{
public:
    AbstractStepperDriver() = default;
    AbstractStepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                          std::shared_ptr<dynamixel::PacketHandler> packetHandler);

public:
    // AbstractMotorDriver interface
    std::string str() const override;

    int writeSingleCmd(std::unique_ptr<common::model::AbstractTtlSingleMotorCmd> && cmd) override;
    int writeSyncCmd(int type, const std::vector<uint8_t>& ids, const std::vector<uint32_t>& params) override;

protected:
    // AbstractTtlDriver interface
    std::string interpreteFirmwareVersion(uint32_t fw_version) const override;

public:
    // specific Stepper commands

    // ram write
    virtual int startHoming(uint8_t id) = 0;
    virtual int readHomingStatus(uint8_t id, uint32_t& status) = 0;
    virtual int writeHomingSetup(uint8_t id, uint8_t direction, uint8_t stall_threshold) = 0;

    virtual int readVelocityProfile(uint8_t id, std::vector<uint32_t>& data_list) = 0;
    virtual int writeVelocityProfile(uint8_t id, const std::vector<uint32_t>& data_list) = 0;
    // read
    virtual int readFirmwareRunning(uint8_t id, bool &is_running) = 0;
};

} // ttl_driver

#endif // ABSTRACT_STEPPER_DRIVER_HPP
