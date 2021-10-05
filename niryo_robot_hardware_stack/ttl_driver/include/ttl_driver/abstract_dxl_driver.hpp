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

//std
#include <memory>

// common
#include "common/model/single_motor_cmd.hpp"
#include "common/model/synchronize_motor_cmd.hpp"

namespace ttl_driver
{

class AbstractDxlDriver : public AbstractMotorDriver
{
    public:
        AbstractDxlDriver() {}
        AbstractDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                          std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        virtual ~AbstractDxlDriver() override;


    public:
        // AbstractMotorDriver interface
        virtual std::string str() const override;

        virtual int writeSingleCmd(const std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd) override;
        virtual int writeSyncCmd(int type, const std::vector<uint8_t> &ids, const std::vector<uint32_t> &params) override;

    public:
        // specific DXL commands

        // ram write
        virtual int setLed(uint8_t id, uint32_t led_value ) = 0;
        virtual int syncWriteLed(const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& led_list ) = 0;

        virtual int setGoalTorque(uint8_t id, uint32_t torque ) = 0;
        virtual int syncWriteTorqueGoal(const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& torque_list ) = 0;

        virtual int setPositionPGain(uint8_t id, uint32_t gain ) = 0;
        virtual int setPositionIGain(uint8_t id, uint32_t gain ) = 0;
        virtual int setPositionDGain(uint8_t id, uint32_t gain ) = 0;
        virtual int setVelocityPGain(uint8_t id, uint32_t gain ) = 0;
        virtual int setVelocityIGain(uint8_t id, uint32_t gain ) = 0;
        virtual int setff1Gain(uint8_t id, uint32_t gain ) = 0;
        virtual int setff2Gain(uint8_t id, uint32_t gain ) = 0;

        // ram read
        virtual int readLoad(uint8_t id, uint32_t& present_load ) = 0;
        virtual int syncReadLoad(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& load_list ) = 0;

        virtual int readVelocity(uint8_t id, uint32_t& present_velocity) = 0;
        virtual int syncReadVelocity(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& velocity_list) = 0;
};

} // ttl_driver

#endif // ABSTRACT_DXL_DRIVER_HPP
