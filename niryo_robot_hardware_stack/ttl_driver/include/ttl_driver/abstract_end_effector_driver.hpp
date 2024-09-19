/*
abstract_ttl_driver.hpp
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

#ifndef ABSTRACT_END_EFFECTOR_DRIVER_HPP
#define ABSTRACT_END_EFFECTOR_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "abstract_ttl_driver.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common/common_defs.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/abstract_synchronize_motor_cmd.hpp"

#include "common/model/action_type_enum.hpp"

namespace ttl_driver
{

/**
 * @brief The AbstractEndEffectorDriver class
 */
class AbstractEndEffectorDriver : public AbstractTtlDriver
{

public:
    AbstractEndEffectorDriver() = default;
    AbstractEndEffectorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                              std::shared_ptr<dynamixel::PacketHandler> packetHandler);

public:
    virtual int readButton0Status(uint8_t id, common::model::EActionType& action) = 0;
    virtual int readButton1Status(uint8_t id, common::model::EActionType& action) = 0;
    virtual int readButton2Status(uint8_t id, common::model::EActionType& action) = 0;
    virtual int syncReadButtonsStatus(const uint8_t& id, std::vector<common::model::EActionType>& action_list) = 0;

    virtual int readAccelerometerXValue(uint8_t id, uint32_t& x_value) = 0;
    virtual int readAccelerometerYValue(uint8_t id, uint32_t& y_value) = 0;
    virtual int readAccelerometerZValue(uint8_t id, uint32_t& z_value) = 0;

    virtual int readCollisionStatus(uint8_t id, bool& status) = 0;

    virtual int readDigitalInput(uint8_t id, bool& in) = 0;
    virtual int writeDigitalOutput(uint8_t id, bool out) = 0;

    virtual int writeCollisionThresh(uint8_t id, int thresh) = 0;
    virtual int writeCollisionThreshAlgo2(uint8_t id, int thresh) = 0;

    std::string interpretErrorState(uint32_t hw_state) const override;

    common::model::EActionType interpretActionValue(uint32_t value) const;

    // AbstractTtlDriver interface
protected:
    std::string str() const override;
    std::string interpretFirmwareVersion(uint32_t fw_version) const override;

    // AbstractTtlDriver interface
public:
    int writeSingleCmd(const std::unique_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd) override;
    int writeSyncCmd(int type, const std::vector<uint8_t> &ids, const std::vector<uint32_t> &params) override;
};

} // ttl_driver

#endif // ABSTRACT_END_EFFECTOR_DRIVER_HPP
