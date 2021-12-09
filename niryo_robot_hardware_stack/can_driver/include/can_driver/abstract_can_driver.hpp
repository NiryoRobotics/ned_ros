/*
abstract_can_driver.hpp
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

#ifndef ABSTRACT_CAN_DRIVER_HPP
#define ABSTRACT_CAN_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>

#include "ros/ros.h"

#include "mcp_can_rpi/mcp_can_rpi.h"
#include "common/common_defs.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"

namespace can_driver
{

/**
 * @brief The AbstractCanDriver class
 */
class AbstractCanDriver
{

public:
    static constexpr int MAX_MESSAGE_LENGTH                     = 8;
    static constexpr int MESSAGE_LENGTH                         = 4;
    static constexpr double PING_TIME_OUT                       = 0.5;  // timeout using if ping fail
    static constexpr double STEPPER_MOTOR_TIMEOUT_VALUE         = 2.0;

public:
    AbstractCanDriver() = default;
    AbstractCanDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can);
    virtual ~AbstractCanDriver() = default;
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    AbstractCanDriver( const AbstractCanDriver& ) = delete;
    AbstractCanDriver( AbstractCanDriver&& ) = delete;
    AbstractCanDriver& operator= ( AbstractCanDriver && ) = delete;
    AbstractCanDriver& operator= ( const AbstractCanDriver& ) = delete;

    virtual bool canReadData() const;

    virtual int ping(uint8_t id);
    virtual int scan(std::set<uint8_t>& motors_unfound, std::vector<uint8_t> &id_list);

    virtual int writeSingleCmd(const std::unique_ptr<common::model::AbstractCanSingleMotorCmd >& cmd) = 0;

public:
    virtual std::string str() const;

    // read
    virtual uint8_t readData(uint8_t& id, int& control_byte,
                             std::array<uint8_t, MAX_MESSAGE_LENGTH>& rxBuf,
                             std::string& error_message);

    // Interpret data received
    virtual int32_t interpretPositionStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) = 0;
    virtual uint8_t interpretTemperatureStatus(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) = 0;
    virtual std::string interpretFirmwareVersion(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) = 0;
    virtual std::pair<common::model::EStepperCalibrationStatus, int32_t> interpretHomingData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) = 0;
    virtual std::tuple<bool, uint8_t, uint16_t> interpretConveyorData(const std::array<uint8_t, MAX_MESSAGE_LENGTH> &data) = 0;

protected:
    uint8_t read(INT32U *id, uint8_t *len, std::array<uint8_t, MAX_MESSAGE_LENGTH> &buf);
    uint8_t write(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);

private:
    std::shared_ptr<mcp_can_rpi::MCP_CAN> _mcp_can;

};

/**
 * @brief CanManager::canReadData
 * @return
 */
inline
bool AbstractCanDriver::canReadData() const
{
  return _mcp_can->canReadData();
}

} // can_driver

#endif // ABSTRACT_CAN_DRIVER_HPP
