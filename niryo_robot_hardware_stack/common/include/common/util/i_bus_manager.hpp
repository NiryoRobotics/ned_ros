/*
i_bus_manager.hpp
Copyright (C) 2017 Niryo
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

#ifndef I_BUS_MANAGER_H
#define I_BUS_MANAGER_H

#include <cstdint>
#include <string>
#include <vector>

#include "common/model/abstract_hardware_state.hpp"

#include "common/model/hardware_type_enum.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"

#include "ros/node_handle.h"

namespace common
{
namespace util
{

/**
 * @brief The IBusManager class is an interface to deal with Buses. Not built to manage polymorphism
 */
class IBusManager
{
public:
    virtual ~IBusManager() = default;
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    IBusManager( const IBusManager& ) = delete;
    IBusManager( IBusManager&& ) = delete;
    IBusManager& operator= ( IBusManager && ) = delete;
    IBusManager& operator= ( const IBusManager& ) = delete;

    virtual bool init(ros::NodeHandle& nh) = 0;

    virtual int addHardwareComponent(std::shared_ptr<common::model::AbstractHardwareState>&& state) = 0;
    virtual void removeHardwareComponent(uint8_t id) = 0;
    virtual bool isConnectionOk() const = 0;
    virtual int scanAndCheck() = 0;
    virtual bool ping(uint8_t id) = 0;

    virtual size_t getNbMotors() const = 0;
    virtual std::vector<uint8_t> getRemovedMotorList() const = 0;
    virtual void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const = 0;
    virtual std::string getErrorMessage() const = 0;

    //calibration
    virtual void startCalibration() = 0;
    virtual void resetCalibration() = 0;
    virtual int32_t getCalibrationResult(uint8_t id) const = 0;
    virtual common::model::EStepperCalibrationStatus getCalibrationStatus() const = 0;
protected:
    IBusManager() = default;

private:
    virtual int setupCommunication() = 0;
    virtual void addHardwareDriver(common::model::EHardwareType hardware_type) = 0;
};

} // namespace util
} // namespace common

#endif // I_BUS_MANAGER
