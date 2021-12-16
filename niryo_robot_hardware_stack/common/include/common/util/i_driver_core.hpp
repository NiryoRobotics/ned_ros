/*
idriver_core.hpp
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

#ifndef I_DriverCore_H
#define I_DriverCore_H

// C++
#include <cstdint>
#include <string>
#include <thread>
#include <mutex>

// niryo
#include "niryo_robot_msgs/BusState.h"

#include "common/model/joint_state.hpp"
#include "common/model/conveyor_state.hpp"

#include "common/model/abstract_single_motor_cmd.hpp"
#include "common/model/abstract_synchronize_motor_cmd.hpp"

#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/hardware_type_enum.hpp"

namespace common
{
namespace util
{

/**
 * @brief The IDriverCore class is an interface, intended to be used as a polymorphic base class
 */
class IDriverCore
{
public:
    virtual ~IDriverCore() = default;
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    IDriverCore( const IDriverCore& ) = delete;
    IDriverCore( IDriverCore&& ) = delete;
    IDriverCore& operator= ( IDriverCore && ) = delete;
    IDriverCore& operator= ( const IDriverCore& ) = delete;

    virtual common::model::EBusProtocol getBusProtocol() const = 0;

    virtual void startControlLoop() = 0;
    virtual bool isConnectionOk() const = 0;
    virtual bool scanMotorId(uint8_t motor_to_find) = 0;
    virtual void addSingleCommandToQueue(std::unique_ptr<common::model::ISingleMotorCmd>&& cmd) = 0;
    virtual void addSingleCommandToQueue(std::vector<std::unique_ptr<common::model::ISingleMotorCmd> > cmd) = 0;
    virtual void addSyncCommandToQueue(std::unique_ptr<common::model::ISynchronizeMotorCmd>&& cmd) = 0;

    virtual bool rebootHardware(const std::shared_ptr<common::model::AbstractHardwareState>& motor_state) = 0;

    // driver for conveyor
    virtual int setConveyor(const std::shared_ptr<common::model::ConveyorState>& state) = 0;
    virtual void unsetConveyor(uint8_t motor_id, uint8_t default_conveyor_id) = 0;
    virtual int changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id) = 0;

    // calibration
    virtual void startCalibration() = 0;
    virtual void resetCalibration() = 0;
    virtual int32_t getCalibrationResult(uint8_t id) const = 0;
    virtual common::model::EStepperCalibrationStatus getCalibrationStatus() const = 0;
    virtual void setCalibrationStatus(const common::model::EStepperCalibrationStatus status) = 0;

    virtual void activeDebugMode(bool mode) = 0;

    virtual int launchMotorsReport() = 0;
    virtual niryo_robot_msgs::BusState getBusState() const = 0;

    virtual std::vector<std::shared_ptr<common::model::JointState> > getJointStates() const = 0;
    virtual std::shared_ptr<common::model::JointState> getJointState(uint8_t motor_id) const = 0;
    virtual std::vector<uint8_t> getRemovedMotorList() const = 0;
protected:
    IDriverCore() = default;

private:
    virtual void resetHardwareControlLoopRates() = 0;
    virtual void controlLoop() = 0;
    virtual void _executeCommand() = 0;
};

} // namespace util
} // namespace common

#endif // I_DRIVER_CORE_H
