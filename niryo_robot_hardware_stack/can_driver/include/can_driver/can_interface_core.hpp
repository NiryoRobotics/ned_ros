/*
can_interface_core.hpp
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

#ifndef CAN_DRIVER_CORE_H
#define CAN_DRIVER_CORE_H

#include <cstdint>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
#include <queue>
#include <vector>

#include "common/model/hardware_type_enum.hpp"
#include "common/util/i_driver_core.hpp"
#include "common/util/i_interface_core.hpp"
#include "can_driver/can_manager.hpp"
#include "can_driver/StepperArrayMotorHardwareStatus.h"
#include "niryo_robot_msgs/BusState.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include "common/model/abstract_single_motor_cmd.hpp"
#include "common/model/stepper_motor_state.hpp"

namespace can_driver
{
/**
 * @brief The CanInterfaceCore class
 */
class CanInterfaceCore : public common::util::IDriverCore, public common::util::IInterfaceCore
{
    public:
        CanInterfaceCore(ros::NodeHandle& nh);
        ~CanInterfaceCore() override;
        CanInterfaceCore( const CanInterfaceCore& ) = delete;
        CanInterfaceCore( CanInterfaceCore&& ) = delete;
        CanInterfaceCore& operator= ( CanInterfaceCore && ) = delete;
        CanInterfaceCore& operator= ( const CanInterfaceCore& ) = delete;

        bool init(ros::NodeHandle& nh) override;

        // joints control
        int addJoint(const std::shared_ptr<common::model::StepperMotorState>& jointState);

        // Tool control
        // N.A.

        // end effector panel control
        // N.A.

        // conveyor control
        int setConveyor(const std::shared_ptr<common::model::ConveyorState>& state) override;
        void unsetConveyor(uint8_t motor_id, uint8_t default_conveyor_id) override;
        int changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id) override;
        
        void clearSingleCommandQueue();
        void clearConveyorCommandQueue();

        void setTrajectoryControllerCommands(std::vector<std::pair<uint8_t, int32_t> >&& cmd);

        void addSyncCommandToQueue(std::unique_ptr<common::model::ISynchronizeMotorCmd>&& cmd) override;

        void addSingleCommandToQueue(std::unique_ptr<common::model::ISingleMotorCmd>&& cmd) override;
        void addSingleCommandToQueue(std::vector<std::unique_ptr<common::model::ISingleMotorCmd> > cmd) override;

        void startCalibration() override;
        void resetCalibration() override;

        // direct commands
        bool scanMotorId(uint8_t motor_to_find) override;
        bool rebootHardware(const std::shared_ptr<common::model::AbstractHardwareState>& motor_state) override;

        // getters
        int32_t getCalibrationResult(uint8_t id) const override;
        common::model::EStepperCalibrationStatus getCalibrationStatus() const override;
        void setCalibrationStatus(const common::model::EStepperCalibrationStatus status) override;

        std::vector<std::shared_ptr<common::model::JointState> > getJointStates() const override;
        std::shared_ptr<common::model::JointState> getJointState(uint8_t motor_id) const override;

        // IDriverCore interface
        void startControlLoop() override;

        void activeDebugMode(bool mode) override;

        bool isConnectionOk() const override;
        int launchMotorsReport() override;
        niryo_robot_msgs::BusState getBusState() const override;
        common::model::EBusProtocol getBusProtocol() const override;

        std::vector<uint8_t> getRemovedMotorList() const override;
    private:
        void initParameters(ros::NodeHandle &nh) override;
        void startServices(ros::NodeHandle &nh) override;
        void startPublishers(ros::NodeHandle &nh) override;
        void startSubscribers(ros::NodeHandle &nh) override;

        void resetHardwareControlLoopRates() override;
        void controlLoop() override;
        void _executeCommand() override;

        int motorCmdReport(const common::model::JointState &jState, common::model::EHardwareType motor_type);

    private:
        bool _control_loop_flag{false};
        bool _debug_flag{false};

        std::mutex  _control_loop_mutex;
        std::mutex  _joint_trajectory_mutex;

        std::thread _control_loop_thread;

        double _control_loop_frequency{0.0};

        double _delta_time_write{0.0};

        double _time_hw_data_last_read{0.0};
        double _time_hw_data_last_write{0.0};

        double _time_check_connection_last_read{0.0};

        // specific to stepper

        std::unique_ptr<CanManager> _can_manager;

        std::vector<std::pair<uint8_t, int32_t> > _joint_trajectory_cmd;

        // can cmds
        std::queue<std::unique_ptr<common::model::AbstractCanSingleMotorCmd>> _stepper_single_cmds;
        std::queue<std::unique_ptr<common::model::AbstractCanSingleMotorCmd>> _conveyor_cmds;

        static constexpr int QUEUE_OVERFLOW = 20;
};

/**
 * @brief CanInterfaceCore::isConnectionOk
 * @return
 */
inline
bool CanInterfaceCore::isConnectionOk() const
{
    return _can_manager->isConnectionOk();
}

/**
 * @brief CanInterfaceCore::getBusProtocol
 * @return
 */
inline
common::model::EBusProtocol
CanInterfaceCore::getBusProtocol() const
{
    return common::model::EBusProtocol::CAN;
}

/**
 * @brief CanInterfaceCore::getCalibrationResult
 * @param id
 * @return
 */
inline
int32_t CanInterfaceCore::getCalibrationResult(uint8_t id) const
{
    return _can_manager->getCalibrationResult(id);
}

/**
 * @brief CanInterfaceCore::getCalibrationStatus
 * @return
 */
inline
common::model::EStepperCalibrationStatus
CanInterfaceCore::getCalibrationStatus() const
{
    return _can_manager->getCalibrationStatus();
}


/**
 * @brief CanInterfaceCore::setCalibrationStatus
 * @return
 */
inline
void CanInterfaceCore::setCalibrationStatus(const common::model::EStepperCalibrationStatus status)
{
    _can_manager->setCalibrationStatus(status);
}

} // CanManager

#endif // CAN_INTERFACE_CORE_H
