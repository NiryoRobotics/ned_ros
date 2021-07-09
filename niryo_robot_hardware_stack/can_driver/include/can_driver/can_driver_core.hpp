/*
can_driver_core.hpp
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

#ifndef CAN_DRIVER_CORE_HPP
#define CAN_DRIVER_CORE_HPP

#include <memory>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
#include <queue>

#include "common/model/idriver_core.hpp"
#include "common/model/iinterface_core.hpp"

#include "can_driver/can_driver.hpp"
#include "can_driver/StepperArrayMotorHardwareStatus.h"
#include "niryo_robot_msgs/BusState.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include "common/model/synchronize_stepper_motor_cmd.hpp"

namespace can_driver
{
/**
 * @brief The CanDriverCore class
 */
class CanDriverCore : public common::model::IDriverCore, public common::model::IInterfaceCore
{
    public:

        CanDriverCore(ros::NodeHandle& nh);
        virtual ~CanDriverCore() override;

        bool init(ros::NodeHandle& nh) override;

        int setConveyor(uint8_t motor_id, uint8_t default_conveyor_id = 6);
        void unsetConveyor(uint8_t motor_id);

        void clearSingleCommandQueue();
        void clearConveyorCommandQueue();

        void setTrajectoryControllerCommands(const std::vector<std::pair<uint8_t, int32_t> > &cmd);
        void addSingleCommandToQueue(const common::model::StepperMotorCmd& cmd);
        void addSingleCommandToQueue(const std::vector<common::model::StepperMotorCmd>& cmd);

        void startCalibration();
        void resetCalibration();

        // direct commands
        bool scanMotorId(uint8_t motor_to_find);

        // getters
        bool isCalibrationInProgress() const;
        int32_t getCalibrationResult(uint8_t id) const;
        common::model::EStepperCalibrationStatus getCalibrationStatus() const;

        can_driver::StepperArrayMotorHardwareStatus getHwStatus() const;

        std::vector<std::shared_ptr<common::model::StepperMotorState> > getStepperStates() const;
        common::model::StepperMotorState getStepperState(uint8_t motor_id) const;

        // IDriverCore interface
        void startControlLoop() override;

        void activeDebugMode(bool mode) override;

        bool isConnectionOk() const override;
        int launchMotorsReport() override;

        niryo_robot_msgs::BusState getBusState() const override;

    private:
        virtual void initParameters(ros::NodeHandle &nh) override;
        virtual void startServices(ros::NodeHandle &nh) override;
        virtual void startPublishers(ros::NodeHandle &nh) override;
        virtual void startSubscribers(ros::NodeHandle &nh) override;

        void resetHardwareControlLoopRates() override;
        void controlLoop() override;
        void _executeCommand() override;

        int motorCmdReport(uint8_t motor_id);

    private:
        ros::NodeHandle _nh;
        bool _control_loop_flag{false};
        bool _debug_flag{false};

        std::mutex _control_loop_mutex;
        std::mutex _joint_trajectory_mutex;

        std::thread _control_loop_thread;

        double _control_loop_frequency{0.0};

        double _delta_time_write{0.0};

        double _time_hw_data_last_read{0.0};
        double _time_hw_data_last_write{0.0};

        double _time_check_connection_last_read{0.0};

        // specific to stepper
        double _delta_time_calib_read{0.0};
        double _time_hw_calib_last_read{0.0};

        std::unique_ptr<CanDriver> _can_driver;

        std::vector<std::pair<uint8_t, int32_t> > _joint_trajectory_cmd;
        std::queue<common::model::StepperMotorCmd> _stepper_single_cmds;
        std::queue<common::model::StepperMotorCmd> _conveyor_cmds;

        static constexpr int QUEUE_OVERFLOW = 20;
};

/**
 * @brief CanDriverCore::isConnectionOk
 * @return
 */
inline
bool CanDriverCore::isConnectionOk() const
{
    return _can_driver->isConnectionOk();
}

/**
 * @brief CanDriverCore::isCalibrationInProgress
 * @return
 */
inline
bool CanDriverCore::isCalibrationInProgress() const
{
    return _can_driver->isCalibrationInProgress();
}

/**
 * @brief CanDriverCore::getCalibrationResult
 * @param id
 * @return
 */
inline
int32_t CanDriverCore::getCalibrationResult(uint8_t id) const
{
    return _can_driver->getCalibrationResult(id);
}

/**
 * @brief CanDriverCore::getCalibrationStatus
 * @return
 */
inline
common::model::EStepperCalibrationStatus
CanDriverCore::getCalibrationStatus() const
{
    return _can_driver->getCalibrationStatus();
}

/**
 * @brief CanDriverCore::getStepperStates
 * @return
 */
inline
std::vector<std::shared_ptr<common::model::StepperMotorState> >
CanDriverCore::getStepperStates() const
{
    return _can_driver->getMotorsStates();
}

/**
 * @brief CanDriverCore::getStepperState
 * @param motor_id
 * @return
 */
inline
common::model::StepperMotorState
CanDriverCore::getStepperState(uint8_t motor_id) const
{
    return _can_driver->getMotorState(motor_id);

}
} // CanDriver

#endif
