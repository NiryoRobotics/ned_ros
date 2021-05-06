/*
    stepper_driver_core.hpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_DRIVER_CORE_HPP
#define STEPPER_DRIVER_CORE_HPP

#include <memory>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
#include <queue>

#include "model/idriver_core.hpp"
#include "stepper_driver/stepper_driver.hpp"
#include "stepper_driver/StepperArrayMotorHardwareStatus.h"
#include "niryo_robot_msgs/BusState.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include <std_msgs/Int64MultiArray.h>
#include "model/synchronize_stepper_motor_cmd.hpp"

namespace StepperDriver
{
    class StepperDriverCore : public common::model::IDriverCore
    {
        public:

            StepperDriverCore();
            virtual ~StepperDriverCore() override;

            void clearCommandQueue();
            void clearConveyorCommandQueue();

            void addCommandToQueue(const common::model::StepperMotorCmd& cmd);
            void addCommandToQueue(const std::vector<common::model::StepperMotorCmd>& cmd);

            void setSyncCommand(const common::model::SynchronizeStepperMotorCmd &cmd);

            int setConveyor(uint8_t motor_id);
            void unsetConveyor(uint8_t motor_id);

            void startCalibration();

            //direct commands
            bool scanMotorId(uint8_t motor_to_find);

            //getters
            bool isCalibrationInProgress() const;
            int32_t getCalibrationResult(uint8_t id) const;
            common::model::EStepperCalibrationStatus getCalibrationStatus() const;

            stepper_driver::StepperArrayMotorHardwareStatus getHwStatus() const;

            std::vector<common::model::StepperMotorState> getStepperStates() const;
            common::model::StepperMotorState getStepperState(uint8_t motor_id) const;

            // IDriverCore interface
            void startControlLoop() override;

            void activeDebugMode(bool mode) override;

            bool isConnectionOk() const override;
            int launchMotorsReport() override;

            niryo_robot_msgs::BusState getBusState() const override;

        private:
            void init() override;
            void initParameters() override;
            void resetHardwareControlLoopRates() override;
            void controlLoop() override;
            void _executeCommand() override;

            int motorCmdReport(uint8_t motor_id,  common::model::EMotorType motor_type = common::model::EMotorType::MOTOR_TYPE_STEPPER);

        private:
            ros::NodeHandle _nh;
            bool _control_loop_flag;
            bool _debug_flag;

            std::mutex _control_loop_mutex;
            std::mutex _joint_trajectory_mutex;
            std::thread _control_loop_thread;

            double _control_loop_frequency;
            double _check_connection_frequency;

            double _delta_time_data_read;
            double _delta_time_calib_read;
            double _delta_time_write;

            double _time_hw_data_last_read;
            double _time_hw_calib_last_read;
            double _time_hw_data_last_write;

            double _time_check_connection_last_read;

            std::unique_ptr<StepperDriver> _stepper;

            common::model::SynchronizeStepperMotorCmd _joint_trajectory_controller_cmd;
            std::queue<common::model::StepperMotorCmd> _stepper_cmds;
            std::queue<common::model::StepperMotorCmd> _conveyor_cmds;

            ros::Publisher cmd_pub;
    };

    inline
    bool StepperDriverCore::isConnectionOk() const
    {
        return _stepper->isConnectionOk();
    }

    inline
    bool StepperDriverCore::isCalibrationInProgress() const
    {
        return _stepper->isCalibrationInProgress();
    }

    inline
    common::model::EStepperCalibrationStatus
    StepperDriverCore::getCalibrationStatus() const
    {
        return _stepper->getCalibrationStatus();
    }

    inline
    std::vector<common::model::StepperMotorState>
    StepperDriverCore::getStepperStates() const
    {
        return _stepper->getMotorsStates();
    }

    inline
    common::model::StepperMotorState
    StepperDriverCore::getStepperState(uint8_t motor_id) const
    {
        return _stepper->getMotorState(motor_id);

    }
}

#endif
