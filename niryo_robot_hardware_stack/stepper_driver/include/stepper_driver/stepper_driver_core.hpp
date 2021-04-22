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

namespace StepperDriver
{
    class StepperDriverCore : public common::model::IDriverCore
    {
        public:

            StepperDriverCore();
            virtual ~StepperDriverCore() override;

            void clearCommandQueue();
            void clearConveyorCommandQueue();

            void addCommandToQueue(const common::model::StepperMotorCmd &cmd);
            void addCommandToQueue(const std::vector<common::model::StepperMotorCmd> &cmd);

            void setTrajectoryControllerCommands(const std::vector<int32_t> &cmd);

            int setConveyor(uint8_t motor_id);
            void unsetConveyor(uint8_t motor_id);

            void clearCalibrationTab();
            void startCalibration(bool enable);

            //direct commands
            bool scanMotorId(int motor_to_find);
            int motorReport(int motor_id);

            //getters
            bool getCalibrationState() const;
            e_CanStepperCalibrationStatus getCalibrationResult(uint8_t id, int32_t& calibration_result) const;
            const std::vector<common::model::ConveyorState> &getConveyorStates() const;

            stepper_driver::StepperArrayMotorHardwareStatus getHwStatus() const;
            niryo_robot_msgs::BusState getCanBusState() const;

            const std::vector<common::model::StepperMotorState>& getStepperStates() const;

            // IDriverCore interface
            void startControlLoop() override;

            void activeDebugMode(bool mode) override;

            bool isConnectionOk() const override;
            int launchMotorsReport() override;

        private:
            void init() override;
            void initParameters() override;
            void resetHardwareControlLoopRates() override;
            void controlLoop() override;
            void _executeCommand() override;

        private:
            ros::NodeHandle _nh;
            bool _control_loop_flag;
            bool _debug_flag;

            std::mutex _control_loop_mutex;
            std::thread _control_loop_thread;

            bool _calibration_in_progress;

            double _control_loop_frequency;
            double _write_frequency;
            double _check_connection_frequency;

            double _time_hw_last_write;
            double _time_hw_last_check_connection;

            std::unique_ptr<StepperDriver> _stepper;

            std::vector<int32_t> _joint_trajectory_controller_cmd;
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
    bool StepperDriverCore::getCalibrationState() const
    {
        return _calibration_in_progress;
    }

    inline
    const std::vector<common::model::ConveyorState> &StepperDriverCore::getConveyorStates() const
    {
        return _stepper->getConveyorsState();
    }

    inline
    const std::vector<common::model::StepperMotorState> &StepperDriverCore::getStepperStates() const
    {
        return _stepper->getMotorsState();
    }
}

#endif
