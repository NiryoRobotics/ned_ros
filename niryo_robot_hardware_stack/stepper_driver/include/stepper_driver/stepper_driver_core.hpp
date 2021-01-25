/*
    dxl_driver.hpp
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

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <mutex>
#include <functional>

#include "stepper_driver/stepper_driver.hpp"
#include "stepper_driver/StepperArrayMotorHardwareStatus.h"
#include "niryo_robot_msgs/BusState.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include <std_msgs/Int64MultiArray.h>

namespace StepperDriver
{
    class StepperDriverCore
    {
        public:

            StepperDriverCore();

            void init();

            void initParameters();

            void startControlLoop();
            void resetHardwareControlLoopRates();
            void controlLoop();

            stepper_driver::StepperArrayMotorHardwareStatus getHwStatus();
            niryo_robot_msgs::BusState getCanBusState();

            std::vector<StepperMotorState>& getStepperStates();

            void setStepperCommands(StepperMotorCmd &cmd);

            void setTrajectoryControllerCommands(std::vector<int32_t> &cmd); 
            std::vector<int32_t>& getTrajectoryControllerStates();

            int setConveyor(uint8_t motor_id);
            void unsetConveyor(uint8_t motor_id);
            void setConveyorCommands(StepperMotorCmd &cmd);
            std::vector<ConveyorState> &getConveyorStates();

            void clearCalibrationTab();
            bool getCalibrationState();
            e_CanStepperCalibrationStatus getCalibrationResult(uint8_t id, boost::shared_ptr<int32_t> &calibration_result);
            void startCalibration(bool enable);

            void activeDebugMode(bool mode);
            int motorReport(int motor_id);
            int launchMotorsReport();

            bool scanMotorId(int motor_to_find);

            bool isConnectionOk() const;

        private:

            ros::NodeHandle _nh;

            bool _control_loop_flag;
            bool _calibration_in_progress;
            bool _debug_flag;

            std::mutex _control_loop_mutex;

            boost::shared_ptr<std::thread> _control_loop_thread;

            std::vector<int32_t> _joint_trajectory_controller_cmd;
            std::shared_ptr<StepperMotorCmd> _stepper_cmd;
            std::shared_ptr<StepperMotorCmd> _conveyor_cmd;

            double _control_loop_frequency;
            double _write_frequency;
            double _check_connection_frequency;

            double _time_hw_last_write;
            double _time_hw_last_check_connection;

            boost::shared_ptr<StepperDriver> _stepper;

            std::vector<std::thread> _calibration_thread_list;

            void _executeCommand();

            ros::Publisher cmd_pub;

    };    
}

#endif
