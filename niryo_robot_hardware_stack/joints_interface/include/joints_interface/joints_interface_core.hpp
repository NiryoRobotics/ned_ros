/*
    joints_hardware_interface_node.cpp
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

#ifndef JOINTS_INTERFACE_CORE_HPP
#define JOINTS_INTERFACE_CORE_HPP

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <functional>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <controller_manager/controller_manager.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joints_interface/JointHardwareInterface.hpp"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/SetBool.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include "niryo_robot_msgs/Trigger.h"

namespace JointsInterface {

    class JointsInterfaceCore
    {
        public:

            JointsInterfaceCore(
                std::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel,
                std::shared_ptr<StepperDriver::StepperDriverCore> stepper);

            virtual ~JointsInterfaceCore();

            void sendMotorsParams();
            void activateLearningMode(bool learning_mode_on, int &resp_status, std::string &resp_message);
            void calibrateJoints();

            std::string jointIdToJointName(int id);

            bool getFreeDriveMode() const;
            void getCalibrationState(bool &need_calibration, bool &calibration_in_progress) const;

            const std::vector<std::shared_ptr<common::model::JointState> >& getJointsState() const;

        private:    
            void init(std::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel,
                      std::shared_ptr<StepperDriver::StepperDriverCore> stepper);
            void initParams();

            void startServices();
            void startSubscribers();

            void rosControlLoop();

        private:
            ros::NodeHandle _nh;

            bool _callbackResetController(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
            void _callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult& msg);
            bool _callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
            bool _callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
            bool _callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req, niryo_robot_msgs::SetBool::Response &res);

            void _publishPackageStates();
            void _publishLearningMode();

            std::shared_ptr<JointHardwareInterface> _robot;

            double _publish_learning_mode_frequency;
            double _ros_control_frequency;

            std::shared_ptr<controller_manager::ControllerManager> _cm;

            ros::Subscriber _trajectory_result_subscriber;

            ros::Publisher _learning_mode_publisher;

            ros::ServiceServer _reset_controller_server; // workaround to compensate missed steps
            ros::ServiceServer _calibrate_motors_server;
            ros::ServiceServer _request_new_calibration_server;

            ros::ServiceServer _activate_learning_mode_server;

            std::thread _publish_learning_mode_thread;
            std::thread _control_loop_thread;

            bool _enable_control_loop;
            bool _previous_state_learning_mode;
            bool _reset_controller;
    };
} // JointsInterface
#endif
