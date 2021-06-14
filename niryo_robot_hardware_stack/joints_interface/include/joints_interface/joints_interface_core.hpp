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
#include "model/motor_type_enum.hpp"

namespace joints_interface {

    class JointsInterfaceCore
    {
        public:

            JointsInterfaceCore(
                std::shared_ptr<ttl_driver::TtlDriverCore> ttl_driver,
                std::shared_ptr<can_driver::CanDriverCore> can_driver);

            virtual ~JointsInterfaceCore();

            void sendMotorsParams();
            void activateLearningMode(bool learning_mode_on, int &resp_status, std::string &resp_message);
            void calibrateJoints();

            std::string jointIdToJointName(uint8_t id, uint8_t motor_type) const;

            bool getFreeDriveMode() const;
            void getCalibrationState(bool &need_calibration, bool &calibration_in_progress) const;

            const std::vector<std::shared_ptr<common::model::JointState> >& getJointsState() const;

        private:    
            void init(std::shared_ptr<ttl_driver::TtlDriverCore> ttl_driver,
                      std::shared_ptr<can_driver::CanDriverCore> can_driver);
            void initParams();

            void startServices();
            void startSubscribers();

            void rosControlLoop();

            bool _callbackResetController(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
            void _callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult& msg);
            bool _callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
            bool _callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
            bool _callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req, niryo_robot_msgs::SetBool::Response &res);

            void _publishLearningMode();

        private:
            ros::NodeHandle _nh;

            bool _enable_control_loop{true};
            bool _previous_state_learning_mode{true};
            bool _reset_controller{false};

            double _control_loop_frequency{0.0};
            double _publish_learning_mode_frequency{0.0};

            std::shared_ptr<JointHardwareInterface> _robot;
            std::shared_ptr<controller_manager::ControllerManager> _cm;

            std::thread _publish_learning_mode_thread;
            std::thread _control_loop_thread;

            ros::Subscriber _trajectory_result_subscriber;
            ros::Publisher _learning_mode_publisher;

            ros::ServiceServer _reset_controller_server; // workaround to compensate missed steps
            ros::ServiceServer _calibrate_motors_server;
            ros::ServiceServer _request_new_calibration_server;
            ros::ServiceServer _activate_learning_mode_server;

    };

    /**
     * @brief JointsInterfaceCore::jointIdToJointName
     * @param id
     * @param motor_type
     * @return
     */
    inline
    std::string JointsInterfaceCore::jointIdToJointName(uint8_t id, uint8_t motor_type) const
    {
        return _robot->jointIdToJointName(id, static_cast<common::model::EMotorType>(motor_type));
    }

    /**
     * @brief JointsInterfaceCore::getFreeDriveMode
     * @return
     */
    inline
    bool JointsInterfaceCore::getFreeDriveMode() const
    {
        return _previous_state_learning_mode;
    }

    /**
     * @brief JointsInterfaceCore::getCalibrationState
     * @param need_calibration
     * @param calibration_in_progress
     */
    inline
    void JointsInterfaceCore::getCalibrationState(bool &need_calibration, bool &calibration_in_progress) const
    {
        need_calibration = _robot->needCalibration();
        calibration_in_progress = _robot->isCalibrationInProgress();
    }

    /**
     * @brief JointsInterfaceCore::getJointsState
     * @return
     */
    inline
    const std::vector<std::shared_ptr<common::model::JointState> > &JointsInterfaceCore::getJointsState() const
    {
        return _robot->getJointsState();
    }
} // JointsInterface
#endif
