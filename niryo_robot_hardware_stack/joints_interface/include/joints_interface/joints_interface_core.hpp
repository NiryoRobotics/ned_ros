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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
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

#include "joints_interface/joint_hardware_interface.hpp"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/SetBool.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include "niryo_robot_msgs/Trigger.h"
#include "common/model/hardware_type_enum.hpp"

#include "common/model/i_interface_core.hpp"

namespace joints_interface
{

class JointsInterfaceCore : common::model::IInterfaceCore
{
    public:

        JointsInterfaceCore(ros::NodeHandle& rootnh, 
                            ros::NodeHandle& robot_hwnh,
                            std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                            std::shared_ptr<can_driver::CanInterfaceCore> can_interface);
        virtual ~JointsInterfaceCore() override;

        virtual bool init(ros::NodeHandle& nh) override;

        void sendMotorsParams();
        void activateLearningMode(bool activate, int &ostatus, std::string &omessage);

        bool getFreeDriveMode() const;
        void getCalibrationState(bool &need_calibration, bool &calibration_in_progress) const;

        const std::vector<std::shared_ptr<common::model::JointState> >& getJointsState() const;

    private:
        virtual void initParameters(ros::NodeHandle& nh) override;
        virtual void startServices(ros::NodeHandle& nh) override;
        virtual void startPublishers(ros::NodeHandle& nh) override;
        virtual void startSubscribers(ros::NodeHandle& nh) override;

        void rosControlLoop();

        bool _callbackResetController(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        bool _callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
        bool _callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        bool _callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req, niryo_robot_msgs::SetBool::Response &res);

        void _callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult& msg);

        void _publishLearningMode();

    private:
        ros::NodeHandle _nh;

        // TODO(CC) create a thread to update hw status from can and ttl into joint_states

        bool _enable_control_loop{true};
        bool _previous_state_learning_mode{true};
        bool _reset_controller{false};

        double _control_loop_frequency{0.0};
        double _publish_learning_mode_frequency{0.0};
        std::string _joint_controller_name;

        std::shared_ptr<JointHardwareInterface> _robot;
        std::shared_ptr<controller_manager::ControllerManager> _cm;

        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;
        std::shared_ptr<can_driver::CanInterfaceCore> _can_interface;

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
