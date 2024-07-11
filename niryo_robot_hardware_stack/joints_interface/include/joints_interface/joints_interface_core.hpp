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
#include <mutex>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <actionlib_msgs/GoalID.h>

#include <ros/ros.h>

#include "common/util/i_interface_core.hpp"

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

namespace joints_interface
{

class JointsInterfaceCore : common::util::IInterfaceCore
{
    public:

        JointsInterfaceCore(ros::NodeHandle& rootnh, 
                            ros::NodeHandle& robot_hwnh,
                            std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                            std::shared_ptr<can_driver::CanInterfaceCore> can_interface);
        ~JointsInterfaceCore() override;

        // non copyable class
        JointsInterfaceCore( const JointsInterfaceCore& ) = delete;
        JointsInterfaceCore( JointsInterfaceCore&& ) = delete;

        JointsInterfaceCore& operator= ( JointsInterfaceCore && ) = delete;
        JointsInterfaceCore& operator= ( const JointsInterfaceCore& ) = delete;

        bool init(ros::NodeHandle& nh) override;

        void activateLearningMode(bool activate, int &ostatus, std::string &omessage);
        bool rebootAll(bool torque_on);

        bool needCalibration() const;
        bool isCalibrationInProgress() const;
        bool isFreeMotion() const;

        void setEstopFlag(bool value);

        const std::vector<std::shared_ptr<common::model::JointState> >& getJointsState() const;

    private:
        void initParameters(ros::NodeHandle& nh) override;
        void startServices(ros::NodeHandle& nh) override;
        void startPublishers(ros::NodeHandle& nh) override;
        void startSubscribers(ros::NodeHandle& nh) override;

        void rosControlLoop();
        void resetController();

        bool _callbackResetController(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        bool _callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
        bool _callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res);
        bool _callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req, niryo_robot_msgs::SetBool::Response &res);
        bool _callbackFactoryCalibrateMotors(FactoryCalibration::Request &req, FactoryCalibration::Response &res);

        void _callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult& msg);

        void _publishLearningMode();

    private:
        ros::NodeHandle _nh;

        bool _enable_control_loop{true};
        bool _previous_state_learning_mode{true};
        bool _reset_controller{true};
        bool _estop_flag{false};

        std::string _joint_controller_name;

        std::shared_ptr<JointHardwareInterface> _robot;
        std::shared_ptr<controller_manager::ControllerManager> _cm;

        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;
        std::shared_ptr<can_driver::CanInterfaceCore> _can_interface;

        std::thread _control_loop_thread;
        ros::Rate _control_loop_rate{1.0};

        ros::Publisher _learning_mode_publisher;

        ros::Subscriber _trajectory_result_subscriber;

        ros::ServiceServer _reset_controller_server; // workaround to compensate missed steps
        ros::ServiceServer _calibrate_motors_server;
        ros::ServiceServer _factory_calibrate_motors_server;
        ros::ServiceServer _request_new_calibration_server;
        ros::ServiceServer _activate_learning_mode_server;

        std::string _hardware_version;
        bool _simulation_mode{false};

        int _lock_write_cnt{-1};
};


inline
void JointsInterfaceCore::setEstopFlag(bool value)
{
    _estop_flag = value;
}

/**
 * @brief JointsInterfaceCore::needCalibration
 * @return
 */
inline
bool JointsInterfaceCore::needCalibration() const
{
    return _robot->needCalibration();
}

/**
 * @brief JointsInterfaceCore::isCalibrationInProgress
 * @return
 */
inline
bool JointsInterfaceCore::isCalibrationInProgress() const
{
    return _robot->isCalibrationInProgress();
}

/**
 * @brief JointsInterfaceCore::isFreeMotion
 * @return
 */
inline
bool JointsInterfaceCore::isFreeMotion() const
{
    return _previous_state_learning_mode;
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
