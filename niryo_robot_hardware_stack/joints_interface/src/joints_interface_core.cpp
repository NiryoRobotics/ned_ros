/*
    joints_hardware_interface_core.cpp
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

// C++
#include <functional>
#include <string>

#include "joints_interface/joints_interface_core.hpp"
#include "common/util/util_defs.hpp"

namespace joints_interface
{

/**
 * @brief JointsInterfaceCore::JointsInterfaceCore
 * @param rootnh
 * @param robot_hwnh
 * @param ttl_driver
 * @param can_driver
 */
JointsInterfaceCore::JointsInterfaceCore(ros::NodeHandle& rootnh, 
                                         ros::NodeHandle& robot_hwnh,
                                         std::shared_ptr<ttl_driver::TtlDriverCore> ttl_driver,
                                         std::shared_ptr<can_driver::CanDriverCore> can_driver)
{
    init(robot_hwnh);

    ROS_DEBUG("JointsInterfaceCore::init - Start joint hardware interface");
    _robot.reset(new JointHardwareInterface(rootnh, robot_hwnh, ttl_driver, can_driver));

    ROS_DEBUG("JointsInterfaceCore::init - Create controller manager");
    _cm.reset(new controller_manager::ControllerManager(_robot.get(), _nh));

    ROS_DEBUG("JointsInterfaceCore::init - Starting ros control thread...");
    _control_loop_thread = std::thread(&JointsInterfaceCore::rosControlLoop, this);

    ROS_INFO("JointsInterfaceCore::init - Started");
    rootnh.setParam("/niryo_robot_joint_interface/initialized", true);
}

/**
 * @brief JointsInterfaceCore::~JointsInterfaceCore
 */
JointsInterfaceCore::~JointsInterfaceCore()
{
    if (_publish_learning_mode_thread.joinable())
        _publish_learning_mode_thread.join();

    if (_control_loop_thread.joinable())
        _control_loop_thread.join();
}

/**
 * @brief JointsInterfaceCore::init
 * @param nh
 * @return
 */
bool JointsInterfaceCore::init(ros::NodeHandle& nh)
{
    ROS_DEBUG("JointsInterfaceCore::init - Initializing parameters...");
    initParameters(nh);

    ROS_DEBUG("JointsInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("JointsInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("JointsInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief JointsInterfaceCore::initParameters
 * @param nh
 */
void JointsInterfaceCore::initParameters(ros::NodeHandle& nh)
{
    nh.getParam("ros_control_loop_frequency", _control_loop_frequency);
    nh.getParam("publish_learning_mode_frequency", _publish_learning_mode_frequency);

    ROS_DEBUG("JointsInterfaceCore::initParams - Ros control loop frequency %f",
              _control_loop_frequency);
    ROS_DEBUG("JointsInterfaceCore::initParams - Publish learning mode frequency : %f",
              _publish_learning_mode_frequency);
}

/**
 * @brief JointsInterfaceCore::startServices
 * @param nh
 */
void JointsInterfaceCore::startServices(ros::NodeHandle& nh)
{
    _calibrate_motors_server = nh.advertiseService("/niryo_robot/joints_interface/calibrate_motors",
                                                    &JointsInterfaceCore::_callbackCalibrateMotors, this);

    _request_new_calibration_server = nh.advertiseService("/niryo_robot/joints_interface/request_new_calibration",
                                                           &JointsInterfaceCore::_callbackRequestNewCalibration, this);

    _activate_learning_mode_server = nh.advertiseService("/niryo_robot/learning_mode/activate",
                                                          &JointsInterfaceCore::_callbackActivateLearningMode, this);

    _reset_controller_server = nh.advertiseService("/niryo_robot/joints_interface/steppers_reset_controller",
                                                    &JointsInterfaceCore::_callbackResetController, this);
}

/**
 * @brief JointsInterfaceCore::startPublishers
 * @param nh
 */
void JointsInterfaceCore::startPublishers(ros::NodeHandle& nh)
{
    _learning_mode_publisher = nh.advertise<std_msgs::Bool>("niryo_robot/learning_mode/state", 10);
    _publish_learning_mode_thread = std::thread(&JointsInterfaceCore::_publishLearningMode, this);
}

/**
 * @brief JointsInterfaceCore::startSubscribers
 * @param nh
 */
void JointsInterfaceCore::startSubscribers(ros::NodeHandle& nh)
{
    _trajectory_result_subscriber = nh.subscribe("/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory/result",
                                                  10, &JointsInterfaceCore::_callbackTrajectoryResult, this);
}

// *********************
//      commands
// *********************

/**
 * @brief JointsInterfaceCore::activateLearningMode
 * @param activate
 * @param ostatus
 * @param omessage
 */
void JointsInterfaceCore::activateLearningMode(bool activate, int &ostatus, std::string &omessage)
{
    omessage.clear();

    if (!_robot->isCalibrationInProgress())  // if not in calibration
    {
        if (_previous_state_learning_mode != activate)  // if state different
        {
            if (!activate)
            {
                _reset_controller = true;
                _robot->deactivateLearningMode();
                omessage = "Deactivating learning mode";
            }
            else
            {
                _robot->activateLearningMode();
                omessage = "Activating learning mode";
            }

            _previous_state_learning_mode = activate;
            // publish new state
            std_msgs::Bool msg;
            msg.data = _previous_state_learning_mode;
            _learning_mode_publisher.publish(msg);
        }
        else
        {
            omessage = activate ? "Learning mode already activated"
                                              : "Learning mode already deactivating";
        }
        ostatus = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    else
    {
        omessage = "Joints Interface Core - You can't activate/deactivate learning mode during motors calibration";
        ostatus = niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
    }
}

/**
 * @brief JointsInterfaceCore::sendMotorsParams
 */
void JointsInterfaceCore::sendMotorsParams()
{
    _robot->sendInitMotorsParams();
}

// *********************
//      control loop
// *********************

/**
 * @brief JointsInterfaceCore::rosControlLoop
 */
void JointsInterfaceCore::rosControlLoop()
{
    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time;
    ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);

    while (ros::ok())
    {
        if (_enable_control_loop)
        {
            current_time = ros::Time::now();
            elapsed_time = ros::Duration(current_time - last_time);
            last_time = current_time;
            _robot->read(current_time, elapsed_time);

            if (_reset_controller)
            {
                ROS_DEBUG("JointsInterfaceCore::rosControlLoop - Reset Controller");
                _robot->setCommandToCurrentPosition();
                _cm->update(ros::Time::now(), elapsed_time, true);
                _reset_controller = false;
            }
            else
            {
                _cm->update(ros::Time::now(), elapsed_time, false);
            }

            if (!_previous_state_learning_mode)
            {
                _robot->write(current_time, elapsed_time);
            }

            bool isFreqMet = control_loop_rate.sleep();
            ROS_DEBUG_COND(!isFreqMet,
                           "JointsInterfaceCore::rosControlLoop : freq not met : expected (%f s) vs actual (%f s)",
                           control_loop_rate.expectedCycleTime().toSec(),
                           control_loop_rate.cycleTime().toSec());
        }
    }
}

// ********************
//  Callbacks
// ********************

/**
 * @brief JointsInterfaceCore::_callbackResetController
 * @param req
 * @param res
 * @return
 *
 * Problem : for joint_trajectory_controller, position command has no discontinuity
 * --> If the stepper motor missed some steps, we need to start at current position (given by the encoder)
 *  So current real position != current trajectory command, we need a discontinuity in controller command.
 *  We have to reset controllers to start from sensor position.
 *  If we subscribe to trajectory /goal topic and reset when we receive a goal, it is often
 *  too late and trajectory will just be preempted.
 *
 *  So, in order to start from encoder position, we need to reset controller before we send the goal. If you
 *  send a new goal, be sure to send a message on /joints_interface/steppers_reset_controller BEFORE sending the goal.
 *
 *  This behavior is used in robot_commander node.
 */
bool JointsInterfaceCore::_callbackResetController(niryo_robot_msgs::Trigger::Request &/*req*/,
                                                   niryo_robot_msgs::Trigger::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackResetController - Reset Controller");

    // set current command to encoder position
    _robot->setCommandToCurrentPosition();

    // reset controllers to allow a discontinuity in position command
    _cm->update(ros::Time::now(), ros::Duration(0.00), true);
    _robot->synchronizeMotors(true);

    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Reset done";

    return (niryo_robot_msgs::CommandStatus::SUCCESS == res.status);
}

/**
 * @brief JointsInterfaceCore::_callbackCalibrateMotors
 * @param req
 * @param res
 * @return
 */
bool JointsInterfaceCore::_callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req,
                                                   niryo_robot_msgs::SetInt::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackTrajectoryResult - Received a calibration request");
    int calibration_mode = req.value;
    std::string result_message = "";
    _enable_control_loop = false;

    int result = _robot->calibrateJoints(calibration_mode, result_message);
    res.status = result;
    res.message = result_message;

    // special case here
    // we set flag learning_mode_on, but we don't activate from here
    // learning_mode should be activated in comm, AFTER motors have been calibrated
    // --> this fixes an issue where motors will jump back to a previous cmd after being calibrated
    if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
    {
        _previous_state_learning_mode = true;
        _robot->activateLearningMode();
        _enable_control_loop = true;
    }

    return true;
}

/**
 * @brief JointsInterfaceCore::_callbackRequestNewCalibration
 * @param req
 * @param res
 * @return
 */
bool JointsInterfaceCore::_callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &/*req*/,
                                                         niryo_robot_msgs::Trigger::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackRequestNewCalibration - New calibration requested");
    _previous_state_learning_mode = true;
    _robot->activateLearningMode();
    _robot->setNeedCalibration();

    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Joints Interface Core - New calibration request has been made, you will be requested to confirm it.";

    return (niryo_robot_msgs::CommandStatus::SUCCESS == res.status);
}

/**
 * @brief JointsInterfaceCore::_callbackActivateLearningMode
 * @param req
 * @param res
 * @return
 *
 * Deactivating learning mode (= activating motors) is possible only if motors are calibrated
 * Activating learning mode is also possible when waiting for calibration
 */
bool JointsInterfaceCore::_callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req,
                                                        niryo_robot_msgs::SetBool::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackActivateLearningMode - activate learning mode");

    activateLearningMode(req.value, res.status, res.message);
    return (niryo_robot_msgs::CommandStatus::SUCCESS == res.status);
}

/**
 * @brief JointsInterfaceCore::_callbackTrajectoryResult
 * @param msg
 */
void JointsInterfaceCore::_callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult &msg)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackTrajectoryResult - Received trajectory RESULT");
    _robot->synchronizeMotors(false);
}

/**
 * @brief JointsInterfaceCore::_publishLearningMode
 *  // cc maybe put this in ros control loop ?
 *
 */
void JointsInterfaceCore::_publishLearningMode()
{
    ROS_DEBUG("JointsInterfaceCore::_publishLearningMode");

    ros::Rate publish_learning_mode_rate = ros::Rate(_publish_learning_mode_frequency);
    while (ros::ok())
    {
        std_msgs::Bool msg;
        msg.data = _previous_state_learning_mode;
        _learning_mode_publisher.publish(msg);
        publish_learning_mode_rate.sleep();
    }
}

}  // namespace joints_interface
