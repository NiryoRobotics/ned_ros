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
#include <memory>
#include <string>
#include <utility>

#include "common/util/util_defs.hpp"
#include "joints_interface/joints_interface_core.hpp"
#include "joints_interface/FactoryCalibration.h"

namespace joints_interface
{

/**
 * @brief JointsInterfaceCore::JointsInterfaceCore
 * @param rootnh
 * @param robot_hwnh
 * @param ttl_interface
 * @param can_interface
 */
JointsInterfaceCore::JointsInterfaceCore(ros::NodeHandle &rootnh, ros::NodeHandle &robot_hwnh, std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                                         std::shared_ptr<can_driver::CanInterfaceCore> can_interface)
    : _joint_controller_name("/niryo_robot_follow_joint_trajectory_controller"), _ttl_interface(std::move(ttl_interface)), _can_interface(std::move(can_interface))
{
    init(robot_hwnh);

    ROS_DEBUG("JointsInterfaceCore::init - Start joint hardware interface");
    _robot.reset(new JointHardwareInterface(rootnh, robot_hwnh, _ttl_interface, _can_interface));

    ROS_DEBUG("JointsInterfaceCore::init - Create controller manager");
    _cm.reset(new controller_manager::ControllerManager(_robot.get(), _nh));

    ROS_DEBUG("JointsInterfaceCore::init - Starting ros control thread...");
    _control_loop_thread = std::thread(&JointsInterfaceCore::rosControlLoop, this);

    ROS_INFO("JointsInterfaceCore::init - Started");
    rootnh.setParam("/niryo_robot_joints_interface/initialized", true);
}

/**
 * @brief JointsInterfaceCore::~JointsInterfaceCore
 */
JointsInterfaceCore::~JointsInterfaceCore()
{
    if (_control_loop_thread.joinable())
        _control_loop_thread.join();
}

/**
 * @brief JointsInterfaceCore::init
 * @param nh
 * @return
 */
bool JointsInterfaceCore::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("JointsInterfaceCore::init - Initializing parameters...");
    initParameters(nh);

    // in ned2, mode is not learning mode in the first time
    _previous_state_learning_mode = (("ned2" != _hardware_version && "ned3pro" != _hardware_version) && !_simulation_mode);

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
void JointsInterfaceCore::initParameters(ros::NodeHandle &nh)
{
    double control_loop_frequency{1.0};

    nh.getParam("ros_control_loop_frequency", control_loop_frequency);
    nh.getParam("/niryo_robot_hardware_interface/hardware_version", _hardware_version);
    nh.getParam("simulation_mode", _simulation_mode);
    ROS_DEBUG("JointsInterfaceCore::initParams - Ros control loop frequency %f", control_loop_frequency);
    ROS_DEBUG("Joint Hardware Interface - hardware_version %s", _hardware_version.c_str());

    _control_loop_rate = ros::Rate(control_loop_frequency);
}

/**
 * @brief JointsInterfaceCore::startServices
 * @param nh
 */
void JointsInterfaceCore::startServices(ros::NodeHandle &nh)
{
    _calibrate_motors_server = nh.advertiseService("/niryo_robot/joints_interface/calibrate_motors", &JointsInterfaceCore::_callbackCalibrateMotors, this);

    _request_new_calibration_server = nh.advertiseService("/niryo_robot/joints_interface/request_new_calibration", &JointsInterfaceCore::_callbackRequestNewCalibration, this);

    _activate_learning_mode_server = nh.advertiseService("/niryo_robot/learning_mode/activate", &JointsInterfaceCore::_callbackActivateLearningMode, this);

    _reset_controller_server = nh.advertiseService("/niryo_robot/joints_interface/steppers_reset_controller", &JointsInterfaceCore::_callbackResetController, this);

    _factory_calibrate_motors_server = nh.advertiseService("/niryo_robot/joints_interface/factory_calibrate_motors", &JointsInterfaceCore::_callbackFactoryCalibrateMotors, this);
}

/**
 * @brief JointsInterfaceCore::startPublishers
 * @param nh
 */
void JointsInterfaceCore::startPublishers(ros::NodeHandle &nh)
{
    _learning_mode_publisher = nh.advertise<std_msgs::Bool>("/niryo_robot/learning_mode/state", 10, true);
    _publishLearningMode();
}

/**
 * @brief JointsInterfaceCore::startSubscribers
 * @param nh
 */
void JointsInterfaceCore::startSubscribers(ros::NodeHandle &nh)
{
    _trajectory_result_subscriber = nh.subscribe(_joint_controller_name + "/follow_joint_trajectory/result", 10, &JointsInterfaceCore::_callbackTrajectoryResult, this);
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
        if (activate != _previous_state_learning_mode)  // if state different
        {
            _robot->activateLearningMode(activate);

            if (!activate)
            {
                _reset_controller = true;
                omessage = "Deactivating learning mode";
            }
            else
            {
                omessage = "Activating learning mode";
            }

            _previous_state_learning_mode = activate;
            _publishLearningMode();
        }
        else
        {
            omessage = activate ? "Learning mode already activated" : "Learning mode already deactivating";
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
 * @brief JointsInterfaceCore::rebootAll
 * @param torque_on
 * @return
 */
bool JointsInterfaceCore::rebootAll(bool torque_on)
{
    int result = niryo_robot_msgs::CommandStatus::FAILURE;
    std::string result_message;
    activateLearningMode((("ned2" != _hardware_version && "ned3pro" != _hardware_version) && !_simulation_mode), result, result_message);
    _previous_state_learning_mode = (("ned2" != _hardware_version && "ned3pro" != _hardware_version) && !_simulation_mode);

    _enable_control_loop = false;
    bool res = _robot->rebootAll(torque_on);

    // reset ros controller to update command to be equal with position actual to avoid movement undefined
    _enable_control_loop = true;
    _reset_controller = true;

    // need calibration after reset joints (ned2 only)
    if ("ned2" == _hardware_version && !_simulation_mode)
        _robot->setNeedCalibration();

    return res;
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

    while (ros::ok())
    {
        if (_enable_control_loop && !_estop_flag)
        {
            _robot->read(current_time, elapsed_time);

            // check if a collision is occurred, reset controller to stop robot
            if (_ttl_interface->getCollisionStatus() && !_previous_state_learning_mode && !_robot->needCalibration())
            {
                resetController();
                ROS_WARN_THROTTLE(2.0, "JointsInterfaceCore: collision detected by End Effector");
            }
            else
                _lock_write_cnt = -1;

            current_time = ros::Time::now();
            elapsed_time = ros::Duration(current_time - last_time);
            last_time = current_time;

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

            // we just use cmd from moveit only in torque on + calibration finished
            if (!_previous_state_learning_mode && _lock_write_cnt == -1 && !_robot->needCalibration())
            {
                _robot->write(current_time, elapsed_time);
            }
            else if (_lock_write_cnt > 0)
            {
                _lock_write_cnt--;
            }
            else if (_lock_write_cnt == 0)
            {
                _lock_write_cnt = -1;
                _reset_controller = true;
            }
            // bool isFreqMet = _control_loop_rate.sleep();
            // ROS_DEBUG_COND(!isFreqMet, "JointsInterfaceCore::rosControlLoop : freq not met : expected (%f s) vs actual (%f s)", _control_loop_rate.expectedCycleTime().toSec(),
            //                _control_loop_rate.cycleTime().toSec());
        }
    }
}

/**
 * @brief JointsInterfaceCore::resetController
 */
void JointsInterfaceCore::resetController()
{
    _robot->setCommandToCurrentPosition();
    // set pos and command equal
    if ("ned2" == _hardware_version || "ned3pro" == _hardware_version)
    {
        _robot->write(ros::Time::now(), ros::Duration(0.0));
        _lock_write_cnt = 150;
        _cm->update(ros::Time::now(), ros::Duration(0.0), true);
    }
    else if (_hardware_version == "ned" || _hardware_version == "one")
    {
        _cm->update(ros::Time::now(), ros::Duration(0.0), true);
        _robot->synchronizeMotors(true);
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
 * NED && ONE
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
 *
 * NED2
 * Callback call when we have to make a discontinuity of position command in ros control (like if we have a collision, we have
 * to update the position command to be equal with the position actual to avoid joints continue moving)
 * This way is a treat to block the write function of robot and wait a little bit and then reset controller to make ros controller
 * update cmd. If ros controller is reset rapidly, The pid return unvalid position.
 */
bool JointsInterfaceCore::_callbackResetController(niryo_robot_msgs::Trigger::Request & /*req*/, niryo_robot_msgs::Trigger::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackResetController - Reset Controller");

    // update position of joints
    _robot->read(ros::Time::now(), ros::Duration(0.0));

    resetController();

    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Reset done";

    return true;
}

/**
 * @brief JointsInterfaceCore::_callbackCalibrateMotors
 * @param req
 * @param res
 * @return
 */
bool JointsInterfaceCore::_callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackCalibrateMotors - Received a calibration request");
    int calibration_mode = req.value;
    std::string result_message;
    _enable_control_loop = false;
    int result = niryo_robot_msgs::CommandStatus::FAILURE;
    // first activate learning mode for ned and one
    activateLearningMode((("ned2" != _hardware_version && "ned3pro" != _hardware_version) && !_simulation_mode), result, result_message);

    result = _robot->calibrateJoints(calibration_mode, result_message);
    res.status = result;
    res.message = result_message;

    if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
    {
        // we have to reset controller to avoid ros controller set command to the previous position
        // before the calibration
        _reset_controller = true;
        _previous_state_learning_mode = (("ned2" != _hardware_version && "ned3pro" != _hardware_version) && !_simulation_mode);
        _enable_control_loop = true;
    }

    return true;
}

/**
 * @brief JointsInterfaceCore::_callbackFactoryCalibrateMotors
 * @param req
 * @param res
 * @return
 */
bool JointsInterfaceCore::_callbackFactoryCalibrateMotors(FactoryCalibration::Request &req, FactoryCalibration::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackFactoryCalibrateMotors - Received a calibration request");

    if ("ned3pro" != _hardware_version)
    {
        res.status = niryo_robot_msgs::CommandStatus::FAILURE;
        res.message = "JointHardwareInterface::factoryCalibrateJoints - Can't be calibratd wrong hardware version: " + _hardware_version;
        return true;
    }

    _enable_control_loop = false;

    res.status = _robot->factoryCalibrateJoints(req.command, req.ids, res.message);
    if (niryo_robot_msgs::CommandStatus::SUCCESS == res.status)
    {
        // we have to reset controller to avoid ros controller set command to the previous position
        // before the calibration
        _reset_controller = true;
        _previous_state_learning_mode = (!_simulation_mode);
    }

    _enable_control_loop = true;

    return true;
}

/**
 * @brief JointsInterfaceCore::_callbackRequestNewCalibration
 * @param req
 * @param res
 * @return
 */
bool JointsInterfaceCore::_callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request & /*req*/, niryo_robot_msgs::Trigger::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackRequestNewCalibration - New calibration requested");
    std::string result_message;

    _robot->setNeedCalibration();

    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Joints Interface Core - New calibration request has been made, you will be requested to confirm it.";

    return true;
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
bool JointsInterfaceCore::_callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req, niryo_robot_msgs::SetBool::Response &res)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackActivateLearningMode - activate learning mode");

    activateLearningMode(req.value, res.status, res.message);
    return true;
}

/**
 * @brief JointsInterfaceCore::_callbackTrajectoryResult
 * @param msg
 */
void JointsInterfaceCore::_callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult & /*msg*/)
{
    ROS_DEBUG("JointsInterfaceCore::_callbackTrajectoryResult - Received trajectory RESULT");
    _robot->synchronizeMotors(false);
}

/**
 * @brief JointsInterfaceCore::_publishLearningMode
 */
void JointsInterfaceCore::_publishLearningMode()
{
    ROS_DEBUG("JointsInterfaceCore::_publishLearningMode");

    std_msgs::Bool msg;
    msg.data = _previous_state_learning_mode;
    _learning_mode_publisher.publish(msg);
}

}  // namespace joints_interface
