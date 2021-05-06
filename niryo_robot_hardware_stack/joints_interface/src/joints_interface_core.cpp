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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "joints_interface/joints_interface_core.hpp"

JointsInterfaceCore::JointsInterfaceCore(
    boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> &dynamixel,
    boost::shared_ptr<StepperDriver::StepperDriverCore> &stepper)
    : _dynamixel(dynamixel), _stepper(stepper)
{
    init();
}

void JointsInterfaceCore::init()
{
    initParams();
    startServices();
    startSubscribers();
    ROS_DEBUG("Joints Interface Core - Start joint hardware interface");
    _robot.reset(new JointHardwareInterface(_dynamixel, _stepper));

    ROS_DEBUG("Joints Interface Core - Create controller manager");
    _cm.reset(new controller_manager::ControllerManager(_robot.get(), _nh));

    _enable_control_loop = true;
    _previous_state_learning_mode = true;
    _reset_controller = false;

    ROS_DEBUG("Joints Interface Core - Starting ros control thread...");
    _ros_control_thread.reset(new std::thread(&JointsInterfaceCore::rosControlLoop, this));

    ROS_INFO("Joints Interface Core - Started");
    _nh.setParam("/niryo_robot_joint_interface/initialized", true);
}

void JointsInterfaceCore::initParams()
{
    _nh.getParam("/niryo_robot_hardware_interface/ros_control_loop_frequency", _ros_control_frequency);
    _nh.getParam("/niryo_robot_hardware_interface/publish_learning_mode_frequency", _publish_learning_mode_frequency);
    ROS_DEBUG("Joints Interface Core - Ros control loop frequency %f", _ros_control_frequency);
    ROS_DEBUG("Joints Interface Core - Publish learning mode frequency : %f", _publish_learning_mode_frequency);
}

void JointsInterfaceCore::startServices()
{
    _calibrate_motors_server = _nh.advertiseService("/niryo_robot/joints_interface/calibrate_motors", &JointsInterfaceCore::_callbackCalibrateMotors, this);
    _request_new_calibration_server = _nh.advertiseService("/niryo_robot/joints_interface/request_new_calibration", &JointsInterfaceCore::_callbackRequestNewCalibration, this);
    _activate_learning_mode_server = _nh.advertiseService("niryo_robot/learning_mode/activate", &JointsInterfaceCore::_callbackActivateLearningMode, this);
    _reset_controller_server = _nh.advertiseService("/niryo_robot/joints_interface/steppers_reset_controller", &JointsInterfaceCore::_callbackResetController, this);
}

void JointsInterfaceCore::startSubscribers()
{
    _trajectory_result_subscriber = _nh.subscribe("/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory/result",
                                                  10, &JointsInterfaceCore::_callbackTrajectoryResult, this);
    _learning_mode_publisher = _nh.advertise<std_msgs::Bool>("niryo_robot/learning_mode/state", 10);
    _publish_learning_mode_thread.reset(new std::thread(boost::bind(&JointsInterfaceCore::_publishLearningMode, this)));
}

void JointsInterfaceCore::rosControlLoop()
{
    ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time;
    ros::Rate control_loop_rate = ros::Rate(_ros_control_frequency);
    while (ros::ok())
    {
        if (_enable_control_loop)
        {
            _robot->read();
            current_time = ros::Time::now();
            elapsed_time = ros::Duration(current_time - last_time);
            last_time = current_time;

            if (_reset_controller)
            {
                ROS_DEBUG("Joints Interface Core - Reset Controller");
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
                _robot->write();
            }
            control_loop_rate.sleep();
        }
    }
}

/*
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
*/
bool JointsInterfaceCore::_callbackResetController(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res)
{
    (void)req;
    ROS_DEBUG("Joints Interface Core - Reset Controller");
    _robot->setCommandToCurrentPosition();                    // set current command to encoder position
    _cm->update(ros::Time::now(), ros::Duration(0.00), true); // reset controllers to allow a discontinuity in position command
    _robot->synchronizeMotors(true);

    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Reset done";
    return true;
}

void JointsInterfaceCore::_callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult &msg)
{
    (void)msg;
    ROS_DEBUG("Joints Interface Core - Received trajectory RESULT");
    _robot->synchronizeMotors(false);
}

bool JointsInterfaceCore::_callbackCalibrateMotors(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res)
{
    ROS_DEBUG("Joints Interface Core - Received a calibration request");
    int calibration_mode = req.value;
    std::string result_message = "";
    _enable_control_loop = false;

    int result = _robot->calibrateJoints(calibration_mode, result_message);
    res.status = result;
    res.message = result_message;

    // // special case here
    // // we set flag learning_mode_on, but we don't activate from here
    // // learning_mode should be activated in comm, AFTER motors have been calibrated
    // // --> this fixes an issue where motors will jump back to a previous cmd after being calibrated
    if (result == niryo_robot_msgs::CommandStatus::SUCCESS)
    {
        _previous_state_learning_mode = true;
        _robot->activateLearningMode();
        _enable_control_loop = true;
    }
    return true;
}

bool JointsInterfaceCore::_callbackRequestNewCalibration(niryo_robot_msgs::Trigger::Request &req, niryo_robot_msgs::Trigger::Response &res)
{
    (void)req;
    ROS_DEBUG("Joints Interface Core - New calibration requested");
    _previous_state_learning_mode = true;
    _robot->activateLearningMode();
    _robot->newCalibration();
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    res.message = "Joints Interface Core - New calibration request has been made, you will be requested to confirm it.";
    return true;
}

/*
 * Deactivating learning mode (= activating motors) is possible only if motors are calibrated
 * Activating learning mode is also possible when waiting for calibration
 */
bool JointsInterfaceCore::_callbackActivateLearningMode(niryo_robot_msgs::SetBool::Request &req, niryo_robot_msgs::SetBool::Response &res)
{
    activateLearningMode(req.value, res.status, res.message);
    return true;
}

void JointsInterfaceCore::activateLearningMode(bool learning_mode_on, int &resp_status, std::string &resp_message)
{
    std_msgs::Bool msg;
    resp_message = " ";

    if (_robot->isCalibrationInProgress())
    {
        resp_status = niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
        resp_message = "Joints Interface Core - You can't activate/deactivate learning mode during motors calibration";
        return;
    }

    if (_previous_state_learning_mode != learning_mode_on)
    {
        if (!learning_mode_on)
        {
            _reset_controller = true;
            _robot->deactivateLearningMode();
            resp_message = "Deactivating learning mode";
        }
        else
        {
            _robot->activateLearningMode();
            resp_message = "Activating learning mode";
        }
        _previous_state_learning_mode = learning_mode_on;
        msg.data = _previous_state_learning_mode;
        _learning_mode_publisher.publish(msg);
    }
    else
    {
        resp_message = (learning_mode_on) ? "Learning mode already activated" : "Learning mode already deactivating";
    }
    resp_status = niryo_robot_msgs::CommandStatus::SUCCESS;
}

bool JointsInterfaceCore::getFreeDriveMode()
{
    return _previous_state_learning_mode;
}

void JointsInterfaceCore::getCalibrationState(boost::shared_ptr<bool> &need_calibration, boost::shared_ptr<bool> &calibration_in_progress)
{
    (*need_calibration) = _robot->needCalibration();
    (*calibration_in_progress) = _robot->isCalibrationInProgress();
}

std::vector<JointState> &JointsInterfaceCore::getJointsState()
{
    return _robot->getJointsState();
}

void JointsInterfaceCore::_publishLearningMode()
{
    ros::Rate publish_learning_mode_rate = ros::Rate(_publish_learning_mode_frequency);
    while (ros::ok())
    {
        std_msgs::Bool msg;
        msg.data = _previous_state_learning_mode;
        _learning_mode_publisher.publish(msg);
        publish_learning_mode_rate.sleep();
    }
}

std::string JointsInterfaceCore::jointIdToJointName(int id, uint8_t motor_type)
{
    return _robot->jointIdToJointName(id, motor_type);
}


void JointsInterfaceCore::sendMotorsParams()
{
    _robot->sendInitMotorsParams();
}
