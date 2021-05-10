/*
    stepper_driver_core.cpp
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

#include <functional>

#include "util/util_defs.hpp"
#include "stepper_driver/stepper_driver_core.hpp"
#include "model/conveyor_state.hpp"
#include "model/motor_type_enum.hpp"

using namespace std;
using namespace common::model;

namespace StepperDriver
{
    /**
     * @brief StepperDriverCore::StepperDriverCore
     */
    StepperDriverCore::StepperDriverCore() :
        _control_loop_flag(false),
        _debug_flag(false)
    {
        ROS_DEBUG("StepperDriverCore::StepperDriverCore - ctor");

        initParameters();
        init();
    }

    /**
     * @brief StepperDriverCore::~StepperDriverCore
     */
    StepperDriverCore::~StepperDriverCore()
    {
        if(_control_loop_thread.joinable())
            _control_loop_thread.join();

        if(_publish_command_thread.joinable())
            _publish_command_thread.join();
    }

    /**
     * @brief StepperDriverCore::init
     */
    void StepperDriverCore::init()
    {
        _stepper.reset(new StepperDriver());
        startControlLoop();
        initServices();
        initPublishers();

    }

    /**
     * @brief StepperDriverCore::initServices
     */
    void StepperDriverCore::initServices()
    {

    }

    /**
     * @brief StepperDriverCore::initPublishers
     */
    void StepperDriverCore::initPublishers()
    {
        _command_publisher = _nh.advertise<std_msgs::Int64MultiArray>("stepper_cmd", 1000);
        _publish_command_thread = std::thread(&StepperDriverCore::_publishCommand, this);
    }

    /**
     * @brief StepperDriverCore::initParameters
     */
    void StepperDriverCore::initParameters()
    {
        _control_loop_frequency = 0.0;
        double write_frequency = 0.0;
        double read_frequency = 0.0;

        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hardware_control_loop_frequency", _control_loop_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hw_write_frequency", write_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hw_read_frequency", read_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/publish_command_frequency", _publish_command_frequency);

        ROS_DEBUG("StepperDriverCore::initParameters - can_hardware_control_loop_frequency : %f", _control_loop_frequency);
        ROS_DEBUG("StepperDriverCore::initParameters - can_hw_write_frequency : %f", write_frequency);
        ROS_DEBUG("StepperDriverCore::initParameters - can_hw_read_frequency : %f", read_frequency);
        ROS_DEBUG("StepperDriverCore::initParameters - publish_command_frequency : %f", _publish_command_frequency);

        _delta_time_data_read = 1.0 / read_frequency;
        _delta_time_write = 1.0 / write_frequency;
    }

    //***************
    //  Commands
    //***************

    /**
     * @brief StepperDriverCore::scanMotorId
     * @param motor_to_find
     * @return
     */
    bool StepperDriverCore::scanMotorId(uint8_t motor_to_find)
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        return _stepper->scanMotorId(motor_to_find);
    }

    /**
     * @brief StepperDriverCore::startCalibration
     */
    void StepperDriverCore::startCalibration()
    {
        if(_stepper)
            _stepper->startCalibration();
    }


    // CC to reformat
    int StepperDriverCore::motorCmdReport(uint8_t motor_id, EMotorType /*motor_type*/)
    {
        int response = niryo_robot_msgs::CommandStatus::ABORTED;
        ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send torque on command on motor %d", motor_id);

        {
            lock_guard<mutex> lck(_control_loop_mutex);
            _stepper->sendTorqueOnCommand(motor_id, 1);
        }
        ros::Duration(0.2).sleep();

        int direction = (motor_id == 2)? -1 : 1;
        int old_position = _stepper->getStepperPose(motor_id);
        ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, old_position);

        {
            lock_guard<mutex> lck(_control_loop_mutex);
            ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send move command on motor %d", motor_id);
            _stepper->sendRelativeMoveCommand(motor_id, -1000*direction, 1500);
            ros::Duration(0.2).sleep();
        }
        ros::Duration(3).sleep();

        int new_position = _stepper->getStepperPose(motor_id);
        ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, new_position);
        if (abs(new_position - old_position) < 250)
        {
            ROS_WARN("StepperDriverCore::launchMotorsReport - Debug - Pose error on motor %d", motor_id);
            response = niryo_robot_msgs::CommandStatus::FAILURE;
        }
        {
            lock_guard<mutex> lck(_control_loop_mutex);

            ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send move command on motor %d", motor_id);
            _stepper->sendRelativeMoveCommand(motor_id, 1000*direction, 1000);
            ros::Duration(0.2).sleep();
        }
        ros::Duration(3).sleep();

        int new_position2 = _stepper->getStepperPose(motor_id);
        ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, new_position2);
        if (abs(new_position2 - new_position) < 250)
        {
            ROS_WARN("StepperDriverCore::launchMotorsReport - Debug - Pose error on motor %d", motor_id);
            response = niryo_robot_msgs::CommandStatus::FAILURE;
        }

        {
            lock_guard<mutex> lck(_control_loop_mutex);
            ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send torque off command on motor %d", motor_id);
            _stepper->sendTorqueOnCommand(motor_id, 0);
            ros::Duration(0.2).sleep();
        }
        if (!_debug_flag)
        {
            ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Debug motor aborted");
            response = niryo_robot_msgs::CommandStatus::ABORTED;
        }

        return response;
    }


    /**
     * @brief StepperDriverCore::launchMotorsReport
     * @return
     */
    int StepperDriverCore::launchMotorsReport()
    {
        int response = niryo_robot_msgs::CommandStatus::ABORTED;
        if (_debug_flag)
        {
            ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Start Stepper Motor Report");
            ros::Duration(0.5).sleep();
            if (CAN_OK == _stepper->scanAndCheck())
            {
                if (_debug_flag)
                {
                    for (auto const& state: _stepper->getMotorsStates())
                    {
                        response = motorCmdReport(state.getId());
                    }
                }
                else
                {
                    ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Debug motor aborted");
                    response = niryo_robot_msgs::CommandStatus::ABORTED;
                }
            }
            else
            {
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            }
        }
        else
        {
            ROS_ERROR("StepperDriverCore::launchMotorsReport - Debug - Debug mode not enabled");
        }

        return response;
    }

    //****************
    //  Control Loop
    //****************

    /**
     * @brief StepperDriverCore::startControlLoop
     */
    void StepperDriverCore::startControlLoop()
    {
        resetHardwareControlLoopRates();
        if (!_control_loop_flag)
        {
            ROS_DEBUG("StepperDriverCore::startControlLoop - Start control loop thread");
            _control_loop_flag = true;
            _control_loop_thread = thread(&StepperDriverCore::controlLoop, this);
        }
    }

    /**
     * @brief StepperDriverCore::resetHardwareControlLoopRates
     */
    void StepperDriverCore::resetHardwareControlLoopRates()
    {
        ROS_DEBUG("StepperDriverCore::resetHardwareControlLoopRates - Reset control loop rates");
        double now = ros::Time::now().toSec();
        _time_hw_data_last_write = now;
        _time_hw_data_last_read = now;

        _time_check_connection_last_read = now;
    }

    /**
     * @brief StepperDriverCore::activeDebugMode
     * @param mode
     */
    void StepperDriverCore::activeDebugMode(bool mode)
    {
        ROS_INFO("StepperDriverCore::activeDebugMode - Activate debug mode for dynamixel driver core: %d", mode);
        _debug_flag = mode;

        if (!mode)
        {
            lock_guard<mutex> lck(_control_loop_mutex);
            _stepper->sendTorqueOnCommand(2, 0);
            ros::Duration(0.2).sleep();
            _stepper->sendTorqueOnCommand(3, 0);
        }
    }

    /**
     * @brief StepperDriverCore::controlLoop
     */
    void StepperDriverCore::controlLoop()
    {
        ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);
        resetHardwareControlLoopRates();
        while (ros::ok())
        {
            if (_control_loop_flag)
            {
                lock_guard<mutex> lck(_control_loop_mutex);

                // cc calibration results ???
                _stepper->readMotorsState();

                if (_stepper->isConnectionOk())
                {
                    if (ros::Time::now().toSec() - _time_hw_data_last_write >= _delta_time_write)
                    {
                        _time_hw_data_last_write = ros::Time::now().toSec();
                        _executeCommand();
                    }
                }

                bool isFreqMet = control_loop_rate.sleep();
                ROS_WARN_COND(!isFreqMet, "StepperDriverCore::rosControlLoop : freq not met : expected (%f s) vs actual (%f s)", control_loop_rate.expectedCycleTime().toSec(), control_loop_rate.cycleTime().toSec());
                ROS_DEBUG_COND(isFreqMet, "StepperDriverCore::rosControlLoop : freq met : expected (%f s) vs actual (%f s)", control_loop_rate.expectedCycleTime().toSec(), control_loop_rate.cycleTime().toSec());
            }
            else
            {
                ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                resetHardwareControlLoopRates();
            }
        }
    }

    /**
     * @brief StepperDriverCore::_executeCommand
     */
    void StepperDriverCore::_executeCommand()
    {
        bool need_sleep = false;
        /*if (_joint_trajectory_cmd.isValid())
        {
            //we need a mutex here to ensure this data is not being modify
            //by another thread during its usage
            lock_guard<mutex> lck(_joint_trajectory_mutex);
            _stepper->readSynchronizeCommand(_joint_trajectory_cmd);
            _joint_trajectory_cmd.reset();

            need_sleep = true;
        }*/
        if (_joint_trajectory_cmd_vec.size() != 0)
        {
            _stepper->executeJointTrajectoryCmd(_joint_trajectory_cmd_vec);
            need_sleep = true;
        }

        if (!_stepper_single_cmds.empty())
        {
            //as we use a queue, we don't need a mutex
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _stepper->readSingleCommand(_stepper_single_cmds.front());
            _stepper_single_cmds.pop();
            need_sleep = true;
        }
        if (!_conveyor_cmds.empty())
        {
            //as we use a queue, we don't need a mutex
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _stepper->readSingleCommand(_conveyor_cmds.front());
            _conveyor_cmds.pop();
        }
    }

    //*************
    //  Setters
    //*************

    /**
     * @brief StepperDriverCore::setConveyor
     * @param motor_id
     * @return
     */
    int StepperDriverCore::setConveyor(uint8_t motor_id)
    {
        int result = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;

        lock_guard<mutex> lck(_control_loop_mutex);

        //try to find motor id 6 (default motor id for conveyor
        if (_stepper->scanMotorId(6))
        {
            if (CAN_OK == _stepper->sendUpdateConveyorId(6, motor_id)) {
                //add stepper as a new conveyor
                _stepper->addMotor(motor_id, true);
                result = niryo_robot_msgs::CommandStatus::SUCCESS;
            }
            else {
                ROS_ERROR("StepperDriverCore::setConveyor : unable to change conveyor ID");
                result = niryo_robot_msgs::CommandStatus::CAN_WRITE_ERROR;
            }
        }
        else
        {
            ROS_WARN("StepperDriverCore::setConveyor - No conveyor found");
        }

        return result;
    }

    /**
     * @brief StepperDriverCore::unsetConveyor
     * @param motor_id
     */
    void StepperDriverCore::unsetConveyor(uint8_t motor_id)
    {
        lock_guard<mutex> lck(_control_loop_mutex);

        ROS_DEBUG("StepperDriverCore::unsetConveyor - UnsetEndEffector: id %d", motor_id);

        if(_stepper->sendUpdateConveyorId(motor_id, 6))
            _stepper->removeMotor(motor_id);
        else
            ROS_ERROR("StepperDriverCore::unsetConveyor : unable to change conveyor ID");
    }

    /**
     * @brief StepperDriverCore::clearSingleCommandQueue
     */
    void StepperDriverCore::clearSingleCommandQueue()
    {
        while(!_stepper_single_cmds.empty())
            _stepper_single_cmds.pop();
    }

    /**
     * @brief StepperDriverCore::clearConveyorCommandQueue
     */
    void StepperDriverCore::clearConveyorCommandQueue()
    {
        while(!_conveyor_cmds.empty())
            _conveyor_cmds.pop();
    }

    void StepperDriverCore::setTrajectoryControllerCommands(std::vector<int32_t> &cmd)
    {
        _joint_trajectory_cmd_vec = cmd;
    }

    /**
     * @brief StepperDriverCore::setTrajectoryControllerCommands
     * @param cmd
     */
    void StepperDriverCore::setSyncCommand(const SynchronizeStepperMotorCmd& cmd)
    {

        if(cmd.isValid()) {
            //keep position cmd apart
            if(cmd.getType() == EStepperCommandType::CMD_TYPE_POSITION) {
                lock_guard<mutex> lck(_joint_trajectory_mutex);

                _joint_trajectory_cmd = cmd;
            }
        }
        else {
            ROS_WARN("StepperDriverCore::setSyncCommand : Invalid command");
        }
    }

    /**
     * @brief StepperDriverCore::addSingleCommandToQueue
     * @param cmd
     */
    void StepperDriverCore::addSingleCommandToQueue(const StepperMotorCmd &cmd)
    {
        ROS_DEBUG("StepperDriverCore::addSingleCommandToQueue - %s", cmd.str().c_str());

        if(cmd.isValid()) {
            if(cmd.getType() == EStepperCommandType::CMD_TYPE_CONVEYOR) {//keep position cmd apart
                if(_conveyor_cmds.size() > QUEUE_OVERFLOW)
                    ROS_WARN("StepperDriverCore::addCommandToQueue: Cmd queue overflow ! %lu", _conveyor_cmds.size());
                else {
                    _conveyor_cmds.push(cmd);
                }
            }
            else {
                if(_stepper_single_cmds.size() > QUEUE_OVERFLOW)
                    ROS_WARN("StepperDriverCore::addCommandToQueue: Cmd queue overflow ! %lu", _stepper_single_cmds.size());
                else {
                    _stepper_single_cmds.push(cmd);
                }
            }
        }
    }

    /**
     * @brief StepperDriverCore::addSingleCommandToQueue
     * @param cmd
     */
    void StepperDriverCore::addSingleCommandToQueue(const vector<StepperMotorCmd> &cmd)
    {
        for(auto&& c : cmd)
            addSingleCommandToQueue(c);
    }

    //********************
    //  getters
    //********************

    int32_t StepperDriverCore::getCalibrationResult(uint8_t id) const
    {
        return _stepper->getCalibrationResult(id);
    }

    /**
     * @brief StepperDriverCore::getHwStatus
     * @return
     */
    stepper_driver::StepperArrayMotorHardwareStatus StepperDriverCore::getHwStatus() const
    {
        stepper_driver::StepperMotorHardwareStatus data;
        stepper_driver::StepperArrayMotorHardwareStatus hw_state;

        for (auto const& stepperState : _stepper->getMotorsStates())
        {
            data.motor_identity.motor_id = stepperState.getId();
            data.motor_identity.motor_type = static_cast<uint8_t>(stepperState.getType());
            data.temperature = static_cast<int32_t>(stepperState.getTemperatureState());
            data.error = static_cast<int32_t>(stepperState.getHardwareErrorState());
            data.firmware_version = stepperState.getFirmwareVersion();
            hw_state.motors_hw_status.push_back(data);
        }
        return hw_state;
    }

    /**
     * @brief StepperDriverCore::getBusState
     * @return
     */
    niryo_robot_msgs::BusState StepperDriverCore::getBusState() const
    {
        niryo_robot_msgs::BusState can_bust_state;

        string error;
        bool connection;
        vector<uint8_t> motor_id;

        _stepper->getBusState(connection, motor_id, error);
        can_bust_state.connection_status = connection;
        can_bust_state.motor_id_connected = motor_id;
        can_bust_state.error = error;
        return can_bust_state;
    }

    //*******************
    //    Callbacks     *
    //*******************

    /**
     * @brief StepperDriverCore::_publishCommand
     */
    void StepperDriverCore::_publishCommand()
    {
        ros::Rate publish_command_rate = ros::Rate(_publish_command_frequency);
        std_msgs::Int64MultiArray msg;

        while (ros::ok())
        {
            if(_joint_trajectory_cmd_vec.size() == 3) {
                msg.data.emplace_back(_joint_trajectory_cmd_vec.at(0));
                msg.data.emplace_back(_joint_trajectory_cmd_vec.at(1));
                msg.data.emplace_back(_joint_trajectory_cmd_vec.at(2));
                _command_publisher.publish(msg);

                publish_command_rate.sleep();
            }
        }
    }


} // namespace StepperDriver
