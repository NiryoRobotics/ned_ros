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

using namespace common::model;

namespace StepperDriver
{
    StepperDriverCore::StepperDriverCore()
    {
        ROS_DEBUG("StepperDriverCore::StepperDriverCore - ctor");

        initParameters();

        init();
    }

    StepperDriverCore::~StepperDriverCore()
    {
        if(_control_loop_thread.joinable())
            _control_loop_thread.join();
    }


    void StepperDriverCore::init()
    {
        _control_loop_flag = false;
        _debug_flag = false;
        _joint_trajectory_controller_cmd.setType(EStepperCommandType::CMD_TYPE_POSITION);
        _stepper.reset(new StepperDriver());
        _stepper->scanAndCheck();
        startControlLoop();

        cmd_pub = _nh.advertise<std_msgs::Int64MultiArray>("stepper_cmd", 1000);
    }

    void StepperDriverCore::initParameters()
    {
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hardware_control_loop_frequency", _control_loop_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hw_write_frequency", _write_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hw_check_connection_frequency", _check_connection_frequency);

        ROS_DEBUG("StepperDriverCore::initParameters - can_hardware_control_loop_frequency : %f", _control_loop_frequency);
        ROS_DEBUG("StepperDriverCore::initParameters - can_hardware_write_frequency : %f", _write_frequency);
        ROS_DEBUG("StepperDriverCore::initParameters - can_hardware_check_connection_frequency : %f", _check_connection_frequency);
    }

    /**
     * @brief StepperDriverCore::clearCommandQueue
     */
    void StepperDriverCore::clearCommandQueue()
    {
        while(!_stepper_cmds.empty())
            _stepper_cmds.pop();
    }

    /**
     * @brief StepperDriverCore::clearConveyorCommandQueue
     */
    void StepperDriverCore::clearConveyorCommandQueue()
    {
        while(!_conveyor_cmds.empty())
            _conveyor_cmds.pop();
    }

    /**
     * @brief StepperDriverCore::setTrajectoryControllerCommands
     * @param cmd
     */
    void StepperDriverCore::setSyncCommand(const SynchronizeStepperMotorCmd& cmd)
    {
        _joint_trajectory_controller_cmd = cmd;
    }

    /**
     * @brief StepperDriverCore::addCommandToQueue
     * @param cmd
     */
    void StepperDriverCore::addCommandToQueue(const StepperMotorCmd &cmd)
    {
        ROS_DEBUG("StepperDriverCore::addSingleCommandToQueue - %s", cmd.str().c_str());

        if(cmd.isValid()) {
            if(cmd.getType() == EStepperCommandType::CMD_TYPE_CONVEYOR) {//keep position cmd apart
                if(_conveyor_cmds.size() > 20)
                    ROS_WARN("StepperDriverCore::addCommandToQueue: Cmd queue overflow ! %lu", _conveyor_cmds.size());
                else {
                    _conveyor_cmds.push(cmd);
                }
            }
            else {
                if(_stepper_cmds.size() > 20)
                    ROS_WARN("StepperDriverCore::addCommandToQueue: Cmd queue overflow ! %lu", _stepper_cmds.size());
                else {
                    _stepper_cmds.push(cmd);
                }
            }
        }
    }

    /**
     * @brief StepperDriverCore::addCommandToQueue
     * @param cmd
     */
    void StepperDriverCore::addCommandToQueue(const std::vector<StepperMotorCmd> &cmd)
    {
        for(auto&& c : cmd)
            addCommandToQueue(c);
    }

    void StepperDriverCore::resetHardwareControlLoopRates()
    {
        ROS_DEBUG_THROTTLE(1,"StepperDriverCore::resetHardwareControlLoopRates - Reset control loop rates");
        double now = ros::Time::now().toSec();
        _time_hw_last_write = now;
        _time_hw_last_check_connection = now;
    }

    void StepperDriverCore::activeDebugMode(bool mode)
    {
        ROS_INFO("StepperDriverCore::activeDebugMode - Activate debug mode for dynamixel driver core: %d", mode);
        _debug_flag = mode;

        if (!mode)
        {
            std::lock_guard<std::mutex> lck(_control_loop_mutex);
            _stepper->sendTorqueOnCommand(2, 0);
            ros::Duration(0.2).sleep();
            _stepper->sendTorqueOnCommand(3, 0);
        }
    }

    int StepperDriverCore::motorReport(uint8_t motor_id)
    {
        int response = niryo_robot_msgs::CommandStatus::SUCCESS;
        if (_debug_flag)
        {
            bool motor_found;
            std::lock_guard<std::mutex> lck(_control_loop_mutex);
            ros::Duration(0.1).sleep();

            motor_found = _stepper->scanMotorId(motor_id);
            if (!motor_found)
            {
                ROS_ERROR("StepperDriverCore::motorReport - Stepper Motor %d not found", motor_id);
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            else
            {
                ROS_INFO("StepperDriverCore::motorReport - Stepper Motor %d found", motor_id);
            }
            return response;
        }
        ROS_ERROR("StepperDriverCore::motorReport - Debug mode not enabled");
        return response;
    }

    int StepperDriverCore::launchMotorsReport()
    {
        int response = niryo_robot_msgs::CommandStatus::SUCCESS;
        if (_debug_flag)
        {
            ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Start Stepper Motor Report");
            ros::Duration(0.5).sleep();
            for (int motor_id = 1; motor_id < 4; motor_id++)
            {
                ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Motor %d report start :", motor_id);
                if (motorReport(motor_id) != niryo_robot_msgs::CommandStatus::SUCCESS)
                {
                    response = niryo_robot_msgs::CommandStatus::FAILURE;
                }
            }

            if (!_debug_flag)
            {
                ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Debug motor aborted");
                return niryo_robot_msgs::CommandStatus::ABORTED;
            }

            for (int motor_id = 1; motor_id < 4; motor_id++)
            {
                ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send torque on command on motor %d", motor_id);

                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    _stepper->sendTorqueOnCommand(motor_id, 1);
                }
                ros::Duration(0.2).sleep();

                int direction = (motor_id == 2)? -1 : 1;
                int old_position = _stepper->getStepperPose(motor_id);
                ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, old_position);

                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send move command on motor %d", motor_id);
                    _stepper->sendRelativeMoveCommand(motor_id, -1000*direction, 1500);
                    ros::Duration(0.2).sleep();
                }
                ros::Duration(3).sleep();

                int new_position = _stepper->getStepperPose(motor_id);
                ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, new_position);
                if (std::abs(new_position - old_position) < 250)
                {
                    ROS_WARN("StepperDriverCore::launchMotorsReport - Debug - Pose error on motor %d", motor_id);
                    response = niryo_robot_msgs::CommandStatus::FAILURE;
                }
                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);

                    ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send move command on motor %d", motor_id);
                    _stepper->sendRelativeMoveCommand(motor_id, 1000*direction, 1000);
                    ros::Duration(0.2).sleep();
                }
                ros::Duration(3).sleep();

                int new_position2 = _stepper->getStepperPose(motor_id);
                ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, new_position2);
                if (std::abs(new_position2 - new_position) < 250)
                {
                    ROS_WARN("StepperDriverCore::launchMotorsReport - Debug - Pose error on motor %d", motor_id);
                    response = niryo_robot_msgs::CommandStatus::FAILURE;
                }

                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Send torque off command on motor %d", motor_id);
                    _stepper->sendTorqueOnCommand(motor_id, 0);
                    ros::Duration(0.2).sleep();
                }
                if (!_debug_flag)
                {
                    ROS_INFO("StepperDriverCore::launchMotorsReport - Debug - Debug motor aborted");
                    return niryo_robot_msgs::CommandStatus::ABORTED;
                }
            }
            return response;
        }
        ROS_ERROR("StepperDriverCore::launchMotorsReport - Debug - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }


    int StepperDriverCore::setConveyor(uint8_t motor_id)
    {
        bool motor_found = false;
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        int result;

        ros::Duration(0.1).sleep();
        motor_found = _stepper->scanMotorId(6);
        ros::Duration(0.1).sleep();
        if (!motor_found)
        {
            ROS_WARN("StepperDriverCore::setConveyor - No conveyor found");
            result = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;
        }
        else
        {
            result = _stepper->sendUpdateConveyorId(6, motor_id);
            if (result != CAN_OK)
            {
                result = niryo_robot_msgs::CommandStatus::CAN_WRITE_ERROR;
            }
            else
            {
                result = niryo_robot_msgs::CommandStatus::SUCCESS;
            }

            ros::Duration(0.1).sleep();
            _stepper->addConveyor(motor_id);
        }

        return result;
    }

    void StepperDriverCore::unsetConveyor(uint8_t motor_id)
    {
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        _stepper->sendUpdateConveyorId(motor_id, 6);
        ros::Duration(0.1).sleep();
        _stepper->removeConveyor(motor_id);
    }

    void StepperDriverCore::startCalibration()
    {
        _stepper->startCalibration();
    }

    void StepperDriverCore::stopCalibration()
    {
        _stepper->stopCalibration();
    }

    int32_t StepperDriverCore::getCalibrationResult(uint8_t id) const
    {
        return _stepper->getCalibrationResult(id);
    }

    stepper_driver::StepperArrayMotorHardwareStatus StepperDriverCore::getHwStatus() const
    {
        stepper_driver::StepperMotorHardwareStatus data;
        stepper_driver::StepperArrayMotorHardwareStatus hw_state;

        for (size_t i = 0; i < _stepper->getNbMotors(); ++i)
        {
            StepperMotorState stepperState = _stepper->getMotorState(static_cast<uint8_t>(i));

            data.motor_identity.motor_id = stepperState.getId();
            data.motor_identity.motor_type = static_cast<uint8_t>(stepperState.getType());
            data.temperature = static_cast<int32_t>(stepperState.getTemperatureState());
            data.error = static_cast<int32_t>(stepperState.getHardwareErrorState());
            data.firmware_version = stepperState.getFirmwareVersion();
            hw_state.motors_hw_status.push_back(data);
        }
        return hw_state;
    }

    niryo_robot_msgs::BusState StepperDriverCore::getBusState() const
    {
        niryo_robot_msgs::BusState can_bust_state;

        std::string error;
        bool connection;
        std::vector<uint8_t> motor_id;
        _stepper->getBusState(connection, motor_id, error);
        can_bust_state.connection_status = connection;
        can_bust_state.motor_id_connected = motor_id;
        can_bust_state.error = error;
        return can_bust_state;
    }

    void StepperDriverCore::startControlLoop()
    {
        resetHardwareControlLoopRates();
        if (!_control_loop_flag)
        {
            ROS_DEBUG("StepperDriverCore::startControlLoop - Start control loop thread");
            _control_loop_flag = true;
            _control_loop_thread = std::thread(&StepperDriverCore::controlLoop, this);
        }
    }

    void StepperDriverCore::controlLoop()
    {
        ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);
        resetHardwareControlLoopRates();
        while (ros::ok())
        {
            if (!_stepper->isConnectionOk())
            {
                ROS_WARN("StepperDriverCore::controlLoop - Stepper connection error");
                ros::Duration(0.1).sleep();

                while (!_stepper->isConnectionOk())
                { // wait for connection to be up
                    ROS_INFO("StepperDriverCore::controlLoop - Scan to find stepper motors");
                    {
                        std::lock_guard<std::mutex> lck(_control_loop_mutex);
                        _stepper->scanAndCheck();
                    }
                    ros::Duration(0.1).sleep();
                }
                ROS_INFO("StepperDriverCore::controlLoop - Stepper Can bus ok");
            }
            if (_control_loop_flag)
            {
                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    if (!_stepper->isCalibrationInProgress())
                    {
                        _stepper->readMotorsState();
                    }

                    if (ros::Time::now().toSec() - _time_hw_last_write > 1.0 / _write_frequency)
                    {
                        _time_hw_last_write += 1.0 / _write_frequency;
                        _executeCommand();
                    }
                }

                control_loop_rate.sleep();
            }
            else
            {
                ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                resetHardwareControlLoopRates();
            }
    }
    }

    void StepperDriverCore::_executeCommand()
    {
        bool need_sleep = false;

        std_msgs::Int64MultiArray cmd;
        if (_joint_trajectory_controller_cmd.isValid())
        {
            _stepper->readSynchronizeCommand(_joint_trajectory_controller_cmd);
            cmd.data.push_back(_joint_trajectory_controller_cmd.getParam(0));
            cmd.data.push_back(_joint_trajectory_controller_cmd.getParam(1));
            cmd.data.push_back(_joint_trajectory_controller_cmd.getParam(2));
            cmd_pub.publish(cmd);

            _joint_trajectory_controller_cmd.clear();
            need_sleep = true;
        }
        if (!_stepper_cmds.empty())
        {
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _stepper->readSingleCommand(_stepper_cmds.front());
            _stepper_cmds.pop();
            need_sleep = true;
        }
        if (!_conveyor_cmds.empty())
        {
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _stepper->readSingleCommand(_conveyor_cmds.front());
            _conveyor_cmds.pop();
        }
    }

    bool StepperDriverCore::scanMotorId(uint8_t motor_to_find)
    {
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        return _stepper->scanMotorId(motor_to_find);
    }


} // namespace StepperDriver
