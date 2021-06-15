/*
    can_driver_core.cpp
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

#include <std_msgs/Int64MultiArray.h>

#include "common/util/util_defs.hpp"
#include "can_driver/can_driver_core.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/motor_type_enum.hpp"

using namespace std;
using namespace common::model;

namespace can_driver
{
    /**
     * @brief CanDriverCore::CanDriverCore
     */
    CanDriverCore::CanDriverCore() :
        _control_loop_flag(false),
        _debug_flag(false),
        _control_loop_frequency(0.0),
        _delta_time_write(0.0),
        _time_hw_data_last_read(0.0),
        _time_hw_data_last_write(0.0),
        _time_check_connection_last_read(0.0),
        _delta_time_calib_read(0.0),
        _time_hw_calib_last_read(0.0)
    {
        ROS_DEBUG("CanDriverCore::CanDriverCore - ctor");

        init();
    }

    /**
     * @brief CanDriverCore::~CanDriverCore
     */
    CanDriverCore::~CanDriverCore()
    {
        if(_control_loop_thread.joinable())
            _control_loop_thread.join();
    }

    /**
     * @brief CanDriverCore::init
     */
    void CanDriverCore::init()
    {
        initParameters();

        _can_driver = std::make_unique<CanDriver>();

        startControlLoop();
    }

    /**
     * @brief CanDriverCore::initParameters
     */
    void CanDriverCore::initParameters()
    {
        _control_loop_frequency = 0.0;
        double write_frequency = 1.0;

        _nh.getParam("/niryo_robot_hardware_interface/can_driver/can_hardware_control_loop_frequency", _control_loop_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/can_driver/can_hw_write_frequency", write_frequency);

        ROS_DEBUG("CanDriverCore::initParameters - can_hardware_control_loop_frequency : %f", _control_loop_frequency);
        ROS_DEBUG("CanDriverCore::initParameters - can_hw_write_frequency : %f", write_frequency);

        _delta_time_write = 1.0 / write_frequency;
    }

    //***************
    //  Commands
    //***************

    /**
     * @brief CanDriverCore::scanMotorId
     * @param motor_to_find
     * @return
     */
    bool CanDriverCore::scanMotorId(uint8_t motor_to_find)
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        return _can_driver->ping(motor_to_find);
    }

    /**
     * @brief CanDriverCore::startCalibration
     */
    void CanDriverCore::startCalibration()
    {
        if(_can_driver)
            _can_driver->startCalibration();
    }

    /**
     * @brief CanDriverCore::resetCalibration
     */
    void CanDriverCore::resetCalibration()
    {
        if(_can_driver)
            _can_driver->resetCalibration();
    }

    /**
     * @brief CanDriverCore::motorCmdReport
     * @param motor_id
     * @return
     */
    int CanDriverCore::motorCmdReport(uint8_t motor_id)
    {
        int ret = niryo_robot_msgs::CommandStatus::ABORTED;

        if (_debug_flag)
        {
            // torque on
            {
                lock_guard<mutex> lck(_control_loop_mutex);
                ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Send torque on command on motor %d", motor_id);
                _can_driver->sendTorqueOnCommand(motor_id, 1);
            }
            ros::Duration(0.2).sleep();

            // move one direction
            // WARNING : the stepper motor 2 direction is hardcoded here
            // there is no easy way to change that for now
            int direction = (motor_id == 2) ? -1 : 1;
            int32_t old_position = _can_driver->getPosition(motor_id);
            ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, old_position);
            {
                lock_guard<mutex> lck(_control_loop_mutex);
                ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Send move command on motor %d", motor_id);
                _can_driver->sendRelativeMoveCommand(motor_id, -1000*direction, 1500);
                ros::Duration(0.2).sleep();
            }
            ros::Duration(3).sleep();

            // move back
            int32_t new_position = _can_driver->getPosition(motor_id);
            ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, new_position);
            {
                lock_guard<mutex> lck(_control_loop_mutex);

                ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Send move command on motor %d", motor_id);
                _can_driver->sendRelativeMoveCommand(motor_id, 1000*direction, 1000);
                ros::Duration(0.2).sleep();
            }
            int rest = static_cast<int>(new_position - old_position);

            ros::Duration(3).sleep();

            // torque off
            int32_t new_position2 = _can_driver->getPosition(motor_id);
            ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Get pose on motor %d: %d", motor_id, new_position2);
            {
                lock_guard<mutex> lck(_control_loop_mutex);
                ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Send torque off command on motor %d", motor_id);
                _can_driver->sendTorqueOnCommand(motor_id, 0);
                ros::Duration(0.2).sleep();
            }
            int rest2 = static_cast<int>(new_position2 - new_position);

            if (abs(rest) < 250 || abs(rest2) < 250)
            {
                ROS_WARN("CanDriverCore::launchMotorsReport - Debug - Pose error on motor %d", motor_id);
                ret = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            else
            {
                ROS_INFO("DynamixelDriverCore::motorCmdReport - Debug - Dynamixel Motor %d OK", motor_id);
                ret = niryo_robot_msgs::CommandStatus::SUCCESS;
            }
        }
        else
        {
            ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Debug motor aborted");
        }

        return ret;
    }


    /**
     * @brief CanDriverCore::launchMotorsReport
     * @return
     */
    int CanDriverCore::launchMotorsReport()
    {
        int response = niryo_robot_msgs::CommandStatus::ABORTED;
        unsigned int nbSuccess = 0;
        unsigned int nbFailure = 0;

        std::vector<std::tuple<uint8_t, EMotorType, int> > results;

        if (_debug_flag)
        {
            ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Start Stepper Motor Report");
            ros::Duration(0.5).sleep();
            if (CAN_OK == _can_driver->scanAndCheck())
            {
                for (auto&& state: _can_driver->getMotorsStates())
                {
                    if(state) {

                        ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Motor %d report start :", static_cast<int>(state->getId()));

                        int cmd_res = motorCmdReport(state->getId());

                        if(niryo_robot_msgs::CommandStatus::SUCCESS == cmd_res)
                        {
                            nbSuccess++;
                        }
                        else
                        {
                            nbFailure++;
                            response = niryo_robot_msgs::CommandStatus::FAILURE;
                        }

                        results.emplace_back(state->getId(), state->getType(), cmd_res);

                        if (niryo_robot_msgs::CommandStatus::ABORTED == cmd_res)
                        {
                            ROS_INFO("CanDriverCore::launchMotorsReport - Debug - Debug motor aborted");
                            break;
                        }
                    }
                }
            }
            else
            {
                ROS_ERROR("DynamixelDriverCore::launchMotorsReport - Debug - unable to contact all steppers");
                response = niryo_robot_msgs::CommandStatus::FAILURE;
                nbFailure++;
            }

            //display synthesis
            ROS_INFO("Synthesis of Stepper motors report :");
            ROS_INFO("--------");
            ROS_INFO("Nb Success : %d, Nb Failure: %d", nbSuccess, nbFailure);
            ROS_INFO("Details: ");
            ROS_INFO("--------");
            ROS_INFO("id \t| type \t| cmd result");

            for(auto const& res : results)
            {
                ROS_INFO("%d \t| %s \t| %d", std::get<0>(res),
                                              MotorTypeEnum(std::get<1>(res)).toString().c_str(),
                                              std::get<2>(res));
            }
        }
        else
        {
            ROS_ERROR("CanDriverCore::launchMotorsReport - Debug - Debug mode not enabled");
        }

        return response;
    }

    //****************
    //  Control Loop
    //****************

    /**
     * @brief CanDriverCore::startControlLoop
     */
    void CanDriverCore::startControlLoop()
    {
        resetHardwareControlLoopRates();
        if (!_control_loop_flag)
        {
            ROS_DEBUG("CanDriverCore::startControlLoop - Start control loop thread");
            _control_loop_flag = true;
            _control_loop_thread = thread(&CanDriverCore::controlLoop, this);
        }
    }

    /**
     * @brief CanDriverCore::resetHardwareControlLoopRates
     */
    void CanDriverCore::resetHardwareControlLoopRates()
    {
        ROS_DEBUG("CanDriverCore::resetHardwareControlLoopRates - Reset control loop rates");
        double now = ros::Time::now().toSec();
        _time_hw_data_last_write = now;
        _time_hw_data_last_read = now;

        _time_check_connection_last_read = now;
    }

    /**
     * @brief CanDriverCore::activeDebugMode
     * @param mode
     */
    void CanDriverCore::activeDebugMode(bool mode)
    {
        ROS_INFO("CanDriverCore::activeDebugMode - Activate debug mode for stepper driver core: %d", mode);
        _debug_flag = mode;

        if (!mode)
        {
            lock_guard<mutex> lck(_control_loop_mutex);
            _can_driver->sendTorqueOnCommand(2, 0);
            ros::Duration(0.2).sleep();
            _can_driver->sendTorqueOnCommand(3, 0);
        }
    }

    /**
     * @brief CanDriverCore::controlLoop
     */
    void CanDriverCore::controlLoop()
    {
        ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);
        resetHardwareControlLoopRates();
        while (ros::ok())
        {
            if (_control_loop_flag)
            {
                lock_guard<mutex> lck(_control_loop_mutex);

                _can_driver->readStatus();

                if (_can_driver->isConnectionOk())
                {
                    if (ros::Time::now().toSec() - _time_hw_data_last_write >= _delta_time_write)
                    {
                        _time_hw_data_last_write = ros::Time::now().toSec();
                        _executeCommand();
                    }
                }

                bool isFreqMet = control_loop_rate.sleep();
                ROS_DEBUG_COND(!isFreqMet, "CanDriverCore::rosControlLoop : freq not met : expected (%f s) vs actual (%f s)", control_loop_rate.expectedCycleTime().toSec(), control_loop_rate.cycleTime().toSec());
            }
            else
            {
                ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                resetHardwareControlLoopRates();
            }
        }
    }

    /**
     * @brief CanDriverCore::_executeCommand
     */
    void CanDriverCore::_executeCommand()
    {
        bool need_sleep = false;
        if (!_joint_trajectory_cmd.empty())
        {
            _can_driver->executeJointTrajectoryCmd(_joint_trajectory_cmd);
            _joint_trajectory_cmd.clear();
            need_sleep = true;
        }

        if (!_stepper_single_cmds.empty())
        {
            //as we use a queue, we don't need a mutex
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _can_driver->readSingleCommand(_stepper_single_cmds.front());
            _stepper_single_cmds.pop();
            need_sleep = true;
        }
        if (!_conveyor_cmds.empty())
        {
            //as we use a queue, we don't need a mutex
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _can_driver->readSingleCommand(_conveyor_cmds.front());
            _conveyor_cmds.pop();
        }
    }

    //*************
    //  Setters
    //*************

    /**
     * @brief CanDriverCore::setConveyor
     * @param new_motor_id
     * @param default_conveyor_id
     * @return
     */
    int CanDriverCore::setConveyor(uint8_t new_motor_id, uint8_t default_conveyor_id)
    {
        int result = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;

        lock_guard<mutex> lck(_control_loop_mutex);

        //try to find motor id 6 (default motor id for conveyor
        if (_can_driver->ping(default_conveyor_id))
        {
            if (CAN_OK == _can_driver->sendUpdateConveyorId(default_conveyor_id, new_motor_id)) {
                //add stepper as a new conveyor
                _can_driver->addMotor(new_motor_id, true);
                result = niryo_robot_msgs::CommandStatus::SUCCESS;
            }
            else {
                ROS_ERROR("CanDriverCore::setConveyor : unable to change conveyor ID");
                result = niryo_robot_msgs::CommandStatus::CAN_WRITE_ERROR;
            }
        }
        else
        {
            ROS_WARN("CanDriverCore::setConveyor - No conveyor found");
        }

        return result;
    }

    /**
     * @brief CanDriverCore::unsetConveyor
     * @param motor_id
     */
    void CanDriverCore::unsetConveyor(uint8_t motor_id)
    {
        lock_guard<mutex> lck(_control_loop_mutex);

        ROS_DEBUG("CanDriverCore::unsetConveyor - UnsetEndEffector: id %d", motor_id);

        if(_can_driver->sendUpdateConveyorId(motor_id, 6))
            _can_driver->removeMotor(motor_id);
        else
            ROS_ERROR("CanDriverCore::unsetConveyor : unable to change conveyor ID");
    }

    /**
     * @brief CanDriverCore::clearSingleCommandQueue
     */
    void CanDriverCore::clearSingleCommandQueue()
    {
        while(!_stepper_single_cmds.empty())
            _stepper_single_cmds.pop();
    }

    /**
     * @brief CanDriverCore::clearConveyorCommandQueue
     */
    void CanDriverCore::clearConveyorCommandQueue()
    {
        while(!_conveyor_cmds.empty())
            _conveyor_cmds.pop();
    }

    /**
     * @brief CanDriverCore::setTrajectoryControllerCommands
     * @param cmd
     */
    void CanDriverCore::setTrajectoryControllerCommands(const std::vector<std::pair<uint8_t, int32_t> > &cmd)
    {
        _joint_trajectory_cmd = cmd;
    }

    /**
     * @brief CanDriverCore::addSingleCommandToQueue
     * @param cmd
     */
    void CanDriverCore::addSingleCommandToQueue(const common::model::StepperMotorCmd &cmd)
    {
        ROS_DEBUG("CanDriverCore::addSingleCommandToQueue - %s", cmd.str().c_str());

        if(cmd.isValid()) {
            if(cmd.getType() == EStepperCommandType::CMD_TYPE_CONVEYOR) {//keep position cmd apart
                if(_conveyor_cmds.size() > QUEUE_OVERFLOW)
                    ROS_WARN("CanDriverCore::addCommandToQueue: Cmd queue overflow ! %lu", _conveyor_cmds.size());
                else {
                    _conveyor_cmds.push(cmd);
                }
            }
            else {
                if(_stepper_single_cmds.size() > QUEUE_OVERFLOW)
                    ROS_WARN("CanDriverCore::addCommandToQueue: Cmd queue overflow ! %lu", _stepper_single_cmds.size());
                else {
                    _stepper_single_cmds.push(cmd);
                }
            }
        }
    }

    /**
     * @brief CanDriverCore::addSingleCommandToQueue
     * @param cmd
     */
    void CanDriverCore::addSingleCommandToQueue(const std::vector<common::model::StepperMotorCmd> &cmd)
    {
        for(auto&& c : cmd)
            addSingleCommandToQueue(c);
    }

    //********************
    //  getters
    //********************

    /**
     * @brief CanDriverCore::getHwStatus
     * @return
     */
    can_driver::StepperArrayMotorHardwareStatus CanDriverCore::getHwStatus() const
    {
        can_driver::StepperMotorHardwareStatus data;
        can_driver::StepperArrayMotorHardwareStatus hw_state;

        for (auto && stepperState : _can_driver->getMotorsStates())
        {
            if(stepperState) {
                data.motor_identity.motor_id = stepperState->getId();
                data.motor_identity.motor_type = static_cast<uint8_t>(stepperState->getType());
                data.temperature = static_cast<int32_t>(stepperState->getTemperatureState());
                data.error = static_cast<int32_t>(stepperState->getHardwareErrorState());
                data.firmware_version = stepperState->getFirmwareVersion();
                hw_state.motors_hw_status.push_back(data);
            }
        }
        return hw_state;
    }

    /**
     * @brief CanDriverCore::getBusState
     * @return
     */
    niryo_robot_msgs::BusState CanDriverCore::getBusState() const
    {
        niryo_robot_msgs::BusState can_bus_state;

        string error;
        bool connection;
        vector<uint8_t> motor_id;

        _can_driver->getBusState(connection, motor_id, error);
        can_bus_state.connection_status = connection;
        can_bus_state.motor_id_connected = motor_id;
        can_bus_state.error = error;
        return can_bus_state;
    }

} // namespace can_driver
