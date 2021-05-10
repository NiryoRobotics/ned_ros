/*
    dxl_driver_core.cpp
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

#include "dynamixel_driver/dxl_driver_core.hpp"

//c++
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <functional>

using namespace std;
using namespace common::model;

static constexpr double DXL_VOLTAGE_DIVISOR = 10.0;

namespace DynamixelDriver
{
    /**
     * @brief DxlDriverCore::DxlDriverCore
     */
    DxlDriverCore::DxlDriverCore() :
        _control_loop_flag(false),
        _debug_flag(false)
    {
        ROS_DEBUG("DynamixelDriverCore - ctor");

        initParameters();

        init();
    }

    /**
     * @brief DxlDriverCore::~DxlDriverCore
     */
    DxlDriverCore::~DxlDriverCore()
    {
        if(_control_loop_thread.joinable())
            _control_loop_thread.join();
    }

    /**
     * @brief DxlDriverCore::init
     */
    void DxlDriverCore::init()
    {

        _dynamixel.reset(new DxlDriver());
        _dynamixel->scanAndCheck();
        startControlLoop();

        //advertise services

        _activate_leds_server = _nh.advertiseService("niryo_robot/dynamixel_driver/set_dxl_leds", &DxlDriverCore::callbackActivateLeds, this);
        _custom_cmd_server = _nh.advertiseService("niryo_robot/dynamixel_driver/send_custom_dxl_value", &DxlDriverCore::callbackSendCustomDxlValue, this);
        _custom_cmd_getter = _nh.advertiseService("niryo_robot/dynamixel_driver/read_custom_dxl_value", &DxlDriverCore::callbackReadCustomDxlValue, this);
    }

    /**
     * @brief DxlDriverCore::initParameters
     */
    void DxlDriverCore::initParameters()
    {
        _control_loop_frequency = 0.0;
        double write_frequency = 0.0;
        double read_data_frequency = 0.0;
        double read_status_frequency = 0.0;

        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_control_loop_frequency", _control_loop_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_write_frequency", write_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_read_data_frequency", read_data_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_read_status_frequency", read_status_frequency);

        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_control_loop_frequency : %f", _control_loop_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_write_frequency : %f", write_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_read_data_frequency : %f", read_data_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_read_status_frequency : %f", read_status_frequency);

        _delta_time_data_read = 1.0 / read_data_frequency;
        _delta_time_status_read = 1.0 / read_status_frequency;
        _delta_time_write = 1.0 / write_frequency;
    }

    //***************
    //  Commands
    //***************

    /**
     * @brief DxlDriverCore::rebootMotors
     * @return
     */
    int DxlDriverCore::rebootMotors()
    {
        ROS_INFO("DynamixelDriverCore::rebootMotors - Reboot motors");
        lock_guard<mutex> lck(_control_loop_mutex);
        int result = _dynamixel->rebootMotors();
        ros::Duration(1.5).sleep();
        return result;
    }

    /**
     * @brief DynamixelDriverCore::scanTools
     * @return
     */
    vector<uint8_t> DxlDriverCore::scanTools()
    {
        vector<uint8_t> motor_list;
        lock_guard<mutex> lck(_control_loop_mutex);
        ROS_INFO("DynamixelDriverCore::scanTools - Scan tools...");
        int result = COMM_PORT_BUSY;

        for(int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
        {
            result = _dynamixel->getAllIdsOnBus(motor_list);
            ROS_DEBUG_COND(COMM_SUCCESS != result, "DxlDriver::scanTools status: %d (counter: %d)", result, counter);
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }
        ROS_DEBUG("DynamixelDriverCore::scanTools - Result getAllIdsOnDxlBus: %d", result);

        ostringstream ss;
        for(auto const& m : motor_list)
            ss << static_cast<int>(m) << " ";
        string motor_id_list_string = ss.str();
        if(!motor_id_list_string.empty())
            motor_id_list_string.pop_back(); //remove trailing " "

        ROS_DEBUG("DynamixelDriverCore::scanTools - All id on dxl bus: [%s]", motor_id_list_string.c_str());
        return motor_list;
    }

    /**
     * @brief DynamixelDriverCore::update_leds
     * @return
     */
    int DxlDriverCore::update_leds(void)
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        int result = _dynamixel->setLeds(_dynamixel->getLedState(), EMotorType::MOTOR_TYPE_XL320);
        return result;
    }

    /**
     * @brief DxlDriverCore::motorScanReport
     * @param motor_id
     * @param motor_type
     * @return
     */
    int DxlDriverCore::motorScanReport(uint8_t motor_id, EMotorType motor_type)
    {
        if (_debug_flag)
        {
            int result;

            ros::Duration(1.0).sleep();
            result = _dynamixel->type_ping_id(motor_id, motor_type);
            if (result == COMM_SUCCESS)
            {
                ROS_INFO("DynamixelDriverCore::motorScanReport - Debug - Dynamixel Motor %d found", motor_id);
            }
            else
            {
                ROS_ERROR("DynamixelDriverCore::motorScanReport - Debug - Dynamixel Motor %d not found", motor_id);
                ROS_ERROR("DynamixelDriverCore::motorScanReport - Debug - code error : %d ", result);
                return niryo_robot_msgs::CommandStatus::FAILURE;
            }
            return niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        ROS_ERROR("DynamixelDriverCore::motorScanReport - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }

    int DxlDriverCore::motorCmdReport(uint8_t motor_id, EMotorType motor_type)
    {
        if (_debug_flag)
        {
            uint32_t old_position;
            uint32_t new_position;
            DxlMotorState dynamixel_motor = DxlMotorState(motor_type, motor_id);

            ros::Duration(0.5).sleep();
            ROS_INFO("DynamixelDriverCore::motorCmdReport - Debug - Send torque on command on dxl %d", int(motor_id));
            _dynamixel->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_TORQUE, motor_id, 1));
            ros::Duration(0.5).sleep();


            old_position = _dynamixel->getPosition(dynamixel_motor);
            ROS_INFO("DynamixelDriverCore::motorCmdReport - Debug - get dxl %d pose: %d ", int(motor_id), int(old_position));
            ros::Duration(0.5).sleep();
            ROS_INFO("DynamixelDriverCore::motorCmdReport - Debug - Send dxl %d pose: %d ", int(motor_id), int(old_position + 200));
            _dynamixel->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION, motor_id, old_position + 200));

            ros::Duration(2).sleep();
            new_position = _dynamixel->getPosition(dynamixel_motor);
            ROS_INFO("DynamixelDriverCore::motorCmdReport - Debug - get dxl %d pose: %d ", int(motor_id), int(new_position));
            int rest = new_position - old_position;

            ROS_INFO("Dynamixel Driver Core - Debug - Send dxl %d pose: %d ", int(motor_id), int(old_position));
            _dynamixel->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION, motor_id, old_position));

            ros::Duration(2).sleep();
            uint32_t new_position2 = _dynamixel->getPosition(dynamixel_motor);
            ROS_INFO("DynamixelDriverCore::motorCmdReport - Debug - get dxl %d pose: %d ", int(motor_id), int(new_position2));
            int rest2 = new_position2 - new_position;

            _dynamixel->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_TORQUE, motor_id, 0));

            if (abs(rest) < 50 or abs(rest2) < 50)
            {
                ROS_WARN("DynamixelDriverCore::motorCmdReport - Debug - Dynamixel Motor %d problem", motor_id);
                return niryo_robot_msgs::CommandStatus::FAILURE;
            }
            else
            {
                ROS_INFO("DynamixelDriverCore::motorCmdReport - Debug - Dynamixel Motor %d OK", motor_id);
            }
            return niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        ROS_ERROR("DynamixelDriverCore::motorCmdReport - Debug - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }

    int DxlDriverCore::launchMotorsReport()
    {
        if (_debug_flag)
        {
            int motor_found;
            int response = niryo_robot_msgs::CommandStatus::SUCCESS;

            ROS_INFO("DynamixelDriverCore::launchMotorsReport - Debug - Start Dynamixel Motor Report");
            ros::Duration(1.0).sleep();
            ROS_INFO("DynamixelDriverCore::launchMotorsReport - Debug - Motor 4 report start :");
            if (motorScanReport(2, EMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            if (motorCmdReport(2, EMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            ROS_INFO("DynamixelDriverCore::launchMotorsReport - Debug - Motor 5 report start :");
            if (motorScanReport(3, EMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            if (motorCmdReport(3, EMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            ROS_INFO("DynamixelDriverCore::launchMotorsReport - Debug - Motor 6 report start :");
            if (motorScanReport(6, EMotorType::MOTOR_TYPE_XL320) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            if (motorCmdReport(6, EMotorType::MOTOR_TYPE_XL320) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;

            ros::Duration(1.0).sleep();
            ROS_INFO("DynamixelDriverCore::launchMotorsReport - Debug - Check for unflash dynamixel motors");
            motor_found = _dynamixel->type_ping_id(1, EMotorType::MOTOR_TYPE_XL430);
            if (motor_found == COMM_SUCCESS)
            {
                ROS_ERROR("DynamixelDriverCore::launchMotorsReport - Debug - Find a dynamixel motor unflash");
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            motor_found = _dynamixel->type_ping_id(1, EMotorType::MOTOR_TYPE_XL320);
            if (motor_found == COMM_SUCCESS)
            {
                ROS_ERROR("DynamixelDriverCore::launchMotorsReport - Debug - Find a dynamixel motor unflash");
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            return response;
        }
        ROS_ERROR("DynamixelDriverCore::launchMotorsReport - Debug - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }

    //****************
    //  Control Loop
    //****************

    /**
     * @brief DxlDriverCore::startControlLoop
     */
    void DxlDriverCore::startControlLoop()
    {
        resetHardwareControlLoopRates();
        if (!_control_loop_flag)
        {
            ROS_INFO("DynamixelDriverCore::startControlLoop - Start control loop");
            _control_loop_flag = true;
            _control_loop_thread = std::thread(&DxlDriverCore::controlLoop, this);
        }
    }

    /**
     * @brief DxlDriverCore::resetHardwareControlLoopRates
     */
    void DxlDriverCore::resetHardwareControlLoopRates()
    {
        ROS_DEBUG("DynamixelDriverCore::resetHardwareControlLoopRates - Reset control loop rates");
        double now = ros::Time::now().toSec();
        _time_hw_data_last_write = now;
        _time_hw_data_last_read = now;
        _time_hw_status_last_read = now;
        _time_check_connection_last_read = now;
        _time_check_end_effector_last_read = now;
    }

    /**
     * @brief DxlDriverCore::activeDebugMode
     * @param mode
     */
    void DxlDriverCore::activeDebugMode(bool mode)
    {
        ROS_INFO("DynamixelDriverCore::activeDebugMode - Activate debug mode for dynamixel driver core: %d", mode);
        _debug_flag = mode;
        _control_loop_flag = !mode;

        if (mode)
        {
            _control_loop_mutex.lock();
        }
        else
        {
            _control_loop_mutex.unlock();
        }
    }

    /**
     * @brief DynamixelDriverCore::controlLoop
     */
    void DxlDriverCore::controlLoop()
    {
        ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);
        resetHardwareControlLoopRates();
        while (ros::ok())
        {
            if (!_debug_flag)
            {
                if (!_dynamixel->isConnectionOk())
                {
                    ROS_WARN("DynamixelDriverCore::controlLoop - Dynamixel connection error");
                    ros::Duration(0.1).sleep();

                    vector<uint8_t> missing_ids;
                    ROS_DEBUG("DynamixelDriverCore::controlLoop - Scan to find Dxl motors");

                    int bus_state = COMM_PORT_BUSY;
                    while (DXL_SCAN_OK != bus_state)
                    {
                        {
                            lock_guard<mutex> lck(_control_loop_mutex);
                            bus_state = _dynamixel->scanAndCheck();
                        }
                        missing_ids = getRemovedMotorList();
                        for(auto const& id : missing_ids) {
                            ROS_WARN("DynamixelDriverCore::controlLoop - Dynamixel %d do not seem to be connected", id);
                        }
                        ros::Duration(0.25).sleep();
                    }
                    ROS_INFO("DynamixelDriverCore::controlLoop - Dxl Bus ok");
                }

                if (_control_loop_flag)
                {
                    lock_guard<mutex> lck(_control_loop_mutex);
                    if (ros::Time::now().toSec() - _time_hw_data_last_read >= _delta_time_data_read)
                    {
                        _time_hw_data_last_read = ros::Time::now().toSec();
                        _dynamixel->readPositionStatus();
                    }
                    if (ros::Time::now().toSec() - _time_hw_status_last_read >= _delta_time_status_read)
                    {
                        _time_hw_status_last_read = ros::Time::now().toSec();
                        _dynamixel->readHwStatus();
                    }
                    if (ros::Time::now().toSec() - _time_hw_data_last_write >= _delta_time_write)
                    {
                        _time_hw_data_last_write = ros::Time::now().toSec();
                        _executeCommand();
                    }
                    bool isFreqMet = control_loop_rate.sleep();
                    ROS_WARN_COND(!isFreqMet, "DxlDriverCore::controlLoop : control loop rate (%f) not met !", _control_loop_frequency);
                }
                else
                {
                    ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                    resetHardwareControlLoopRates();
                }
            }
            else
            {
                ros::Duration(0.5).sleep();
            }
        }
    }

    /**
     * @brief DynamixelDriverCore::_executeCommand : execute all the cmd in the current queue
     */ // create a unique queue using polymorphism
    void DxlDriverCore::_executeCommand()
    {
        bool need_sleep = false;

        if (_joint_trajectory_cmd.isValid())
        {
            //we need a mutex here to ensure this data is not being modify
            //by another thread during its usage
            lock_guard<mutex> lck(_joint_trajectory_mutex);

            _dynamixel->readSynchronizeCommand(_joint_trajectory_cmd);
            _joint_trajectory_cmd.reset();
            need_sleep = true;
        }
        if (!_dxl_single_cmds.empty())
        {
            _dynamixel->readSingleCommand(_dxl_single_cmds.front());
            _dxl_single_cmds.pop();
            need_sleep = true;
        }
        if (!_end_effector_cmds.empty())
        {
            //as we use a queue, we don't need a mutex
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _dynamixel->readSingleCommand(_end_effector_cmds.front());
            _end_effector_cmds.pop();
            need_sleep = true;
        }
        if (_dxl_sync_cmds.isValid())
        {
            //as we use a queue, we don't need a mutex
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _dynamixel->readSynchronizeCommand(_dxl_sync_cmds);
            _dxl_sync_cmds.reset();
        }
    }

    //*************
    //  Setters
    //*************

    /**
     * @brief DxlDriverCore::setEndEffector
     * @param type
     * @param id
     * @return
     */
    int DxlDriverCore::setEndEffector(EMotorType type, uint8_t motor_id)
    {
        int result = niryo_robot_msgs::CommandStatus::DXL_READ_ERROR;

        lock_guard<mutex> lck(_control_loop_mutex);

        //try to find motor
        int result_ping = _dynamixel->type_ping_id(motor_id, type);

        if (COMM_SUCCESS == result_ping)
        {
            //add dynamixel as a new tool
            _dynamixel->addMotor(type, motor_id, true);
            result = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        else
        {
            ROS_WARN("DynamixelDriverCore::setEndEffector - No end effector found with motor id %d (ping return value %d)", motor_id, result_ping);
        }

        return result;
    }

    /**
     * @brief DxlDriverCore::unsetEndEffector
     * @param id
     */
    void DxlDriverCore::unsetEndEffector(uint8_t motor_id)
    {
        lock_guard<mutex> lck(_control_loop_mutex);

        ROS_DEBUG("DynamixelDriverCore::unsetEndEffector - UnsetEndEffector: id %d", motor_id);
        _dynamixel->removeMotor(motor_id);
    }


    /**
     * @brief DxlDriverCore::clearSingleCommandQueue
     */
    void DxlDriverCore::clearSingleCommandQueue()
    {
        while(!_dxl_single_cmds.empty())
            _dxl_single_cmds.pop();
    }

    /**
     * @brief DxlDriverCore::clearEndEffectorCommandQueue
     */
    void DxlDriverCore::clearEndEffectorCommandQueue()
    {
        while(!_end_effector_cmds.empty())
            _end_effector_cmds.pop();
    }

    /**
     * @brief DynamixelDriverCore::setSyncCommand
     * @param cmd
     */
    void DxlDriverCore::setSyncCommand(const SynchronizeMotorCmd &cmd)
    {
        if(cmd.isValid()) {
            //keep position cmd apart
            if(cmd.getType() == EDxlCommandType::CMD_TYPE_POSITION) {
                std::lock_guard<std::mutex> lck(_joint_trajectory_mutex);
                _joint_trajectory_cmd = cmd;
            }
            else
                _dxl_sync_cmds = cmd;
        }
        else {
            ROS_WARN("DynamixelDriverCore::setSyncCommand : Invalid command");
        }
    }

    /**
     * @brief DynamixelDriverCore::addSingleCommandToQueue
     * @param cmd
     *
     *  Not very good, nothing prevents the user from providing an end effector command here
     * and vice versa with addEndEffectorCmd. To be changed
     */
    void DxlDriverCore::addSingleCommandToQueue(const SingleMotorCmd &cmd)
    {
        ROS_DEBUG("DynamixelDriverCore::addSingleCommandToQueue - %s", cmd.str().c_str());

        if(cmd.isValid())
        {
            if(_dxl_single_cmds.size() > QUEUE_OVERFLOW)
                ROS_WARN("DynamixelDriverCore::addSingleCommandToQueue: Cmd queue overflow ! %lu", _dxl_single_cmds.size());
            else
                _dxl_single_cmds.push(cmd);
        }
    }

    /**
     * @brief DxlDriverCore::addSingleCommandToQueue
     * @param cmd
     */
    void DxlDriverCore::addSingleCommandToQueue(const std::vector<SingleMotorCmd> &cmd)
    {
        for(auto&& c : cmd)
            addSingleCommandToQueue(c);
    }

    /**
     * @brief DynamixelDriverCore::addEndEffectorCommandToQueue
     * @param cmd
     */
    void DxlDriverCore::addEndEffectorCommandToQueue(const SingleMotorCmd &cmd)
    {
        if(_end_effector_cmds.size() > QUEUE_OVERFLOW)
            ROS_WARN_THROTTLE(0.5, "DynamixelDriverCore::addEndEffectorCommandToQueue: Cmd queue overflow ! %lu", _end_effector_cmds.size());
        else
            _end_effector_cmds.push(cmd);
    }

    /**
     * @brief DxlDriverCore::addEndEffectorCommandToQueue
     * @param cmd
     */
    void DxlDriverCore::addEndEffectorCommandToQueue(const vector<SingleMotorCmd> &cmd)
    {
        for(auto&& c : cmd)
            addEndEffectorCommandToQueue(c);
    }

    //***************
    //  Getters
    //***************

    /**
     * @brief DxlDriverCore::getEndEffectorState
     * @param id
     * @return
     */
    double DxlDriverCore::getEndEffectorState(uint8_t id) const
    {
        DxlMotorState motor_state = _dynamixel->getMotorState(id);
        return static_cast<double>(motor_state.getPositionState());
    }

    /**
     * @brief DxlDriverCore::getHwStatus
     * @return
     */
    dynamixel_driver::DxlArrayMotorHardwareStatus DxlDriverCore::getHwStatus() const
    {
        dynamixel_driver::DxlMotorHardwareStatus data;
        dynamixel_driver::DxlArrayMotorHardwareStatus hw_state;

        for (auto const& dxlState : _dynamixel->getMotorsStates())
        {
            data.motor_identity.motor_id = dxlState.getId();
            data.motor_identity.motor_type = static_cast<uint8_t>(dxlState.getType());
            data.temperature = static_cast<uint32_t>(dxlState.getTemperatureState());
            data.voltage = static_cast<double>(dxlState.getVoltageState()) / DXL_VOLTAGE_DIVISOR;
            data.error = static_cast<uint32_t>(dxlState.getHardwareErrorState());
            data.error_msg = dxlState.getHardwareErrorMessageState();
            hw_state.motors_hw_status.push_back(data);
        }
        return hw_state;
    }

    /**
     * @brief DxlDriverCore::getBusState
     * @return
     */
    niryo_robot_msgs::BusState DxlDriverCore::getBusState() const
    {
        niryo_robot_msgs::BusState dxl_bus_state;

        string error;
        bool connection;
        vector<uint8_t> motor_id;

        _dynamixel->getBusState(connection, motor_id, error);
        dxl_bus_state.connection_status = connection;
        dxl_bus_state.motor_id_connected = motor_id;
        dxl_bus_state.error = error;
        return dxl_bus_state;
    }

    //*******************
    //    Callbacks     *
    //*******************

    /**
     * @brief DynamixelDriverCore::callbackSendCustomDxlValue
     * @param req
     * @param res
     * @return
     */
    bool DxlDriverCore::callbackSendCustomDxlValue(dynamixel_driver::SendCustomDxlValue::Request &req,
                                                         dynamixel_driver::SendCustomDxlValue::Response &res)
    {
        int result;

        EMotorType motor_type;

        if(2 <= req.motor_type  && 5 >= req.motor_type)
            motor_type = static_cast<EMotorType>(req.motor_type);
        else
        {
            res.status = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
            res.message = "Dynamixel Driver Core - Invalid motor type: should be 2 (XL-430) or 3 (XL-320) or 4 (XL-330) or 5 (XC-430)";
            return true;
        }

        lock_guard<mutex> lck(_control_loop_mutex);
        result = _dynamixel->sendCustomDxlCommand(motor_type,
                                                  req.id,
                                                  req.reg_address,
                                                  req.value,
                                                  req.byte_number);

        if (result != COMM_SUCCESS) {
            res.message = "Dynamixel Driver Core - Send custom command failed";
        }
        else
        {
            res.message = "Dynamixel Driver Core - Send custom command done";
            result = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        res.status = result;
        return true;
    }

    /**
     * @brief DynamixelDriverCore::callbackReadCustomDxlValue
     * @param req
     * @param res
     * @return
     */
    bool DxlDriverCore::callbackReadCustomDxlValue(dynamixel_driver::ReadCustomDxlValue::Request &req,
                                                        dynamixel_driver::ReadCustomDxlValue::Response &res)
    {
        int result;
        EMotorType motor_type;
        if(2 <= req.motor_type  && 5 >= req.motor_type)
            motor_type = static_cast<EMotorType>(req.motor_type);
        else
        {
            res.status = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
            res.message = "Dynamixel Driver Core - Invalid motor type: should be 2 (XL-430) or 3 (XL-320) or 4 (XL-330) or 5 (XC-430)";
            return true;
        }

        lock_guard<mutex> lck(_control_loop_mutex);
        int value = 0;
        result = _dynamixel->readCustomDxlCommand(motor_type,
                                                  req.id,
                                                  req.reg_address,
                                                  value,
                                                  req.byte_number);

        if (result != COMM_SUCCESS) {
            res.message = "Dynamixel Driver Core - Reading custom registry failed";
        }
        else
        {
            res.message = "Dynamixel Driver Core - Reading successful. Registry value : " + to_string(value);
            result = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        res.status = result;
        return true;
    }

    /**
     * @brief DynamixelDriverCore::callbackActivateLeds
     * @param req
     * @param res
     * @return
     */
    bool DxlDriverCore::callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res)
    {
        int led = req.value;
        string message = "";

        lock_guard<mutex> lck(_control_loop_mutex);
        int result = _dynamixel->setLeds(led, EMotorType::MOTOR_TYPE_XL320);

        res.status = result;
        res.message = message;
        return true;
    }

} // namespace DynamixelDriver
