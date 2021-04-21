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
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <functional>

using namespace std;
using namespace common::model;

static constexpr double DXL_VOLTAGE_DIVISOR = 10.0;

namespace DynamixelDriver
{
    DynamixelDriverCore::DynamixelDriverCore() :
        _control_loop_flag(false),
        _debug_flag(false)
    {
        ROS_DEBUG("DynamixelDriverCore - ctor");

        init();
    }

    DynamixelDriverCore::~DynamixelDriverCore()
    {
        if(_control_loop_thread.joinable())
                _control_loop_thread.join();
    }

    void DynamixelDriverCore::init()
    {
        _control_loop_flag = false;
        _debug_flag = false;

        initParameters();
        _dynamixel.reset(new DxlDriver());
        _dynamixel->scanAndCheck();
        startControlLoop();

        _activate_leds_server = _nh.advertiseService("niryo_robot/dynamixel_driver/set_dxl_leds", &DynamixelDriverCore::callbackActivateLeds, this);
        _custom_cmd_server = _nh.advertiseService("niryo_robot/dynamixel_driver/send_custom_dxl_value", &DynamixelDriverCore::callbackSendCustomDxlValue, this);
        _custom_cmd_getter = _nh.advertiseService("niryo_robot/dynamixel_driver/read_custom_dxl_value", &DynamixelDriverCore::callbackReadCustomDxlValue, this);
    }

    void DynamixelDriverCore::initParameters()
    {

        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_control_loop_frequency", _control_loop_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_write_frequency", _write_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_read_data_frequency", _read_data_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_read_status_frequency", _read_status_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_check_connection_frequency", _check_connection_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_end_effector_frequency", _check_end_effector_frequency);

        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_control_loop_frequency : %f", _control_loop_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_write_frequency : %f", _write_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_read_data_frequency : %f", _read_data_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_read_status_frequency : %f", _read_status_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_hardware_check_connection_frequency : %f", _check_connection_frequency);
        ROS_DEBUG("DynamixelDriverCore::initParameters - dxl_end_effector_frequency : %f", _check_end_effector_frequency);
    }

    void DynamixelDriverCore::startControlLoop()
    {
        resetHardwareControlLoopRates();
        if (!_control_loop_flag)
        {
            ROS_INFO("DynamixelDriverCore::startControlLoop - Start control loop");
            _control_loop_flag = true;
            _control_loop_thread = std::thread(&DynamixelDriverCore::controlLoop, this);
        }
    }

    void DynamixelDriverCore::clearDxlSyncCommandQueue()
    {
        while(!_dxl_sync_cmds.empty())
            _dxl_sync_cmds.pop();
    }

    void DynamixelDriverCore::clearDxlCommandQueue()
    {
        while(!_dxl_single_cmds.empty())
            _dxl_single_cmds.pop();
    }

    void DynamixelDriverCore::clearDxlEndEffectorCommandQueue()
    {
        while(!_end_effector_cmds.empty())
            _end_effector_cmds.pop();
    }

    void DynamixelDriverCore::resetHardwareControlLoopRates()
    {
        ROS_DEBUG("DynamixelDriverCore::resetHardwareControlLoopRates - Reset control loop rates");
        double now = ros::Time::now().toSec();
        _time_hw_data_last_write = now;
        _time_hw_data_last_read = now;
        _time_hw_status_last_read = now;
        _time_check_connection_last_read = now;
        _time_check_end_effector_last_read = now;
    }

    void DynamixelDriverCore::activeDebugMode(bool mode)
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

    int DynamixelDriverCore::rebootMotors()
    {
        ROS_INFO("DynamixelDriverCore::rebootMotors - Reboot motors");
        lock_guard<mutex> lck(_control_loop_mutex);
        int result = _dynamixel->rebootMotors();
        ros::Duration(1.5).sleep();
        return result;
    }

    int DynamixelDriverCore::motorScanReport(uint8_t motor_id, EMotorType motor_type)
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

    int DynamixelDriverCore::motorCmdReport(uint8_t motor_id, EMotorType motor_type)
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

    int DynamixelDriverCore::launchMotorsReport()
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

    /**
     * @brief DynamixelDriverCore::addDxlCommandToQueue
     * @param cmd
     */
    void DynamixelDriverCore::addDxlSyncCommandToQueue(const SynchronizeMotorCmd &cmd)
    {
        ROS_DEBUG_THROTTLE(0.5, "DynamixelDriverCore::addDxlSyncCommandToQueue(%lu) - %s", _dxl_sync_cmds.size(), cmd.str().c_str());

        if(_dxl_sync_cmds.size() > 20)
            ROS_WARN_THROTTLE(0.5, "DynamixelDriverCore::addDxlSyncCommandToQueue: Cmd queue overflow ! %lu", _dxl_sync_cmds.size());
        else {

            //add only if the command is different from the last one added
            if(_dxl_sync_cmds.empty() || cmd != _dxl_sync_cmds.back())
                _dxl_sync_cmds.push(cmd);
        }
    }

    void DynamixelDriverCore::addDxlSyncCommandToQueue(const std::vector<SynchronizeMotorCmd> &cmd)
    {
        for(auto&& c : cmd)
            addDxlSyncCommandToQueue(c);
    }

    /**
     * @brief DynamixelDriverCore::addDxlCommandToQueue
     * @param cmd
     */
    void DynamixelDriverCore::addDxlCommandToQueue(const SingleMotorCmd &cmd)
    {
        ROS_DEBUG_THROTTLE(0.5, "DynamixelDriverCore::addDxlCommandToQueue - %s", cmd.str().c_str());


        if(_dxl_single_cmds.size() > 20)
            ROS_WARN_THROTTLE(0.5, "DynamixelDriverCore::addDxlCommandToQueue: Cmd queue overflow ! %lu", _dxl_single_cmds.size());
        else
            _dxl_single_cmds.push(cmd);
    }

    void DynamixelDriverCore::addDxlCommandToQueue(const std::vector<SingleMotorCmd> &cmd)
    {
        for(auto&& c : cmd)
            addDxlCommandToQueue(c);
    }


    void DynamixelDriverCore::addEndEffectorCommandToQueue(const SingleMotorCmd &cmd)
    {
        ROS_DEBUG_THROTTLE(0.5, "DynamixelDriverCore::addEndEffectorCommandToQueue - %s", cmd.str().c_str());

        if(_end_effector_cmds.size() > 20)
            ROS_WARN_THROTTLE(0.5, "DynamixelDriverCore::addEndEffectorCommandToQueue: Cmd queue overflow ! %lu", _end_effector_cmds.size());
        else
            _end_effector_cmds.push(cmd);
    }

    void DynamixelDriverCore::addEndEffectorCommandToQueue(const vector<SingleMotorCmd> &cmd)
    {

        for(auto&& c : cmd)
            addEndEffectorCommandToQueue(c);
    }

    vector<uint8_t> DynamixelDriverCore::scanTools()
    {
        vector<uint8_t> motor_list;
        lock_guard<mutex> lck(_control_loop_mutex);
        ROS_INFO("DynamixelDriverCore::scanTools - Scan tools...");
        int result = COMM_PORT_BUSY;

        for(int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
        {
            result = _dynamixel->getAllIdsOnDxlBus(motor_list);
            ROS_DEBUG_COND(COMM_SUCCESS != result, "DxlDriver::scanAndCheck status: %d (counter: %d)", result, counter);
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

    int DynamixelDriverCore::setEndEffector(EMotorType type, uint8_t id)
    {
        int result = this->ping_id(id, type);

        if (result != COMM_SUCCESS)
        {
            ROS_WARN("DynamixelDriverCore::setEndEffector - End effector dxl error: %d with motor id %d", result, id);
            result = niryo_robot_msgs::CommandStatus::DXL_READ_ERROR;
        }
        else
        {
            //add dynamixel as a new tool
            _dynamixel->addDynamixel(type, id, true);
            result = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        return result;
    }

    int DynamixelDriverCore::ping_id(uint8_t id, EMotorType type)
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        int result = _dynamixel->type_ping_id(id, type);
        ROS_DEBUG("DynamixelDriverCore::ping_id - Ping_id result: %d", result);
        return result;
    }

    void DynamixelDriverCore::unsetEndEffector(uint8_t id)
    {
        ROS_DEBUG("DynamixelDriverCore::unsetEndEffector - UnsetEndEffector: id %d", id);
        lock_guard<mutex> lck(_control_loop_mutex);
        _dynamixel->removeDynamixel(id);
    }

    // CC remove type
    uint32_t DynamixelDriverCore::getEndEffectorState(uint8_t id)
    {
        DxlMotorState motor_state = _dynamixel->getMotorsState(id);

        return static_cast<uint32_t>(motor_state.getPositionState());
    }

    dynamixel_driver::DxlArrayMotorHardwareStatus DynamixelDriverCore::getHwStatus() const
    {
        dynamixel_driver::DxlMotorHardwareStatus data;
        dynamixel_driver::DxlArrayMotorHardwareStatus hw_state;
        vector<DxlMotorState> motor_states = _dynamixel->getMotorsStates();

        for (int i = 0; i < motor_states.size(); i++)
        {
            data.motor_identity.motor_id = motor_states.at(i).getId();
            data.motor_identity.motor_type = static_cast<uint8_t>(motor_states.at(i).getType());
            data.temperature = motor_states.at(i).getTemperatureState();
            data.voltage = double(motor_states.at(i).getVoltageState()) / DXL_VOLTAGE_DIVISOR;
            data.error = motor_states.at(i).getHardwareErrorState();
            data.error_msg = motor_states.at(i).getHardwareErrorMessageState();
            hw_state.motors_hw_status.push_back(data);
        }
        return hw_state;
    }

    niryo_robot_msgs::BusState DynamixelDriverCore::getDxlBusState() const
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

    /**
     * @brief DynamixelDriverCore::update_leds
     * @return
     */
    int DynamixelDriverCore::update_leds(void)
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        int result = _dynamixel->setLeds(_dynamixel->getLedState(), EMotorType::MOTOR_TYPE_XL320);
        return result;
    }

    /**
     * @brief DynamixelDriverCore::_executeCommand : execute all the cmd in the current queue
     */ // create a unique queue using polymorphism
    void DynamixelDriverCore::_executeCommand()
    {
        bool need_sleep = false;
        if (!_dxl_sync_cmds.empty() && _dxl_single_cmds.empty())
        {
            _dynamixel->readSynchronizeCommand(_dxl_sync_cmds.front());
            _dxl_sync_cmds.pop();
            need_sleep = true;
        }
        if (!_dxl_single_cmds.empty())
        {
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _dynamixel->readSingleCommand(_dxl_single_cmds.front());
            _dxl_single_cmds.pop();
            need_sleep = true;
        }
        if (!_end_effector_cmds.empty())
        {
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _dynamixel->readSingleCommand(_end_effector_cmds.front());
            _end_effector_cmds.pop();
        }
    }

    /**
     * @brief DynamixelDriverCore::controlLoop
     */
    void DynamixelDriverCore::controlLoop()
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

                    int bus_state;
                    {
                        lock_guard<mutex> lck(_control_loop_mutex);
                        bus_state = _dynamixel->scanAndCheck();
                    }
                    while (bus_state != DXL_SCAN_OK)
                    { // wait for connection to be up
                        missing_ids = getRemovedMotorList();
                        for(vector<uint8_t>::const_iterator it = missing_ids.cbegin(); it != missing_ids.cend(); ++it) {
                            ROS_WARN_THROTTLE(2, "DynamixelDriverCore::controlLoop - Dynamixel %d do not seem to be connected", *it);
                        }
                        ros::Duration(0.25).sleep();
                        {
                            lock_guard<mutex> lck(_control_loop_mutex);
                            bus_state = _dynamixel->scanAndCheck();
                        }
                    }
                    ROS_INFO("DynamixelDriverCore::controlLoop - Dxl Bus ok");
                }

                if (_control_loop_flag)
                {
                    lock_guard<mutex> lck(_control_loop_mutex);
                    if (ros::Time::now().toSec() - _time_hw_data_last_read > 1.0 / _read_data_frequency)
                    {
                        _time_hw_data_last_read += 1.0 / _read_data_frequency;
                        _dynamixel->readPositionStatus();
                    }
                    if (ros::Time::now().toSec() - _time_hw_status_last_read > 1.0 / _read_status_frequency)
                    {
                        _time_hw_status_last_read += 1.0 / _read_status_frequency;
                        _dynamixel->readHwStatus();
                    }
                    if (ros::Time::now().toSec() - _time_hw_data_last_write > 1.0 / _write_frequency)
                    {
                        _time_hw_data_last_write += 1.0 / _write_frequency;
                        _executeCommand();
                    }
                    control_loop_rate.sleep();
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
     * @brief DynamixelDriverCore::callbackSendCustomDxlValue
     * @param req
     * @param res
     * @return
     */
    bool DynamixelDriverCore::callbackSendCustomDxlValue(dynamixel_driver::SendCustomDxlValue::Request &req,
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
    bool DynamixelDriverCore::callbackReadCustomDxlValue(dynamixel_driver::ReadCustomDxlValue::Request &req,
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
    bool DynamixelDriverCore::callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res)
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
// namespace DynamixelDriver
