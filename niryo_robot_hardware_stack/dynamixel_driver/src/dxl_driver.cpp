/*
    dxl_driver.cpp
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

#include "dynamixel_driver/dxl_driver.hpp"
#include "model/motor_type_enum.hpp"
#include "model/dxl_command_type_enum.hpp"
#include <string>
#include <algorithm>
#include <sstream>
#include <unordered_map>

using namespace std;
using namespace common::model;

namespace DynamixelDriver
{
    /**
     * @brief DxlDriver::DxlDriver
     */
    DxlDriver::DxlDriver() :
        _is_connection_ok(false),
        _debug_error_message("Dxl Driver - No connection with Dynamixel motors has been made yet"),
      _hw_fail_counter_read(0)
    {
        ROS_DEBUG("DxlDriver - ctor");

        init();

        if (COMM_SUCCESS != setupCommunication())
            ROS_WARN("DxlDriver - Dynamixel Communication Failed");
    }


    DxlDriver::~DxlDriver()
    {
        //we use an "init()" in the ctor. Thus there should be some kind of "uninit" in the dtor

    }

    /**
     * @brief DxlDriver::init
     * @return
     */
    bool DxlDriver::init()
    {
        // get params from rosparams
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_bus/dxl_uart_device_name", _device_name);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_bus/dxl_baudrate", _uart_baudrate);

        _dxlPortHandler.reset(dynamixel::PortHandler::getPortHandler(_device_name.c_str()));
        _dxlPacketHandler.reset(dynamixel::PacketHandler::getPacketHandler(DXL_BUS_PROTOCOL_VERSION));

        ROS_DEBUG("DxlDriver::init - Dxl : set port name (%s), baudrate(%d)", _device_name.c_str(), _uart_baudrate);

        //retrieve motor config
        vector<int> idList;
        vector<string> typeList;

        if (_nh.hasParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_id_list"))
            _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_id_list", idList);
        else
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/dxl_motor_id_list", idList);

        if (_nh.hasParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_type_list"))
            _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_type_list", typeList);
        else
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/dxl_motor_type_list", typeList);

        //debug - display info
        ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < idList.size() && i < typeList.size() ; ++i)
            ss << " id " << idList.at(i) << ": " << typeList.at(i) << ",";

        string motor_string_list = ss.str();
        motor_string_list.pop_back(); //remove last ","
        motor_string_list += "]";

        ROS_INFO("DxlDriver::init - Dxl motor list: %s ", motor_string_list.c_str());

        //check that the two lists have the same size
        if(idList.size() != typeList.size())
            ROS_ERROR("DxlDriver::init - wrong dynamixel configuration. Please check your configuration file dxl_motor_id_list and dxl_motor_type_list");

        //put everything in maps
        for(size_t i = 0; i < idList.size(); ++i) {

            uint8_t id = static_cast<uint8_t>(idList.at(i));
            EMotorType type = MotorTypeEnum(typeList.at(i).c_str());

            if(0 == _state_map.count(id))
            {
                if(EMotorType::MOTOR_TYPE_UNKNOWN != type)
                    addMotor(type, id);
                else
                    ROS_ERROR("DxlDriver::init - unknown type %s. Please check your configuration file (niryo_robot_hardware_stack/dynamixel_driver/config/motors_config.yaml)", typeList.at(id).c_str());
            }
            else
                ROS_ERROR("DxlDriver::init - duplicate id %d. Please check your configuration file (niryo_robot_hardware_stack/dynamixel_driver/config/motors_config.yaml)", id);
        }

        //display internal data for debug
        for(auto const &s : _state_map)
            ROS_DEBUG("DxlDriver::init - State map: %d => %s", s.first, s.second.str().c_str());

        for(auto const &id : _ids_map) {
            string listOfId;
            for(auto const &i : id.second) listOfId += to_string(i) + " ";

            ROS_DEBUG("DxlDriver::init - Id map: %s => %s", MotorTypeEnum(id.first).toString().c_str(), listOfId.c_str());
        }

        for(auto const &d : _xdriver_map) {
            ROS_DEBUG("DxlDriver::init - Driver map: %s => %s", MotorTypeEnum(d.first).toString().c_str(), d.second->str().c_str());
        }

        return COMM_SUCCESS;
    }

    /**
     * @brief DxlDriver::addDynamixel
     * @param id
     * @param type
     */
    void DxlDriver::addMotor(EMotorType type, uint8_t id, bool isTool)
    {
        ROS_DEBUG("DxlDriver::addMotor - Add motor id: %d", id);

        //add id to _state_map
        _state_map.insert(make_pair(id, DxlMotorState(type, id, isTool)));

        // if not already instanciated
        if(0 == _ids_map.count(type)) {
            _ids_map.insert(make_pair(type, vector<uint8_t>({id})));
        }
        else {
            _ids_map.at(type).push_back(id);
        }

        // if not already instanciated
        if(0 == _xdriver_map.count(type))
        {
            switch(type)
            {
                case EMotorType::MOTOR_TYPE_XL430:
                    _xdriver_map.insert(make_pair(type, make_shared<XL430Driver>(_dxlPortHandler, _dxlPacketHandler)));
                break;
                case EMotorType::MOTOR_TYPE_XC430:
                    _xdriver_map.insert(make_pair(type, make_shared<XC430Driver>(_dxlPortHandler, _dxlPacketHandler)));
                break;
                case EMotorType::MOTOR_TYPE_XL320:
                    _xdriver_map.insert(make_pair(type, make_shared<XL320Driver>(_dxlPortHandler, _dxlPacketHandler)));
                break;
                case EMotorType::MOTOR_TYPE_XL330:
                    _xdriver_map.insert(make_pair(type, make_shared<XL330Driver>(_dxlPortHandler, _dxlPacketHandler)));
                break;
                default:
                    ROS_ERROR("Dxl Driver - Unable to instanciate driver, unknown type");
                break;
            }
        }

    }

    /**
     * @brief DxlDriver::removeDynamixel
     * @param id
     */
    void DxlDriver::removeMotor(uint8_t id)
    {
        ROS_DEBUG("DxlDriver::removeMotor - Remove motor id: %d", id);

        if(_state_map.count(id)) {
            EMotorType type = _state_map.at(id).getType();

            if(_ids_map.count(type)) {
                auto ids = _ids_map.at(type);
                auto pos = find(ids.begin(), ids.end(), id);
                if (pos != ids.end())
                    ids.erase(pos);
            }

            _state_map.erase(id);
        }

        _removed_motor_id_list.push_back(id);
    }

    /**
     * @brief DxlDriver::setupCommunication
     * @return
     */
    int DxlDriver::setupCommunication()
    {
        int ret = COMM_NOT_AVAILABLE;

        ROS_DEBUG("DxlDriver::setupCommunication - initializing connection...");

        if(_dxlPortHandler) {
            _debug_error_message.clear();

            // setup half-duplex direction GPIO
            // see schema http://support.robotis.com/en/product/actuator/dynamixel_x/xl-series_main.htm
            if (!_dxlPortHandler->setupGpio())
            {
                ROS_ERROR("DxlDriver::setupCommunication - Failed to setup direction GPIO pin for Dynamixel half-duplex serial");
                _debug_error_message = "Dxl Driver -  Failed to setup direction GPIO pin for Dynamixel half-duplex serial";
                return DXL_FAIL_SETUP_GPIO;
            }

            // Open port
            if (!_dxlPortHandler->openPort())
            {
                ROS_ERROR("DxlDriver::setupCommunication - Failed to open Uart port for Dynamixel bus");
                _debug_error_message = "Dxl Driver - Failed to open Uart port for Dynamixel bus";
                return DXL_FAIL_OPEN_PORT;
            }

            // Set baudrate
            if (!_dxlPortHandler->setBaudRate(_uart_baudrate))
            {
                ROS_ERROR("DxlDriver::setupCommunication - Failed to set baudrate for Dynamixel bus");
                _debug_error_message = "Dxl Driver - Failed to set baudrate for Dynamixel bus";
                return DXL_FAIL_PORT_SET_BAUDRATE;
            }

            //wait a bit to be sure the connection is established
            ros::Duration(0.1).sleep();
            ret = COMM_SUCCESS;
        }
        else
            ROS_ERROR("DxlDriver::setupCommunication - Invalid port handler");


        return ret;
    }

    //****************
    //  commands
    //****************

    /**
     * @brief DxlDriver::scanAndCheck
     * @return
     */
    int DxlDriver::scanAndCheck()
    {
        ROS_DEBUG("DxlDriver::scanAndCheck");
        int result = COMM_PORT_BUSY;

        _all_motor_connected.clear();
        _is_connection_ok = false;

        for(int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
        {
            result = getAllIdsOnBus(_all_motor_connected);
            ROS_DEBUG_COND(COMM_SUCCESS != result, "DxlDriver::scanAndCheck status: %d (counter: %d)", result, counter);
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (COMM_SUCCESS == result)
        {
            checkRemovedMotors();

            if (_removed_motor_id_list.empty())
            {
                _is_connection_ok = true;
                _debug_error_message = "";
                result = DXL_SCAN_OK;
            }
            else
            {
                _debug_error_message = "Dynamixel(s):";
                for (auto const &id : _removed_motor_id_list) {
                    _debug_error_message += " " + to_string(id);
                }
                _debug_error_message += " do not seem to be connected";
            }
        }
        else
        {
            _debug_error_message = "Dxl Driver - Failed to scan motors, Dynamixel bus is too busy. Will retry...";
            ROS_WARN_THROTTLE(1, "DxlDriver::scanAndCheck - Failed to scan motors, dxl bus is too busy");
        }

        return result;
    }

    /**
     * @brief DxlDriver::ping
     * @param targeted_dxl
     * @return
     */
    int DxlDriver::ping(DxlMotorState &targeted_dxl)
    {
        int result = 0;

        if(_xdriver_map.count(targeted_dxl.getType())) {
            result = _xdriver_map.at(targeted_dxl.getType())->ping(targeted_dxl.getId());
        }
        else
            ROS_ERROR_THROTTLE(1, "DxlDriver::ping - Wrong dxl type detected: %d", static_cast<int>(targeted_dxl.getType()));

        return result;
    }

    /**
     * @brief DxlDriver::type_ping_id
     * @param id
     * @param type
     * @return
     */
    int DxlDriver::type_ping_id(uint8_t id, EMotorType type)
    {
        if(_xdriver_map.count(type) && _xdriver_map.at(type)) {
            return _xdriver_map.at(type)->ping(id);
        }

        return COMM_RX_FAIL;
    }

    /**
     * @brief DxlDriver::rebootMotors
     * @return
     */
    int DxlDriver::rebootMotors()
    {
        int return_value = COMM_SUCCESS;
        int result = 0;

        for(auto const &it : _state_map)
        {
            EMotorType type = it.second.getType();
            ROS_DEBUG("DxlDriver::rebootMotors - Reboot Dxl motor with ID: %d", it.first);
            if(_xdriver_map.count(type)) {
                result = _xdriver_map.at(type)->reboot(it.first);
                if (result != COMM_SUCCESS)
                {
                    ROS_WARN("DxlDriver::rebootMotors - Failed to reboot motor: %d", result);
                    return_value = result;
                }
            }
        }

        return return_value;
    }

    //******************
    //  Read operations
    //******************

    /**
     * @brief DxlDriver::getPosition
     * @param targeted_dxl
     * @return
     */
    uint32_t DxlDriver::getPosition(DxlMotorState &targeted_dxl)
    {
        uint32_t result = 0;
        EMotorType dxl_type = targeted_dxl.getType();

        if(_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type)) {

            for(_hw_fail_counter_read = 0; _hw_fail_counter_read < 25; _hw_fail_counter_read++)
            {
                if (_xdriver_map.at(dxl_type)->readPosition(targeted_dxl.getId(), &result) == COMM_SUCCESS)
                {
                    _hw_fail_counter_read = 0;
                    break;
                }
            }
        }

        if (0 < _hw_fail_counter_read)
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::getPosition - Dxl connection problem - Failed to read from Dxl bus");
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
            _hw_fail_counter_read = 0;
            _is_connection_ok = false;
        }

        return result;
    }


    /**
     * @brief DxlDriver::readPositionState
     */
    void DxlDriver::readPositionStatus()
    {
        if (!hasMotors())
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::readPositionStatus - No motor");
            _debug_error_message = "DxlDriver::readPositionStatus -  No motor";
            return;
        }
        fillPositionStatus();

        if (_hw_fail_counter_read > 25)
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::readPositionStatus - Dxl connection problem - Failed to read from Dxl bus");
            _hw_fail_counter_read = 0;
            _is_connection_ok = false;
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
        }
    }

    /**
     * @brief DxlDriver::readHwStatus
     */
    void DxlDriver::readHwStatus()
    {
        if (!hasMotors())
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::readHwStatus - No motor");
            _debug_error_message = "Dxl Driver - No motor";
            return;
        }
        fillTemperatureStatus();
        fillVoltageStatus();
        fillErrorStatus();

        interpreteErrorState();

        if (_hw_fail_counter_read > 25 )
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::readHwStatus - Dxl connection problem - Failed to read from Dxl bus");
            _hw_fail_counter_read = 0;

            _is_connection_ok = false;
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
        }
    }

    /**
     * @brief DxlDriver::getAllIdsOnDxlBus
     * @param id_list
     * @return
     */
    int DxlDriver::getAllIdsOnBus(vector<uint8_t> &id_list)
    {
        int result = COMM_RX_FAIL;

        // 1. Get all ids from dxl bus. We can use any driver for that
        auto it = _xdriver_map.begin();
        if(it != _xdriver_map.end() && it->second)
        {
            result = it->second->scan(id_list);

            string ids_str;
            for(auto const &id : id_list)
                ids_str += to_string(id) + " ";

            ROS_DEBUG_THROTTLE(1, "DxlDriver::getAllIdsOnDxlBus - Found ids (%s) on bus using first driver (type: %s)", ids_str.c_str(),
                               MotorTypeEnum(it->first).toString().c_str());

            if (COMM_SUCCESS != result)
            {
                if (COMM_RX_TIMEOUT != result)
                { // -3001
                    _debug_error_message = "Dxl Driver - No Dynamixel motor found. Make sure that motors are correctly connected and powered on.";
                }
                else
                { // -3002 or other
                    _debug_error_message = "Dxl Driver - Failed to scan Dynamixel bus.";
                }
                ROS_WARN_THROTTLE(1, "DxlDriver::getAllIdsOnDxlBus - Broadcast ping failed , result : %d (-3001: timeout, -3002: corrupted packet)", result);
            }
        }

        return result;
    }

    //******************
    //  Write operations
    //******************

    /**
     * @brief DxlDriver::readSynchronizeCommand
     * @param cmd
     */
    int DxlDriver::readSynchronizeCommand(SynchronizeMotorCmd cmd)
    {   
        int result = COMM_TX_ERROR;
        ROS_DEBUG_THROTTLE(0.5, "DxlDriver::readSynchronizeCommand:  %s", cmd.str().c_str());

        if(cmd.isValid()) {
            switch(cmd.getType())
            {
                case EDxlCommandType::CMD_TYPE_POSITION:
                    result = _syncWrite(&XDriver::syncWritePositionGoal, cmd);
                break;
                case EDxlCommandType::CMD_TYPE_VELOCITY:
                    result = _syncWrite(&XDriver::syncWriteVelocityGoal, cmd);
                break;
                case EDxlCommandType::CMD_TYPE_EFFORT:
                    result = _syncWrite(&XDriver::syncWriteTorqueGoal, cmd);
                break;
                case EDxlCommandType::CMD_TYPE_TORQUE:
                    result = _syncWrite(&XDriver::syncWriteTorqueEnable, cmd);
                break;
                case EDxlCommandType::CMD_TYPE_LEARNING_MODE:
                    result = _syncWrite(&XDriver::syncWriteTorqueEnable, cmd);
                break;
                default:
                    ROS_ERROR("DxlDriver::readSynchronizeCommand - Unsupported command type: %d", static_cast<int>(cmd.getType()));
                break;
            }
        }
        else {
            ROS_ERROR("DxlDriver::readSynchronizeCommand - Invalid command");
        }

        return result;
    }

    /**
     * @brief DxlDriver::readSingleCommand
     * @param cmd
     */
    int DxlDriver::readSingleCommand(SingleMotorCmd cmd)
    {
        int result = COMM_TX_ERROR;
        uint8_t id = cmd.getId();

        if(cmd.isValid()) {
            int counter = 0;

            ROS_DEBUG_THROTTLE(0.5, "DxlDriver::readSingleCommand:  %s", cmd.str().c_str());

            if(_state_map.count(id) != 0)
            {
                DxlMotorState state = _state_map.at(id);

                while ((COMM_SUCCESS != result) && (counter < 50))
                {
                    switch(cmd.getType())
                    {
                    case EDxlCommandType::CMD_TYPE_VELOCITY:
                        result = _singleWrite(&XDriver::setGoalVelocity, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_POSITION:
                        result = _singleWrite(&XDriver::setGoalPosition, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_EFFORT:
                        result = _singleWrite(&XDriver::setGoalTorque, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_TORQUE:
                        result = _singleWrite(&XDriver::setTorqueEnable, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_P_GAIN:
                        result = _singleWrite(&XDriver::setPGain, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_I_GAIN:
                        result = _singleWrite(&XDriver::setIGain, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_D_GAIN:
                        result = _singleWrite(&XDriver::setDGain, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_FF1_GAIN:
                        result = _singleWrite(&XDriver::setff1Gain, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_FF2_GAIN:
                        result = _singleWrite(&XDriver::setff2Gain, state.getType(), cmd);
                        break;
                    case EDxlCommandType::CMD_TYPE_PING:
                        result = ping(state);
                        break;
                    default:
                        break;
                    }

                    counter += 1;

                    ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                }
            }
        }

        if (result != COMM_SUCCESS)
        {
            ROS_WARN("DxlDriver::readSingleCommand - Failed to write a single command on dxl motor id : %d", id);
            _debug_error_message = "Dxl Driver - Failed to write a single command";
        }

        return result;
    }


    /**
     * @brief DxlDriver::setLeds
     * @param led
     * @param type
     * @return
     */
    int DxlDriver::setLeds(int led, EMotorType type)
    {
        int ret = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
        _led_state = led;

        //get list of motors of the given type
        vector<uint8_t> id_list;
        if(_ids_map.count(type) && _xdriver_map.count(type)) {
            id_list = _ids_map.at(type);

            auto driver = _xdriver_map.at(type);

            //sync write led state
            vector<uint32_t> command_led_id(id_list.size(), static_cast<uint32_t>(led));
            if (0 <= led && 7 >= led)
            {
                int result = COMM_TX_FAIL;
                for(int error_counter = 0; result != COMM_SUCCESS && error_counter < 5; ++error_counter)
                {
                    ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                    result = driver->syncWriteLed(id_list, command_led_id);
                }

                if (COMM_SUCCESS == result)
                    ret = niryo_robot_msgs::CommandStatus::SUCCESS;
                else
                    ROS_WARN("DxlDriver::setLeds - Failed to write LED");
            }
        }

        return ret;
    }

    /**
     * @brief DxlDriver::sendCustomDxlCommand
     * @param motor_type
     * @param id
     * @param reg_address
     * @param value
     * @param byte_number
     * @return
     */
    int DxlDriver::sendCustomDxlCommand(EMotorType motor_type, uint8_t id,
                                        int reg_address, int value,  int byte_number)
    {
        int result = COMM_TX_FAIL;
        ROS_DEBUG("DxlDriver::sendCustomDxlCommand:\n"
                  "\t\t Motor type: %d, ID: %d, Value: %d, Address: %d, Size: %d",
                  static_cast<int>(motor_type), static_cast<int>(id), value,
                  reg_address, byte_number);

        if(_xdriver_map.count(motor_type) && _xdriver_map.at(motor_type))
        {
            result = _xdriver_map.at(motor_type)->write(static_cast<uint8_t>(reg_address),
                                                        static_cast<uint8_t>(byte_number),
                                                        id,
                                                        static_cast<uint32_t>(value));
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("DxlDriver::sendCustomDxlCommand - Failed to write custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::sendCustomDxlCommand - driver for motor %s not available", MotorTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
        ros::Duration(0.005).sleep();
        return result;
    }

    /**
     * @brief DxlDriver::readCustomDxlCommand
     * @param motor_type
     * @param id
     * @param reg_address
     * @param value
     * @param byte_number
     * @return
     */
    int DxlDriver::readCustomDxlCommand(EMotorType motor_type, uint8_t id,
                                        int32_t reg_address, int& value, int byte_number)
    {
        int result = COMM_RX_FAIL;
        ROS_DEBUG("DxlDriver::readCustomDxlCommand: "
                  "Motor type: %d, ID: %d, Address: %d, Size: %d",
                  static_cast<int>(motor_type), static_cast<int>(id),
                  static_cast<int>(reg_address), byte_number);

        if(_xdriver_map.count(motor_type) && _xdriver_map.at(motor_type))
        {
            uint32_t data = 0;
            result = _xdriver_map.at(motor_type)->read(static_cast<uint8_t>(reg_address),
                                                       static_cast<uint8_t>(byte_number),
                                                       id,
                                                       &data);
            value = static_cast<int>(data);

            if (result != COMM_SUCCESS)
            {
                ROS_WARN("DxlDriver::readCustomDxlCommand - Failed to read custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::readCustomDxlCommand - driver for motor %s not available", MotorTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
        ros::Duration(0.005).sleep();
        return result;
    }

    size_t DxlDriver::getNbMotors() const
    {
        return _state_map.size();
    }


    //********************
    //  Private
    //********************

    /**
     * @brief DxlDriver::interpreteErrorState
     */
    void DxlDriver::interpreteErrorState()
    {
        for (auto const &it : _state_map)
        {
            DxlMotorState motor = it.second;
            EMotorType motor_type = motor.getType();

            uint32_t hw_state = static_cast<uint32_t>(motor.getHardwareErrorState());
            string hardware_message;

            if(_xdriver_map.count(motor_type) && _xdriver_map.at(motor_type)) {
                hardware_message = _xdriver_map.at(motor_type)->interpreteErrorState(hw_state);
            }

            motor.setHardwareError(hardware_message);
        }
    }

    /**
     * @brief DxlDriver::checkRemovedMotors
     */
    void DxlDriver::checkRemovedMotors()
    {
        //get list of ids
        vector<uint8_t> motor_list;

        for(auto const& istate: _state_map) {
            auto it = find(_all_motor_connected.begin(), _all_motor_connected.end(), istate.first);
            if (it == _all_motor_connected.end())
                motor_list.push_back(istate.first);
        }
        _removed_motor_id_list = motor_list;
    }

    /**
     * @brief DxlDriver::fillPositionStatus
     */
    void DxlDriver::fillPositionStatus()
    {
        readAndFillState(&XDriver::syncReadPosition,
                         &DxlMotorState::setPositionState);
    }

    /**
     * @brief DxlDriver::fillTemperatureStatus
     */
    void DxlDriver::fillTemperatureStatus()
    {
        readAndFillState(&XDriver::syncReadTemperature,
                         &DxlMotorState::setTemperatureState);
    }

    /**
     * @brief DxlDriver::fillVoltageStatus
     */
    void DxlDriver::fillVoltageStatus()
    {
        readAndFillState(&XDriver::syncReadVoltage,
                         &DxlMotorState::setVoltageState);
    }

    /**
     * @brief DxlDriver::fillErrorStatus
     */
    void DxlDriver::fillErrorStatus()
    {
        readAndFillState(&XDriver::syncReadHwErrorStatus,
                         &DxlMotorState::setHardwareError);
    }


    /**
     * @brief DxlDriver::readAndFillState
     */
    void DxlDriver::readAndFillState(
            int (XDriver::*syncReadFunction)(const vector<uint8_t>&, vector<uint32_t>&),
            void (DxlMotorState::*setFunction)(int))
    {
        // syncread from all drivers for all motors
        for(auto const& it : _xdriver_map)
        {
            EMotorType type = it.first;
            shared_ptr<XDriver> driver = it.second;

            if(driver && _ids_map.count(type) != 0)
            {
                //we retrieve all the associated id for the type of the current driver
                vector<uint8_t> id_list = _ids_map.at(type);
                vector<uint32_t> position_list;

                if ((driver.get()->*syncReadFunction)(id_list, position_list) == COMM_SUCCESS)
                {
                    _hw_fail_counter_read = 0;
                    if(id_list.size() == position_list.size())
                    {
                        // set motors states accordingly
                        for(size_t i = 0; i < id_list.size(); ++i) {
                            if(_state_map.count(id_list.at(i)) != 0)
                                (_state_map.at(id_list.at(i)).*setFunction)(static_cast<int>(position_list.at(i)));
                        }
                    }
                    else {
                        ROS_ERROR( "DxlDriver::readAndFillState : syncreadfunction return vectors mismatch (id_list size %d, position_list size %d)",
                                       static_cast<int>(id_list.size()), static_cast<int>(position_list.size()));
                        _hw_fail_counter_read++;
                    }
                }
                else
                    _hw_fail_counter_read++;
            }
        }

        // no need for fillMotorState
    }

    /**
     * @brief DxlDriver::_syncWrite
     * @param motor_list
     * @param param_list
     *
     * // to be reformatted, not beautiful
     */
    int DxlDriver::_syncWrite(int (XDriver::*syncWriteFunction)(const vector<uint8_t>&, const vector<uint32_t>&),
                                     const SynchronizeMotorCmd& cmd)
    {
        int result = COMM_TX_ERROR;

        set<EMotorType> typesToProcess = cmd.getMotorTypes();

        //process all the motors using each successive drivers
        for(int counter = 0; counter < 25; ++counter)
        {
            ROS_DEBUG_THROTTLE(0.5, "DxlDriver::_syncWrite: try to sync write (counter %d)", counter);

            for(auto const& it : _xdriver_map) {
                if(it.second && typesToProcess.count(it.first) != 0) {
                    //syncwrite for this driver. The driver is responsible for sync write only to its associated motors
                    int results = ((it.second.get())->*syncWriteFunction)(cmd.getMotorsId(it.first), cmd.getParams(it.first));
                    ros::Duration(0.05).sleep();
                    //if successful, don't process this driver in the next loop
                    if (COMM_SUCCESS == results) {
                        typesToProcess.erase(typesToProcess.find(it.first));
                    }
                    else {
                        ROS_ERROR("DxlDriver::_syncWrite : unable to sync write function : %d", results);
                    }
                }
            }

            //if all drivers are processed, go out of for loop
            if(typesToProcess.empty()) {
                result = COMM_SUCCESS;
                break;
            }

            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if(COMM_SUCCESS != result)
        {
            ROS_ERROR_THROTTLE(0.5, "DxlDriver::_syncWrite - Failed to write synchronize position");
            _debug_error_message = "Dxl Driver - Failed to write synchronize position";
        }

        return result;
    }

    /**
     * @brief DxlDriver::_singleWrite
     * @param dxl_type
     * @param cmd
     * @return
     */
    int DxlDriver::_singleWrite(int (XDriver::*singleWriteFunction)(uint8_t, uint32_t),
                                       EMotorType dxl_type,
                                       const SingleMotorCmd &cmd)
    {
        int result = COMM_TX_ERROR;

        if (_xdriver_map.count(dxl_type) != 0 && _xdriver_map.at(dxl_type))
        {
            result = (_xdriver_map.at(dxl_type).get()->*singleWriteFunction)(cmd.getId(), cmd.getParam());
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::_singleWrite - Wrong dxl type detected: %s", MotorTypeEnum(dxl_type).toString().c_str());
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    DxlMotorState DxlDriver::getMotorState(uint8_t motor_id) const
    {
        if(!_state_map.count(motor_id))
            throw std::out_of_range("DxlDriver::getMotorsState: Unknown motor id");

        return _state_map.at(motor_id);
    }

    std::vector<DxlMotorState> DxlDriver::getMotorsStates() const
    {
        std::vector<common::model::DxlMotorState> states;
        for (auto const& it : _state_map)
            states.push_back(it.second);

        return states;
    }


} // namespace DynamixelDriver
