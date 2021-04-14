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

namespace DynamixelDriver
{
    /**
     * @brief DxlDriver::DxlDriver
     */
    DxlDriver::DxlDriver() :
        _is_dxl_connection_ok(false),
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

    // CC to be moved outside of ctor ?
    /**
     * @brief DxlDriver::init
     * @return
     */
    void DxlDriver::init()
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
            ;
            common::model::EMotorType type = common::model::MotorTypeEnum(typeList.at(i).c_str());

            if(0 == _state_map.count(id))
            {
                if(common::model::EMotorType::MOTOR_TYPE_UNKNOWN != type)
                    addDynamixel(id, type);
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

            ROS_DEBUG("DxlDriver::init - Id map: %s => %s", common::model::MotorTypeEnum(id.first).toString().c_str(), listOfId.c_str());
        }

        for(auto const &d : _xdriver_map) {
            ROS_DEBUG("DxlDriver::init - Driver map: %s => %s", common::model::MotorTypeEnum(d.first).toString().c_str(), d.second->str().c_str());
        }
    }

    /**
     * @brief DxlDriver::addDynamixel
     * @param id
     * @param type
     */
    void DxlDriver::addDynamixel(uint8_t id, common::model::EMotorType type, bool isTool)
    {
        //add id to _ids_map
        _state_map.insert(make_pair(id, common::model::DxlMotorState(id, type, isTool)));

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
                case common::model::EMotorType::MOTOR_TYPE_XL430:
                    _xdriver_map.insert(make_pair(type, make_shared<XL430Driver>(_dxlPortHandler, _dxlPacketHandler)));
                break;
                case common::model::EMotorType::MOTOR_TYPE_XC430:
                    _xdriver_map.insert(make_pair(type, make_shared<XC430Driver>(_dxlPortHandler, _dxlPacketHandler)));
                break;
                case common::model::EMotorType::MOTOR_TYPE_XL320:
                    _xdriver_map.insert(make_pair(type, make_shared<XL320Driver>(_dxlPortHandler, _dxlPacketHandler)));
                break;
                case common::model::EMotorType::MOTOR_TYPE_XL330:
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
    void DxlDriver::removeDynamixel(uint8_t id, common::model::EMotorType )
    {
        if(_state_map.count(id)) {
            common::model::EMotorType type = _state_map.at(id).getType();

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

        _all_motor_connected.clear();
        int result = COMM_PORT_BUSY;

        for(int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
        {
            result = getAllIdsOnDxlBus(_all_motor_connected);
            ROS_DEBUG_COND(COMM_SUCCESS != result, "DxlDriver::scanAndCheck status: %d (counter: %d)", result, counter);
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (COMM_SUCCESS != result)
        {
            _debug_error_message = "Dxl Driver - Failed to scan motors, Dynamixel bus is too busy. Will retry...";
            ROS_WARN_THROTTLE(1, "DxlDriver::scanAndCheck - Failed to scan motors, dxl bus is too busy");
            return result;
        }

        checkRemovedMotors();
        if (_removed_motor_id_list.empty())
        {
            _is_dxl_connection_ok = true;
            _debug_error_message = "";
            return DXL_SCAN_OK;
        }
        _is_dxl_connection_ok = false;

        _debug_error_message = "Dynamixel(s):";
        for (auto const &it : _removed_motor_id_list)
        {
            _debug_error_message += " ";
            _debug_error_message += to_string(it);
        }
        _debug_error_message += " do not seem to be connected";

        return DXL_SCAN_MISSING_MOTOR;
    }

    /**
     * @brief DxlDriver::ping
     * @param targeted_dxl
     * @return
     */
    int DxlDriver::ping(common::model::DxlMotorState &targeted_dxl)
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
    int DxlDriver::type_ping_id(uint8_t id, common::model::EMotorType type)
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
            common::model::EMotorType type = it.second.getType();
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
    uint32_t DxlDriver::getPosition(common::model::DxlMotorState &targeted_dxl)
    {
        uint32_t result;
        common::model::EMotorType dxl_type = targeted_dxl.getType();

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
            _is_dxl_connection_ok = false;
            result = COMM_RX_FAIL;
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
            _is_dxl_connection_ok = false;
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

            _is_dxl_connection_ok = false;
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
        }
    }

    /**
     * @brief DxlDriver::getAllIdsOnDxlBus
     * @param id_list
     * @return
     */
    int DxlDriver::getAllIdsOnDxlBus(vector<uint8_t> &id_list)
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
                               common::model::MotorTypeEnum(it->first).toString().c_str());

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
     * @brief DxlDriver::executeJointTrajectoryCmd
     * @param cmd : always of size 3 -> only applies to the arm without the tool
     */
    void DxlDriver::executeJointTrajectoryCmd(vector<uint32_t> &cmd)
    {
        common::model::SynchronizeMotorCmd syncCmd(common::model::EDxlCommandType::CMD_TYPE_POSITION, getArmMotors(), cmd);

        ROS_DEBUG_THROTTLE(0.5, "DxlDriver::executeJointTrajectoryCmd");

        readSynchronizeCommand(syncCmd);
    }

    /**
     * @brief DxlDriver::readSynchronizeCommand
     * @param cmd
     */
    void DxlDriver::readSynchronizeCommand(common::model::SynchronizeMotorCmd cmd)
    {
        ROS_DEBUG_THROTTLE(0.5, "DxlDriver::readSynchronizeCommand:  %s", cmd.str().c_str());

        switch(cmd.getType())
        {
            case common::model::EDxlCommandType::CMD_TYPE_POSITION:
                syncWritePositionCommand(cmd.getMotorsId(), cmd.getParams());
            break;
            case common::model::EDxlCommandType::CMD_TYPE_VELOCITY:
                syncWriteVelocityCommand(cmd.getMotorsId(), cmd.getParams());
            break;
            case common::model::EDxlCommandType::CMD_TYPE_EFFORT:
                syncWriteEffortCommand(cmd.getMotorsId(), cmd.getParams());
            break;
            case common::model::EDxlCommandType::CMD_TYPE_TORQUE:
                syncWriteTorqueEnable(cmd.getMotorsId(), cmd.getParams());
            break;
            case common::model::EDxlCommandType::CMD_TYPE_LEARNING_MODE:
            {
                vector<uint8_t> id_list = getArmMotors();
                vector<uint32_t> cmd_param(id_list.size(), cmd.getParams()[0]);

                syncWriteTorqueEnable(id_list, cmd_param);
            }
            break;
            default:
                ROS_ERROR("DxlDriver::readSynchronizeCommand - Unknown command type: %d", static_cast<int>(cmd.getType()));
            break;
        }
    }

    /**
     * @brief DxlDriver::readSingleCommand
     * @param cmd
     */
    void DxlDriver::readSingleCommand(common::model::SingleMotorCmd cmd)
    {
        int result = -1;
        int counter = 0;
        uint8_t id = cmd.getId();

        ROS_DEBUG_THROTTLE(0.5, "DxlDriver::readSingleCommand:  %s", cmd.str().c_str());

        if(_state_map.count(id) != 0)
        {
            common::model::DxlMotorState state = _state_map.at(id);

            while ((COMM_SUCCESS != result) && (counter < 50))
            {
                switch(cmd.getType())
                {
                case common::model::EDxlCommandType::CMD_TYPE_VELOCITY:
                    result = setGoalVelocity(state, cmd.getParam());
                    break;
                case common::model::EDxlCommandType::CMD_TYPE_POSITION:
                    result = setGoalPosition(state, cmd.getParam());
                    break;
                case common::model::EDxlCommandType::CMD_TYPE_EFFORT:
                    result = setGoalTorque(state, cmd.getParam());
                    break;
                case common::model::EDxlCommandType::CMD_TYPE_TORQUE:
                    result = setTorqueEnable(state, cmd.getParam());
                    break;
                case common::model::EDxlCommandType::CMD_TYPE_PING:
                    result = ping(state);
                    break;
                default:
                    break;
                }

                counter += 1;

                ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
            }
        }

        if (result != COMM_SUCCESS)
        {
            ROS_WARN("DxlDriver::readSingleCommand - Failed to write a single command on dxl motor id : %d", id);
            _debug_error_message = "Dxl Driver - Failed to write a single command";
        }
    }

    /**
     * @brief DxlDriver::setGoalVelocity
     * @param targeted_dxl
     * @param velocity
     * @return
     */
    int DxlDriver::setGoalVelocity(common::model::DxlMotorState &targeted_dxl, uint32_t velocity)
    {
        int result = -1;

        common::model::EMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setGoalVelocity(targeted_dxl.getId(), velocity);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::setGoalVelocity - Wrong dxl type detected: %s", common::model::MotorTypeEnum(dxl_type).toString().c_str());
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }

        return result;
    }

    /**
     * @brief DxlDriver::setGoalPosition
     * @param targeted_dxl
     * @param position
     * @return
     */
    int DxlDriver::setGoalPosition(common::model::DxlMotorState &targeted_dxl, uint32_t position)
    {
        int result = -1;

        common::model::EMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) != 0 && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setGoalPosition(targeted_dxl.getId(), position);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::setGoalPosition - Wrong dxl type detected: %s", common::model::MotorTypeEnum(dxl_type).toString().c_str());
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    /**
     * @brief DxlDriver::setGoalTorque
     * @param targeted_dxl
     * @param torque
     * @return
     */
    int DxlDriver::setGoalTorque(common::model::DxlMotorState &targeted_dxl, uint32_t torque)
    {
        int result = -1;

        common::model::EMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setGoalTorque(targeted_dxl.getId(), torque);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::setGoalTorque - Wrong dxl type detected: %s", common::model::MotorTypeEnum(dxl_type).toString().c_str());
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }

        return result;
    }

    /**
     * @brief DxlDriver::setTorqueEnable
     * @param targeted_dxl
     * @param torque_enable
     * @return
     */
    int DxlDriver::setTorqueEnable(common::model::DxlMotorState &targeted_dxl, uint32_t torque_enable)
    {
        int result = -1;

        common::model::EMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setTorqueEnable(targeted_dxl.getId(), torque_enable);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::setTorqueEnable - Wrong dxl type detected: %s", common::model::MotorTypeEnum(dxl_type).toString().c_str());
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    /**
     * @brief DxlDriver::setLeds
     * @param led
     * @param type
     * @return
     */
    int DxlDriver::setLeds(int led, common::model::EMotorType type)
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
    int DxlDriver::sendCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id, uint32_t reg_address, uint32_t value,  uint32_t byte_number)
    {
        int result;
        ROS_DEBUG("DxlDriver::sendCustomDxlCommand:\n"
                  "\t\t Motor type: %d, ID: %d, Value: %d, Address: %d, Size: %d",
                  static_cast<int>(motor_type), static_cast<int>(id), static_cast<int>(value),
                  static_cast<int>(reg_address), static_cast<int>(byte_number));

        if(_xdriver_map.count(motor_type) && _xdriver_map.at(motor_type))
        {
            result = _xdriver_map.at(motor_type)->customWrite(id, static_cast<uint8_t>(reg_address), value, static_cast<uint8_t>(byte_number));
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("DxlDriver::sendCustomDxlCommand - Failed to write custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::sendCustomDxlCommand - driver for motor %s not available", common::model::MotorTypeEnum(motor_type).toString().c_str());
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
    int DxlDriver::readCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id,
                                        uint32_t reg_address, uint32_t &value, uint32_t byte_number)
    {
        int result;
        ROS_DEBUG("DxlDriver::readCustomDxlCommand:\n"
                  "Motor type: %d, ID: %d, Address: %d, Size: %d",
                  static_cast<int>(motor_type), static_cast<int>(id),
                  static_cast<int>(reg_address), static_cast<int>(byte_number));

        if(_xdriver_map.count(motor_type) && _xdriver_map.at(motor_type))
        {
            result = _xdriver_map.at(motor_type)->customRead(id, reg_address, value, byte_number);
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("DxlDriver::readCustomDxlCommand - Failed to write custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "DxlDriver::readCustomDxlCommand - driver for motor %s not available", common::model::MotorTypeEnum(motor_type).toString().c_str());
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
        ros::Duration(0.005).sleep();
        return result;
    }


    //********************
    //  Private
    //********************

    /**
     * @brief DxlDriver::getArmMotors
     * @return
     */
    vector<uint8_t> DxlDriver::getArmMotors()
    {
        vector<uint8_t> motors_list;

        for(auto const& m: _state_map) {
            if(!m.second.isTool())
                motors_list.push_back(m.first);
        }

        return motors_list;
    }

    /**
     * @brief DxlDriver::interpreteErrorState
     */
    void DxlDriver::interpreteErrorState()
    {
        for (auto const &it : _state_map)
        {
            common::model::DxlMotorState motor = it.second;
            common::model::EMotorType motor_type = motor.getType();

            uint32_t hw_state = motor.getHardwareErrorState();
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
                         &common::model::DxlMotorState::setPositionState);
    }

    /**
     * @brief DxlDriver::fillTemperatureStatus
     */
    void DxlDriver::fillTemperatureStatus()
    {
        readAndFillState(&XDriver::syncReadTemperature,
                         &common::model::DxlMotorState::setTemperatureState);
    }

    /**
     * @brief DxlDriver::fillVoltageStatus
     */
    void DxlDriver::fillVoltageStatus()
    {
        readAndFillState(&XDriver::syncReadVoltage,
                         &common::model::DxlMotorState::setVoltageState);
    }

    /**
     * @brief DxlDriver::fillErrorStatus
     */
    void DxlDriver::fillErrorStatus()
    {
        readAndFillState(&XDriver::syncReadHwErrorStatus,
                         &common::model::DxlMotorState::setHardwareError);
    }


    /**
     * @brief DxlDriver::readAndFillState
     */
    void DxlDriver::readAndFillState(
            int (XDriver::*syncReadFunction)(const vector<uint8_t>&, vector<uint32_t>&),
            void (common::model::DxlMotorState::*setFunction)(uint32_t))
    {
        // syncread from all drivers for all motors
        for(auto const& it : _xdriver_map)
        {
            common::model::EMotorType type = it.first;
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
                                (_state_map.at(id_list.at(i)).*setFunction)(position_list.at(i));
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
     * @brief DxlDriver::syncWriteTorqueEnable
     * @param motor_list
     * @param torque_list
     */
    void DxlDriver::syncWriteTorqueEnable(const vector<uint8_t>& motor_list,
                                          const vector<uint32_t>& torque_list)
    {
        syncWriteCommand(&XDriver::syncWriteTorqueEnable,
                         motor_list, torque_list);
    }

    /**
     * @brief DxlDriver::syncWritePositionCommand
     * @param motor_list
     * @param position_list
     */
    void DxlDriver::syncWritePositionCommand(const vector<uint8_t>& motor_list,
                                             const vector<uint32_t>& position_list)
    {
        syncWriteCommand(&XDriver::syncWritePositionGoal,
                         motor_list, position_list);
    }

    /**
     * @brief DxlDriver::syncWriteEffortCommand
     * @param motor_list
     * @param effort_list
     */
    void DxlDriver::syncWriteEffortCommand(const vector<uint8_t>& motor_list,
                                           const vector<uint32_t>& effort_list)
    {
        syncWriteCommand(&XDriver::syncWriteTorqueGoal,
                         motor_list, effort_list);
    }

    /**
     * @brief DxlDriver::syncWriteVelocityCommand
     * @param motor_list
     * @param velocity_list
     */
    void DxlDriver::syncWriteVelocityCommand(const vector<uint8_t>& motor_list,
                                             const vector<uint32_t>& velocity_list)
    {
        syncWriteCommand(&XDriver::syncWriteVelocityGoal,
                         motor_list, velocity_list);
    }

    /**
     * @brief DxlDriver::syncWriteCommand
     * @param motor_list
     * @param param_list
     *
     * // to be reformatted, not beautiful
     */
    void DxlDriver::syncWriteCommand(int (XDriver::*syncWriteFunction)(const vector<uint8_t>&, const vector<uint32_t>&),
                                     const vector<uint8_t>& motor_list,
                                     const vector<uint32_t>& param_list)
    {

        set<common::model::EMotorType> typesToProcess;
        for(auto const& it : _xdriver_map)
            typesToProcess.insert(it.first);

        //process all the motors using each successive drivers
        for(int counter = 0; counter < 25; ++counter)
        {
            ROS_DEBUG("DxlDriver::syncWriteCommand: try to sync write %d", counter);

            for(auto const& it : _xdriver_map) {
                if(it.second && typesToProcess.count(it.first) != 0) {
                    //syncwrite for this driver. The driver is responsible for sync write only to its associated motors
                    int results = ((it.second.get())->*syncWriteFunction)(motor_list, param_list);
                    //if successful, don't process this driver in the next loop
                    if (COMM_SUCCESS == results) {
                        typesToProcess.erase(typesToProcess.find(it.first));
                    }
                    else {
                        ROS_ERROR("DxlDriver::syncWriteCommand : unable to sync write function : %d", results);
                    }
                }
            }

            //if all drivers are processed, go out of for loop
            if(typesToProcess.empty())
                break;

            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (!typesToProcess.empty())
        {
            ROS_WARN("DxlDriver::syncWriteCommand - Failed to write synchronize position");
            _debug_error_message = "Dxl Driver - Failed to write synchronize position";
        }
    }

} // namespace DynamixelDriver
