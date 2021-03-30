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
#include "dynamixel_driver/dxl_enum.hpp"
#include <string>
#include <algorithm>
#include <sstream>

namespace DynamixelDriver
{
    /**
     * @brief DxlDriver::DxlDriver
     */
    DxlDriver::DxlDriver() :
        _is_dxl_connection_ok(false),
        _hw_fail_counter_read(0),
        _debug_error_message("Dxl Driver - No connection with Dynamixel motors has been made yet")
    {

        init();

        if (COMM_SUCCESS != setupCommunication())
            ROS_WARN("Dxl Driver - Dynamixel Communication Failed");
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
    int DxlDriver::init()
    {
        //retrieve debug_mode from config file
        bool debug_mode = false;
        _nh.getParam("/niryo_robot_hardware_interface/debug", debug_mode);

        if (debug_mode && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
            ros::console::notifyLoggerLevelsChanged();
        }

        // get params from rosparams
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_bus/dxl_uart_device_name", _device_name);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_bus/dxl_baudrate", _uart_baudrate);

        _dxlPortHandler.reset(dynamixel::PortHandler::getPortHandler(_device_name.c_str()));
        _dxlPacketHandler.reset(dynamixel::PacketHandler::getPacketHandler(DXL_BUS_PROTOCOL_VERSION));

        ROS_DEBUG("Dxl Driver - Dxl : set port name (%s), baudrate(%d)", _device_name.c_str(), _uart_baudrate);

        //retrieve motor config
        std::vector<int> idList;
        std::vector<std::string> typeList;

        if (_nh.hasParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_id_list"))
            _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_id_list", idList);
        else
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/dxl_motor_id_list", idList);

        if (_nh.hasParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_type_list"))
            _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_type_list", typeList);
        else
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/dxl_motor_type_list", typeList);

        //debug - display info
        std::ostringstream ss;
        ss << "[";
        for (auto i = 0; i < idList.size(); ++i)
            ss << " id " << idList.at(i) << ": " << typeList.at(i) << ",";

        std::string motor_string_list = ss.str();
        motor_string_list.pop_back(); //remove last ","
        motor_string_list += "]";

        ROS_INFO("Dxl Driver - Dxl motor list: %s ", motor_string_list.c_str());

        //check that the two lists have the same size
        if(idList.size() != typeList.size())
            ROS_ERROR("Dxl Driver - wrong dynamixel configuration. Please check your configuration file dxl_motor_id_list and dxl_motor_type_list");

        //put everything in maps
        for(auto const &id : idList) {

            DxlMotorType type = dxlMotorTypeFromString(typeList.at(id));

            if(DxlMotorType::UNKNOWN_TYPE != type)
            {
                // add dynamixel to the maps
                addDynamixel(id, type);
            }
            else
            {
                ROS_ERROR("Dxl Driver - unknown type %s. Please check your configuration file dxl_motor_id_list and dxl_motor_type_list", typeList.at(id).c_str());
            }
        }

    }


    /**
     * @brief DxlDriver::addDynamixel
     * @param id
     * @param type
     */
    void DxlDriver::addDynamixel(uint8_t id, DxlMotorType type)
    {
        //add id to _ids_map
        _state_map[id] = DxlMotorState(id, type);
        if(_ids_map.count(type)) {
            _ids_map[type] = std::vector<uint8_t>({id});
        }
        else
            _ids_map[type].push_back(id);

        //only if not already instanciated
        if(_xdriver_map.count(type))
        {
            switch(type)
            {
                case DxlMotorType::MOTOR_TYPE_XL430:
                    _xdriver_map[type] = std::make_shared<XL430Driver>(_dxlPortHandler, _dxlPacketHandler);
                break;
                case DxlMotorType::MOTOR_TYPE_XC430:
                    _xdriver_map[type] = std::make_shared<XC430Driver>(_dxlPortHandler, _dxlPacketHandler);
                break;
                case DxlMotorType::MOTOR_TYPE_XL320:
                    _xdriver_map[type] = std::make_shared<XL320Driver>(_dxlPortHandler, _dxlPacketHandler);
                break;
                case DxlMotorType::MOTOR_TYPE_XL330:
                    _xdriver_map[type] = std::make_shared<XL330Driver>(_dxlPortHandler, _dxlPacketHandler);
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
    void DxlDriver::removeDynamixel(uint8_t id, DxlMotorType )
    {
        if(_state_map.count(id)) {
            DxlMotorType type = _state_map.at(id).getType();

            if(_ids_map.count(type)) {
                auto ids = _ids_map.at(type);
                auto pos = std::find(ids.begin(), ids.end(), id);
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

        if(!_dxlPortHandler) {
            _debug_error_message.clear();

            // setup half-duplex direction GPIO
            // see schema http://support.robotis.com/en/product/actuator/dynamixel_x/xl-series_main.htm
            if (!_dxlPortHandler->setupGpio())
            {
                ROS_ERROR("Dxl Driver - Failed to setup direction GPIO pin for Dynamixel half-duplex serial");
                _debug_error_message = "Dxl Driver -  Failed to setup direction GPIO pin for Dynamixel half-duplex serial";
                return DXL_FAIL_SETUP_GPIO;
            }

            // Open port
            if (!_dxlPortHandler->openPort())
            {
                ROS_ERROR("Dxl Driver - Failed to open Uart port for Dynamixel bus");
                _debug_error_message = "Dxl Driver - Failed to open Uart port for Dynamixel bus";
                return DXL_FAIL_OPEN_PORT;
            }

            // Set baudrate
            if (!_dxlPortHandler->setBaudRate(_uart_baudrate))
            {
                ROS_ERROR("Dxl Driver - Failed to set baudrate for Dynamixel bus");
                _debug_error_message = "Dxl Driver - Failed to set baudrate for Dynamixel bus";
                return DXL_FAIL_PORT_SET_BAUDRATE;
            }

            //wait a bit to be sure the connection is established
            ros::Duration(0.1).sleep();
            ret = COMM_SUCCESS;
        }

        return ret;
    }

    //****************
    //  commands
    //****************

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
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)targeted_dxl.getType());

        return result;
    }

    /**
     * @brief DxlDriver::type_ping_id
     * @param id
     * @param type
     * @return
     */
    int DxlDriver::type_ping_id(uint8_t id, DxlMotorType type)
    {
        if(_xdriver_map.count(type)) {
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
            DxlMotorType type = it.second.getType();
            ROS_DEBUG("Dxl Driver - Reboot Dxl motor with ID: %d", it.first);
            if(_xdriver_map.count(type)) {
                result = _xdriver_map.at(type)->reboot(it.first);
                if (result != COMM_SUCCESS)
                {
                    ROS_WARN("Dxl Driver - Failed to reboot motor: %d", result);
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
        uint32_t result;
        int read_result;
        DxlMotorType dxl_type = targeted_dxl.getType();

        _hw_fail_counter_read = 0;
        if(_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type)) {
            while (_hw_fail_counter_read < 25)
            {
                read_result = _xdriver_map.at(dxl_type)->readPosition(targeted_dxl.getId(), &result);

                if (read_result == COMM_SUCCESS)
                {
                    _hw_fail_counter_read = 0;
                    break;
                }
                else
                {
                    _hw_fail_counter_read++;
                }
            }
        }

        if (0 < _hw_fail_counter_read)
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Dxl connection problem - Failed to read from Dxl bus");
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
            _hw_fail_counter_read = 0;
            _is_dxl_connection_ok = false;
            result = COMM_RX_FAIL;
        }

        return result;
    }



    //******************
    //  Write operations
    //******************

    /**
     * @brief DxlDriver::setTorqueEnable
     * @param targeted_dxl
     * @param torque_enable
     * @return
     */
    int DxlDriver::setTorqueEnable(DxlMotorState &targeted_dxl, uint32_t torque_enable)
    {
        int result = -1;

        DxlMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setTorqueEnable(targeted_dxl.getId(), torque_enable);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    /**
     * @brief DxlDriver::setGoalVelocity
     * @param targeted_dxl
     * @param velocity
     * @return
     */
    int DxlDriver::setGoalVelocity(DxlMotorState &targeted_dxl, uint32_t velocity)
    {
        int result = -1;

        DxlMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setGoalVelocity(targeted_dxl.getId(), velocity);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
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
    int DxlDriver::setGoalTorque(DxlMotorState &targeted_dxl, uint32_t torque)
    {
        int result = -1;

        DxlMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setGoalTorque(targeted_dxl.getId(), torque);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
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
    int DxlDriver::setGoalPosition(DxlMotorState &targeted_dxl, uint32_t position)
    {
        int result = -1;

        DxlMotorType dxl_type = targeted_dxl.getType();

        if (_xdriver_map.count(dxl_type) != 0 && _xdriver_map.at(dxl_type))
        {
            result = _xdriver_map.at(dxl_type)->setGoalPosition(targeted_dxl.getId(), position);
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
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
    int DxlDriver::setLeds(int led, DxlMotorType type)
    {
        _led_state = led;

        //get list of motors of the given type
        std::vector<uint8_t> id_list;
        if(_ids_map.count(type) && _xdriver_map.count(type)) {
            id_list = _ids_map.at(type);

            auto driver = _xdriver_map.at(type);

            //sync write led state
            std::vector<uint32_t> command_led_id(id_list.size(), (uint32_t)led);
            if (0 <= led && 7 >= led)
            {
                int error_counter = 0;
                int result = driver->syncWriteLed(id_list, command_led_id);
                while (result != COMM_SUCCESS && error_counter < 5)
                {
                    ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                    result = driver->syncWriteLed(id_list, command_led_id);
                    error_counter++;
                }

                if (result != COMM_SUCCESS)
                {
                    ROS_WARN("Dxl Driver - Failed to write LED");
                    return niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
                }
            }
        }


        return niryo_robot_msgs::CommandStatus::SUCCESS;
    }

    /**
     * @brief DxlDriver::readAndFillState
     */
    void DxlDriver::readAndFillState(
            int (XDriver::*syncReadFunction)(std::vector<uint8_t> &, std::vector<uint32_t> &),
            void (DxlMotorState::*setFunction)(uint32_t))
    {
        // syncread from all drivers for all motors
        for(auto it : _xdriver_map)
        {
            DxlMotorType type = it.first;
            std::shared_ptr<XDriver> driver = it.second;

            if(driver && _ids_map.count(type) != 0)
            {
                std::vector<uint8_t> id_list = _ids_map.at(type);
                std::vector<uint32_t> position_list;

                if ((driver.get()->*syncReadFunction)(id_list, position_list) == COMM_SUCCESS)
                {
                    _hw_fail_counter_read = 0;

                    // set motors states accordingly
                    for(auto const &i : id_list)
                        (_state_map.at(i).*setFunction)(position_list.at(i));
                }
            }
        }

        // no need for fillMotorState

    }

    /**
     * @brief DxlDriver::fillPositionStatus
     */
    void DxlDriver::fillPositionStatus()
    {
        readAndFillState(&XDriver::syncReadPosition, &DxlMotorState::setTemperatureState);
    }

    /**
     * @brief DxlDriver::fillTemperatureStatus
     */
    void DxlDriver::fillTemperatureStatus()
    {
        readAndFillState(&XDriver::syncReadTemperature, &DxlMotorState::setTemperatureState);
    }

    /**
     * @brief DxlDriver::fillVoltageStatus
     */
    void DxlDriver::fillVoltageStatus()
    {
        readAndFillState(&XDriver::syncReadVoltage, &DxlMotorState::setVoltageState);
    }

    /**
     * @brief DxlDriver::fillErrorStatus
     */
    void DxlDriver::fillErrorStatus()
    {
        readAndFillState(&XDriver::syncReadHwErrorStatus, &DxlMotorState::setHardwareError);
    }

    /**
     * @brief DxlDriver::readPositionState
     */
    void DxlDriver::readPositionStatus()
    {
        if (!hasMotors())
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - No motor");
            _debug_error_message = "Dxl Driver -  No motor";
            return;
        }
        fillPositionStatus();

        if (_hw_fail_counter_read > 25)
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Dxl connection problem - Failed to read from Dxl bus");
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
            ROS_ERROR_THROTTLE(1, "Dxl Driver - No motor");
            _debug_error_message = "Dxl Driver - No motor";
            return;
        }
        fillTemperatureStatus();
        fillVoltageStatus();
        fillErrorStatus();

        interpreteErrorState();

        if (_hw_fail_counter_read > 25 )
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Dxl connection problem - Failed to read from Dxl bus");
            _hw_fail_counter_read = 0;

            _is_dxl_connection_ok = false;
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
        }
    }

    /**
     * @brief DxlDriver::interpreteErrorState
     */
    void DxlDriver::interpreteErrorState()
    {
        for (auto const &it : _state_map)
        {
            DxlMotorState motor = it.second;

            uint32_t hw_state = motor.getHardwareErrorState();
            std::string hardware_message = "";
            DxlMotorType motor_type = motor.getType();

            switch(motor_type)
            {
                case DxlMotorType::MOTOR_TYPE_XL430:
                case DxlMotorType::MOTOR_TYPE_XL330:
                case DxlMotorType::MOTOR_TYPE_XC430:
                    if (hw_state & 0b00000001)
                    {
                        hardware_message += "Input Voltage";
                    }
                    if (hw_state & 0b00000100)
                    {
                        if (hardware_message != "")
                            hardware_message += ", ";
                        hardware_message += "OverHeating";
                    }
                    if (hw_state & 0b00001000)
                    {
                        if (hardware_message != "")
                            hardware_message += ", ";
                        hardware_message += "Motor Encoder";
                    }
                    if (hw_state & 0b00010000)
                    {
                        if (hardware_message != "")
                            hardware_message += ", ";
                        hardware_message += "Electrical Shock";
                    }
                    if (hw_state & 0b00100000)
                    {
                        if (hardware_message != "")
                            hardware_message += ", ";
                        hardware_message += "Overload";
                    }
                    if (hardware_message != "")
                        hardware_message += " Error";
                break;
                case DxlMotorType::MOTOR_TYPE_XL320:
                    if (hw_state & 0b00000001)
                    {
                        hardware_message += "Overload";
                    }
                    if (hw_state & 0b00000010)
                    {
                        if (hardware_message != "")
                            hardware_message += ", ";
                        hardware_message += "OverHeating";
                    }
                    if (hw_state & 0b00000100)
                    {
                        if (hardware_message != "")
                            hardware_message += ", ";
                        hardware_message += "Input voltage out of range";
                    }
                break;
                default:
                    break;
            }

            motor.setHardwareError(hardware_message);
        }
    }


    void DxlDriver::executeJointTrajectoryCmd(std::vector<uint32_t> &cmd)
    {
        std::string cmd_string;
        std::for_each(std::begin(cmd), std::end(cmd),
                      [&cmd_string](const uint32_t &x) {
                          cmd_string += std::to_string(x) + " ";
                      });
        // ROS_DEBUG_THROTTLE(0.5, "Dxl Driver - execute JointTrajectoryCmd called with %s", cmd_string.c_str());
        int counter = 0;
        std::vector<uint32_t> xl430_cmd;
        uint32_t xl320_cmd;
        xl430_cmd.push_back(cmd.at(0));
        xl430_cmd.push_back(cmd.at(1));
        xl320_cmd = cmd.at(2);
        int result_xl430 = _xl430->syncWritePositionGoal(_xl430_arm_id_list, xl430_cmd);
        int result_xl320 = _xl320->setGoalPosition(_xl320_arm_id, xl320_cmd);
        if (result_xl320 != COMM_SUCCESS || result_xl430 != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Failed to write position");
            _debug_error_message = "Dxl Driver - Failed to write position";
        }
    }

    /**
     * @brief DxlDriver::readSynchronizeCommand
     * @param cmd
     */
    void DxlDriver::readSynchronizeCommand(SynchronizeMotorCmd cmd)
    {
        ROS_DEBUG("Dxl Driver - Received dxl syncronized cmd with type %d", int(cmd.getType()));

        std::string ids_string = "";
        for (int i = 0; i < cmd.getMotorsId().size(); i++)
            ids_string += std::to_string(cmd.getMotorsId().at(i)) + " ";
        ROS_DEBUG("Dxl Driver - Received syncronized dxl cmd with ids %s", ids_string.c_str());

        std::string params_string = "";
        for (int i = 0; i < cmd.getParams().size(); i++)
            params_string += std::to_string(cmd.getParams().at(i)) + " ";
        ROS_DEBUG("Dxl Driver - Received syncronized dxl cmd with params %s", params_string.c_str());

        switch(cmd.getType())
        {
            case DxlCommandType::CMD_TYPE_POSITION:
                syncWritePositionCommand(cmd.getMotorsId(), cmd.getParams());
            break;
            case DxlCommandType::CMD_TYPE_VELOCITY:
                syncWriteVelocityCommand(cmd.getMotorsId(), cmd.getParams());
            break;
            case DxlCommandType::CMD_TYPE_EFFORT:
                syncWriteEffortCommand(cmd.getMotorsId(), cmd.getParams());
            break;
            case DxlCommandType::CMD_TYPE_TORQUE:
                syncWriteTorqueEnable(cmd.getMotorsId(), cmd.getParams());
            break;
            case DxlCommandType::CMD_TYPE_LEARNING_MODE:
            {
                std::vector<uint8_t> id_list;
                for(auto const& element: _state_map) {
                    id_list.push_back(element.first);
                }
                std::vector<uint32_t> cmd_param(id_list.size(), cmd.getParams()[0]);

                syncWriteTorqueEnable(id_list, cmd_param);
            }
            break;
            default:
                ROS_ERROR("Unknown command type: %d", cmd.getType());
            break;
        }
    }

    /**
     * @brief DxlDriver::readSingleCommand
     * @param cmd
     */
    void DxlDriver::readSingleCommand(SingleMotorCmd cmd)
    {
        int result = -1;
        int counter = 0;
        int id = (int)cmd.getId();

        ROS_DEBUG("Dxl Driver - Received dxl cmd with type %d", int(cmd.getType()));
        ROS_DEBUG("Dxl Driver - Received dxl cmd with ids %s", std::to_string(id).c_str());
        ROS_DEBUG("Dxl Driver - Received dxl cmd with params %s", std::to_string(cmd.getParam()).c_str());

        if(_state_map.count(id) != 0)
        {
            DxlMotorState state = _state_map.at(id);

            while ((COMM_SUCCESS != result) && (counter < 50))
            {
                switch(cmd.getType())
                {
                case DxlCommandType::CMD_TYPE_VELOCITY:
                    result = setGoalVelocity(state, cmd.getParam());
                    break;
                case DxlCommandType::CMD_TYPE_POSITION:
                    result = setGoalPosition(state, cmd.getParam());
                    break;
                case DxlCommandType::CMD_TYPE_EFFORT:
                    result = setGoalTorque(state, cmd.getParam());
                    break;
                case DxlCommandType::CMD_TYPE_TORQUE:
                    result = setTorqueEnable(state, cmd.getParam());
                    break;
                case DxlCommandType::CMD_TYPE_PING:
                    result = ping(id);
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
            ROS_WARN("Dxl Driver - Failed to write a single command on dxl motor id : %d", id);
            _debug_error_message = "Dxl Driver - Failed to write a single command";
        }
    }

    void DxlDriver::syncWritePositionCommand(std::vector<uint8_t> &motor_list, std::vector<uint32_t> &param_list)
    {
        std::vector<uint8_t> xl320_motor;
        std::vector<uint32_t> xl320_cmd;
        std::vector<uint8_t> xl430_motor;
        std::vector<uint32_t> xl430_cmd;
        for (int i = 0; i < motor_list.size(); i++)
        {
            auto xl320 = std::find(_xl320_id_list.begin(), _xl320_id_list.end(), motor_list.at(i));
            if (xl320 != _xl320_id_list.end())
            {
                xl320_motor.push_back(*xl320);
                xl320_cmd.push_back(param_list.at(i));
            }
            auto xl430 = std::find(_xl430_id_list.begin(), _xl430_id_list.end(), motor_list.at(i));
            if (xl430 != _xl430_id_list.end())
            {
                xl430_motor.push_back(*xl430);
                xl430_cmd.push_back(param_list.at(i));
            }
        }

        int counter = 0;
        int result_xl320 = _xl320->syncWritePositionGoal(xl320_motor, xl320_cmd);
        int result_xl430 = _xl430->syncWritePositionGoal(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter < 50))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWritePositionGoal(xl320_motor, xl320_cmd);
                counter += 1;
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWritePositionGoal(xl430_motor, xl430_cmd);
                counter += 1;
            }
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result_xl320 != COMM_SUCCESS || result_xl430 != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Failed to write synchronize position");
            _debug_error_message = "Dxl Driver - Failed to write synchronize position";
        }
    }

    void DxlDriver::syncWriteEffortCommand(std::vector<uint8_t> &motor_list, std::vector<uint32_t> &param_list)
    {
        std::vector<uint8_t> xl320_motor;
        std::vector<uint32_t> xl320_cmd;
        std::vector<uint8_t> xl430_motor;
        std::vector<uint32_t> xl430_cmd;
        for (int i = 0; i < motor_list.size(); i++)
        {
            auto xl320 = std::find(_xl320_id_list.begin(), _xl320_id_list.end(), motor_list.at(i));
            if (xl320 != _xl320_id_list.end())
            {
                xl320_motor.push_back(*xl320);
                xl320_cmd.push_back(param_list.at(i));
            }
            auto xl430 = std::find(_xl430_id_list.begin(), _xl430_id_list.end(), motor_list.at(i));
            if (xl430 != _xl430_id_list.end())
            {
                xl430_motor.push_back(*xl430);
                xl430_cmd.push_back(param_list.at(i));
            }
        }

        int counter = 0;
        int result_xl320 = _xl320->syncWriteTorqueGoal(xl320_motor, xl320_cmd);
        int result_xl430 = _xl430->syncWriteTorqueGoal(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter < 50))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWriteTorqueGoal(xl320_motor, xl320_cmd);
                counter += 1;
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWriteTorqueGoal(xl430_motor, xl430_cmd);
                counter += 1;
            }
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result_xl320 != COMM_SUCCESS || result_xl430 != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Failed to write synchronize effort");
            _debug_error_message = "Dxl Driver - Failed to write synchronize effort";
        }
    }

    void DxlDriver::syncWriteVelocityCommand(std::vector<uint8_t> &motor_list, std::vector<uint32_t> &param_list)
    {
        std::vector<uint8_t> xl320_motor;
        std::vector<uint32_t> xl320_cmd;
        std::vector<uint8_t> xl430_motor;
        std::vector<uint32_t> xl430_cmd;
        for (int i = 0; i < motor_list.size(); i++)
        {
            auto xl320 = std::find(_xl320_id_list.begin(), _xl320_id_list.end(), motor_list.at(i));
            if (xl320 != _xl320_id_list.end())
            {
                xl320_motor.push_back(*xl320);
                xl320_cmd.push_back(param_list.at(i));
            }
            auto xl430 = std::find(_xl430_id_list.begin(), _xl430_id_list.end(), motor_list.at(i));
            if (xl430 != _xl430_id_list.end())
            {
                xl430_motor.push_back(*xl430);
                xl430_cmd.push_back(param_list.at(i));
            }
        }

        int counter = 0;
        int result_xl320 = _xl320->syncWriteVelocityGoal(xl320_motor, xl320_cmd);
        int result_xl430 = _xl430->syncWriteVelocityGoal(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter < 50))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWriteVelocityGoal(xl320_motor, xl320_cmd);
                counter += 1;
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWriteVelocityGoal(xl430_motor, xl430_cmd);
                counter += 1;
            }
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result_xl320 != COMM_SUCCESS || result_xl430 != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Failed to write synchronize velocity");
            _debug_error_message = "Dxl Driver - Failed to write synchronize velocity";
        }
    }

    void DxlDriver::syncWriteTorqueEnable(std::vector<uint8_t> &motor_list, std::vector<uint32_t> &torque_enable)
    {


        std::vector<uint8_t> xl320_motor;
        std::vector<uint32_t> xl320_cmd;
        std::vector<uint8_t> xl430_motor;
        std::vector<uint32_t> xl430_cmd;
        for (int i = 0; i < motor_list.size(); i++)
        {
            auto xl320 = std::find(_xl320_id_list.begin(), _xl320_id_list.end(), motor_list.at(i));
            if (xl320 != _xl320_id_list.end())
            {
                xl320_motor.push_back(*xl320);
                xl320_cmd.push_back(torque_enable.at(i));
            }
            auto xl430 = std::find(_xl430_id_list.begin(), _xl430_id_list.end(), motor_list.at(i));
            if (xl430 != _xl430_id_list.end())
            {
                xl430_motor.push_back(*xl430);
                xl430_cmd.push_back(torque_enable.at(i));
            }
        }

        int counter = 0;
        int result_xl320 = _xl320->syncWriteTorqueEnable(xl320_motor, xl320_cmd);
        ros::Duration(0.05).sleep();
        int result_xl430 = _xl430->syncWriteTorqueEnable(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter < 50))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWriteTorqueEnable(xl320_motor, xl320_cmd);
                counter += 1;
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWriteTorqueEnable(xl430_motor, xl430_cmd);
                counter += 1;
            }
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result_xl320 != COMM_SUCCESS || result_xl430 != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Failed to write synchronize torque enable");
            _debug_error_message = "Dxl Driver - Failed to write synchronize torque";
        }
    }

    /**
     * @brief DxlDriver::scanAndCheck
     * @return
     */
    int DxlDriver::scanAndCheck()
    {
        int counter = 0;
        _all_motor_connected.clear();
        int result = getAllIdsOnDxlBus(_all_motor_connected);
        while (COMM_SUCCESS != result && counter < 50)
        {
            result = getAllIdsOnDxlBus(_all_motor_connected);
            counter += 1;
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result != COMM_SUCCESS)
        {
            _debug_error_message = "Dxl Driver - Failed to scan motors, Dynamixel bus is too busy. Will retry...";
            ROS_WARN("Dxl Driver - Failed to scan motors, dxl bus is too busy (counter max : %d)", counter);
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
            _debug_error_message += std::to_string(*it);
        }
        _debug_error_message += " do not seem to be connected";

        return DXL_SCAN_MISSING_MOTOR;
    }

    /**
     * @brief DxlDriver::checkRemovedMotors
     */
    void DxlDriver::checkRemovedMotors()
    {
        //get list of ids
        std::vector<uint8_t> motor_list;

        for(auto const& istate: _state_map) {
            auto it = std::find(_all_motor_connected.begin(), _all_motor_connected.end(), istate.first);
            if (it == _all_motor_connected.end())
                motor_list.push_back(istate.first);
        }
        _removed_motor_id_list = motor_list;
    }

    /**
     * @brief DxlDriver::getAllIdsOnDxlBus
     * @param id_list
     * @return
     */
    int DxlDriver::getAllIdsOnDxlBus(std::vector<uint8_t> &id_list)
    {
        int result = COMM_RX_FAIL;

        // 1. Get all ids from dxl bus. We can use any driver for that
        auto it = _xdriver_map.begin();
        if(it != _xdriver_map.end() && it->second)
        {
            int result = it->second->scan(id_list);

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
                ROS_WARN("Dxl Driver - Broadcast ping failed , result : %d (-3001: timeout, -3002: corrupted packet)", result);
            }
        }

        return result;
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
    int DxlDriver::sendCustomDxlCommand(DxlMotorType motor_type, uint8_t id, uint32_t reg_address, uint32_t value,  uint32_t byte_number)
    {
        int result;
        ROS_DEBUG("Dxl Driver - Sending custom command to Dynamixel:\n"
                  "Motor type: %d, ID: %d, Value: %d, Address: %d, Size: %d",
                  (int)motor_type, (int)id, (int)value,
                  (int)reg_address, (int)byte_number);

        if(_xdriver_map.count(motor_type) && _xdriver_map.at(motor_type))
        {
            result = _xdriver_map.at(motor_type)->customWrite(id, reg_address, value, byte_number);
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("Dxl Driver - Failed to write custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong motor type, should be 2 (XL-430) or 3 (XL-320) or 4 (XL-330) or 5 (XC-430). Detected: %d", (int)motor_type);
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
    int DxlDriver::readCustomDxlCommand(DxlMotorType motor_type, uint8_t id,
                                        uint32_t reg_address, uint32_t &value, uint32_t byte_number)
    {
        int result;
        ROS_DEBUG("Dxl Driver - Reading custom registry from Dynamixel:\n"
                  "Motor type: %d, ID: %d, Address: %d, Size: %d",
                  (int)motor_type, (int)id, (int)value,
                  (int)reg_address, (int)byte_number);

        if(_xdriver_map.count(motor_type) && _xdriver_map.at(motor_type))
        {
            result = _xdriver_map.at(motor_type)->customRead(id, reg_address, value, byte_number);
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("Dxl Driver - Failed to write custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong motor type, should be 2 (XL-430) or 3 (XL-320) or 4 (XL-330) or 5 (XC-430). Detected: %d", (int)motor_type);
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
        ros::Duration(0.005).sleep();
        return result;
    }

} // namespace DynamixelDriver
