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
namespace DynamixelDriver
{
    static const unsigned int MAX_RETRIES = 25;

    DxlDriver::DxlDriver(): _xl320_hw_fail_counter_read(0), _xl430_hw_fail_counter_read(0), _led_state(-1)
    {
        bool debug_mode = false;
        _nh.getParam("/niryo_robot_hardware_interface/debug", debug_mode);
        if (debug_mode)
        {
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            {
                ros::console::notifyLoggerLevelsChanged();
            }
        }

        if (init() != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Dynamixel Communication Failed");
        }
    }

    int DxlDriver::init()
    {
        initParameters();

        _dxlPortHandler.reset(dynamixel::PortHandler::getPortHandler(_device_name.c_str()));
        _dxlPacketHandler.reset(dynamixel::PacketHandler::getPacketHandler(DXL_BUS_PROTOCOL_VERSION));

        ROS_DEBUG("Dxl Driver - Dxl : set port name (%s), baudrate(%d)", _device_name.c_str(), _uart_baudrate);

        _xl320.reset(new XL320Driver(_dxlPortHandler, _dxlPacketHandler));
        _xl430.reset(new XL430Driver(_dxlPortHandler, _dxlPacketHandler));

        _is_dxl_connection_ok = false;
        _debug_error_message = "Dxl Driver - No connection with Dynamixel motors has been made yet";

        updateMotorTypeList();

        return setupCommunication();
    }

    void DxlDriver::initParameters()
    {
        // get params from rosparams
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_bus/dxl_uart_device_name", _device_name);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_bus/dxl_baudrate", _uart_baudrate);

        if (_nh.hasParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_id_list"))
        {
            _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_id_list", _motor_id_list);
        }
        else
        {
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/dxl_motor_id_list", _motor_id_list);
        }

        if (_nh.hasParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_type_list"))
        {
            _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/motors_params/dxl_motor_type_list", _motor_id_list);
        }
        else
        {
            _nh.getParam("/niryo_robot_hardware_interface/motors_params/dxl_motor_type_list", _motor_type_list);
        }

        _xl430_arm_id_list.clear();
        _xl430_arm_id_list.push_back(2);
        _xl430_arm_id_list.push_back(3);
        _xl320_arm_id = 6;
        _removed_motor_id_list.clear();

        std::string motor_string_list = "[";
        for (unsigned int i = 0; i < _motor_id_list.size(); i++)
        {
            if (i != 0)
                motor_string_list += ", ";
            motor_string_list += "id " + std::to_string(_motor_id_list.at(i)) + ": " + _motor_type_list.at(i).c_str();
        }
        motor_string_list += "]";
        ROS_INFO("Dxl Driver - Dxl motor list: %s ", motor_string_list.c_str());
    }

    int DxlDriver::setupCommunication()
    {

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

        ros::Duration(0.1).sleep();
        return COMM_SUCCESS;
    }

    std::vector<DxlMotorState> &DxlDriver::getMotorsState()
    {
        return _motor_list;
    }

    void DxlDriver::getBusState(bool &connection_state, std::vector<uint8_t> &motor_id, std::string &debug_msg)
    {
        debug_msg = _debug_error_message;
        motor_id = _all_motor_connected;
        connection_state = isConnectionOk();
    }

    bool DxlDriver::addDynamixel(uint8_t id, DxlMotorType type)
    {
        bool added_successfully = false;
        switch(type)
        {
            case DxlMotorType::MOTOR_TYPE_XL320:
            {
                _motor_type_list.push_back("xl320");
                added_successfully = true;
                break;
            }
            case DxlMotorType::MOTOR_TYPE_XL430:
            {
                _motor_type_list.push_back("xl430");
                added_successfully = true;
                break;
            }
            default:
            {
                ROS_WARN("DxlDriver - Unknown type given to addDynamixel function");
                break;
            }
        }
        if (added_successfully)
        {
            _motor_id_list.push_back(id);
            updateMotorTypeList();
        }
        return added_successfully;
    }

    void DxlDriver::removeDynamixel(uint8_t id, DxlMotorType type)
    {
        (void)type;
        for (unsigned int i = 0; i < _motor_id_list.size(); i++)
        {
            if (_motor_id_list.at(i) == id)
            {
                _motor_id_list.erase(_motor_id_list.begin() + i);
                _motor_type_list.erase(_motor_type_list.begin() + i);
            }
        }
        for (unsigned int i = 0; i < _removed_motor_id_list.size(); i++)
        {
            if (_removed_motor_id_list.at(i) == id)
            {
                _removed_motor_id_list.erase(_removed_motor_id_list.begin() + i);
            }
        }
        updateMotorTypeList();
    }

    void DxlDriver::updateMotorTypeList(void)
    {
        std::vector<uint8_t> new_xl430_id_list;
        std::vector<uint8_t> new_xl320_id_list;

        std::vector<DxlMotorState> new_motor_list;
        std::vector<DxlMotorState> new_xl320_motor_list;
        std::vector<DxlMotorState> new_xl430_motor_list;

        for (unsigned int i = 0; i < _motor_id_list.size(); i++)
        {
            if (_motor_type_list.at(i) == "xl430")
            {
                std::vector<DxlMotorState>::iterator it = std::find_if(_motor_list.begin(), _motor_list.end(), [&](DxlMotorState &dxl_state) { return dxl_state.getId() == _motor_id_list.at(i); });
                if (it != _motor_list.end())
                {
                    it->setType(DxlMotorType::MOTOR_TYPE_XL430);
                    new_xl430_motor_list.push_back(*it);
                    new_motor_list.push_back(*it);
                }
                else
                {
                    DxlMotorState motor(_motor_id_list.at(i), DxlMotorType::MOTOR_TYPE_XL430);
                    new_xl430_motor_list.push_back(motor);
                    new_motor_list.push_back(motor);
                }
                new_xl430_id_list.push_back(_motor_id_list.at(i));
            }
            else if (_motor_type_list.at(i) == "xl320")
            {
                std::vector<DxlMotorState>::iterator it = std::find_if(_motor_list.begin(), _motor_list.end(), [&](DxlMotorState &dxl_state) { return dxl_state.getId() == _motor_id_list.at(i); });
                if (it != _motor_list.end())
                {
                    it->setType(DxlMotorType::MOTOR_TYPE_XL320);
                    new_xl320_motor_list.push_back(*it);
                    new_motor_list.push_back(*it);
                }
                else
                {
                    DxlMotorState motor(_motor_id_list.at(i), DxlMotorType::MOTOR_TYPE_XL320);
                    new_xl320_motor_list.push_back(motor);
                    new_motor_list.push_back(motor);
                }
                new_xl320_id_list.push_back(_motor_id_list.at(i));
            }
        }

        _xl430_id_list = new_xl430_id_list;
        _xl320_id_list = new_xl320_id_list;
        _xl320_motor_list = new_xl320_motor_list;
        _xl430_motor_list = new_xl430_motor_list;
        _motor_list = new_motor_list;
    }

    void DxlDriver::fillPositionState()
    {
        readAndFillStateXL320(&XL320Driver::syncReadPosition, &DxlMotorState::setPositionState);
        readAndFillStateXL430(&XL430Driver::syncReadPosition, &DxlMotorState::setPositionState);
        FillMotorsState(&DxlMotorState::setPositionState, &DxlMotorState::getPositionState);
    }

    void DxlDriver::fillTemperatureStatus()
    {
        readAndFillStateXL320(&XL320Driver::syncReadTemperature, &DxlMotorState::setTemperatureState);
        readAndFillStateXL430(&XL430Driver::syncReadTemperature, &DxlMotorState::setTemperatureState);
        FillMotorsState(&DxlMotorState::setTemperatureState, &DxlMotorState::getTemperatureState);
    }

    void DxlDriver::fillVoltageStatus()
    {
        readAndFillStateXL320(&XL320Driver::syncReadVoltage, &DxlMotorState::setVoltageState);
        readAndFillStateXL430(&XL430Driver::syncReadVoltage, &DxlMotorState::setVoltageState);
        FillMotorsState(&DxlMotorState::setVoltageState, &DxlMotorState::getVoltageState);
    }

    void DxlDriver::fillErrorStatus()
    {
        readAndFillStateXL320(&XL320Driver::syncReadHwErrorStatus, &DxlMotorState::setHardwareError);
        readAndFillStateXL430(&XL430Driver::syncReadHwErrorStatus, &DxlMotorState::setHardwareError);
        FillMotorsState(&DxlMotorState::setHardwareError, &DxlMotorState::getHardwareErrorState);
    }

    void DxlDriver::FillMotorsState(void (DxlMotorState::*setFunction)(uint32_t), uint32_t (DxlMotorState::*getFunction)())
    {
        for (auto motor = _motor_list.begin(); motor != _motor_list.end(); ++motor)
        {
            for (auto xl430 = _xl430_motor_list.begin(); xl430 != _xl430_motor_list.end(); ++xl430)
            {
                if (motor->getId() == xl430->getId())
                {
                    ((*motor).*setFunction)(((*xl430).*getFunction)());
                }
            }
            for (auto xl320 = _xl320_motor_list.begin(); xl320 != _xl320_motor_list.end(); ++xl320)
            {
                if (motor->getId() == xl320->getId())
                {
                    ((*motor).*setFunction)(((*xl320).*getFunction)());
                }
            }
        }
    }

    void DxlDriver::readAndFillStateXL320(
        int (XL320Driver::*readFunction)(std::vector<uint8_t> &, std::vector<uint32_t> &),
        void (DxlMotorState::*setFunction)(uint32_t))
    {
        std::vector<uint32_t> read_elements_list;
        int read_result = (_xl320.get()->*readFunction)(_xl320_id_list, read_elements_list);
        if (read_result == COMM_SUCCESS && read_elements_list.size() == _xl320_motor_list.size())
        {
            _xl320_hw_fail_counter_read = 0;
            for (unsigned int i = 0; i < _xl320_motor_list.size(); i++)
            {
                (_xl320_motor_list.at(i).*setFunction)(read_elements_list.at(i));
            }
        }
        else
        {
            _xl320_hw_fail_counter_read++;
        }
    }

    void DxlDriver::readAndFillStateXL430(
        int (XL430Driver::*readFunction)(std::vector<uint8_t> &, std::vector<uint32_t> &),
        void (DxlMotorState::*setFunction)(uint32_t))
    {
        std::vector<uint32_t> read_elements_list;
        int read_result = (_xl430.get()->*readFunction)(_xl430_id_list, read_elements_list);
        if (read_result == COMM_SUCCESS && read_elements_list.size() == _xl430_motor_list.size())
        {
            _xl430_hw_fail_counter_read = 0;
            for (unsigned int i = 0; i < _xl430_motor_list.size(); i++)
            {
                (_xl430_motor_list.at(i).*setFunction)(read_elements_list.at(i));
            }
        }
        else
        {
            _xl430_hw_fail_counter_read++;
        }
    }

    void DxlDriver::readPositionState()
    {
        bool can_read_dxl = (_motor_list.size() > 0);
        if (!can_read_dxl)
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - No motor");
            _debug_error_message = "Dxl Driver -  No motor";
            return;
        }
        fillPositionState();
        if (_xl320_hw_fail_counter_read > 25 || _xl430_hw_fail_counter_read > 25)
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Dxl connection problem - Failed to read from Dxl bus");
            _xl320_hw_fail_counter_read = 0;
            _xl430_hw_fail_counter_read = 0;
            _is_dxl_connection_ok = false;
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
        }
    }

    void DxlDriver::readHwStatus()
    {
        bool can_read_dxl = (_motor_list.size() > 0);
        if (!can_read_dxl)
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - No motor");
            _debug_error_message = "Dxl Driver - No motor";
            return;
        }
        fillTemperatureStatus();
        fillVoltageStatus();
        fillErrorStatus();
        interpreteErrorState();

        if (_xl320_hw_fail_counter_read > 25 || _xl430_hw_fail_counter_read > 25)
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Dxl connection problem - Failed to read from Dxl bus");
            _xl320_hw_fail_counter_read = 0;
            _xl430_hw_fail_counter_read = 0;
            _is_dxl_connection_ok = false;
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
        }
    }

    void DxlDriver::interpreteErrorState()
    {

        for (auto motor = _motor_list.begin(); motor != _motor_list.end(); ++motor)
        {
            uint32_t hw_state = motor->getHardwareErrorState();
            std::string hardware_message = "";
            DxlMotorType motor_type = motor->getType();
            if(motor_type == DxlMotorType::MOTOR_TYPE_XL430)
            {
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
            }
            else if (motor_type == DxlMotorType::MOTOR_TYPE_XL320)
            {
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
            }
    
            motor->setHardwareError(hardware_message);
        }
    }

    void DxlDriver::executeJointTrajectoryCmd(std::vector<uint32_t> &cmd)
    {
        std::string cmd_string;
        std::for_each(std::begin(cmd), std::end(cmd),
                      [&cmd_string](const uint32_t &x) {
                          cmd_string += std::to_string(x) + " ";
                      });
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

    void DxlDriver::readSynchronizeCommand(SynchronizeMotorCmd cmd)
    {
        ROS_DEBUG("Dxl Driver - Received dxl syncronized cmd with type %d", int(cmd.getType()));

        std::string ids_string = "";
        for (unsigned int i = 0; i < cmd.getMotorsId().size(); i++)
            ids_string += std::to_string(cmd.getMotorsId().at(i)) + " ";
        ROS_DEBUG("Dxl Driver - Received syncronized dxl cmd with ids %s", ids_string.c_str());

        std::string params_string = "";
        for (unsigned int i = 0; i < cmd.getParams().size(); i++)
            params_string += std::to_string(cmd.getParams().at(i)) + " ";
        ROS_DEBUG("Dxl Driver - Received syncronized dxl cmd with params %s", params_string.c_str());

        if (cmd.getType() == DxlCommandType::CMD_TYPE_POSITION)
        {
            syncWritePositionCommand(cmd.getMotorsId(), cmd.getParams());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_VELOCITY)
        {
            syncWriteVelocityCommand(cmd.getMotorsId(), cmd.getParams());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_EFFORT)
        {
            syncWriteEffortCommand(cmd.getMotorsId(), cmd.getParams());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_TORQUE)
        {
            syncWriteTorqueEnable(cmd.getMotorsId(), cmd.getParams());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_LEARNING_MODE)
        {
            std::vector<uint32_t> cmd_param(_motor_id_list.size(), cmd.getParams()[0]);
            std::vector<uint8_t> id_list(_motor_id_list.begin(), _motor_id_list.end());
            syncWriteTorqueEnable(id_list, cmd_param);
        }
    }

    int DxlDriver::_sendDxl320Command(const SingleMotorCmd &cmd)
    {
        int result = -1;
        if (cmd.getType() == DxlCommandType::CMD_TYPE_VELOCITY)
        {
            result = _xl320->setGoalVelocity(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_POSITION)
        {
            result = _xl320->setGoalPosition(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_EFFORT)
        {
            result = _xl320->setGoalTorque(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_TORQUE)
        {
            result = _xl320->setTorqueEnable(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_PING)
        {
            result = _xl320->ping(cmd.getId());
        }
        return result;
    }

    int DxlDriver::_sendDxl430Command(const SingleMotorCmd &cmd)
    {
        int result = -1;
        if (cmd.getType() == DxlCommandType::CMD_TYPE_VELOCITY)
        {
            result = _xl430->setGoalVelocity(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_POSITION)
        {
            result = _xl430->setGoalPosition(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_EFFORT)
        {
            result = _xl430->setGoalTorque(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_TORQUE)
        {
            result = _xl430->setTorqueEnable(cmd.getId(), cmd.getParam());
        }
        else if (cmd.getType() == DxlCommandType::CMD_TYPE_PING)
        {
            result = _xl430->ping(cmd.getId());
        }
        return result;
    }

    void DxlDriver::readSingleCommand(SingleMotorCmd cmd)
    {
        int result = -1;
        unsigned int counter = 0;

        ROS_DEBUG("Dxl Driver - Received dxl cmd with type %d", int(cmd.getType()));

        ROS_DEBUG("Dxl Driver - Received dxl cmd with ids %s", std::to_string(cmd.getId()).c_str());

        ROS_DEBUG("Dxl Driver - Received dxl cmd with params %s", std::to_string(cmd.getParam()).c_str());

        while ((result != COMM_SUCCESS) && (counter++ < MAX_RETRIES))
        {
            auto xl320 = std::find(_xl320_id_list.begin(), _xl320_id_list.end(), cmd.getId());
            if (xl320 != _xl320_id_list.end())
            {
                result = _sendDxl320Command(cmd);
            }

            auto xl430 = std::find(_xl430_id_list.begin(), _xl430_id_list.end(), cmd.getId());
            if (xl430 != _xl430_id_list.end())
            {
                result = _sendDxl430Command(cmd);
            }
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Failed to write a single command on dxl motor id : %d", cmd.getId());
            _debug_error_message = "Dxl Driver - Failed to write a single command";
        }
    }

    void DxlDriver::syncWritePositionCommand(std::vector<uint8_t> &motor_list, std::vector<uint32_t> &param_list)
    {
        std::vector<uint8_t> xl320_motor;
        std::vector<uint32_t> xl320_cmd;
        std::vector<uint8_t> xl430_motor;
        std::vector<uint32_t> xl430_cmd;
        for (unsigned int i = 0; i < motor_list.size(); i++)
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

        unsigned int counter = 0;
        int result_xl320 = _xl320->syncWritePositionGoal(xl320_motor, xl320_cmd);
        int result_xl430 = _xl430->syncWritePositionGoal(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter++ < MAX_RETRIES))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWritePositionGoal(xl320_motor, xl320_cmd);
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWritePositionGoal(xl430_motor, xl430_cmd);
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
        for (unsigned int i = 0; i < motor_list.size(); i++)
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

        unsigned int counter = 0;
        int result_xl320 = _xl320->syncWriteTorqueGoal(xl320_motor, xl320_cmd);
        int result_xl430 = _xl430->syncWriteTorqueGoal(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter++ < MAX_RETRIES))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWriteTorqueGoal(xl320_motor, xl320_cmd);
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWriteTorqueGoal(xl430_motor, xl430_cmd);
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
        for (unsigned int i = 0; i < motor_list.size(); i++)
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

        unsigned int counter = 0;
        int result_xl320 = _xl320->syncWriteVelocityGoal(xl320_motor, xl320_cmd);
        int result_xl430 = _xl430->syncWriteVelocityGoal(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter++ < MAX_RETRIES))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWriteVelocityGoal(xl320_motor, xl320_cmd);
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWriteVelocityGoal(xl430_motor, xl430_cmd);
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
        for (unsigned int i = 0; i < motor_list.size(); i++)
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

        unsigned int counter = 0;
        int result_xl320 = _xl320->syncWriteTorqueEnable(xl320_motor, xl320_cmd);
        ros::Duration(0.05).sleep();
        int result_xl430 = _xl430->syncWriteTorqueEnable(xl430_motor, xl430_cmd);

        while ((result_xl320 != COMM_SUCCESS && result_xl430 != COMM_SUCCESS) && (counter++ < MAX_RETRIES))
        {
            if (result_xl320 != COMM_SUCCESS)
            {
                result_xl320 = _xl320->syncWriteTorqueEnable(xl320_motor, xl320_cmd);
            }
            if (result_xl430 != COMM_SUCCESS)
            {
                result_xl430 = _xl430->syncWriteTorqueEnable(xl430_motor, xl430_cmd);
            }
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result_xl320 != COMM_SUCCESS || result_xl430 != COMM_SUCCESS)
        {
            ROS_WARN("Dxl Driver - Failed to write synchronize torque enable");
            _debug_error_message = "Dxl Driver - Failed to write synchronize torque";
        }
    }

    int DxlDriver::setGoalPosition(DxlMotorState &targeted_dxl, uint32_t position)
    {
        DxlMotorType dxl_type = targeted_dxl.getType();
        int result;

        if (dxl_type == DxlMotorType::MOTOR_TYPE_XL320)
        {
            result = _xl320->setGoalPosition(targeted_dxl.getId(), position);
        }
        else if (dxl_type == DxlMotorType::MOTOR_TYPE_XL430)
        {
            result = _xl430->setGoalPosition(targeted_dxl.getId(), position);
        }
        else
        {
            result = -1;
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    uint32_t DxlDriver::getPosition(DxlMotorState &targeted_dxl)
    {
        uint32_t result;
        int read_result;
        DxlMotorType dxl_type = targeted_dxl.getType();
        while (_hw_fail_counter_read < 25)
        {
            if (dxl_type == DxlMotorType::MOTOR_TYPE_XL320)
            {
                read_result = _xl320->readPosition(targeted_dxl.getId(), &result);
            }
            else if (dxl_type == DxlMotorType::MOTOR_TYPE_XL430)
            {
                read_result = _xl430->readPosition(targeted_dxl.getId(), &result);
            }
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
        if (_hw_fail_counter_read != 0)
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Dxl connection problem - Failed to read from Dxl bus");
            _debug_error_message = "Dxl Driver - Connection problem with Dynamixel Bus.";
            _hw_fail_counter_read = 0;
            _is_dxl_connection_ok = false;
            result = COMM_RX_FAIL;
        }

        return result;
    }

    int DxlDriver::setTorqueEnable(DxlMotorState &targeted_dxl, uint32_t torque_enable)
    {
        DxlMotorType dxl_type = targeted_dxl.getType();
        int result;

        if (dxl_type == DxlMotorType::MOTOR_TYPE_XL320)
        {
            result = _xl320->setTorqueEnable(targeted_dxl.getId(), torque_enable);
        }
        else if (dxl_type == DxlMotorType::MOTOR_TYPE_XL430)
        {
            result = _xl430->setTorqueEnable(targeted_dxl.getId(), torque_enable);
        }
        else
        {
            result = -1;
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    int DxlDriver::setGoalVelocity(DxlMotorState &targeted_dxl, uint32_t velocity)
    {
        DxlMotorType dxl_type = targeted_dxl.getType();
        int result;

        if (dxl_type == DxlMotorType::MOTOR_TYPE_XL320)
        {
            result = _xl320->setGoalVelocity(targeted_dxl.getId(), velocity);
        }
        else if (dxl_type == DxlMotorType::MOTOR_TYPE_XL430)
        {
            result = _xl430->setGoalVelocity(targeted_dxl.getId(), velocity);
        }
        else
        {
            result = -1;
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    int DxlDriver::setGoalTorque(DxlMotorState &targeted_dxl, uint32_t torque)
    {
        DxlMotorType dxl_type = targeted_dxl.getType();
        int result;

        if (dxl_type == DxlMotorType::MOTOR_TYPE_XL320)
        {
            result = _xl320->setGoalTorque(targeted_dxl.getId(), torque);
        }
        else if (dxl_type == DxlMotorType::MOTOR_TYPE_XL430)
        {
            result = _xl430->setGoalTorque(targeted_dxl.getId(), torque);
        }
        else
        {
            result = -1;
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
            _debug_error_message = "Dxl Driver - Wrong dxl type detected";
        }
        return result;
    }

    bool DxlDriver::isConnectionOk()
    {
        return _is_dxl_connection_ok;
    }

    int DxlDriver::scanAndCheck()
    {
        unsigned int counter = 0;
        _all_motor_connected.clear();
        int result = getAllIdsOnDxlBus(_all_motor_connected);
        while (result != COMM_SUCCESS && counter++ < MAX_RETRIES)
        {
            result = getAllIdsOnDxlBus(_all_motor_connected);
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
        }

        if (result != COMM_SUCCESS)
        {
            _debug_error_message = "Dxl Driver - Failed to scan motors, Dynamixel bus is too busy. Will retry...";
            ROS_WARN("Dxl Driver - Failed to scan motors, dxl bus is too busy (counter max : %d)", counter);
            return result;
        }

        checkRemovedMotors();
        if (_removed_motor_id_list.begin() == _removed_motor_id_list.end())
        {
            _is_dxl_connection_ok = true;
            _debug_error_message = "";
            return DXL_SCAN_OK;
        }
        _is_dxl_connection_ok = false;

        _debug_error_message = "Dynamixel(s):";
        for (std::vector<int>::iterator it = _removed_motor_id_list.begin(); it != _removed_motor_id_list.end(); ++it)
        {
            _debug_error_message += " ";
            _debug_error_message += std::to_string(*it);
        }
        _debug_error_message += " do not seem to be connected";

        return DXL_SCAN_MISSING_MOTOR;
    }

    void DxlDriver::checkRemovedMotors()
    {
        std::vector<int> motor_list;
        for (unsigned int i = 0; i < _motor_id_list.size(); i++)
        {
            auto it = std::find(_all_motor_connected.begin(), _all_motor_connected.end(), _motor_id_list.at(i));
            if (it == _all_motor_connected.end())
            {
                motor_list.push_back(_motor_id_list.at(i));
            }
        }
        _removed_motor_id_list = motor_list;
    }

    std::vector<int> &DxlDriver::getRemovedMotorList()
    {
        return _removed_motor_id_list;
    }

    int DxlDriver::checkCommPortAvailable()
    {
        unsigned int counter = 0;

        while (counter < MAX_RETRIES)
        {
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
            counter++;
        }

        if (counter == MAX_RETRIES)
        {
            _debug_error_message = "Dxl Driver - Failed to scan motors, Dynamixel bus is too busy. Will retry...";
            ROS_WARN("Dxl Driver - Failed to scan motors, dxl bus is too busy (counter max : %d)", counter);
            return COMM_PORT_BUSY;
        }
        return 0;
    }

    int DxlDriver::getAllIdsOnDxlBus(std::vector<uint8_t> &id_list)
    {

        // 1. Get all ids from dxl bus
        int result = _xl320->scan(id_list);

        if (result != COMM_SUCCESS)
        {
            if (result == COMM_RX_TIMEOUT)
            { // -3001
                _debug_error_message = "Dxl Driver - No Dynamixel motor found. Make sure that motors are correctly connected and powered on.";
            }
            else
            { // -3002 or other
                _debug_error_message = "Dxl Driver - Failed to scan Dynamixel bus.";
            }
            ROS_WARN("Dxl Driver - Broadcast ping failed , result : %d (-3001: timeout, -3002: corrupted packet)", result);
            return result;
        }
        return result;
    }

    int DxlDriver::ping(DxlMotorState &targeted_dxl)
    {
        DxlMotorType dxl_type = targeted_dxl.getType();
        int result = 0;

        if (dxl_type == DxlMotorType::MOTOR_TYPE_XL320)
        {
            result = _xl320->ping(targeted_dxl.getId());
        }
        else if (dxl_type == DxlMotorType::MOTOR_TYPE_XL430)
        {
            result = _xl430->ping(targeted_dxl.getId());
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type detected: %d", (int)dxl_type);
        }
        return result;
    }

    int DxlDriver::ping_id(int dxl_id)
    {
        auto xl320 = std::find(_xl320_id_list.begin(), _xl320_id_list.end(), dxl_id);
        if (xl320 != _xl320_id_list.end())
        {
            return _xl320->ping(dxl_id);
        }

        auto xl430 = std::find(_xl430_id_list.begin(), _xl430_id_list.end(), dxl_id);
        if (xl430 != _xl430_id_list.end())
        {
            return _xl430->ping(dxl_id);
        }

        ROS_ERROR_THROTTLE(1, "Dxl Driver - Cannot ping motor ID %d ", dxl_id);
        return TOOL_STATE_PING_ERROR;
    }

    int DxlDriver::type_ping_id(uint8_t id, DxlMotorType type)
    {
        if (type == DxlMotorType::MOTOR_TYPE_XL320)
            return _xl320->ping(id);
        else if (type == DxlMotorType::MOTOR_TYPE_XL430)
            return _xl430->ping(id);

        ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong dxl type ");
        return TOOL_STATE_PING_ERROR;
    }

     int DxlDriver::rebootMotor(uint8_t motor_id, DxlMotorType motor_type)
    {
        int result = COMM_NOT_AVAILABLE;
        if(motor_type == DxlMotorType::MOTOR_TYPE_XL430)
        {
            result= _xl430->reboot(motor_id);
            if (result==COMM_SUCCESS)
            {
                ros::Time start_time = ros::Time::now();
                uint32_t tmp=0;
                int wait_result =_xl430->readTemperature(motor_id, &tmp);
                while(wait_result!=COMM_SUCCESS || tmp==0){
                    if ((ros::Time::now() - start_time).toSec() > 1)
                        return wait_result;
                    ros::Duration(0.1).sleep();
                    wait_result =_xl430->readTemperature(motor_id, &tmp);
                }
            }
        }
        if(motor_type == DxlMotorType::MOTOR_TYPE_XL320)
        {
            result = _xl320->reboot(motor_id);
            uint32_t tmp=0;
            if (result==COMM_SUCCESS)
            {
                ros::Time start_time = ros::Time::now();
                uint32_t tmp=0;
                int wait_result =_xl320->readTemperature(motor_id, &tmp);
                while(wait_result!=COMM_SUCCESS || tmp==0){
                    if ((ros::Time::now() - start_time).toSec() > 1)
                        return wait_result;
                    ros::Duration(0.1).sleep();
                    wait_result =_xl320->readTemperature(motor_id, &tmp);
                }
            }
        }
        return result;
    }
            // wait for new data



    int DxlDriver::rebootMotors()
    {
        int result;
        int return_value = COMM_SUCCESS;
        for (unsigned int i = 0; i < _xl430_motor_list.size(); i++)
        {
            ROS_DEBUG("Dxl Driver - Reboot Dxl motor with ID: %d", (int)_xl430_motor_list.at(i).getId());
            result = _xl430->reboot(_xl430_motor_list.at(i).getId());
            // if (result != COMM_SUCCESS)
            // {
            //     ROS_WARN("Dxl Driver - Failed to reboot motor: %d", result);
            //     return_value = result;
            // }
            ROS_INFO("Dxl Driver - Reboot motor: %d", _motor_list.at(i).getId());
        }
        for (unsigned int i = 0; i < _xl320_motor_list.size(); i++)
        {
            ROS_DEBUG("Dxl Driver - Reboot Dxl motor with ID: %d", (int)_xl320_motor_list.at(i).getId());
            result = _xl320->reboot(_xl320_motor_list.at(i).getId());
            // if (result != COMM_SUCCESS)
            // {
            //     ROS_WARN("Dxl Driver - Failed to reboot motor: %d", result);
            //     return_value = result;
            // }
            ROS_INFO("Dxl Driver - Reboot motor: %d", _xl320_motor_list.at(i).getId());
        }
        return return_value;
    }

    int DxlDriver::sendCustomDxlCommand(DxlMotorType motor_type, uint8_t id, uint32_t value, uint32_t reg_address, uint32_t byte_number)
    {
        DxlCustomCommand cmd = DxlCustomCommand(motor_type, id, value, reg_address, byte_number);
        int result;
        ROS_DEBUG("Dxl Driver - Sending custom command to Dynamixel:\n"
                  "Motor type: %d, ID: %d, Value: %d, Address: %d, Size: %d",
                  (int)cmd.motor_type, (int)cmd.id, (int)cmd.value,
                  (int)cmd.reg_address, (int)cmd.byte_number);

        if (cmd.motor_type == DxlMotorType::MOTOR_TYPE_XL320)
        {
            result = _xl320->customWrite(cmd.id, cmd.value, cmd.reg_address, cmd.byte_number);
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("Dxl Driver - Failed to write custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else if (cmd.motor_type == DxlMotorType::MOTOR_TYPE_XL430)
        {
            result = _xl430->customWrite(cmd.id, cmd.value, cmd.reg_address, cmd.byte_number);
            if (result != COMM_SUCCESS)
            {
                ROS_WARN("Dxl Driver - Failed to write custom command: %d", result);
                result = niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Dxl Driver - Wrong motor type, should be 1 (XL-320) or 2 (XL-430).");
            result = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        }
        ros::Duration(0.005).sleep();
        return result;
    }

    std::string DxlDriver::getErrorMessage()
    {
        return _debug_error_message;
    }

    int DxlDriver::getledstate(void)
    {
        return _led_state;
    }

    int DxlDriver::setLeds(int led)
    {
        _led_state = led;
        std::vector<uint32_t> command_led_id(_xl320_id_list.size(), (uint32_t)led);
        if (led >= 0 && led <= 7)
        {
            int error_counter = 0;
            int result = _xl320->syncWriteLed(_xl320_id_list, command_led_id);
            while (result != COMM_SUCCESS && error_counter < 5)
            {
                ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
                result = _xl320->syncWriteLed(_xl320_id_list, command_led_id);
                error_counter++;
            }

            if (result != COMM_SUCCESS)
            {
                ROS_WARN("Dxl Driver - Failed to write LED");
                return niryo_robot_msgs::CommandStatus::DXL_WRITE_ERROR;
            }
        }
        return niryo_robot_msgs::CommandStatus::SUCCESS;
    }

} // namespace DynamixelDriver
