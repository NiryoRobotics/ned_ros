#include "dynamixel_driver/dxl_driver_core.hpp"
#include <cstdlib>
#include <algorithm>
#include <cmath>

namespace DynamixelDriver
{
    DynamixelDriverCore::DynamixelDriverCore()
    {
        // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        //     ros::console::notifyLoggerLevelsChanged();
        // }
        init();
    }

    void DynamixelDriverCore::init()
    {
        _control_loop_flag = false;
        _debug_flag = false;
        _joint_trajectory_controller_cmd.clear();
        _dxl_cmd.reset();
        _end_effector_cmd.clear();
        initParameters();
        _dynamixel.reset(new DxlDriver());
        _dynamixel->scanAndCheck();
        startControlLoop();

        _activate_leds_server = _nh.advertiseService("niryo_robot/dynamixel_driver/set_dxl_leds", &DynamixelDriverCore::callbackActivateLeds, this);
        _custom_cmd_server = _nh.advertiseService("niryo_robot/dynamixel_driver/send_custom_dxl_value", &DynamixelDriverCore::callbackSendCustomDxlValue, this);
    }

    void DynamixelDriverCore::initParameters()
    {
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_control_loop_frequency", _control_loop_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_write_frequency", _write_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_read_data_frequency", _read_data_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_read_status_frequency", _read_status_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_hardware_check_connection_frequency", _check_connection_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixel_driver/dxl_end_effector_frequency", _check_end_effector_frequency);

        ROS_DEBUG("Dynamixel Driver Core - dxl_hardware_control_loop_frequency : %f", _control_loop_frequency);
        ROS_DEBUG("Dynamixel Driver Core - dxl_hardware_write_frequency : %f", _write_frequency);
        ROS_DEBUG("Dynamixel Driver Core - dxl_hardware_read_data_frequency : %f", _read_data_frequency);
        ROS_DEBUG("Dynamixel Driver Core - dxl_hardware_read_status_frequency : %f", _read_status_frequency);
        ROS_DEBUG("Dynamixel Driver Core - dxl_hardware_check_connection_frequency : %f", _check_connection_frequency);
        ROS_DEBUG("Dynamixel Driver Core - dxl_end_effector_frequency : %f", _check_end_effector_frequency);
    }

    void DynamixelDriverCore::startControlLoop()
    {
        resetHardwareControlLoopRates();
        if (!_control_loop_thread)
        {
            ROS_INFO("Dynamixel Driver Core - Start control loop");
            _control_loop_flag = true;
            _control_loop_thread.reset(new std::thread(boost::bind(&DynamixelDriverCore::controlLoop, this)));
        }
    }

    void DynamixelDriverCore::resetHardwareControlLoopRates()
    {
        ROS_DEBUG("Dynamixel Driver Core - Reset control loop rates");
        double now = ros::Time::now().toSec();
        _time_hw_data_last_write = now;
        _time_hw_data_last_read = now;
        _time_hw_status_last_read = now;
        _time_check_connection_last_read = now;
        _time_check_end_effector_last_read = now;
    }

    void DynamixelDriverCore::activeDebugMode(bool mode)
    {
        ROS_INFO("Dynamixel Driver Core - Activate debug mode for dynamixel driver core: %d", mode);
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
        ROS_INFO("Dynamixel Driver Core - Reboot motors");
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        int result = _dynamixel->rebootMotors();
        ros::Duration(1.5).sleep();
        return result;
    }

    bool DynamixelDriverCore::rebootMotor(uint8_t motor_id, DxlMotorType motor_type)
    {
        ROS_INFO("Dynamixel Driver Core - Reboot motor %d", motor_id);
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        int result = _dynamixel->rebootMotor(motor_id, motor_type);
        return (result==COMM_SUCCESS) ? true : false;
    }

    int DynamixelDriverCore::motorScanReport(uint8_t motor_id, DxlMotorType motor_type)
    {
        if (_debug_flag)
        {
            int result;

            ros::Duration(1.0).sleep();
            result = _dynamixel->type_ping_id(motor_id, motor_type);
            if (result == COMM_SUCCESS)
            {
                ROS_INFO("Dynamixel Driver Core - Debug - Dynamixel Motor %d found", motor_id);
            }
            else
            {
                ROS_ERROR("Dynamixel Driver Core - Debug - Dynamixel Motor %d not found", motor_id);
                ROS_ERROR("Dynamixel Driver Core - Debug - code error : %d ", result);
                return niryo_robot_msgs::CommandStatus::FAILURE;
            }
            return niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        ROS_ERROR("Dynamixel Driver Core - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }

    int DynamixelDriverCore::motorCmdReport(uint8_t motor_id, DxlMotorType motor_type)
    {
        if (_debug_flag)
        {

            int result;
            uint32_t old_position;
            uint32_t new_position;
            DxlMotorState dynamixel_motor = DxlMotorState(motor_id, motor_type);

            ros::Duration(0.5).sleep();
            ROS_INFO("Dynamixel Driver Core - Debug - Send torque on command on dxl %d", int(motor_id));
            _dynamixel->setTorqueEnable(dynamixel_motor, 1);
            ros::Duration(0.5).sleep();


            old_position = _dynamixel->getPosition(dynamixel_motor);
            ROS_INFO("Dynamixel Driver Core - Debug - get dxl %d pose: %d ", int(motor_id), int(old_position));
            ros::Duration(0.5).sleep();
            ROS_INFO("Dynamixel Driver Core - Debug - Send dxl %d pose: %d ", int(motor_id), int(old_position + 200));
            result = _dynamixel->setGoalPosition(dynamixel_motor, old_position + 200);
            ros::Duration(2).sleep();
            new_position = _dynamixel->getPosition(dynamixel_motor);
            ROS_INFO("Dynamixel Driver Core - Debug - get dxl %d pose: %d ", int(motor_id), int(new_position));
            int rest = new_position - old_position;

            ROS_INFO("Dynamixel Driver Core - Debug - Send dxl %d pose: %d ", int(motor_id), int(old_position));
            result = _dynamixel->setGoalPosition(dynamixel_motor, old_position);
            ros::Duration(2).sleep();
            uint32_t new_position2 = _dynamixel->getPosition(dynamixel_motor);
            ROS_INFO("Dynamixel Driver Core - Debug - get dxl %d pose: %d ", int(motor_id), int(new_position2));
            int rest2 = new_position2 - new_position;

            _dynamixel->setTorqueEnable(dynamixel_motor, 0);

            if (std::abs(rest) < 50 or std::abs(rest2) < 50)
            {
                ROS_WARN("Dynamixel Driver Core - Debug - Dynamixel Motor %d problem: %d", motor_id, result);
                return niryo_robot_msgs::CommandStatus::FAILURE;
            }
            else
            {
                ROS_INFO("Dynamixel Driver Core - Debug - Dynamixel Motor %d OK", motor_id);
            }
            return niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        ROS_ERROR("Dynamixel Driver Core - Debug - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }

    int DynamixelDriverCore::launchMotorsReport()
    {
        if (_debug_flag)
        {
            int motor_found;
            int response = niryo_robot_msgs::CommandStatus::SUCCESS;

            ROS_INFO("Dynamixel Driver Core - Debug - Start Dynamixel Motor Report");
            ros::Duration(1.0).sleep();
            ROS_INFO("Dynamixel Driver Core - Debug - Motor 4 report start :");
            if (motorScanReport(2, DxlMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            if (motorCmdReport(2, DxlMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            ROS_INFO("Dynamixel Driver Core - Debug - Motor 5 report start :");
            if (motorScanReport(3, DxlMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            if (motorCmdReport(3, DxlMotorType::MOTOR_TYPE_XL430) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            ROS_INFO("Dynamixel Driver Core - Debug - Motor 6 report start :");
            if (motorScanReport(6, DxlMotorType::MOTOR_TYPE_XL320) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            if (!_debug_flag)
                return niryo_robot_msgs::CommandStatus::ABORTED;
            if (motorCmdReport(6, DxlMotorType::MOTOR_TYPE_XL320) != niryo_robot_msgs::CommandStatus::SUCCESS)
                response = niryo_robot_msgs::CommandStatus::FAILURE;

            ros::Duration(1.0).sleep();
            ROS_INFO("Dynamixel Driver Core - Debug - Check for unflash dynamixel motors");
            motor_found = _dynamixel->type_ping_id(1, DxlMotorType::MOTOR_TYPE_XL430);
            if (motor_found == COMM_SUCCESS)
            {
                ROS_ERROR("Dynamixel Driver Core - Debug - Find a dynamixel motor unflash");
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            motor_found = _dynamixel->type_ping_id(1, DxlMotorType::MOTOR_TYPE_XL320);
            if (motor_found == COMM_SUCCESS)
            {
                ROS_ERROR("Dynamixel Driver Core - Debug - Find a dynamixel motor unflash");
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            return response;
        }
        ROS_ERROR("Dynamixel Driver Core - Debug - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }

    void DynamixelDriverCore::setDxlCommands(SynchronizeMotorCmd &cmd)
    {
        _dxl_cmd.reset(new SynchronizeMotorCmd(cmd));
    }

    void DynamixelDriverCore::setTrajectoryControllerCommands(std::vector<uint32_t> &cmd)
    {
        _joint_trajectory_controller_cmd = cmd;
    }

    std::vector<uint8_t> DynamixelDriverCore::scanTools()
    {
        std::vector<uint8_t> motor_list;
        ROS_INFO("Dynamixel Driver Core - Scan tools...");
        {
            std::lock_guard<std::mutex> lck(_control_loop_mutex);
            int result = _dynamixel->getAllIdsOnDxlBus(motor_list);
            ROS_DEBUG("Dynamixel Driver Core - Result getAllIdsOnDxlBus: %d", result);
        }
        std::string motor_id_list_string;
        std::for_each(std::begin(motor_list), std::end(motor_list),
                      [&motor_id_list_string](int x) {
                          motor_id_list_string += std::to_string(x) + " ";
                      });
        ROS_DEBUG("Dynamixel Driver Core - All id on dxl bus: [%s]", motor_id_list_string.c_str());
        return motor_list;
    }

    int DynamixelDriverCore::setEndEffector(uint8_t id, DxlMotorType type)
    {
        int result = this->ping_id(id, type);

        if (result == COMM_SUCCESS)
        {
            {
                std::lock_guard<std::mutex> lck(_control_loop_mutex);
                bool dynamixel_added_successfully = _dynamixel->addDynamixel(id, type);
                result = dynamixel_added_successfully ? niryo_robot_msgs::CommandStatus::SUCCESS
                                                        : niryo_robot_msgs::CommandStatus::FAILURE;
            }
        }
        else
        {
            ROS_WARN("Dynamixel Driver Core - End effector dxl error: %d with motor id %d", result, id);
            result = niryo_robot_msgs::CommandStatus::DXL_READ_ERROR;
        }
        return result;
    }

    int DynamixelDriverCore::ping_id(uint8_t id, DxlMotorType type)
    {
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        int result = _dynamixel->type_ping_id(id, type);
        ROS_DEBUG("Dynamixel Driver Core - Ping_id result: %d", result);
        return result;
    }

    void DynamixelDriverCore::unsetEndEffector(uint8_t id, DxlMotorType type)
    {
        ROS_DEBUG("Dynamixel Driver Core - UnsetEndEffector: id %d with type %d", id, static_cast<int>(type));
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        _dynamixel->removeDynamixel(id, type);
    }

    void DynamixelDriverCore::setEndEffectorCommands(std::vector<SingleMotorCmd> &cmd)
    {
        _end_effector_cmd = cmd;
    }

    uint32_t DynamixelDriverCore::getEndEffectorState(uint8_t id, DxlMotorType type)
    {
        DxlMotorState motor(id, type);
        uint32_t result = 0;
        std::vector<DxlMotorState> list_motor_states = _dynamixel->getMotorsState();
        auto it = std::find_if(list_motor_states.begin(), list_motor_states.end(), [&](DxlMotorState &dms) {
            return dms.getId() == motor.getId();
        });
        if (it != list_motor_states.end())
        {
            result = it->getPositionState();
        }
        return result;
    }

    std::vector<DxlMotorState> &DynamixelDriverCore::getDxlStates()
    {
        return _dynamixel->getMotorsState();
    }

    std::vector<int> &DynamixelDriverCore::getRemovedMotorList()
    {
        return _dynamixel->getRemovedMotorList();
    }

    dynamixel_driver::DxlArrayMotorHardwareStatus DynamixelDriverCore::getHwStatus()
    {
        dynamixel_driver::DxlMotorHardwareStatus data;
        dynamixel_driver::DxlArrayMotorHardwareStatus hw_state;
        std::vector<DxlMotorState> motor_states = _dynamixel->getMotorsState();

        for (int i = 0; i < motor_states.size(); i++)
        {
            data.motor_identity.motor_id = motor_states.at(i).getId();
            data.motor_identity.motor_type = (uint8_t)motor_states.at(i).getType();
            data.temperature = motor_states.at(i).getTemperatureState();
            data.voltage = double(motor_states.at(i).getVoltageState()) / DXL_VOLTAGE_DIVISOR;
            data.error = motor_states.at(i).getHardwareErrorState();
            data.error_msg = motor_states.at(i).getHardwareErrorMessageState();
            hw_state.motors_hw_status.push_back(data);
        }
        return hw_state;
    }

    niryo_robot_msgs::BusState DynamixelDriverCore::getDxlBusState()
    {
        niryo_robot_msgs::BusState dxl_bus_state;
        std::string error;
        bool connection;
        std::vector<uint8_t> motor_id;
        _dynamixel->getBusState(connection, motor_id, error);
        dxl_bus_state.connection_status = connection;
        dxl_bus_state.motor_id_connected = motor_id;
        dxl_bus_state.error = error;
        return dxl_bus_state;
    }

    void DynamixelDriverCore::_executeCommand()
    {
        bool need_sleep = false;
        if (!_joint_trajectory_controller_cmd.empty())
        {
            _dynamixel->executeJointTrajectoryCmd(_joint_trajectory_controller_cmd);
            _joint_trajectory_controller_cmd.clear();
            need_sleep = true;
        }
        if (_dxl_cmd.get() != nullptr)
        {
            if (need_sleep)
                ros::Duration(0.01).sleep();
            _dynamixel->readSynchronizeCommand(*_dxl_cmd.get());
            _dxl_cmd.reset();
            need_sleep = true;
        }
        if (!_end_effector_cmd.empty())
        {
            if (need_sleep)
                ros::Duration(0.01).sleep();
            SingleMotorCmd end_effector_command = _end_effector_cmd.at(0);
            if (_dynamixel->ping_id(end_effector_command.getId()) == COMM_SUCCESS)
            {
                _dynamixel->readSingleCommand(end_effector_command);
                _end_effector_cmd.erase (_end_effector_cmd.begin());
            }
        }
    }

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
                    ROS_WARN("Dynamixel Driver Core - Dynamixel connection error");
                    ros::Duration(0.1).sleep();

                    std::vector<int> missing_ids;
                    ROS_DEBUG("Dynamixel Driver Core - Scan to find Dxl motors");

                    int bus_state;
                    {
                        std::lock_guard<std::mutex> lck(_control_loop_mutex);
                        bus_state = _dynamixel->scanAndCheck();
                    }
                    while (bus_state != DXL_SCAN_OK)
                    { // wait for connection to be up
                        missing_ids = getRemovedMotorList();
                        for(std::vector<int>::iterator it = missing_ids.begin(); it != missing_ids.end(); ++it) {
                            ROS_WARN_THROTTLE(0.5, "Dynamixel Driver Core - Dynamixel %d do not seem to be connected", *it);
                        }
                        ros::Duration(0.25).sleep();
                        {
                            std::lock_guard<std::mutex> lck(_control_loop_mutex);
                            bus_state = _dynamixel->scanAndCheck();
                        }
                    }
                    ROS_INFO("Dynamixel Driver Core - Dxl Bus ok");
                }
                if (_control_loop_flag)
                {
                    {
                        std::lock_guard<std::mutex> lck(_control_loop_mutex);
                        if (ros::Time::now().toSec() - _time_hw_data_last_read > 1.0 / _read_data_frequency)
                        {
                            _time_hw_data_last_read += 1.0 / _read_data_frequency;
                            _dynamixel->readPositionState();
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

    bool DynamixelDriverCore::callbackSendCustomDxlValue(dynamixel_driver::SendCustomDxlValue::Request &req,
                                                         dynamixel_driver::SendCustomDxlValue::Response &res)
    {
        int result;
        DxlMotorType motor_type;
        if (req.motor_type == (uint8_t)DxlMotorType::MOTOR_TYPE_XL430)
        {
            motor_type = DxlMotorType::MOTOR_TYPE_XL430;
        }
        else if (req.motor_type == (uint8_t)DxlMotorType::MOTOR_TYPE_XL320)
        {
            motor_type = DxlMotorType::MOTOR_TYPE_XL320;
        }
        else
        {
            res.status = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
            res.message = "Dynamixel Driver Core - Invalid motor type: should be 1 (XL-320) or 2 (XL-430)";
            return true;
        }
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        result = _dynamixel->sendCustomDxlCommand(motor_type, req.id, req.value, req.reg_address, req.byte_number);

        if (result != COMM_SUCCESS)
        {
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

    bool DynamixelDriverCore::callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res)
    {
        int led = req.value;
        std::string message = "";

        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        int result = _dynamixel->setLeds(led);

        res.status = result;
        res.message = message;
        return true;
    }

    int DynamixelDriverCore::update_leds(void)
    {
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        int result = _dynamixel->setLeds(_dynamixel->getledstate());
        return result;
    }
} // namespace DynamixelDriver
// namespace DynamixelDriver
