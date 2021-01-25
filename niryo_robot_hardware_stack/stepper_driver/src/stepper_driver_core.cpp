#include "stepper_driver/stepper_driver_core.hpp"

namespace StepperDriver
{
    StepperDriverCore::StepperDriverCore()
    {
        init();
    }

    void StepperDriverCore::init()
    {
        _control_loop_flag = false;
        _calibration_in_progress = false;
        _debug_flag = false;
        _joint_trajectory_controller_cmd.clear();
        _stepper_cmd.reset(new StepperMotorCmd());
        _conveyor_cmd.reset(new StepperMotorCmd());
        initParameters();
        _stepper.reset(new StepperDriver());
        startControlLoop();

        cmd_pub = _nh.advertise<std_msgs::Int64MultiArray>("stepper_cmd", 1000);
    }

    void StepperDriverCore::initParameters()
    {
        // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        // ros::console::notifyLoggerLevelsChanged();
        // }
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hardware_control_loop_frequency", _control_loop_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hw_write_frequency", _write_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/stepper_driver/can_hw_check_connection_frequency", _check_connection_frequency);

        ROS_DEBUG("Stepper Driver Core - can_hardware_control_loop_frequency : %f", _control_loop_frequency);
        ROS_DEBUG("Stepper Driver Core - can_hardware_write_frequency : %f", _write_frequency);
        ROS_DEBUG("Stepper Driver Core - can_hardware_check_connection_frequency : %f", _check_connection_frequency);
    }

    void StepperDriverCore::startControlLoop()
    {
        resetHardwareControlLoopRates();
        if (!_control_loop_thread)
        {
            ROS_DEBUG("Stepper Driver Core - Start control loop thread");
            _control_loop_flag = true;
            _control_loop_thread.reset(new std::thread(boost::bind(&StepperDriverCore::controlLoop, this)));
        }
    }

    void StepperDriverCore::resetHardwareControlLoopRates()
    {
        ROS_DEBUG_THROTTLE(1,"Stepper Driver Core - Reset control loop rates");
        double now = ros::Time::now().toSec();
        _time_hw_last_write = now;
        _time_hw_last_check_connection = now;
    }

    void StepperDriverCore::activeDebugMode(bool mode)
    {
        ROS_INFO("Stepper Driver Core - Activate debug mode for dynamixel driver core: %d", mode);
        _debug_flag = mode;

        if (!mode)
        {
            std::lock_guard<std::mutex> lck(_control_loop_mutex);
            _stepper->sendTorqueOnCommand(2, 0);
            ros::Duration(0.2).sleep();
            _stepper->sendTorqueOnCommand(3, 0);
        }
    }

    int StepperDriverCore::motorReport(int motor_id)
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
                ROS_ERROR("Stepper Driver Core - Stepper Motor %d not found", motor_id);
                response = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            else
            {
                ROS_INFO("Stepper Driver Core - Stepper Motor %d found", motor_id);
            }
            return response;
        }
        ROS_ERROR("Stepper Driver Core - Debug mode not enabled");
        return response;
    }

    int StepperDriverCore::launchMotorsReport()
    {
        int response = niryo_robot_msgs::CommandStatus::SUCCESS;
        if (_debug_flag)
        {
            ROS_INFO("Stepper Driver Core - Debug - Start Stepper Motor Report");
            ros::Duration(0.5).sleep();
            for (int motor_id = 1; motor_id < 4; motor_id++)
            {
                ROS_INFO("Stepper Driver Core - Debug - Motor %d report start :", motor_id);
                if (motorReport(motor_id) != niryo_robot_msgs::CommandStatus::SUCCESS)
                {
                    response = niryo_robot_msgs::CommandStatus::FAILURE;
                }
            }

            if (!_debug_flag)
            {
                ROS_INFO("Stepper Driver Core - Debug - Debug motor aborted");
                return niryo_robot_msgs::CommandStatus::ABORTED;
            }

            for (int motor_id = 1; motor_id < 4; motor_id++)
            {
                ROS_INFO("Stepper Driver Core - Debug - Send torque on command on motor %d", motor_id);

                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    _stepper->sendTorqueOnCommand(motor_id, 1);
                }
                ros::Duration(0.2).sleep();

                int direction = (motor_id == 2)? -1 : 1;
                int old_position = _stepper->getStepperPose(motor_id);
                ROS_INFO("Stepper Driver Core - Debug - Get pose on motor %d: %d", motor_id, old_position);

                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    ROS_INFO("Stepper Driver Core - Debug - Send move command on motor %d", motor_id);
                    _stepper->sendRelativeMoveCommand(motor_id, -1000*direction, 1500);
                    ros::Duration(0.2).sleep();
                }
                ros::Duration(3).sleep();

                int new_position = _stepper->getStepperPose(motor_id);
                ROS_INFO("Stepper Driver Core - Debug - Get pose on motor %d: %d", motor_id, new_position);
                if (std::abs(new_position - old_position) < 250)
                {
                    ROS_WARN("Stepper Driver Core - Debug - Pose error on motor %d", motor_id);
                    response = niryo_robot_msgs::CommandStatus::FAILURE;
                }
                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);

                    ROS_INFO("Stepper Driver Core - Debug - Send move command on motor %d", motor_id);
                    _stepper->sendRelativeMoveCommand(motor_id, 1000*direction, 1000);
                    ros::Duration(0.2).sleep();
                }
                ros::Duration(3).sleep();

                int new_position2 = _stepper->getStepperPose(motor_id);
                ROS_INFO("Stepper Driver Core - Debug - Get pose on motor %d: %d", motor_id, new_position2);
                if (std::abs(new_position2 - new_position) < 250)
                {
                    ROS_WARN("Stepper Driver Core - Debug - Pose error on motor %d", motor_id);
                    response = niryo_robot_msgs::CommandStatus::FAILURE;
                }

                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    ROS_INFO("Stepper Driver Core - Debug - Send torque off command on motor %d", motor_id);
                    _stepper->sendTorqueOnCommand(motor_id, 0);
                    ros::Duration(0.2).sleep();
                }
                if (!_debug_flag)
                {
                    ROS_INFO("Stepper Driver Core - Debug - Debug motor aborted");
                    return niryo_robot_msgs::CommandStatus::ABORTED;
                }
            }
            return response;
        }
        ROS_ERROR("Stepper Driver Core - Debug - Debug mode not enabled");
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }

    void StepperDriverCore::setStepperCommands(StepperMotorCmd &cmd)
    {
        _stepper_cmd.reset(new StepperMotorCmd(cmd));
    }

    void StepperDriverCore::setTrajectoryControllerCommands(std::vector<int32_t> &cmd)
    {
        _joint_trajectory_controller_cmd = cmd;
    }

    std::vector<int32_t> &StepperDriverCore::getTrajectoryControllerStates()
    {
        return _stepper->getJointTrajectoryState();
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
            ROS_WARN("Stepper Driver Core - No conveyor found");
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

    void StepperDriverCore::setConveyorCommands(StepperMotorCmd &cmd)
    {
        _conveyor_cmd.reset(new StepperMotorCmd(cmd));
    }

    std::vector<ConveyorState> &StepperDriverCore::getConveyorStates()
    {
        return _stepper->getConveyorsState();
    }

    std::vector<StepperMotorState> &StepperDriverCore::getStepperStates()
    {
        return _stepper->getMotorsState();
    }

    void StepperDriverCore::startCalibration(bool enable)
    {
        _calibration_in_progress = enable;
        _stepper->setCalibrationInProgress(enable);
    }

    bool StepperDriverCore::getCalibrationState()
    {
        return _calibration_in_progress;
    }

    void StepperDriverCore::clearCalibrationTab()
    {
        _stepper->clearCalibrationTab();
    }

    e_CanStepperCalibrationStatus StepperDriverCore::getCalibrationResult(uint8_t id, boost::shared_ptr<int32_t> &calibration_result)
    {
        return _stepper->getCalibrationResult(id, calibration_result);
    }

    stepper_driver::StepperArrayMotorHardwareStatus StepperDriverCore::getHwStatus()
    {
        stepper_driver::StepperMotorHardwareStatus data;
        stepper_driver::StepperArrayMotorHardwareStatus hw_state;

        std::vector<StepperMotorState> motor_states = _stepper->getMotorsState();

        for (int i = 0; i < motor_states.size(); i++)
        {
            data.motor_identity.motor_id = motor_states.at(i).getId();
            data.motor_identity.motor_type = (uint8_t)StepperMotorType::MOTOR_TYPE_STEPPER;
            data.temperature = motor_states.at(i).getTemperatureState();
            data.error = motor_states.at(i).getHardwareErrorState();
            data.firmware_version = motor_states.at(i).getFirmwareVersion();
            hw_state.motors_hw_status.push_back(data);
        }
        return hw_state;
    }

    niryo_robot_msgs::BusState StepperDriverCore::getCanBusState()
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

    void StepperDriverCore::_executeCommand()
    {
        std_msgs::Int64MultiArray cmd;
        if (_joint_trajectory_controller_cmd.size() != 0)
        {
            _stepper->executeJointTrajectoryCmd(_joint_trajectory_controller_cmd);
            cmd.data.push_back(_joint_trajectory_controller_cmd[0]);
            cmd.data.push_back(_joint_trajectory_controller_cmd[1]);
            cmd.data.push_back(_joint_trajectory_controller_cmd[2]);
            cmd_pub.publish(cmd);
            // _stepper->readCommand(*_controller_cmd.get());
            _joint_trajectory_controller_cmd.clear();
        }
        if (_stepper_cmd.get() != nullptr)
        {
            _stepper->readCommand(*_stepper_cmd.get());
            _stepper_cmd.reset();
        }
        if (_conveyor_cmd.get() != nullptr)
        {
            _stepper->readCommand(*_conveyor_cmd.get());
            _conveyor_cmd.reset();
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
                ROS_WARN("Stepper Driver Core - Stepper connection error");
                ros::Duration(0.1).sleep();

                while (!_stepper->isConnectionOk())
                { // wait for connection to be up
                    ROS_INFO("Stepper Driver Core - Scan to find stepper motors");
                    {
                        std::lock_guard<std::mutex> lck(_control_loop_mutex);
                        _stepper->scanAndCheck();
                    }
                    ros::Duration(0.1).sleep();
                }
                ROS_INFO("Stepper Driver Core - Stepper Can bus ok");
            }
            if (_control_loop_flag)
            {
                {
                    std::lock_guard<std::mutex> lck(_control_loop_mutex);
                    if (!_calibration_in_progress)
                    {
                        _stepper->readMotorsState();
                    }

                    if (ros::Time::now().toSec() - _time_hw_last_write > 1.0 / _write_frequency)
                    {
                        _time_hw_last_write += 1.0 / _write_frequency;
                        _executeCommand();
                    }

                    /*if (ros::Time::now().toSec() - _time_hw_last_check_connection > 1.0/_check_connection_frequency)
                    {
                        _time_hw_last_check_connection += 1.0/_check_connection_frequency;
                        if(!_calibration_in_progress)
                        {
                            _stepper->scanAndCheck();
                        }
                    }*/
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

    bool StepperDriverCore::scanMotorId(int motor_to_find)
    {
        std::lock_guard<std::mutex> lck(_control_loop_mutex);
        return _stepper->scanMotorId(motor_to_find);
    }

    bool StepperDriverCore::isConnectionOk() const
    {
        return _stepper->isConnectionOk();
    }
} // namespace StepperDriver
