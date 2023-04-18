/*
    can_interface_core.cpp
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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include <cstdint>
#include <functional>

#include <std_msgs/Int64MultiArray.h>

// c++
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// common
#include "can_driver/can_interface_core.hpp"
#include "common/model/abstract_single_motor_cmd.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/util/unique_ptr_cast.hpp"
#include "common/util/util_defs.hpp"

using ::std::lock_guard;
using ::std::mutex;
using ::std::string;
using ::std::thread;
using ::std::vector;

using ::common::model::AbstractCanSingleMotorCmd;
using ::common::model::EHardwareType;
using ::common::model::EStepperCommandType;
using ::common::model::HardwareTypeEnum;
using ::common::model::JointState;
using ::common::model::StepperSingleCmd;

namespace can_driver
{
/**
 * @brief CanInterfaceCore::CanInterfaceCore
 */
CanInterfaceCore::CanInterfaceCore(ros::NodeHandle &nh)
{
    ROS_DEBUG("CanInterfaceCore::CanInterfaceCore - ctor");

    init(nh);

    _can_manager = std::make_unique<CanManager>(nh);

    startControlLoop();
}

/**
 * @brief CanInterfaceCore::~CanInterfaceCore
 */
CanInterfaceCore::~CanInterfaceCore()
{
    if (_control_loop_thread.joinable())
        _control_loop_thread.join();
}

/**
 * @brief CanInterfaceCore::init Initialize ros things
 */
bool CanInterfaceCore::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("CanInterfaceCore::init - Initializing parameters...");
    initParameters(nh);

    ROS_DEBUG("CanInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("CanInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("CanInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief CanInterfaceCore::initParameters
 */
void CanInterfaceCore::initParameters(ros::NodeHandle &nh)
{
    _control_loop_frequency = 0.0;
    double write_frequency = 1.0;

    nh.getParam("can_hardware_control_loop_frequency", _control_loop_frequency);

    nh.getParam("can_hw_write_frequency", write_frequency);

    ROS_DEBUG("CanInterfaceCore::initParameters - can_hardware_control_loop_frequency : %f", _control_loop_frequency);

    ROS_DEBUG("CanInterfaceCore::initParameters - can_hw_write_frequency : %f", write_frequency);

    _delta_time_write = 1.0 / write_frequency;
}

/**
 * @brief CanInterfaceCore::startServices
 */
void CanInterfaceCore::startServices(ros::NodeHandle & /*nh*/) { ROS_DEBUG("CanInterfaceCore::startServices - no services to start"); }

/**
 * @brief CanInterfaceCore::startPublishers
 * @param nh
 */
void CanInterfaceCore::startPublishers(ros::NodeHandle & /*nh*/) { ROS_DEBUG("CanInterfaceCore::startServices - no publishers to start"); }

/**
 * @brief CanInterfaceCore::startSubscribers
 */
void CanInterfaceCore::startSubscribers(ros::NodeHandle & /*nh*/) { ROS_DEBUG("CanInterfaceCore::startServices - no subscribers to start"); }

/**
 * @brief TtlInterfaceCore::addJoint add joints from joints_interface
 * @param jointState
 * @return
 */
int CanInterfaceCore::addJoint(const std::shared_ptr<common::model::StepperMotorState> &jointState) { return _can_manager->addHardwareComponent(jointState); }

// ***************
//  Commands
// ***************

/**
 * @brief CanInterfaceCore::scanMotorId
 * @param motor_to_find
 * @return
 */
bool CanInterfaceCore::scanMotorId(uint8_t motor_to_find)
{
    lock_guard<mutex> lck(_control_loop_mutex);
    return _can_manager->ping(motor_to_find);
}

/**
 * @brief CanInterfaceCore::rebootMotor
 * @param motor_state
 * @return
 */
bool CanInterfaceCore::rebootHardware(const std::shared_ptr<common::model::AbstractHardwareState> & /*motor_state*/)
{
    ROS_ERROR("Reboot motors Not available for CAN");
    return false;
}

/**
 * @brief CanInterfaceCore::startCalibration
 */
void CanInterfaceCore::startCalibration()
{
    if (_can_manager)
        _can_manager->startCalibration();
}

/**
 * @brief CanInterfaceCore::resetCalibration
 */
void CanInterfaceCore::resetCalibration()
{
    if (_can_manager)
        _can_manager->resetCalibration();
}

/**
 * @brief CanInterfaceCore::motorCmdReport
 * @param jState
 * @param motor_type
 * @return
 */
int CanInterfaceCore::motorCmdReport(const JointState &jState, common::model::EHardwareType motor_type)
{
    int ret = niryo_robot_msgs::CommandStatus::ABORTED;

    if (motor_type != EHardwareType::UNKNOWN)
    {
        if (_debug_flag)
        {
            uint8_t motor_id = jState.getId();

            // torque on
            ros::Duration(0.5).sleep();
            ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - Send torque on command to motor %d", motor_id);

            _can_manager->writeSingleCommand(std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_TORQUE, motor_id, std::initializer_list<int32_t>{1}));
            ros::Duration(0.5).sleep();

            // move one direction
            auto direction = jState.getDirection();
            int32_t old_position = _can_manager->getPosition(jState);
            ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - Get pose on motor %d: %d", motor_id, old_position);
            ros::Duration(0.5).sleep();

            ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - Send move command on motor %d", motor_id);
            _can_manager->writeSingleCommand(
                std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor_id, std::initializer_list<int32_t>{-1000 * direction, 1500}));
            ros::Duration(2).sleep();
            int32_t new_position = _can_manager->getPosition(jState);
            ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - Get pose on motor %d: %d", motor_id, new_position);

            // set position back to old position
            ros::Duration(0.5).sleep();

            ROS_INFO("CanInterfaceCore::motorCmdReport - Send can motor %d pose: %d ", motor_id, old_position);
            _can_manager->writeSingleCommand(
                std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor_id, std::initializer_list<int32_t>{1000 * direction, 1000}));
            ros::Duration(2).sleep();
            int32_t new_position2 = _can_manager->getPosition(jState);
            ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - get can motor %d pose: %d ", motor_id, new_position2);
            int rest2 = static_cast<int>(new_position2 - new_position);
            ros::Duration(0.5).sleep();

            // torque off
            ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - Send torque off command on can motor %d", motor_id);
            _can_manager->writeSingleCommand(std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_TORQUE, motor_id, std::initializer_list<int32_t>{0}));
            ros::Duration(0.2).sleep();

            if (abs(rest2) < 250)
            {
                ROS_WARN("CanInterfaceCore::motorCmdReport - Debug - Pose error on motor %d", motor_id);
                ret = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            else
            {
                ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - Stepper Motor %d OK", motor_id);
                ret = niryo_robot_msgs::CommandStatus::SUCCESS;
            }
        }
        else
        {
            ROS_INFO("CanInterfaceCore::motorCmdReport - Debug - Debug motor aborted");
        }
    }

    return ret;
}

/**
 * @brief CanInterfaceCore::launchMotorsReport
 * @return
 */
int CanInterfaceCore::launchMotorsReport()
{
    int response = niryo_robot_msgs::CommandStatus::ABORTED;
    unsigned int nbSuccess = 0;
    unsigned int nbFailure = 0;

    std::vector<std::tuple<uint8_t, EHardwareType, int>> results;

    if (_debug_flag)
    {
        ROS_INFO("CanInterfaceCore::launchMotorsReport - Debug - Start Stepper Motor Report");
        ros::Duration(0.5).sleep();
        if (CAN_OK == _can_manager->scanAndCheck())
        {
            for (auto const &state : _can_manager->getMotorsStates())
            {
                if (state)
                {
                    ROS_INFO("CanInterfaceCore::launchMotorsReport - Debug - Motor %d report start :", static_cast<int>(state->getId()));

                    int cmd_res = motorCmdReport(*state, state->getHardwareType());

                    if (niryo_robot_msgs::CommandStatus::SUCCESS == cmd_res)
                    {
                        nbSuccess++;
                    }
                    else
                    {
                        nbFailure++;
                        response = niryo_robot_msgs::CommandStatus::FAILURE;
                    }

                    results.emplace_back(state->getId(), state->getHardwareType(), cmd_res);

                    if (niryo_robot_msgs::CommandStatus::ABORTED == cmd_res)
                    {
                        ROS_INFO("CanInterfaceCore::launchMotorsReport - Debug - Debug motor aborted");
                        break;
                    }
                }
            }
            if (nbFailure == 0)
                response = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("DynamixelDriverCore::launchMotorsReport - Debug - unable to contact all steppers");
            response = niryo_robot_msgs::CommandStatus::FAILURE;
            nbFailure++;
        }

        // display synthesis
        ROS_INFO("Synthesis of Stepper motors report :");
        ROS_INFO("--------");
        ROS_INFO("Nb Success : %d, Nb Failure: %d", nbSuccess, nbFailure);
        ROS_INFO("Details: ");
        ROS_INFO("--------");
        ROS_INFO("id \t| type \t| cmd result");

        for (auto const &res : results)
        {
            ROS_INFO("%d \t| %s \t| %d", std::get<0>(res), HardwareTypeEnum(std::get<1>(res)).toString().c_str(), std::get<2>(res));
        }
    }
    else
    {
        ROS_ERROR("CanInterfaceCore::launchMotorsReport - Debug - Debug mode not enabled");
    }

    return response;
}

// ****************
//  Control Loop
// ****************

/**
 * @brief CanInterfaceCore::startControlLoop
 */
void CanInterfaceCore::startControlLoop()
{
    resetHardwareControlLoopRates();
    if (!_control_loop_flag)
    {
        ROS_DEBUG("CanInterfaceCore::startControlLoop - Start control loop thread");
        _control_loop_flag = true;
        _control_loop_thread = thread(&CanInterfaceCore::controlLoop, this);
    }
}

/**
 * @brief CanInterfaceCore::resetHardwareControlLoopRates
 */
void CanInterfaceCore::resetHardwareControlLoopRates()
{
    ROS_DEBUG("CanInterfaceCore::resetHardwareControlLoopRates - Reset control loop rates");
    double now = ros::Time::now().toSec();
    _time_hw_data_last_write = now;
    _time_hw_data_last_read = now;

    _time_check_connection_last_read = now;
}

/**
 * @brief CanInterfaceCore::activeDebugMode
 * @param mode
 */
void CanInterfaceCore::activeDebugMode(bool mode)
{
    ROS_INFO("CanInterfaceCore::activeDebugMode - Activate debug mode for stepper driver core: %d", mode);
    _debug_flag = mode;

    if (!mode)
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        // disable torque on motor 2
        _can_manager->writeSingleCommand(std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_TORQUE, 2, std::initializer_list<int32_t>{0}));
        ros::Duration(0.2).sleep();

        // disable torque on motor 3
        _can_manager->writeSingleCommand(std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_TORQUE, 3, std::initializer_list<int32_t>{0}));
    }
}

/**
 * @brief CanInterfaceCore::controlLoop
 */
void CanInterfaceCore::controlLoop()
{
    ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);
    resetHardwareControlLoopRates();
    while (ros::ok())
    {
        // Do not readStatus if connection is down or not be established yet
        if (!_can_manager->isConnectionOk())
        {
            _can_manager->scanAndCheck();
            control_loop_rate.sleep();
        }
        else if (_control_loop_flag)
        {
            {
                lock_guard<mutex> lck(_control_loop_mutex);
                _can_manager->readStatus();

                if (ros::Time::now().toSec() - _time_hw_data_last_write >= _delta_time_write)
                {
                    _time_hw_data_last_write = ros::Time::now().toSec();
                    _executeCommand();
                }
            }

            bool isFreqMet = control_loop_rate.sleep();
            if (!isFreqMet)
                ROS_DEBUG_THROTTLE(2, "CanInterfaceCore::rosControlLoop : freq not met : expected (%f s) vs actual (%f s)", control_loop_rate.expectedCycleTime().toSec(),
                                   control_loop_rate.cycleTime().toSec());
        }
        else
        {
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
            resetHardwareControlLoopRates();
        }
    }
}

/**
 * @brief CanInterfaceCore::_executeCommand
 */
void CanInterfaceCore::_executeCommand()
{
    if (!_joint_trajectory_cmd.empty())
    {
        _can_manager->executeJointTrajectoryCmd(_joint_trajectory_cmd);
        _joint_trajectory_cmd.clear();
    }

    if (!_stepper_single_cmds.empty())
    {
        // as we use a queue, we don't need a mutex
        _can_manager->writeSingleCommand(std::move(_stepper_single_cmds.front()));
        _stepper_single_cmds.pop();
    }
    if (!_conveyor_cmds.empty())
    {
        // as we use a queue, we don't need a mutex
        _can_manager->writeSingleCommand(std::move(_conveyor_cmds.front()));
        _conveyor_cmds.pop();
    }
}

// *************
//  Setters
// *************

/**
 * @brief CanInterfaceCore::setConveyor add conveyor from conveyor_interface
 * @param state
 * @return
 */
int CanInterfaceCore::setConveyor(const std::shared_ptr<common::model::ConveyorState> &state)
{
    int result = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;

    // add stepper as a new conveyor
    _can_manager->addHardwareComponent(state);

    bool res;
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        res = _can_manager->ping(state->getId());
    }
    // try to find motor id 6 (default motor id for conveyor
    if (res)
    {
        // send commands to init
        ROS_DEBUG("ConveyorInterfaceCore::addConveyor : Initializing for CAN bus");

        _can_manager->writeSingleCommand(std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_MICRO_STEPS, state->getId(),
                                                                            std::initializer_list<int32_t>{static_cast<int32_t>(state->getMicroSteps())}));

        _can_manager->writeSingleCommand(std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_MAX_EFFORT, state->getId(),
                                                                            std::initializer_list<int32_t>{static_cast<int32_t>(state->getMaxEffort())}));

        _can_manager->writeSingleCommand(std::make_unique<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_CONVEYOR, state->getId(), std::initializer_list<int32_t>{false, 0, -1}));

        result = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    else
    {
        ROS_DEBUG("CanInterfaceCore::setConveyor - No conveyor found");
    }

    return result;
}

/**
 * @brief CanInterfaceCore::unsetConveyor
 * @param motor_id
 * @param default_conveyor_id
 */
void CanInterfaceCore::unsetConveyor(uint8_t motor_id, uint8_t default_conveyor_id)
{
    ROS_DEBUG("CanInterfaceCore::unsetConveyor - unsetConveyor: id %d", motor_id);
    auto state = getJointState(motor_id);

    int res;
    {
        lock_guard<mutex> lck(_control_loop_mutex);
        res = _can_manager->changeId(state->getHardwareType(), motor_id, default_conveyor_id);
    }

    if (CAN_OK == res)
    {
        _can_manager->removeHardwareComponent(default_conveyor_id);
    }
    else
        ROS_ERROR("CanInterfaceCore::unsetConveyor : unable to change conveyor ID");
}

/**
 * @brief CanInterfaceCore::changeId
 * @param motor_type
 * @param old_id
 * @param new_id
 * @return
 */
int CanInterfaceCore::changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id)
{
    if (CAN_OK == _can_manager->changeId(motor_type, old_id, new_id))
        return niryo_robot_msgs::CommandStatus::SUCCESS;

    ROS_ERROR("CanInterfaceCore::setConveyor : unable to change conveyor ID");
    return niryo_robot_msgs::CommandStatus::CAN_WRITE_ERROR;
}

/**
 * @brief CanInterfaceCore::clearSingleCommandQueue
 */
void CanInterfaceCore::clearSingleCommandQueue()
{
    while (!_stepper_single_cmds.empty())
        _stepper_single_cmds.pop();
}

/**
 * @brief CanInterfaceCore::clearConveyorCommandQueue
 */
void CanInterfaceCore::clearConveyorCommandQueue()
{
    while (!_conveyor_cmds.empty())
        _conveyor_cmds.pop();
}

/**
 * @brief CanInterfaceCore::setTrajectoryControllerCommands
 * @param cmd
 */
void CanInterfaceCore::setTrajectoryControllerCommands(std::vector<std::pair<uint8_t, int32_t>> &&cmd) { _joint_trajectory_cmd = cmd; }  // NOLINT

/**
 * @brief CanInterfaceCore::addSingleCommandToQueue
 * @param cmd : needs to be a ptr for two reasons :
 * - cannot be a template method because we need it to be virtual (no static polymorphism possible)
 * - dynamic polymorphism necessary to be able to cast into a derived class
 */
void CanInterfaceCore::addSingleCommandToQueue(std::unique_ptr<common::model::ISingleMotorCmd> &&cmd)  // NOLINT
{
    ROS_DEBUG("CanInterfaceCore::addSingleCommandToQueue - %s", cmd->str().c_str());

    if (cmd && cmd->isValid())
    {
        if (cmd->getCmdType() == static_cast<int>(EStepperCommandType::CMD_TYPE_CONVEYOR))
        {
            // keep position cmd apart
            if (_conveyor_cmds.size() > QUEUE_OVERFLOW)
            {
                ROS_WARN("CanInterfaceCore::addCommandToQueue: Cmd queue overflow ! %d", static_cast<int>(_conveyor_cmds.size()));
            }
            else
            {
                _conveyor_cmds.push(common::util::static_unique_ptr_cast<common::model::AbstractCanSingleMotorCmd>(std::move(cmd)));
            }
        }
        else
        {
            if (_stepper_single_cmds.size() > QUEUE_OVERFLOW)
            {
                ROS_WARN("CanInterfaceCore::addCommandToQueue: Cmd queue overflow ! %d", static_cast<int>(_stepper_single_cmds.size()));
            }
            else
            {
                _stepper_single_cmds.push(common::util::static_unique_ptr_cast<common::model::AbstractCanSingleMotorCmd>(std::move(cmd)));
            }
        }
    }
}

/**
 * @brief CanInterfaceCore::addSingleCommandToQueue
 * @param cmd
 */
void CanInterfaceCore::addSingleCommandToQueue(std::vector<std::unique_ptr<common::model::ISingleMotorCmd>> cmd)
{
    for (auto &c : cmd)
        addSingleCommandToQueue(std::move(c));
}

/**
 * @brief CanInterfaceCore::setSyncCommand
 * @param cmd
 */
void CanInterfaceCore::addSyncCommandToQueue(std::unique_ptr<common::model::ISynchronizeMotorCmd> && /*cmd*/)  // NOLINT
{
    ROS_INFO("CanInterfaceCore::setSyncCommand: need to be implemented");
}
// ********************
//  getters
// ********************

/**
 * @brief CanInterfaceCore::getJointStates
 * @return
 */
std::vector<std::shared_ptr<common::model::JointState>> CanInterfaceCore::getJointStates() const
{
    std::vector<std::shared_ptr<common::model::JointState>> jstates;
    for (size_t i = 0; i < _can_manager->getMotorsStates().size(); i++)
    {
        jstates.push_back(_can_manager->getMotorsStates().at(i));
    }
    return jstates;
}

/**
 * @brief CanInterfaceCore::getBusState
 * @return
 */
niryo_robot_msgs::BusState CanInterfaceCore::getBusState() const
{
    niryo_robot_msgs::BusState can_bus_state;

    string error;
    bool connection;
    vector<uint8_t> motor_id;

    _can_manager->getBusState(connection, motor_id, error);
    can_bus_state.connection_status = connection;
    can_bus_state.motor_id_connected = motor_id;
    can_bus_state.error = error;
    return can_bus_state;
}

/**
 * @brief CanInterfaceCore::getJointState
 * @param motor_id
 * @return
 */
std::shared_ptr<common::model::JointState> CanInterfaceCore::getJointState(uint8_t motor_id) const
{
    return std::dynamic_pointer_cast<common::model::JointState>(_can_manager->getHardwareState(motor_id));
}

/**
 * @brief can_driver::CanInterfaceCore::getRemovedMotorList
 */
std::vector<uint8_t> CanInterfaceCore::getRemovedMotorList() const { return _can_manager->getRemovedMotorList(); }

}  // namespace can_driver
