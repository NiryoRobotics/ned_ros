/*
    ttl_interface_core.cpp
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

#include "ttl_driver/ttl_interface_core.hpp"

// c++
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>
#include <utility>
#include <string>
#include <tuple>

using ::std::lock_guard;
using ::std::mutex;
using ::std::vector;
using ::std::ostringstream;
using ::std::string;
using ::std::to_string;

using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;
using ::common::model::StepperMotorState;
using ::common::model::DxlMotorState;
using ::common::model::JointState;
using ::common::model::EDxlCommandType;
using ::common::model::DxlCommandTypeEnum;
using ::common::model::EStepperCommandType;
using ::common::model::StepperCommandTypeEnum;
using ::common::model::SingleMotorCmd;
using ::common::model::SynchronizeMotorCmd;
using ::common::model::AbstractTtlSingleMotorCmd;
using ::common::model::DxlSingleCmd;

namespace ttl_driver
{
/**
 * @brief TtlInterfaceCore::TtlInterfaceCore
 */
TtlInterfaceCore::TtlInterfaceCore(ros::NodeHandle& nh)
{
    ROS_DEBUG("TtlInterfaceCore - ctor");

    init(nh);
}

/**
 * @brief TtlInterfaceCore::~TtlInterfaceCore
 */
TtlInterfaceCore::~TtlInterfaceCore()
{
    if (_control_loop_thread.joinable())
        _control_loop_thread.join();
}

/**
 * @brief TtlInterfaceCore::init
 */
bool TtlInterfaceCore::init(ros::NodeHandle& nh)
{
    ROS_DEBUG("TtlInterfaceCore::init - Init parameters...");
    initParameters(nh);

    _ttl_manager = std::make_unique<TtlManager>(nh);
    _ttl_manager->scanAndCheck();
    startControlLoop();

    ROS_DEBUG("TtlInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("TtlInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("TtlInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief TtlInterfaceCore::initParameters
 */
void TtlInterfaceCore::initParameters(ros::NodeHandle& nh)
{
    _control_loop_frequency = 0.0;
    double write_frequency = 0.0;
    double read_data_frequency = 0.0;
    double read_status_frequency = 0.0;

    nh.getParam("ttl_hardware_control_loop_frequency",
                 _control_loop_frequency);

    nh.getParam("ttl_hardware_write_frequency",
                 write_frequency);

    nh.getParam("ttl_hardware_read_data_frequency",
                 read_data_frequency);

    nh.getParam("ttl_hardware_read_status_frequency",
                 read_status_frequency);

    ROS_DEBUG("TtlInterfaceCore::initParameters - ttl_hardware_control_loop_frequency : %f", _control_loop_frequency);
    ROS_DEBUG("TtlInterfaceCore::initParameters - ttl_hardware_write_frequency : %f", write_frequency);
    ROS_DEBUG("TtlInterfaceCore::initParameters - ttl_hardware_read_data_frequency : %f", read_data_frequency);
    ROS_DEBUG("TtlInterfaceCore::initParameters - ttl_hardware_read_status_frequency : %f", read_status_frequency);

    _delta_time_data_read = 1.0 / read_data_frequency;
    _delta_time_status_read = 1.0 / read_status_frequency;
    _delta_time_write = 1.0 / write_frequency;
}

/**
 * @brief TtlInterfaceCore::startServices
 * @param nh
 */
void TtlInterfaceCore::startServices(ros::NodeHandle& nh)
{
    // advertise services
    _activate_leds_server = nh.advertiseService("/niryo_robot/ttl_driver/set_dxl_leds",
                                                 &TtlInterfaceCore::_callbackActivateLeds, this);

    _custom_cmd_server = nh.advertiseService("/niryo_robot/ttl_driver/send_custom_value",
                                              &TtlInterfaceCore::_callbackSendCustomValue, this);

    _custom_cmd_getter = nh.advertiseService("/niryo_robot/ttl_driver/read_custom_value",
                                              &TtlInterfaceCore::_callbackReadCustomValue, this);
}

/**
 * @brief TtlInterfaceCore::startPublishers
 * @param nh
 */
void TtlInterfaceCore::startPublishers(ros::NodeHandle &/*nh*/)
{
    ROS_DEBUG("TtlInterfaceCore::startSubscribers - no publishers to start");
}

/**
 * @brief TtlInterfaceCore::startSubscribers
 * @param nh
 */
void TtlInterfaceCore::startSubscribers(ros::NodeHandle &/*nh*/)
{
    ROS_DEBUG("TtlInterfaceCore::startSubscribers - no subscribers to start");
}

// ***************
//  Commands
// ***************

/**
 * @brief TtlInterfaceCore::rebootMotors
 * @return
 */
int TtlInterfaceCore::rebootMotors()
{
    ROS_INFO("TtlInterfaceCore::rebootMotors - Reboot motors");
    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_manager->rebootMotors();
    ros::Duration(1.5).sleep();
    return result;
}


/**
 * @brief TtlInterfaceCore::rebootMotors
 * @return
 */
bool TtlInterfaceCore::rebootMotor(uint8_t motor_id)
{
    ROS_INFO("TtlInterfaceCore::rebootMotor - Reboot motor %d", static_cast<int>(motor_id));
    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_manager->rebootMotor(motor_id);
    ros::Duration(1.5).sleep();
    // return truc if result is COMM_SUCCESS
    return (COMM_SUCCESS == result);
}

/**
 * @brief TtlInterfaceCore::scanTools
 * @return
 */
vector<uint8_t> TtlInterfaceCore::scanTools()
{
    vector<uint8_t> motor_list;
    lock_guard<mutex> lck(_control_loop_mutex);
    ROS_INFO("TtlInterfaceCore::scanTools - Scan tools...");
    int result = COMM_PORT_BUSY;

    for (int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
    {
        result = _ttl_manager->getAllIdsOnBus(motor_list);
        ROS_DEBUG_COND(COMM_SUCCESS != result, "Driver::scanTools status: %d (counter: %d)", result, counter);
        ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
    }
    ROS_DEBUG("TtlInterfaceCore::scanTools - Result getAllIdsOnDxlBus: %d", result);

    ostringstream ss;
    for (auto const& m : motor_list)
        ss << static_cast<int>(m) << " ";
    string motor_id_list_string = ss.str();
    if (!motor_id_list_string.empty())
        motor_id_list_string.pop_back();  // remove trailing " "

    ROS_DEBUG("TtlInterfaceCore::scanTools - All id on bus: [%s]", motor_id_list_string.c_str());
    return motor_list;
}

/**
 * @brief TtlInterfaceCore::update_leds
 * @return
 */
int TtlInterfaceCore::update_leds(void)
{
    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_manager->setLeds(_ttl_manager->getLedState());
    return result;
}

/**
 * @brief TtlInterfaceCore::motorScanReport
 * @param motor_id
 * @return
 */
int TtlInterfaceCore::motorScanReport(uint8_t motor_id)
{
    if (_debug_flag)
    {
        ros::Duration(1.0).sleep();
        if (_ttl_manager->ping(motor_id))
        {
            ROS_INFO("TtlInterfaceCore::motorScanReport - Debug - Motor %d found", motor_id);
        }
        else
        {
            ROS_ERROR("TtlInterfaceCore::motorScanReport - Debug - Motor %d not found", motor_id);
            return niryo_robot_msgs::CommandStatus::FAILURE;
        }
        return niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    ROS_ERROR("TtlInterfaceCore::motorScanReport - Debug mode not enabled");
    return niryo_robot_msgs::CommandStatus::ABORTED;
}

/**
 * @brief TtlInterfaceCore::motorCmdReport
 * @param motor_id
 * @param motor_type
 * @return
 */
int TtlInterfaceCore::motorCmdReport(uint8_t motor_id, EHardwareType motor_type)
{
    int ret = niryo_robot_msgs::CommandStatus::ABORTED;

    if (_debug_flag)
    {
        if (motor_type == EHardwareType::UNKNOWN)
            return 0;

        JointState tmp_state = JointState("unknown", motor_type, common::model::EBusProtocol::TTL, motor_id);

        // torque on
        ros::Duration(0.5).sleep();
        ROS_INFO("TtlInterfaceCore::motorCmdReport - Debug - Send torque on command to motor %d", motor_id);
        if (motor_type != EHardwareType::STEPPER)
            ROS_INFO("TtlInterfaceCore::motorCmdReport: Implement in case we have stepper");
        else
        {
            std::shared_ptr<AbstractTtlSingleMotorCmd> cmd_torque = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_TORQUE,
                                                                                                   motor_id,
                                                                                                   std::initializer_list<uint32_t>{1});
            _ttl_manager->writeSingleCommand(cmd_torque);
            ros::Duration(0.5).sleep();

            // set position to old position + 200
            uint32_t old_position = _ttl_manager->getPosition(tmp_state);
            ROS_INFO("TtlInterfaceCore::motorCmdReport - Debug - get dxl %d pose: %d ", motor_id, old_position);
            ros::Duration(0.5).sleep();
            ROS_INFO("TtlInterfaceCore::motorCmdReport - Debug - Send dxl %d pose: %d ", motor_id, old_position + 200);
            std::shared_ptr<AbstractTtlSingleMotorCmd> cmd_pos = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION,
                                                                                                motor_id,
                                                                                                std::initializer_list<uint32_t>{old_position + 200});
            _ttl_manager->writeSingleCommand(cmd_pos);

            // set position back to old position
            ros::Duration(2).sleep();
            uint32_t new_position = _ttl_manager->getPosition(tmp_state);
            ROS_INFO("TtlInterfaceCore::motorCmdReport - Debug - get dxl %d pose: %d ", motor_id, new_position);
            int rest = static_cast<int>(new_position - old_position);

            ROS_INFO("TtlInterfaceCore - Debug - Send dxl %d pose: %d ", motor_id, old_position);
            std::shared_ptr<AbstractTtlSingleMotorCmd> cmd_pos_2 = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION,
                                                                                                  motor_id,
                                                                                                  std::initializer_list<uint32_t>{old_position});
            _ttl_manager->writeSingleCommand(cmd_pos_2);

            ros::Duration(2).sleep();
            uint32_t new_position2 = _ttl_manager->getPosition(tmp_state);
            ROS_INFO("TtlInterfaceCore::motorCmdReport - Debug - get dxl %d pose: %d ", motor_id, new_position2);
            int rest2 = static_cast<int>(new_position2 - new_position);

            // torque off
            ROS_INFO("TtlInterfaceCore::motorCmdReport - Debug - Send torque off command on dxl %d", motor_id);
            std::shared_ptr<AbstractTtlSingleMotorCmd> cmd_torque_2 = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_TORQUE,
                                                                                                     motor_id,
                                                                                                     std::initializer_list<uint32_t>{0});
            _ttl_manager->writeSingleCommand(cmd_torque_2);

            if (abs(rest) < 50 || abs(rest2) < 50)
            {
                ROS_WARN("TtlInterfaceCore::motorCmdReport - Debug - Dynamixel Motor %d problem", motor_id);
                ret = niryo_robot_msgs::CommandStatus::FAILURE;
            }
            else
            {
                ROS_INFO("TtlInterfaceCore::motorCmdReport - Debug - Dynamixel Motor %d OK", motor_id);
                ret = niryo_robot_msgs::CommandStatus::SUCCESS;
            }
        }
    }
    else
    {
        ROS_ERROR("TtlInterfaceCore::motorCmdReport - Debug - Debug mode not enabled");
    }

    return ret;
}

/**
 * @brief TtlInterfaceCore::launchMotorsReport
 * @return
 */
int TtlInterfaceCore::launchMotorsReport()
{
    int response = niryo_robot_msgs::CommandStatus::SUCCESS;
    unsigned int nbSuccess = 0;
    unsigned int nbFailure = 0;

    std::vector<std::tuple<uint8_t, EHardwareType, int, int> > results;

    if (_debug_flag)
    {
        ROS_INFO("TtlInterfaceCore::launchMotorsReport - Debug - Start Motor Report");
        ros::Duration(1.0).sleep();

        for (auto const& state : _ttl_manager->getMotorsStates())
        {
            if (state)
            {
                ROS_INFO("TtlInterfaceCore::launchMotorsReport - Debug - Motor %d report start :",
                         static_cast<int>(state->getId()));

                int scan_res = motorScanReport(state->getId());
                int cmd_res = motorCmdReport(state->getId(), state->getType());

                if (niryo_robot_msgs::CommandStatus::SUCCESS == scan_res
                        && niryo_robot_msgs::CommandStatus::SUCCESS == cmd_res)
                {
                    nbSuccess++;
                }
                else
                {
                    nbFailure++;
                    response = niryo_robot_msgs::CommandStatus::FAILURE;
                }

                results.emplace_back(state->getId(), state->getType(), scan_res, cmd_res);

                if (niryo_robot_msgs::CommandStatus::ABORTED == scan_res ||
                        niryo_robot_msgs::CommandStatus::ABORTED == cmd_res)
                {
                    ROS_INFO("TtlInterfaceCore::launchMotorsReport - Debug - Debug motor aborted");
                    break;
                }
            }
        }

        ros::Duration(1.0).sleep();
        ROS_INFO("TtlInterfaceCore::launchMotorsReport - Debug - Check for unflashed motors");

        // check if some id 1 are found (id = 1 is for unflashed dxl motors)
        if (_ttl_manager->ping(1))
        {
            ROS_ERROR("TtlInterfaceCore::launchMotorsReport - Debug - Find an unflashed motor");
            results.emplace_back(1, EHardwareType::UNKNOWN, niryo_robot_msgs::CommandStatus::SUCCESS,
                                 niryo_robot_msgs::CommandStatus::FAILURE);
            response = niryo_robot_msgs::CommandStatus::FAILURE;
            nbFailure++;
        }

        // display synthesis
        ROS_INFO("Synthesis of motors report :");
        ROS_INFO("--------");
        ROS_INFO("Nb Success : %d, Nb Failure: %d", nbSuccess, nbFailure);
        ROS_INFO("Details: ");
        ROS_INFO("--------");
        ROS_INFO("id \t| type \t| scan result | cmd result");

        for (auto const& res : results)
        {
            ROS_INFO("%d \t| %s \t| %d | %d", std::get<0>(res),
                                              HardwareTypeEnum(std::get<1>(res)).toString().c_str(),
                                              std::get<2>(res),
                                              std::get<3>(res));
        }
    }
    else
    {
        ROS_ERROR("TtlInterfaceCore::launchMotorsReport - Debug - Debug mode not enabled");
    }

    return response;
}

// ****************
//  Control Loop
// ****************

/**
 * @brief TtlInterfaceCore::startControlLoop
 */
void TtlInterfaceCore::startControlLoop()
{
    resetHardwareControlLoopRates();
    if (!_control_loop_flag)
    {
        ROS_INFO("TtlInterfaceCore::startControlLoop - Start control loop");
        _control_loop_flag = true;
        _control_loop_thread = std::thread(&TtlInterfaceCore::controlLoop, this);
    }
}

/**
 * @brief TtlInterfaceCore::scanMotorId
*/
bool TtlInterfaceCore::scanMotorId(uint8_t motor_to_find)
{
    ROS_INFO("TtlInterfaceCore::scanMotorId need to be implemented used in calibration manager");
    return true;
}

/**
 * @brief TtlInterfaceCore::startCalibration
 */
void TtlInterfaceCore::startCalibration()
{
    if (_ttl_manager)
        _ttl_manager->startCalibration();
}

/**
 * @brief TtlInterfaceCore::resetCalibration
 */
void TtlInterfaceCore::resetCalibration()
{
    if (_ttl_manager)
        _ttl_manager->resetCalibration();
}

/**
 * @brief TtlInterfaceCore::isCalibrationInProgress
 * @return
 */
bool TtlInterfaceCore::isCalibrationInProgress() const
{
    return _ttl_manager->isCalibrationInProgress();
}

/**
 * @brief TtlInterfaceCore::getCalibrationResult
 * @param id
 * @return
 */
int32_t TtlInterfaceCore::getCalibrationResult(uint8_t id) const
{
    return _ttl_manager->getCalibrationResult(id);
}

/**
 * @brief TtlInterfaceCore::getCalibrationStatus
 * @return
 */
inline
common::model::EStepperCalibrationStatus
TtlInterfaceCore::getCalibrationStatus() const
{
    return _ttl_manager->getCalibrationStatus();
}

/**
 * @brief TtlInterfaceCore::resetHardwareControlLoopRates
 */
void TtlInterfaceCore::resetHardwareControlLoopRates()
{
    ROS_DEBUG("TtlInterfaceCore::resetHardwareControlLoopRates - Reset control loop rates");
    double now = ros::Time::now().toSec();
    _time_hw_data_last_write = now;
    _time_hw_data_last_read = now;
    _time_hw_status_last_read = now;
    _time_check_connection_last_read = now;
    _time_check_end_effector_last_read = now;
}

/**
 * @brief TtlInterfaceCore::activeDebugMode
 * @param mode
 */
void TtlInterfaceCore::activeDebugMode(bool mode)
{
    ROS_INFO("TtlInterfaceCore::activeDebugMode - Activate debug mode for driver core: %d", mode);
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
 * @brief TtlInterfaceCore::controlLoop
 */
void TtlInterfaceCore::controlLoop()
{
    ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);
    resetHardwareControlLoopRates();
    while (ros::ok())
    {
        if (!_debug_flag)
        {
            if (!_ttl_manager->isConnectionOk())
            {
                ROS_WARN("TtlInterfaceCore::controlLoop - motor connection error");
                ros::Duration(0.1).sleep();

                vector<uint8_t> missing_ids;
                ROS_DEBUG("TtlInterfaceCore::controlLoop - Scan to find motors");

                int bus_state = COMM_PORT_BUSY;
                while (TTL_SCAN_OK != bus_state)
                {
                    {
                        lock_guard<mutex> lck(_control_loop_mutex);
                        bus_state = _ttl_manager->scanAndCheck();
                    }
                    missing_ids = getRemovedMotorList();
                    for (auto const& id : missing_ids)
                    {
                        ROS_WARN("TtlInterfaceCore::controlLoop - motor %d do not seem to be connected", id);
                    }
                    ros::Duration(0.25).sleep();
                }
                ROS_INFO("TtlInterfaceCore::controlLoop - Bus is ok");
            }

            if (_control_loop_flag)
            {
                bool needSleep = false;
                lock_guard<mutex> lck(_control_loop_mutex);
                if (ros::Time::now().toSec() - _time_hw_data_last_read >= _delta_time_data_read)
                {
                    _time_hw_data_last_read = ros::Time::now().toSec();
                    _ttl_manager->readPositionStatus();
                    needSleep = true;
                }
                if (ros::Time::now().toSec() - _time_hw_data_last_read >= _delta_time_data_read)
                {
                    _time_hw_data_last_read = ros::Time::now().toSec();
                    _ttl_manager->readPositionStatus();
                    needSleep = true;
                }
                if (!needSleep && ros::Time::now().toSec() - _time_hw_status_last_read >= _delta_time_status_read)
                {
                    _time_hw_status_last_read = ros::Time::now().toSec();
                    _ttl_manager->readHwStatus();
                    needSleep = true;
                }
                if (!needSleep && ros::Time::now().toSec() - _time_hw_data_last_write >= _delta_time_write)
                {
                    _time_hw_data_last_write = ros::Time::now().toSec();
                    _executeCommand();
                }
                bool isFreqMet = control_loop_rate.sleep();
                ROS_DEBUG_COND(!isFreqMet,
                               "TtlInterfaceCore::ControlLoop : freq not met : expected (%f s) vs actual (%f s)",
                               control_loop_rate.expectedCycleTime().toSec(),
                               control_loop_rate.cycleTime().toSec());
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
 * @brief TtlInterfaceCore::_executeCommand : execute all the cmd in the current queue
 */  // create a unique queue using polymorphism
void TtlInterfaceCore::_executeCommand()
{
    bool _need_sleep = false;
    if (!_joint_trajectory_cmd.empty())
    {
        _ttl_manager->executeJointTrajectoryCmd(_joint_trajectory_cmd);
        _joint_trajectory_cmd.clear();
        _need_sleep = true;
    }
    if (!_single_cmds_queue.empty())
    {
        _ttl_manager->writeSingleCommand(_single_cmds_queue.front());
        _single_cmds_queue.pop();
        _need_sleep = true;
    }
    if (!_conveyor_cmds_queue.empty())
    {
        // as we use a queue, we don't need a mutex
        if (_need_sleep)
            ros::Duration(0.01).sleep();
        _ttl_manager->writeSingleCommand(_conveyor_cmds_queue.front());
        _conveyor_cmds_queue.pop();
    }
    if (_sync_cmds && _sync_cmds->isValid())
    {
        // as we use a queue, we don't need a mutex
        if (_need_sleep)
            ros::Duration(0.01).sleep();
        _ttl_manager->writeSynchronizeCommand(_sync_cmds);
        _sync_cmds->reset();
    }
}

// *************
//  Setters
// *************

/**
 * @brief TtlInterfaceCore::setTool
 * @param type
 * @param motor_id
 * @return
 */
int TtlInterfaceCore::setTool(EHardwareType type, uint8_t motor_id)
{
    int result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;

    lock_guard<mutex> lck(_control_loop_mutex);

    // try to find motor
    if (_ttl_manager->ping(motor_id))
    {
        // add dynamixel as a new tool
        _ttl_manager->addHardwareComponent(type, motor_id, TtlManager::EType::TOOL);
        // Enable torque
        std::shared_ptr<AbstractTtlSingleMotorCmd> cmd_torque = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_TORQUE,
                                                            motor_id, std::initializer_list<uint32_t>{1});
        _ttl_manager->writeSingleCommand(cmd_torque);
        ros::Duration(0.01).sleep();

        result = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    else
    {
        ROS_WARN("TtlInterfaceCore::setTool - No end effector found with motor id %d", motor_id);
    }

    return result;
}

/**
 * @brief TtlInterfaceCore::unsetTool
 * @param motor_id
 */
void TtlInterfaceCore::unsetTool(uint8_t motor_id)
{
    lock_guard<mutex> lck(_control_loop_mutex);

    ROS_DEBUG("TtlInterfaceCore::unsetTool - UnsetTool: id %d", motor_id);
    _ttl_manager->removeMotor(motor_id);
}

/**
 * @brief TtlInterfaceCore::setEndEffector
 * @param end_effector_id
 * @return
 */
int TtlInterfaceCore::setEndEffector(uint8_t end_effector_id)
{
  int result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;

  lock_guard<mutex> lck(_control_loop_mutex);

  // try to find hw
  if (_ttl_manager->ping(end_effector_id))
  {
      // add end effector
      _ttl_manager->addHardwareComponent(EHardwareType::END_EFFECTOR, end_effector_id, TtlManager::EType::END_EFFECTOR);

      result = niryo_robot_msgs::CommandStatus::SUCCESS;
  }
  else
  {
      ROS_WARN("TtlInterfaceCore::setTool - No end effector found with id %d", end_effector_id);
  }

  return result;
}

/**
 * @brief TtlInterfaceCore::setConveyor
 * @param new_motor_id
 * @param default_conveyor_id
 * @return
 */
int TtlInterfaceCore::setConveyor(uint8_t new_motor_id, uint8_t default_conveyor_id)
{
    int result = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;

    lock_guard<mutex> lck(_control_loop_mutex);

    // try to find motor id 6 (default motor id for conveyor
    if (_ttl_manager->ping(default_conveyor_id))
    {
        if (COMM_SUCCESS == _ttl_manager->changeId(EHardwareType::STEPPER, default_conveyor_id, new_motor_id))
        {
            // add stepper as a new conveyor
            _ttl_manager->addHardwareComponent(EHardwareType::STEPPER, new_motor_id, ttl_driver::TtlManager::EType::CONVOYER);
            result = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("TtlInterfaceCore::setConveyor : unable to change conveyor ID");
            result = niryo_robot_msgs::CommandStatus::TTL_WRITE_ERROR;
        }
    }
    else
    {
        ROS_WARN("TtlInterfaceCore::setConveyor - No conveyor found");
    }

    return result;
}

/**
 * @brief TtlInterfaceCore::unsetConveyor
 * @param motor_id
 */
void TtlInterfaceCore::unsetConveyor(uint8_t motor_id)
{
    lock_guard<mutex> lck(_control_loop_mutex);

    ROS_DEBUG("TtlInterfaceCore::unsetConveyor - unsetConveyor: id %d", motor_id);

    if (COMM_SUCCESS == _ttl_manager->changeId(EHardwareType::STEPPER, motor_id, 6))
        _ttl_manager->removeMotor(motor_id);
    else
        ROS_ERROR("TtlInterfaceCore::unsetConveyor : unable to change conveyor ID");
}

/**
 * @brief TtlInterfaceCore::clearSingleCommandQueue
 */
void TtlInterfaceCore::clearSingleCommandQueue()
{
    while (!_single_cmds_queue.empty())
        _single_cmds_queue.pop();
}

/**
 * @brief TtlInterfaceCore::clearConveyorCommandQueue
 */
void TtlInterfaceCore::clearConveyorCommandQueue()
{
    while (!_conveyor_cmds_queue.empty())
        _conveyor_cmds_queue.pop();
}

/**
 * @brief TtlInterfaceCore::setTrajectoryControllerCommands
 * @param cmd
 */
void TtlInterfaceCore::setTrajectoryControllerCommands(const std::vector<std::pair<uint8_t, uint32_t> > &cmd)
{
    _joint_trajectory_cmd = cmd;
}

/**
 * @brief TtlInterfaceCore::setSyncCommand
 * @param cmd
 */
void TtlInterfaceCore::setSyncCommand(const std::shared_ptr<common::model::ISynchronizeMotorCmd>& cmd)
{
    if (cmd->isValid())
    {
        if (cmd->isStepperCmd())
            _sync_cmds = std::dynamic_pointer_cast<common::model::StepperTtlSyncCmd>(cmd);
        else if (cmd->isDxlCmd())
            _sync_cmds = std::dynamic_pointer_cast<common::model::DxlSyncCmd>(cmd);
    }
    else
        ROS_WARN("TtlInterfaceCore::setSyncCommand : Invalid command %s", cmd->str().c_str());
}

/**
 * @brief TtlInterfaceCore::addSingleCommandToQueue
 * @param cmd
 *
 */
void TtlInterfaceCore::addSingleCommandToQueue(const std::shared_ptr<common::model::ISingleMotorCmd>& cmd)
{
    ROS_DEBUG("TtlInterfaceCore::addSingleCommandToQueue - %s", cmd->str().c_str());

    if (cmd->isValid())
    {
        if (_single_cmds_queue.size() > QUEUE_OVERFLOW)
            ROS_WARN("TtlInterfaceCore::addSingleCommandToQueue: dxl cmd queue overflow ! %lu", _single_cmds_queue.size());

        if (cmd->isDxlCmd())
            _single_cmds_queue.push(std::dynamic_pointer_cast<common::model::DxlSingleCmd>(cmd));
        else if (cmd->isStepperCmd())
        {
            if (cmd->getCmdType() == static_cast<int>(EStepperCommandType::CMD_TYPE_CONVEYOR))
            {
                if (_conveyor_cmds_queue.size() > QUEUE_OVERFLOW)
                    ROS_WARN("TtlInterfaceCore::addCommandToQueue: Cmd queue overflow ! %lu", _conveyor_cmds_queue.size());
                else
                    _conveyor_cmds_queue.push(std::dynamic_pointer_cast<common::model::StepperTtlSingleCmd>(cmd));
            }
            else
                _single_cmds_queue.push(std::dynamic_pointer_cast<common::model::StepperTtlSingleCmd>(cmd));
        }
    }
    else
    {
        ROS_WARN("TtlInterfaceCore::addSingleCommandToQueue : Invalid command %s", cmd->str().c_str());
    }
}

/**
 * @brief TtlInterfaceCore::addSingleCommandToQueue
 * @param cmd
 */
void TtlInterfaceCore::addSingleCommandToQueue(const std::vector<std::shared_ptr<common::model::ISingleMotorCmd> >& cmd)
{
    for (auto const& c : cmd)
        addSingleCommandToQueue(c);
}

/**
 * @brief TtlInterfaceCore::setMotorPID
 * @param motorState
 * @return
 */
bool TtlInterfaceCore::setMotorPID(const std::shared_ptr<JointState> &motorState)
{
    if (motorState->isDynamixel())
    {
        std::shared_ptr<DxlMotorState> dxlState = std::dynamic_pointer_cast<DxlMotorState>(motorState);
        uint8_t motor_id = motorState->getId();

        ROS_DEBUG("TtlInterfaceCore::setMotorPID - Setting PID for motor id: %d", static_cast<int>(motor_id));

        // ** DXL PID configuration **

        // Position Gain
        if (dxlState->getPositionPGain() > 0)
        {
            auto dxl_cmd_pos_p = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION_P_GAIN,
                                                                motor_id, std::initializer_list<uint32_t>{dxlState->getPositionPGain()});

            if (dxl_cmd_pos_p->isValid())
                addSingleCommandToQueue(dxl_cmd_pos_p);
        }
        if (dxlState->getPositionIGain() > 0)
        {
            auto dxl_cmd_pos_i = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION_I_GAIN,
                                                                motor_id, std::initializer_list<uint32_t>{dxlState->getPositionIGain()});

            if (dxl_cmd_pos_i->isValid())
                addSingleCommandToQueue(dxl_cmd_pos_i);
        }

        if (dxlState->getPositionDGain() > 0)
        {
            auto dxl_cmd_pos_d = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_POSITION_D_GAIN,
                                                                motor_id, std::initializer_list<uint32_t>{dxlState->getPositionDGain()});

            if (dxl_cmd_pos_d->isValid())
                addSingleCommandToQueue(dxl_cmd_pos_d);
        }

        // Velocity Gain
        if (dxlState->getVelocityPGain() > 0)
        {
            auto dxl_cmd_vel_p = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_VELOCITY_P_GAIN,
                                                                motor_id, std::initializer_list<uint32_t>{dxlState->getVelocityPGain()});

            if (dxl_cmd_vel_p->isValid())
                addSingleCommandToQueue(dxl_cmd_vel_p);
        }

        if (dxlState->getVelocityIGain() > 0)
        {
            auto dxl_cmd_vel_i = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_VELOCITY_I_GAIN,
                                                                motor_id, std::initializer_list<uint32_t>{dxlState->getVelocityIGain()});

            if (dxl_cmd_vel_i->isValid())
                addSingleCommandToQueue(dxl_cmd_vel_i);
        }
        if (dxlState->getFF1Gain() > 0)
        {
            auto dxl_cmd_ff1 = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_FF1_GAIN,
                                                              motor_id, std::initializer_list<uint32_t>{dxlState->getFF1Gain()});

            if (dxl_cmd_ff1->isValid())
                addSingleCommandToQueue(dxl_cmd_ff1);
        }

        if (dxlState->getFF2Gain() > 0)
        {
            auto dxl_cmd_ff2 = std::make_shared<DxlSingleCmd>(EDxlCommandType::CMD_TYPE_FF2_GAIN,
                                                              motor_id, std::initializer_list<uint32_t>{dxlState->getFF2Gain()});

            if (dxl_cmd_ff2->isValid())
                addSingleCommandToQueue(dxl_cmd_ff2);
        }
    }
    else if (motorState->isStepper())
    {
        ROS_INFO("A stepper motor does not have PID");
    }

    return true;
}

// ***************
//  Getters
// ***************

/**
 * @brief TtlInterfaceCore::getJointStates
 * @return
 */
std::vector<std::shared_ptr<common::model::JointState> >
TtlInterfaceCore::getJointStates() const
{
    return _ttl_manager->getMotorsStates();
}

/**
 * @brief TtlInterfaceCore::getState
 * @param motor_id
 * @return
 */
common::model::JointState
TtlInterfaceCore::getJointState(uint8_t motor_id) const
{
    // Check type of motor type
    return _ttl_manager->getHardwareState<JointState>(motor_id);
}

/**
 * @brief TtlInterfaceCore::getPosition
 * @param id
 * @return
 */
double TtlInterfaceCore::getPosition(uint8_t id) const
{
    JointState motor_state = getJointState(id);
    return static_cast<double>(motor_state.getPositionState());
}

/**
 * @brief TtlInterfaceCore::getHwStatus
 * @return
 */
ttl_driver::ArrayMotorHardwareStatus TtlInterfaceCore::getHwStatus() const
{
    ttl_driver::MotorHardwareStatus data;
    ttl_driver::ArrayMotorHardwareStatus hw_state;

    for (auto const& State : _ttl_manager->getMotorsStates())
    {
        if (State)
        {
            data.motor_identity.motor_id = State->getId();
            data.motor_identity.motor_type = static_cast<uint8_t>(State->getType());
            data.temperature = static_cast<uint32_t>(State->getTemperatureState());
            if (State->isDynamixel())
                data.voltage = static_cast<double>(State->getVoltageState()) / TTL_VOLTAGE_DIVISOR;
            else if (State->isStepper())
            {
                 // TODO(Thuc): Get correctly voltage of stepper
                data.voltage = static_cast<double>(State->getVoltageState());
            }
            data.error = static_cast<uint32_t>(State->getHardwareErrorState());
            data.error_msg = State->getHardwareErrorMessageState();
            hw_state.motors_hw_status.push_back(data);
        }
    }
    return hw_state;
}

/**
 * @brief TtlInterfaceCore::getBusState
 * @return
 */
niryo_robot_msgs::BusState TtlInterfaceCore::getBusState() const
{
    niryo_robot_msgs::BusState bus_state;

    string error;
    bool connection;
    vector<uint8_t> motor_id;

    _ttl_manager->getBusState(connection, motor_id, error);
    bus_state.connection_status = connection;
    bus_state.motor_id_connected = motor_id;
    bus_state.error = error;
    return bus_state;
}

// *******************
//    Callbacks     *
// *******************

/**
 * @brief TtlInterfaceCore::callbackSendCustomValue
 * @param req
 * @param res
 * @return
 */
bool TtlInterfaceCore::_callbackSendCustomValue(ttl_driver::SendCustomValue::Request &req,
                                               ttl_driver::SendCustomValue::Response &res)
{
    int result;

    EHardwareType motor_type;

    if (1 <= req.motor_type  && 5 >= req.motor_type)
        motor_type = static_cast<EHardwareType>(req.motor_type);
    else
    {
        res.status = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        res.message = "TtlInterfaceCore - Invalid motor type: should be 1 (Stepper) 2 (XL-430) or 3 (XL-320) or 4 (XL-330) or 5 (XC-430)";
        return true;
    }

    lock_guard<mutex> lck(_control_loop_mutex);
    result = _ttl_manager->sendCustomCommand(motor_type,
                                              req.id,
                                              req.reg_address,
                                              req.value,
                                              req.byte_number);

    if (result != COMM_SUCCESS)
    {
        res.message = "TtlInterfaceCore - Send custom command failed";
    }
    else
    {
        res.message = "TtlInterfaceCore - Send custom command done";
        result = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    res.status = result;
    return true;
}

/**
 * @brief TtlInterfaceCore::callbackReadCustomValue
 * @param req
 * @param res
 * @return
 */
bool TtlInterfaceCore::_callbackReadCustomValue(ttl_driver::ReadCustomValue::Request &req,
                                                ttl_driver::ReadCustomValue::Response &res)
{
    int result;
    EHardwareType motor_type;
    if (1 <= req.motor_type  && 5 >= req.motor_type)
        motor_type = static_cast<EHardwareType>(req.motor_type);
    else
    {
        res.status = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        res.message = "TtlInterfaceCore - Invalid motor type: should be 1 (Stepper) 2 (XL-430) or 3 (XL-320) or 4 (XL-330) or 5 (XC-430)";
        return true;
    }

    lock_guard<mutex> lck(_control_loop_mutex);
    int value = 0;
    result = _ttl_manager->readCustomCommand(motor_type,
                                              req.id,
                                              req.reg_address,
                                              value,
                                              req.byte_number);

    if (result != COMM_SUCCESS)
    {
        res.message = "TtlInterfaceCore - Reading custom registry failed";
    }
    else
    {
        res.message = "TtlInterfaceCore - Reading successful. Registry value : " + to_string(value);
        result = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    res.status = result;
    return true;
}

/**
 * @brief TtlInterfaceCore::callbackActivateLeds
 * @param req
 * @param res
 * @return
 */
bool TtlInterfaceCore::_callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req,
                                          niryo_robot_msgs::SetInt::Response &res)
{
    int led = req.value;
    string message = "";

    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_manager->setLeds(led);

    res.status = result;
    res.message = message;
    return true;
}

}  // namespace ttl_driver
