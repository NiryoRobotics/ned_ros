/*
    ttl_driver_core.cpp
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

#include "ttl_driver/ttl_driver_core.hpp"

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

using ::common::model::EMotorType;
using ::common::model::MotorTypeEnum;
using ::common::model::DxlMotorState;
using ::common::model::EDxlCommandType;
using ::common::model::SingleMotorCmd;
using ::common::model::SynchronizeMotorCmd;

namespace ttl_driver
{
/**
 * @brief TtlDriverCore::TtlDriverCore
 */
TtlDriverCore::TtlDriverCore(ros::NodeHandle& nh)
{
    ROS_DEBUG("TtlDriverCore - ctor");

    init(nh);
}

/**
 * @brief TtlDriverCore::~TtlDriverCore
 */
TtlDriverCore::~TtlDriverCore()
{
    if (_control_loop_thread.joinable())
        _control_loop_thread.join();
}

/**
 * @brief TtlDriverCore::init
 */
bool TtlDriverCore::init(ros::NodeHandle& nh)
{
    ROS_DEBUG("TtlDriverCore::init - Init parameters...");
    initParameters(nh);

    _ttl_driver = std::make_unique<TtlDriver>(nh);
    _ttl_driver->scanAndCheck();
    startControlLoop();

    ROS_DEBUG("TtlDriverCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("TtlDriverCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("TtlDriverCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief TtlDriverCore::initParameters
 */
void TtlDriverCore::initParameters(ros::NodeHandle& nh)
{
    _control_loop_frequency = 0.0;
    double write_frequency = 0.0;
    double read_data_frequency = 0.0;
    double read_status_frequency = 0.0;

    nh.getParam("dxl_hardware_control_loop_frequency",
                 _control_loop_frequency);

    nh.getParam("dxl_hardware_write_frequency",
                 write_frequency);

    nh.getParam("dxl_hardware_read_data_frequency",
                 read_data_frequency);

    nh.getParam("dxl_hardware_read_status_frequency",
                 read_status_frequency);

    ROS_DEBUG("TtlDriverCore::initParameters - dxl_hardware_control_loop_frequency : %f", _control_loop_frequency);
    ROS_DEBUG("TtlDriverCore::initParameters - dxl_hardware_write_frequency : %f", write_frequency);
    ROS_DEBUG("TtlDriverCore::initParameters - dxl_hardware_read_data_frequency : %f", read_data_frequency);
    ROS_DEBUG("TtlDriverCore::initParameters - dxl_hardware_read_status_frequency : %f", read_status_frequency);

    _delta_time_data_read = 1.0 / read_data_frequency;
    _delta_time_status_read = 1.0 / read_status_frequency;
    _delta_time_write = 1.0 / write_frequency;
}

/**
 * @brief TtlDriverCore::startServices
 * @param nh
 */
void TtlDriverCore::startServices(ros::NodeHandle& nh)
{
    // advertise services
    _activate_leds_server = nh.advertiseService("/niryo_robot/ttl_driver/set_dxl_leds",
                                                 &TtlDriverCore::_callbackActivateLeds, this);

    _custom_cmd_server = nh.advertiseService("/niryo_robot/ttl_driver/send_custom_dxl_value",
                                              &TtlDriverCore::_callbackSendCustomDxlValue, this);

    _custom_cmd_getter = nh.advertiseService("/niryo_robot/ttl_driver/read_custom_dxl_value",
                                              &TtlDriverCore::_callbackReadCustomDxlValue, this);
}

/**
 * @brief TtlDriverCore::startPublishers
 * @param nh
 */
void TtlDriverCore::startPublishers(ros::NodeHandle &/*nh*/)
{
    ROS_DEBUG("TTLDriverCore::startSubscribers - no publishers to start");
}

/**
 * @brief TtlDriverCore::startSubscribers
 * @param nh
 */
void TtlDriverCore::startSubscribers(ros::NodeHandle &/*nh*/)
{
    ROS_DEBUG("TTLDriverCore::startSubscribers - no subscribers to start");
}

// ***************
//  Commands
// ***************

/**
 * @brief TtlDriverCore::rebootMotors
 * @return
 */
int TtlDriverCore::rebootMotors()
{
    ROS_INFO("TtlDriverCore::rebootMotors - Reboot motors");
    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_driver->rebootMotors();
    ros::Duration(1.5).sleep();
    return result;
}


/**
 * @brief TtlDriverCore::rebootMotors
 * @return
 */
bool TtlDriverCore::rebootMotor(uint8_t motor_id)
{
    ROS_INFO("TtlDriverCore::rebootMotor - Reboot motor %d", static_cast<int>(motor_id));
    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_driver->rebootMotor(motor_id);
    ros::Duration(1.5).sleep();
    // return truc if result is COMM_SUCCESS
    return (COMM_SUCCESS == result);
}

/**
 * @brief TtlDriverCore::scanTools
 * @return
 */
vector<uint8_t> TtlDriverCore::scanTools()
{
    vector<uint8_t> motor_list;
    lock_guard<mutex> lck(_control_loop_mutex);
    ROS_INFO("TtlDriverCore::scanTools - Scan tools...");
    int result = COMM_PORT_BUSY;

    for (int counter = 0; counter < 50 && COMM_SUCCESS != result; ++counter)
    {
        result = _ttl_driver->getAllIdsOnBus(motor_list);
        ROS_DEBUG_COND(COMM_SUCCESS != result, "DxlDriver::scanTools status: %d (counter: %d)", result, counter);
        ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
    }
    ROS_DEBUG("TtlDriverCore::scanTools - Result getAllIdsOnDxlBus: %d", result);

    ostringstream ss;
    for (auto const& m : motor_list)
        ss << static_cast<int>(m) << " ";
    string motor_id_list_string = ss.str();
    if (!motor_id_list_string.empty())
        motor_id_list_string.pop_back();  // remove trailing " "

    ROS_DEBUG("TtlDriverCore::scanTools - All id on dxl bus: [%s]", motor_id_list_string.c_str());
    return motor_list;
}

/**
 * @brief TtlDriverCore::update_leds
 * @return
 */
int TtlDriverCore::update_leds(void)
{
    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_driver->setLeds(_ttl_driver->getLedState(), EMotorType::XL320);
    return result;
}

/**
 * @brief TtlDriverCore::motorScanReport
 * @param motor_id
 * @return
 */
int TtlDriverCore::motorScanReport(uint8_t motor_id)
{
    if (_debug_flag)
    {
        ros::Duration(1.0).sleep();
        if (_ttl_driver->ping(motor_id))
        {
            ROS_INFO("TtlDriverCore::motorScanReport - Debug - Dynamixel Motor %d found", motor_id);
        }
        else
        {
            ROS_ERROR("TtlDriverCore::motorScanReport - Debug - Dynamixel Motor %d not found", motor_id);
            return niryo_robot_msgs::CommandStatus::FAILURE;
        }
        return niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    ROS_ERROR("TtlDriverCore::motorScanReport - Debug mode not enabled");
    return niryo_robot_msgs::CommandStatus::ABORTED;
}

/**
 * @brief TtlDriverCore::motorCmdReport
 * @param motor_id
 * @param motor_type
 * @return
 */
int TtlDriverCore::motorCmdReport(uint8_t motor_id, EMotorType motor_type)
{
    int ret = niryo_robot_msgs::CommandStatus::ABORTED;

    if (_debug_flag)
    {
        DxlMotorState dynamixel_motor = DxlMotorState(motor_type, motor_id);

        // torque on
        ros::Duration(0.5).sleep();
        ROS_INFO("TtlDriverCore::motorCmdReport - Debug - Send torque on command on dxl %d", motor_id);
        _ttl_driver->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_TORQUE, motor_id, 1));
        ros::Duration(0.5).sleep();

        // set position to old position + 200
        uint32_t old_position = _ttl_driver->getPosition(dynamixel_motor);
        ROS_INFO("TtlDriverCore::motorCmdReport - Debug - get dxl %d pose: %d ", motor_id, old_position);
        ros::Duration(0.5).sleep();
        ROS_INFO("TtlDriverCore::motorCmdReport - Debug - Send dxl %d pose: %d ", motor_id, old_position + 200);
        _ttl_driver->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION,
                                                      motor_id, old_position + 200));

        // set position back to old position
        ros::Duration(2).sleep();
        uint32_t new_position = _ttl_driver->getPosition(dynamixel_motor);
        ROS_INFO("TtlDriverCore::motorCmdReport - Debug - get dxl %d pose: %d ", motor_id, new_position);
        int rest = static_cast<int>(new_position - old_position);

        ROS_INFO("TtlDriverCore - Debug - Send dxl %d pose: %d ", motor_id, old_position);
        _ttl_driver->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_POSITION, motor_id, old_position));

        ros::Duration(2).sleep();
        uint32_t new_position2 = _ttl_driver->getPosition(dynamixel_motor);
        ROS_INFO("TtlDriverCore::motorCmdReport - Debug - get dxl %d pose: %d ", motor_id, new_position2);
        int rest2 = static_cast<int>(new_position2 - new_position);

        // torque off
        ROS_INFO("TtlDriverCore::motorCmdReport - Debug - Send torque off command on dxl %d", motor_id);
        _ttl_driver->readSingleCommand(SingleMotorCmd(EDxlCommandType::CMD_TYPE_TORQUE, motor_id, 0));

        if (abs(rest) < 50 || abs(rest2) < 50)
        {
            ROS_WARN("TtlDriverCore::motorCmdReport - Debug - Dynamixel Motor %d problem", motor_id);
            ret = niryo_robot_msgs::CommandStatus::FAILURE;
        }
        else
        {
            ROS_INFO("TtlDriverCore::motorCmdReport - Debug - Dynamixel Motor %d OK", motor_id);
            ret = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
    }
    else
    {
        ROS_ERROR("TtlDriverCore::motorCmdReport - Debug - Debug mode not enabled");
    }

    return ret;
}

/**
 * @brief TtlDriverCore::launchMotorsReport
 * @return
 */
int TtlDriverCore::launchMotorsReport()
{
    int response = niryo_robot_msgs::CommandStatus::SUCCESS;
    unsigned int nbSuccess = 0;
    unsigned int nbFailure = 0;

    std::vector<std::tuple<uint8_t, EMotorType, int, int> > results;

    if (_debug_flag)
    {
        ROS_INFO("TtlDriverCore::launchMotorsReport - Debug - Start Dynamixel Motor Report");
        ros::Duration(1.0).sleep();

        for (auto const& state : _ttl_driver->getMotorsStates())
        {
            if (state)
            {
                ROS_INFO("TtlDriverCore::launchMotorsReport - Debug - Motor %d report start :",
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
                    ROS_INFO("TtlDriverCore::launchMotorsReport - Debug - Debug motor aborted");
                    break;
                }
            }
        }

        ros::Duration(1.0).sleep();
        ROS_INFO("TtlDriverCore::launchMotorsReport - Debug - Check for unflashed dynamixel motors");

        // check if some id 1 are found (id = 1 is for unflashed dxl motors)
        if (_ttl_driver->ping(1))
        {
            ROS_ERROR("TtlDriverCore::launchMotorsReport - Debug - Find an unflashed dynamixel motor");
            results.emplace_back(1, EMotorType::UNKNOWN, niryo_robot_msgs::CommandStatus::SUCCESS,
                                 niryo_robot_msgs::CommandStatus::FAILURE);
            response = niryo_robot_msgs::CommandStatus::FAILURE;
            nbFailure++;
        }

        // display synthesis
        ROS_INFO("Synthesis of Dynamixel motors report :");
        ROS_INFO("--------");
        ROS_INFO("Nb Success : %d, Nb Failure: %d", nbSuccess, nbFailure);
        ROS_INFO("Details: ");
        ROS_INFO("--------");
        ROS_INFO("id \t| type \t| scan result | cmd result");

        for (auto const& res : results)
        {
            ROS_INFO("%d \t| %s \t| %d | %d", std::get<0>(res),
                                              MotorTypeEnum(std::get<1>(res)).toString().c_str(),
                                              std::get<2>(res),
                                              std::get<3>(res));
        }
    }
    else
    {
        ROS_ERROR("TtlDriverCore::launchMotorsReport - Debug - Debug mode not enabled");
    }

    return response;
}

// ****************
//  Control Loop
// ****************

/**
 * @brief TtlDriverCore::startControlLoop
 */
void TtlDriverCore::startControlLoop()
{
    resetHardwareControlLoopRates();
    if (!_control_loop_flag)
    {
        ROS_INFO("TtlDriverCore::startControlLoop - Start control loop");
        _control_loop_flag = true;
        _control_loop_thread = std::thread(&TtlDriverCore::controlLoop, this);
    }
}

/**
 * @brief TtlDriverCore::resetHardwareControlLoopRates
 */
void TtlDriverCore::resetHardwareControlLoopRates()
{
    ROS_DEBUG("TtlDriverCore::resetHardwareControlLoopRates - Reset control loop rates");
    double now = ros::Time::now().toSec();
    _time_hw_data_last_write = now;
    _time_hw_data_last_read = now;
    _time_hw_status_last_read = now;
    _time_check_connection_last_read = now;
    _time_check_end_effector_last_read = now;
}

/**
 * @brief TtlDriverCore::activeDebugMode
 * @param mode
 */
void TtlDriverCore::activeDebugMode(bool mode)
{
    ROS_INFO("TtlDriverCore::activeDebugMode - Activate debug mode for dynamixel driver core: %d", mode);
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
 * @brief TtlDriverCore::controlLoop
 */
void TtlDriverCore::controlLoop()
{
    ros::Rate control_loop_rate = ros::Rate(_control_loop_frequency);
    resetHardwareControlLoopRates();
    while (ros::ok())
    {
        if (!_debug_flag)
        {
            if (!_ttl_driver->isConnectionOk())
            {
                ROS_WARN("TtlDriverCore::controlLoop - Dynamixel connection error");
                ros::Duration(0.1).sleep();

                vector<uint8_t> missing_ids;
                ROS_DEBUG("TtlDriverCore::controlLoop - Scan to find Dxl motors");

                int bus_state = COMM_PORT_BUSY;
                while (DXL_SCAN_OK != bus_state)
                {
                    {
                        lock_guard<mutex> lck(_control_loop_mutex);
                        bus_state = _ttl_driver->scanAndCheck();
                    }
                    missing_ids = getRemovedMotorList();
                    for (auto const& id : missing_ids)
                    {
                        ROS_WARN("TtlDriverCore::controlLoop - Dynamixel %d do not seem to be connected", id);
                    }
                    ros::Duration(0.25).sleep();
                }
                ROS_INFO("TtlDriverCore::controlLoop - Dxl Bus ok");
            }

            if (_control_loop_flag)
            {
                bool needSleep = false;
                lock_guard<mutex> lck(_control_loop_mutex);
                if (ros::Time::now().toSec() - _time_hw_data_last_read >= _delta_time_data_read)
                {
                    _time_hw_data_last_read = ros::Time::now().toSec();
                    _ttl_driver->readPositionStatus();
                    needSleep = true;
                }
                if (!needSleep && ros::Time::now().toSec() - _time_hw_status_last_read >= _delta_time_status_read)
                {
                    _time_hw_status_last_read = ros::Time::now().toSec();
                    _ttl_driver->readHwStatus();
                    needSleep = true;
                }
                if (!needSleep && ros::Time::now().toSec() - _time_hw_data_last_write >= _delta_time_write)
                {
                    _time_hw_data_last_write = ros::Time::now().toSec();
                    _executeCommand();
                }
                bool isFreqMet = control_loop_rate.sleep();
                ROS_DEBUG_COND(!isFreqMet,
                               "TtlDriverCore::ControlLoop : freq not met : expected (%f s) vs actual (%f s)",
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
 * @brief TtlDriverCore::_executeCommand : execute all the cmd in the current queue
 */  // create a unique queue using polymorphism
void TtlDriverCore::_executeCommand()
{
    bool need_sleep = false;

    if (!_joint_trajectory_cmd.empty())
    {
        _ttl_driver->executeJointTrajectoryCmd(_joint_trajectory_cmd);
        _joint_trajectory_cmd.clear();
        need_sleep = true;
    }

    if (!_dxl_single_cmds.empty())
    {
        _ttl_driver->readSingleCommand(_dxl_single_cmds.front());
        _dxl_single_cmds.pop();
        need_sleep = true;
    }
    if (!_end_effector_cmds.empty())
    {
        // as we use a queue, we don't need a mutex
        if (need_sleep)
            ros::Duration(0.01).sleep();
        _ttl_driver->readSingleCommand(_end_effector_cmds.front());
        _end_effector_cmds.pop();
        need_sleep = true;
    }
    if (_dxl_sync_cmds.isValid())
    {
        // as we use a queue, we don't need a mutex
        if (need_sleep)
            ros::Duration(0.01).sleep();
        _ttl_driver->readSynchronizeCommand(_dxl_sync_cmds);
        _dxl_sync_cmds.reset();
    }
}

// *************
//  Setters
// *************

/**
 * @brief TtlDriverCore::setEndEffector
 * @param type
 * @param motor_id
 * @return
 */
int TtlDriverCore::setEndEffector(EMotorType type, uint8_t motor_id)
{
    int result = niryo_robot_msgs::CommandStatus::DXL_READ_ERROR;

    lock_guard<mutex> lck(_control_loop_mutex);

    // try to find motor
    if (_ttl_driver->ping(motor_id))
    {
        // add dynamixel as a new tool
        _ttl_driver->addMotor(type, motor_id, true);
        result = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    else
    {
        ROS_WARN("TtlDriverCore::setEndEffector - No end effector found with motor id %d", motor_id);
    }

    return result;
}

/**
 * @brief TtlDriverCore::unsetEndEffector
 * @param motor_id
 */
void TtlDriverCore::unsetEndEffector(uint8_t motor_id)
{
    lock_guard<mutex> lck(_control_loop_mutex);

    ROS_DEBUG("TtlDriverCore::unsetEndEffector - UnsetEndEffector: id %d", motor_id);
    _ttl_driver->removeMotor(motor_id);
}


/**
 * @brief TtlDriverCore::clearSingleCommandQueue
 */
void TtlDriverCore::clearSingleCommandQueue()
{
    while (!_dxl_single_cmds.empty())
        _dxl_single_cmds.pop();
}

/**
 * @brief TtlDriverCore::clearEndEffectorCommandQueue
 */
void TtlDriverCore::clearEndEffectorCommandQueue()
{
    while (!_end_effector_cmds.empty())
        _end_effector_cmds.pop();
}

/**
 * @brief TtlDriverCore::setTrajectoryControllerCommands
 * @param cmd
 */
void TtlDriverCore::setTrajectoryControllerCommands(const std::vector<std::pair<uint8_t, uint32_t> > &cmd)
{
    _joint_trajectory_cmd = cmd;
}

/**
 * @brief TtlDriverCore::setSyncCommand
 * @param cmd
 */
void TtlDriverCore::setSyncCommand(const SynchronizeMotorCmd &cmd)
{
    if (cmd.isValid())
    {
        _dxl_sync_cmds = cmd;
    }
    else
    {
        ROS_WARN("TtlDriverCore::setSyncCommand : Invalid command %s", cmd.str().c_str());
    }
}

/**
 * @brief TtlDriverCore::addSingleCommandToQueue
 * @param cmd
 *
 *  Not very good, nothing prevents the user from providing an end effector command here
 * and vice versa with addEndEffectorCmd. To be changed
 */
void TtlDriverCore::addSingleCommandToQueue(const common::model::SingleMotorCmd &cmd)
{
    ROS_DEBUG("TtlDriverCore::addSingleCommandToQueue - %s", cmd.str().c_str());

    if (cmd.isValid())
    {
        if (_dxl_single_cmds.size() > QUEUE_OVERFLOW)
            ROS_WARN("TtlDriverCore::addSingleCommandToQueue: Cmd queue overflow ! %lu", _dxl_single_cmds.size());
        else
            _dxl_single_cmds.push(cmd);
    }
}

/**
 * @brief TtlDriverCore::addSingleCommandToQueue
 * @param cmd
 */
void TtlDriverCore::addSingleCommandToQueue(const std::vector<common::model::SingleMotorCmd> &cmd)
{
    for (auto const& c : cmd)
        addSingleCommandToQueue(c);
}

/**
 * @brief TtlDriverCore::addEndEffectorCommandToQueue
 * @param cmd
 */
void TtlDriverCore::addEndEffectorCommandToQueue(const common::model::SingleMotorCmd &cmd)
{
    if (_end_effector_cmds.size() > QUEUE_OVERFLOW)
        ROS_WARN_THROTTLE(0.5, "TtlDriverCore::addEndEffectorCommandToQueue: Cmd queue overflow ! %lu",
                          _end_effector_cmds.size());
    else
        _end_effector_cmds.push(cmd);
}

/**
 * @brief TtlDriverCore::addEndEffectorCommandToQueue
 * @param cmd
 */
void TtlDriverCore::addEndEffectorCommandToQueue(const std::vector<common::model::SingleMotorCmd> &cmd)
{
    for (auto const& c : cmd)
        addEndEffectorCommandToQueue(c);
}


/**
 * @brief TtlDriverCore::setMotorPID
 * @param dxlState
 * @return
 */
bool TtlDriverCore::setMotorPID(const std::shared_ptr<DxlMotorState> &dxlState)
{
    uint8_t motor_id = dxlState->getId();

    ROS_DEBUG("TtlDriverCore::setMotorPID - Setting PID for motor id: %d", static_cast<int>(motor_id));

    // ** DXL PID configuration **

    // Position Gain
    if (dxlState->getPositionPGain() > 0)
    {
        SingleMotorCmd dxl_cmd_pos_p(EDxlCommandType::CMD_TYPE_POSITION_P_GAIN, motor_id, dxlState->getPositionPGain());

        if (dxl_cmd_pos_p.isValid())
            addSingleCommandToQueue(dxl_cmd_pos_p);
    }

    if (dxlState->getPositionIGain() > 0)
    {
        SingleMotorCmd dxl_cmd_pos_i(EDxlCommandType::CMD_TYPE_POSITION_I_GAIN, motor_id, dxlState->getPositionIGain());

        if (dxl_cmd_pos_i.isValid())
            addSingleCommandToQueue(dxl_cmd_pos_i);
    }

    if (dxlState->getPositionDGain() > 0)
    {
        SingleMotorCmd dxl_cmd_pos_d(EDxlCommandType::CMD_TYPE_POSITION_D_GAIN, motor_id, dxlState->getPositionDGain());

        if (dxl_cmd_pos_d.isValid())
            addSingleCommandToQueue(dxl_cmd_pos_d);
    }

    // Velocity Gain
    if (dxlState->getVelocityPGain() > 0)
    {
        SingleMotorCmd dxl_cmd_vel_p(EDxlCommandType::CMD_TYPE_VELOCITY_P_GAIN, motor_id, dxlState->getVelocityPGain());

        if (dxl_cmd_vel_p.isValid())
            addSingleCommandToQueue(dxl_cmd_vel_p);
    }

    if (dxlState->getVelocityIGain() > 0)
    {
        SingleMotorCmd dxl_cmd_vel_i(EDxlCommandType::CMD_TYPE_VELOCITY_I_GAIN, motor_id, dxlState->getVelocityIGain());

        if (dxl_cmd_vel_i.isValid())
            addSingleCommandToQueue(dxl_cmd_vel_i);
    }

    // Feed Forward Gain
    if (dxlState->getFF1Gain() > 0)
    {
        SingleMotorCmd dxl_cmd_ff1(EDxlCommandType::CMD_TYPE_FF1_GAIN, motor_id, dxlState->getFF1Gain());

        if (dxl_cmd_ff1.isValid())
            addSingleCommandToQueue(dxl_cmd_ff1);
    }

    if (dxlState->getFF2Gain() > 0)
    {
        SingleMotorCmd dxl_cmd_ff2(EDxlCommandType::CMD_TYPE_FF2_GAIN, motor_id, dxlState->getFF2Gain());

        if (dxl_cmd_ff2.isValid())
            addSingleCommandToQueue(dxl_cmd_ff2);
    }

    return true;
}

// ***************
//  Getters
// ***************

/**
 * @brief TtlDriverCore::getEndEffectorState
 * @param id
 * @return
 */
double TtlDriverCore::getEndEffectorState(uint8_t id) const
{
    DxlMotorState motor_state = _ttl_driver->getMotorState(id);
    return static_cast<double>(motor_state.getPositionState());
}

/**
 * @brief TtlDriverCore::getHwStatus
 * @return
 */
ttl_driver::DxlArrayMotorHardwareStatus TtlDriverCore::getHwStatus() const
{
    ttl_driver::DxlMotorHardwareStatus data;
    ttl_driver::DxlArrayMotorHardwareStatus hw_state;

    for (auto const& dxlState : _ttl_driver->getMotorsStates())
    {
        if (dxlState)
        {
            data.motor_identity.motor_id = dxlState->getId();
            data.motor_identity.motor_type = static_cast<uint8_t>(dxlState->getType());
            data.temperature = static_cast<uint32_t>(dxlState->getTemperatureState());
            data.voltage = static_cast<double>(dxlState->getVoltageState()) / DXL_VOLTAGE_DIVISOR;
            data.error = static_cast<uint32_t>(dxlState->getHardwareErrorState());
            data.error_msg = dxlState->getHardwareErrorMessageState();
            hw_state.motors_hw_status.push_back(data);
        }
    }
    return hw_state;
}

/**
 * @brief TtlDriverCore::getBusState
 * @return
 */
niryo_robot_msgs::BusState TtlDriverCore::getBusState() const
{
    niryo_robot_msgs::BusState dxl_bus_state;

    string error;
    bool connection;
    vector<uint8_t> motor_id;

    _ttl_driver->getBusState(connection, motor_id, error);
    dxl_bus_state.connection_status = connection;
    dxl_bus_state.motor_id_connected = motor_id;
    dxl_bus_state.error = error;
    return dxl_bus_state;
}

// *******************
//    Callbacks     *
// *******************

/**
 * @brief TtlDriverCore::callbackSendCustomDxlValue
 * @param req
 * @param res
 * @return
 */
bool TtlDriverCore::_callbackSendCustomDxlValue(ttl_driver::SendCustomDxlValue::Request &req,
                                               ttl_driver::SendCustomDxlValue::Response &res)
{
    int result;

    EMotorType motor_type;

    if (2 <= req.motor_type  && 5 >= req.motor_type)
        motor_type = static_cast<EMotorType>(req.motor_type);
    else
    {
        res.status = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        res.message = "TtlDriverCore - Invalid motor type: should be";
// (CC) to be changed with loop
//        2(XL - 430) or 3(XL - 320) or 4(XL - 330) or 5(XC - 430) ";
            return true;
    }

    lock_guard<mutex> lck(_control_loop_mutex);
    result = _ttl_driver->sendCustomDxlCommand(motor_type,
                                              req.id,
                                              req.reg_address,
                                              req.value,
                                              req.byte_number);

    if (result != COMM_SUCCESS)
    {
        res.message = "TtlDriverCore - Send custom command failed";
    }
    else
    {
        res.message = "TtlDriverCore - Send custom command done";
        result = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    res.status = result;
    return true;
}

/**
 * @brief TtlDriverCore::callbackReadCustomDxlValue
 * @param req
 * @param res
 * @return
 */
bool TtlDriverCore::_callbackReadCustomDxlValue(ttl_driver::ReadCustomDxlValue::Request &req,
                                               ttl_driver::ReadCustomDxlValue::Response &res)
{
    int result;
    EMotorType motor_type;
    if (2 <= req.motor_type  && 5 >= req.motor_type)
        motor_type = static_cast<EMotorType>(req.motor_type);
    else
    {
        res.status = niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE;
        // (CC) to be changed with for loop
        res.message = "TtlDriverCore - Invalid motor type: should be";
        /*
        2(XL - 430) "
                    " or 3 (XL-320) or 4 (XL-330) or 5 (XC-430)";*/
        return true;
    }

    lock_guard<mutex> lck(_control_loop_mutex);
    int value = 0;
    result = _ttl_driver->readCustomDxlCommand(motor_type,
                                              req.id,
                                              req.reg_address,
                                              value,
                                              req.byte_number);

    if (result != COMM_SUCCESS)
    {
        res.message = "TtlDriverCore - Reading custom registry failed";
    }
    else
    {
        res.message = "TtlDriverCore - Reading successful. Registry value : " + to_string(value);
        result = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    res.status = result;
    return true;
}

/**
 * @brief TtlDriverCore::callbackActivateLeds
 * @param req
 * @param res
 * @return
 */
bool TtlDriverCore::_callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req,
                                          niryo_robot_msgs::SetInt::Response &res)
{
    int led = req.value;
    string message = "";

    lock_guard<mutex> lck(_control_loop_mutex);
    int result = _ttl_driver->setLeds(led, EMotorType::XL320);

    res.status = result;
    res.message = message;
    return true;
}

}  // namespace ttl_driver
