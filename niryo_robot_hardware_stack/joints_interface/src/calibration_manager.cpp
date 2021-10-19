/*
    _calibration_manager.cpp
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

// std
#include <functional>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

// to be migrated to std/filesystem when switching to C++17
#include <boost/filesystem.hpp>

// ros
#include <ros/console.h>

// niryo
#include "common/model/single_motor_cmd.hpp"
#include "common/model/synchronize_motor_cmd.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"

#include "common/util/util_defs.hpp"

#include "joints_interface/calibration_manager.hpp"

using ::common::model::JointState;
using ::common::model::EStepperCommandType;
using ::common::model::DxlSyncCmd;
using ::common::model::StepperSingleCmd;
using ::common::model::StepperTtlSyncCmd;
using ::common::model::StepperMotorState;
using ::common::model::EStepperCalibrationStatus;
using ::common::model::EDxlCommandType;
using ::common::model::EBusProtocol;
using ::common::model::StepperTtlSingleCmd;

namespace joints_interface
{

/**
 * @brief CalibrationManager::CalibrationManager
 * @param nh
 * @param joint_list
 * @param ttl_interface
 * @param can_interface
 */
CalibrationManager::CalibrationManager(ros::NodeHandle& nh,
                                       std::vector<std::shared_ptr<JointState> > joint_list,
                                       std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                                       std::shared_ptr<can_driver::CanInterfaceCore> can_interface) :
    _ttl_interface(std::move(ttl_interface)),
    _can_interface(std::move(can_interface)),
    _joint_states_list(std::move(joint_list))
{
    ROS_DEBUG("CalibrationManager::ctor");

    initParameters(nh);

    ROS_INFO("Calibration Interface - Calibration interface started");
}

/**
 * @brief CalibrationManager::initParameters
 * @param nh
 */
void CalibrationManager::initParameters(ros::NodeHandle &nh)
{
    nh.getParam("calibration_timeout", _calibration_timeout);
    nh.getParam("calibration_stall_threshold", _calibration_stall_threshold);

    nh.getParam("calibration_file", _calibration_file_name);
    nh.getParam("/niryo_robot_hardware_interface/hardware_version", _hardware_version);

    ROS_DEBUG("Calibration Interface - hardware_version %s", _hardware_version.c_str());
    ROS_DEBUG("Calibration Interface - Calibration timeout %d", _calibration_timeout);
    ROS_DEBUG("Calibration Interface - Calibration stall threshold %d", _calibration_stall_threshold);
    ROS_DEBUG("Calibration Interface - Calibration file name %s", _calibration_file_name.c_str());
}

/**
 * @brief CalibrationManager::startCalibration
 * @param mode
 * @param result_message
 * @return
 */
int CalibrationManager::startCalibration(int mode, std::string &result_message)
{
    int res = niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
    result_message.clear();

    if (steppersConnected())
    {
        if (AUTO_CALIBRATION == mode)  // auto
        {
            if (EStepperCalibrationStatus::CALIBRATION_OK == autoCalibration())
            {
                result_message = "Calibration Interface - Calibration done";
                res = niryo_robot_msgs::CommandStatus::SUCCESS;
            }
            else
                result_message = "Calibration Interface - auto calibration failed";
        }
        else if (MANUAL_CALIBRATION == mode)  // manuel
        {
            if (canProcessManualCalibration(result_message))
            {
                if (EStepperCalibrationStatus::CALIBRATION_OK == manualCalibration())
                {
                    result_message = "Calibration Interface - Calibration done";
                    res = niryo_robot_msgs::CommandStatus::SUCCESS;
                }
                else
                    result_message = "Calibration Interface - manual calibration failed";
            }
        }
        else                          // unknown
        {
            result_message = "Calibration Interface - Command error";
            res = niryo_robot_msgs::CommandStatus::FAILURE;
        }
    }
    else
    {
        result_message = "Calibration Interface - Please ensure that all motors are connected";
    }

    ROS_WARN_COND(niryo_robot_msgs::CommandStatus::SUCCESS != res,
                  "Calibration Interface - Calibration error : %s", result_message.c_str());

    return res;
}

/**
 * @brief CalibrationManager::steppersConnected
 * @return
 */
bool CalibrationManager::steppersConnected()
{
    for (auto const& jState : _joint_states_list)
    {
        if (jState && jState->isStepper() && getJointInterface(jState->getBusProtocol()))
        {
            if (!getJointInterface(jState->getBusProtocol())->scanMotorId(jState->getId()))
                    return false;
        }
    }
    return true;
}

/**
 * @brief CalibrationManager::canProcessManualCalibration
 * @param result_message
 * @return
 */
bool CalibrationManager::canProcessManualCalibration(std::string &result_message)
{
    bool res = true;
    result_message.clear();
    if ("one" == _hardware_version)
    {
        if (_can_interface)
        {
            auto stepper_motor_states = _can_interface->getJointStates();

            // 1. Check if motors firmware version is ok
            for (auto const& mState : stepper_motor_states)
            {
                if (mState)
                {
                    std::string firmware_version = std::dynamic_pointer_cast<StepperMotorState>(mState)->getFirmwareVersion();
                    if (!firmware_version.empty())
                    {
                        if (stoi(firmware_version.substr(0, 1)) >= 2)
                        {
                            // 2. Check if motor offset values have been previously saved (with auto calibration)
                            std::vector<int> motor_id_list;
                            std::vector<int> steps_list;

                            if (getMotorsCalibrationOffsets(motor_id_list, steps_list))
                            {
                                // 3. Check if all connected motors have a motor offset value

                                bool found = false;

                                for (auto const& m_id : motor_id_list)
                                {
                                    if (m_id == mState->getId())
                                    {
                                        found = true;
                                        break;
                                    }
                                }

                                if (!found)
                                {
                                    result_message = "Calibration Interface - Motor " +
                                                    std::to_string(mState->getId()) +
                                                    " does not have a saved offset value, " +
                                                    "you need to do one auto calibration";

                                    res = false;
                                }
                            }
                            else
                            {
                                result_message = "Calibration Interface - You need to make an "
                                                 "auto calibration before using the manual calibration";

                                res = false;
                            }
                        }
                        else
                        {
                            result_message = "Calibration Interface - You need to upgrade stepper firmware for motor " +
                                                std::to_string(mState->getId());

                            res = false;
                        }
                    }
                    else
                    {
                        result_message = "Calibration Interface - No firmware version available for motor " +
                                        std::to_string(mState->getId()) +
                                        ". Make sure all motors are connected";

                        res = false;
                    }
                }  // if (mState)
            }  // for (auto const& mState : stepper_motor_states)
        }
        else
        {
          result_message = "Calibration Interface - CAN interface not available";

          res = false;
        }
    }
    else
    {
      result_message = "Calibration Interface - manual calibration available for Niryo One only";
      res = false;
    }

    ROS_WARN_COND(!res, "Calibration Interface - Can't process manual calibration : %s", result_message.c_str());

    return res;
}

/**
 * @brief CalibrationManager::autoCalibration
 * @return
 */
EStepperCalibrationStatus CalibrationManager::autoCalibration()
{
    _calibration_in_progress = true;

    // 1. Move robot back to home
    moveRobotBeforeCalibration();

    // 2. Send calibration cmd 1 + 2 + 3 (from can or ttl depending of which interface is instanciated)
    sendCalibrationToSteppers();

    double timeout = 0.0;
    // 3. wait for calibration status to change
    while ((_can_interface && _can_interface->isCalibrationInProgress()) ||
           (_ttl_interface && _ttl_interface->isCalibrationInProgress()))
    {
        ros::Duration(0.2).sleep();
        timeout += 0.2;
        if (timeout >= 30.0)
        {
            ROS_ERROR("CalibrationManager::autoCalibration - calibration timeout, please try again");
            _calibration_in_progress = false;
            return common::model::EStepperCalibrationStatus::CALIBRATION_TIMEOUT;
        }
    }

    // 4. retrieve values for the calibration
    std::vector<int> sensor_offset_results;
    std::vector<int> sensor_offset_ids;

    for (size_t i = 0; i < 3; ++i)
    {
        auto jState = _joint_states_list.at(i);
        uint8_t motor_id = jState->getId();
        if (getJointInterface(jState->getBusProtocol()))
        {
          int calibration_result = getJointInterface(jState->getBusProtocol())->getCalibrationResult(motor_id);

          sensor_offset_results.emplace_back(calibration_result);
          sensor_offset_ids.emplace_back(motor_id);

          ROS_INFO("CalibrationManager::autoCalibration - Motor %d, calibration cmd result %d ", motor_id, calibration_result);
        }
    }

    if (sensor_offset_results.at(0) && sensor_offset_results.at(1) && sensor_offset_results.at(2))
    {
        ROS_INFO("CalibrationManager::autoCalibration -  Calibration successfull, going back home");

        // 5. Move Motor 1 to 0.0 (back to home)
        moveSteppersToHome();
        ros::Duration(3.5).sleep();

        // 6. Write sensor_offset_steps to file
        setMotorsCalibrationOffsets(sensor_offset_ids, sensor_offset_results);
    }
    else
    {
        ROS_ERROR("CalibrationManager::autoCalibration -  An error occured while calibrating stepper motors");
    }

    // 7 - stop torques
    activateLearningMode(true);

    auto calibration_status = common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;

    if (_can_interface)
        calibration_status = _can_interface->getCalibrationStatus();
    else if (_ttl_interface)
        calibration_status = _ttl_interface->getCalibrationStatus();

    _calibration_in_progress = false;

    return calibration_status;
}

/**
 * @brief CalibrationManager::manualCalibration
 * @return
 *
 * only for niryo one
 */
EStepperCalibrationStatus
CalibrationManager::manualCalibration()
{
    EStepperCalibrationStatus status = EStepperCalibrationStatus::CALIBRATION_FAIL;
    _calibration_in_progress = true;

    if  ("one" == _hardware_version)
    {
        if (_can_interface)
        {
            std::vector<int> motor_id_list;
            std::vector<int> steps_list;

            if (getMotorsCalibrationOffsets(motor_id_list, steps_list))
            {
                _can_interface->startCalibration();

                // 0. Torque ON for motor 2
                auto state = std::dynamic_pointer_cast<common::model::StepperMotorState>(_joint_states_list.at(1));
                if (state)
                {
                    int steps_per_rev = state->stepsPerRev();

                    for (size_t i = 0; i < motor_id_list.size(); i++)
                    {
                        int offset_to_send = 0;
                        int motor_id = motor_id_list.at(i);
                        int sensor_offset_steps = steps_list.at(i);

                        if (motor_id == _joint_states_list.at(0)->getId())
                        {
                            offset_to_send = sensor_offset_steps - _joint_states_list.at(0)->to_motor_pos(_joint_states_list.at(0)->getOffsetPosition()) % steps_per_rev;
                            if (offset_to_send < 0)
                                offset_to_send += steps_per_rev;

                            StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION_OFFSET, _joint_states_list.at(0)->getId(),
                                                         {offset_to_send, offset_to_send});
                            _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(stepper_cmd));
                        }
                        else if (motor_id == _joint_states_list.at(1)->getId())
                        {
                            offset_to_send = sensor_offset_steps - _joint_states_list.at(1)->to_motor_pos(_joint_states_list.at(1)->getOffsetPosition());

                            offset_to_send %= steps_per_rev;
                            if (offset_to_send < 0)
                                offset_to_send += steps_per_rev;

                            StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION_OFFSET, _joint_states_list.at(1)->getId(),
                                                         {offset_to_send, offset_to_send});
                            _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(stepper_cmd));
                        }
                        else if (motor_id == _joint_states_list.at(2)->getId())
                        {
                            offset_to_send = sensor_offset_steps - _joint_states_list.at(2)->to_motor_pos(_joint_states_list.at(2)->getOffsetPosition());

                            StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION_OFFSET, _joint_states_list.at(2)->getId(),
                                                         {offset_to_send, sensor_offset_steps});
                            _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(stepper_cmd));
                        }
                        ros::Duration(0.2).sleep();
                    }

                    status = _can_interface->getCalibrationStatus();
                }
            }  // if (state)
        }  // if (getMotorsCalibrationOffsets(motor_id_list, steps_list))
    }
    else
    {
      ROS_ERROR("CalibrationManager::manualCalibration : manual calibration not available for %s robot. Only supported for Niryo One", _hardware_version.c_str());
    }

    _calibration_in_progress = false;

    return status;
}

//**********************
//  Commands to the robot
//*********************

/**
 * @brief CalibrationManager::setTorqueStepperMotor
 * @param pState
 * @param status
 * TODO(cc) : maybe use JointHardwareInterface instead ?
 */
void CalibrationManager::setTorqueStepperMotor(const std::shared_ptr<JointState>& pState,
                                               bool status)
{
    if (pState->isStepper())
    {
        uint8_t motor_id = pState->getId();

        if (EBusProtocol::CAN == pState->getBusProtocol())
        {
            StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, motor_id, {status});
            if (getJointInterface(pState->getBusProtocol()))
                getJointInterface(pState->getBusProtocol())->addSingleCommandToQueue(
                                        std::make_unique<StepperSingleCmd>(stepper_cmd));
        }
        else if (EBusProtocol::TTL == pState->getBusProtocol())
        {
            StepperTtlSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, motor_id, {status});
            if (getJointInterface(pState->getBusProtocol()))
                getJointInterface(pState->getBusProtocol())->addSingleCommandToQueue(
                                        std::make_unique<StepperTtlSingleCmd>(stepper_cmd));
        }
    }
}

/**
 * @brief CalibrationManager::setStepperCalibrationCommand
 * @param pState
 * @param delay
 * @param calibration_direction
 * @param timeout
 */
void CalibrationManager::setStepperCalibrationCommand(const std::shared_ptr<StepperMotorState>& pState,
                                                        int32_t delay, int32_t calibration_direction, int32_t timeout)
{
    if (pState->isStepper() && getJointInterface(pState->getBusProtocol()))
    {
        uint8_t motor_id = pState->getId();
        int32_t offset = pState->to_motor_pos(pState->getOffsetPosition());
        auto motor_direction = pState->getDirection();

        if (EBusProtocol::CAN == pState->getBusProtocol())
        {
          StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION, motor_id,
                                    {offset, delay, motor_direction * calibration_direction, timeout});
          getJointInterface(pState->getBusProtocol())->addSingleCommandToQueue(
                                  std::make_unique<StepperSingleCmd>(stepper_cmd));
        }
        else if (EBusProtocol::TTL == pState->getBusProtocol())
        {
            StepperTtlSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION, motor_id);
            getJointInterface(pState->getBusProtocol())->addSingleCommandToQueue(
                                  std::make_unique<StepperTtlSingleCmd>(stepper_cmd));
        }
    }
}

/**
 * @brief CalibrationManager::moveRobotBeforeCalibration
 */
void CalibrationManager::moveRobotBeforeCalibration()
{
    // 1 - for can only, first move motor 3 a bit up
    if (_can_interface)
    {
        // move motor 1 back a little bit
        // set Torque motor 1
        setTorqueStepperMotor(_joint_states_list.at(0), true);
        // Relative Move Motor 1
        if (_joint_states_list.at(0)->isStepper() && _joint_states_list.at(0)->getBusProtocol() == common::model::EBusProtocol::CAN)
        {
          uint8_t motor_id = _joint_states_list.at(0)->getId();
          int steps = -500 * _joint_states_list.at(0)->getDirection();
          int delay = 200;

          StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor_id, {steps, delay});
          getJointInterface(_joint_states_list.at(0)->getBusProtocol())->addSingleCommandToQueue(
                                  std::make_unique<StepperSingleCmd>(stepper_cmd));
        }
        ros::Duration(0.3).sleep();

        // Torque ON for motor 2
        setTorqueStepperMotor(_joint_states_list.at(1), true);

        ros::Duration(0.2).sleep();

        // Relative Move Motor 3
        if (_joint_states_list.at(2)->isStepper())
        {
          uint8_t motor_id = _joint_states_list.at(2)->getId();
          int steps = _joint_states_list.at(2)->to_motor_pos(0.25);
          int delay = 500;

          setTorqueStepperMotor(_joint_states_list.at(2), true);

          StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor_id, {steps, delay});
          getJointInterface(_joint_states_list.at(2)->getBusProtocol())->addSingleCommandToQueue(
                                  std::make_unique<StepperSingleCmd>(stepper_cmd));
        }
        ros::Duration(0.5).sleep();
    }

    // 2. Move All Dynamixel to Home Position
    if (_ttl_interface)
    {
        // set torque on
        DxlSyncCmd dynamixel_cmd(EDxlCommandType::CMD_TYPE_TORQUE);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(3)->getHardwareType(), _joint_states_list.at(3)->getId(), 1);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(4)->getHardwareType(), _joint_states_list.at(4)->getId(), 1);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(5)->getHardwareType(), _joint_states_list.at(5)->getId(), 1);

        _ttl_interface->setSyncCommand(std::make_unique<DxlSyncCmd>(dynamixel_cmd));
        ros::Duration(0.2).sleep();

        // move dxls
        dynamixel_cmd.reset();
        dynamixel_cmd.setType(EDxlCommandType::CMD_TYPE_POSITION);

        dynamixel_cmd.addMotorParam(_joint_states_list.at(3)->getHardwareType(), _joint_states_list.at(3)->getId(),
                                    static_cast<uint32_t>(_joint_states_list.at(3)->to_motor_pos(0)));

        dynamixel_cmd.addMotorParam(_joint_states_list.at(4)->getHardwareType(), _joint_states_list.at(4)->getId(),
                                    static_cast<uint32_t>(_joint_states_list.at(4)->to_motor_pos(0)));

        dynamixel_cmd.addMotorParam(_joint_states_list.at(5)->getHardwareType(), _joint_states_list.at(5)->getId(),
                                    static_cast<uint32_t>(_joint_states_list.at(5)->to_motor_pos(0)));

        _ttl_interface->setSyncCommand(std::make_unique<DxlSyncCmd>(dynamixel_cmd));
        ros::Duration(0.2).sleep();
    }
}

/**
 * @brief CalibrationManager::moveSteppersToHome
 */
void CalibrationManager::moveSteppersToHome()
{
    auto pState = _joint_states_list.at(0);

    if (pState->isStepper() && getJointInterface(pState->getBusProtocol()))
    {
        uint8_t motor_id = pState->getId();

        setTorqueStepperMotor(pState, true);
        ros::Duration(0.2).sleep();

        if (EBusProtocol::CAN == pState->getBusProtocol())
        {
            // -0.01 to bypass error

            int steps = -pState->to_motor_pos(pState->getOffsetPosition());
            int delay = 550;
            StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor_id, {steps, delay});
            getJointInterface(pState->getBusProtocol())->addSingleCommandToQueue(
                                    std::make_unique<StepperSingleCmd>(stepper_cmd));
        }
        else if (EBusProtocol::TTL == pState->getBusProtocol())
        {
            auto steps = static_cast<uint32_t>(pState->to_motor_pos(0));

            StepperTtlSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION, motor_id, {steps});
            getJointInterface(pState->getBusProtocol())->addSingleCommandToQueue(
                                    std::make_unique<StepperTtlSingleCmd>(stepper_cmd));
        }
    }
}

/**
 * @brief CalibrationManager::sendCalibrationToSteppers
 */
void CalibrationManager::sendCalibrationToSteppers()
{
    std::shared_ptr<StepperMotorState> pStepperMotorState_1 =
            std::dynamic_pointer_cast<StepperMotorState>(_joint_states_list.at(0));

    std::shared_ptr<StepperMotorState> pStepperMotorState_2 =
            std::dynamic_pointer_cast<StepperMotorState>(_joint_states_list.at(1));

    std::shared_ptr<StepperMotorState> pStepperMotorState_3 =
            std::dynamic_pointer_cast<StepperMotorState>(_joint_states_list.at(2));

    if (pStepperMotorState_1 && pStepperMotorState_1->isValid() &&
        pStepperMotorState_2 && pStepperMotorState_2->isValid() &&
        pStepperMotorState_3 && pStepperMotorState_3->isValid())
    {
        if (_can_interface)
        {
            _can_interface->startCalibration();

            setStepperCalibrationCommand(pStepperMotorState_1, 200, 1, _calibration_timeout);
            setStepperCalibrationCommand(pStepperMotorState_2, 1000, 1, _calibration_timeout);
            setStepperCalibrationCommand(pStepperMotorState_3, 1000, -1, _calibration_timeout);

            // wait for calibration status done
            ros::Duration(0.2).sleep();
        }
        else
        {
            // calibration of steppers Ttl
            setTorqueStepperMotor(pStepperMotorState_1, false);
            setTorqueStepperMotor(pStepperMotorState_2, false);
            setTorqueStepperMotor(pStepperMotorState_3, false);

            _ttl_interface->startCalibration();

            StepperTtlSingleCmd calib_setup_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION_SETUP, pStepperMotorState_2->getId(),
                                                {1, static_cast<uint8_t>(_calibration_stall_threshold)});
            _ttl_interface->addSingleCommandToQueue(std::make_unique<StepperTtlSingleCmd>(calib_setup_cmd));

            setStepperCalibrationCommand(pStepperMotorState_1, 200, 1, _calibration_timeout);
            setStepperCalibrationCommand(pStepperMotorState_2, 1000, 1, _calibration_timeout);
            setStepperCalibrationCommand(pStepperMotorState_3, 1000, 1, _calibration_timeout);

            ros::Duration(0.2).sleep();
        }
    }
}

/**
 * @brief CalibrationManager::activateLearningMode
 * @param activated
 */
void CalibrationManager::activateLearningMode(bool activated)
{
    ROS_DEBUG("CalibrationManager::activateLearningMode - activate learning mode");

    DxlSyncCmd dxl_cmd(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
    StepperTtlSingleCmd stepper_ttl_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);
    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);

    for (auto const& jState : _joint_states_list)
    {
        if (jState)
        {
            if (jState->isDynamixel())
            {
                dxl_cmd.addMotorParam(jState->getHardwareType(), jState->getId(), activated);
            }
            else if ((jState->isStepper() && EBusProtocol::TTL == jState->getBusProtocol()))
            {
                stepper_ttl_cmd.setId(jState->getId());
                stepper_ttl_cmd.setParams({activated});
                _ttl_interface->addSingleCommandToQueue(std::make_unique<StepperTtlSingleCmd>(stepper_ttl_cmd));
            }
            else
            {
                stepper_cmd.setId(jState->getId());
                stepper_cmd.setParams({activated});
                _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(stepper_cmd));
            }
        }
    }

    if (_ttl_interface)
    {
        _ttl_interface->setSyncCommand(std::make_unique<DxlSyncCmd>(dxl_cmd));
    }
}

//********************
//  file IO
//********************

/**
 * @brief CalibrationManager::setMotorsCalibrationOffsets
 * @param motor_id_list
 * @param steps_list
 * @return
 */
bool CalibrationManager::setMotorsCalibrationOffsets(const std::vector<int> &motor_id_list,
                                                     const std::vector<int> &steps_list)
{
    bool res = false;
    if (motor_id_list.size() == steps_list.size())
    {
        size_t found = _calibration_file_name.find_last_of('/');
        std::string folder_name = _calibration_file_name.substr(0, found);

        boost::filesystem::path filepath(_calibration_file_name);
        boost::filesystem::path directory(folder_name);

        // Create dir if not exist
        boost::system::error_code returned_error;
        boost::filesystem::create_directories(directory, returned_error);

        if (!returned_error)
        {
            // Create text to write
            std::string text_to_write;
            for (size_t i = 0; i < motor_id_list.size(); i++)
            {
                text_to_write += std::to_string(motor_id_list.at(i));
                text_to_write += ":";
                text_to_write += std::to_string(steps_list.at(i));
                if (i < motor_id_list.size() - 1)
                {
                    text_to_write += "\n";
                }
            }

            // Write to file
            std::ofstream offset_file(_calibration_file_name.c_str());
            if (offset_file.is_open())
            {
                ROS_DEBUG("CalibrationManager::set_motors_calibration_offsets - Writing calibration offsets to file : \n%s",
                          text_to_write.c_str());

                offset_file << text_to_write.c_str();
                offset_file.close();

                res = true;
            }
            else
            {
                ROS_WARN("CalibrationManager::set_motors_calibration_offsets - Unable to open file : %s",
                         _calibration_file_name.c_str());
            }
        }
        else
        {
            ROS_WARN("CalibrationManager::set_motors_calibration_offsets - Could not create directory : %s",
                     folder_name.c_str());
        }
    }
    else
    {
        ROS_ERROR("CalibrationManager::set_motors_calibration_offsets - Corrupted command"
                  ": motors id list and params list size mismatch");
    }

    return res;
}

/**
 * @brief CalibrationManager::getMotorsCalibrationOffsets
 * @param motor_id_list
 * @param steps_list
 * @return
 */
bool CalibrationManager::getMotorsCalibrationOffsets(std::vector<int> &motor_id_list,
                                                     std::vector<int> &steps_list)
{
    bool res = false;

    std::vector<std::string> lines;
    std::string current_line;

    motor_id_list.clear();
    steps_list.clear();

    std::ifstream offset_file(_calibration_file_name.c_str());

    if (offset_file.is_open())
    {
        // read all lines
        while (getline(offset_file, current_line))
        {
            try
            {
                size_t index = current_line.find(':');
                motor_id_list.emplace_back(stoi(current_line.substr(0, index)));
                steps_list.emplace_back(stoi(current_line.erase(0, index + 1)));
            }
            catch (...)
            {
                ROS_ERROR("CalibrationManager::getMotorsCalibrationOffsets - Exception caught during file reading");
            }
        }

        offset_file.close();
        res = true;
    }
    else
    {
        ROS_WARN("Motor Offset - Unable to open file : %s", _calibration_file_name.c_str());
    }

    return res;
}

}  // namespace joints_interface
