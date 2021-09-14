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
#include <vector>
#include <fstream>

// to be migrated to std/filesystem when switching to C++17
#include <boost/filesystem.hpp>

// ros
#include <ros/console.h>

// niryo
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"

#include "common/util/util_defs.hpp"

#include "joints_interface/calibration_manager.hpp"

using ::common::model::JointState;
using ::common::model::EStepperCommandType;
using ::common::model::StepperSingleCmd;
using ::common::model::StepperTtlSingleCmd;
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
    _joint_states_list(joint_list)
{
    ROS_DEBUG("CalibrationManager::ctor");

    // add interface core into map if not null
    if(ttl_interface)
      _driver_interfaces_map.insert(std::make_pair(EBusProtocol::TTL, ttl_interface));

    if(can_interface)
      _driver_interfaces_map.insert(std::make_pair(EBusProtocol::CAN, can_interface));


    initParameters(nh);

    ROS_INFO("Calibration Interface - Calibration interface started");
}

/**
 * @brief CalibrationManager::~CalibrationManager
 */
CalibrationManager::~CalibrationManager()
{
}

/**
 * @brief CalibrationManager::initParameters
 * @param nh
 */
void CalibrationManager::initParameters(ros::NodeHandle &nh)
{
    nh.getParam("calibration_timeout", _calibration_timeout);
    nh.getParam("calibration_file", _calibration_file_name);
    nh.getParam("/niryo_robot_hardware_interface/hardware_version", _hardware_version);

    ROS_DEBUG("Calibration Interface - hardware_version %s", _hardware_version.c_str());
    ROS_DEBUG("Calibration Interface - Calibration timeout %d", _calibration_timeout);
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
    if (AUTO_CALIBRATION == mode)  // auto
    {
        _calibration_in_progress = true;
        if (!steppersConnected())
        {
            result_message = "Calibration Interface - Please ensure that all motors are connected";
            _calibration_in_progress = false;
            return niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
        }
        autoCalibration();
        _calibration_in_progress = false;
    }
    else if (MANUAL_CALIBRATION == mode)  // manuel
    {
        _calibration_in_progress = true;
        if (!canProcessManualCalibration(result_message))
        {
            result_message = "Calibration Interface - Can't proceed to manual calibration";
            _calibration_in_progress = false;
            return niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
        }
        if (!steppersConnected())
        {
            result_message = "Calibration Interface - Please ensure that all motors are connected";
            _calibration_in_progress = false;
            return niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
        }

        manualCalibration();
        _calibration_in_progress = false;
    }
    else
    {
        result_message = "Calibration Interface - Command error";
        return -1;
    }
    result_message = "Calibration Interface - Calibration done";
    return niryo_robot_msgs::CommandStatus::SUCCESS;
}

/**
 * @brief CalibrationManager::setTorqueStepperMotor
 * @param motor
 * @param status
 */
void CalibrationManager::setTorqueStepperMotor(const std::shared_ptr<JointState>& pState,
                                               bool status)
{
  if(pState->isStepper())
  {
    uint8_t motor_id = pState->getId();

    if (EBusProtocol::CAN == pState->getBusProtocol())
    {
        StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, motor_id, {status});
        if(_driver_interfaces_map.count(pState->getBusProtocol()))
            _driver_interfaces_map.at(pState->getBusProtocol())->addSingleCommandToQueue(
                                    std::make_shared<StepperSingleCmd>(stepper_cmd));
    }
    else if (EBusProtocol::TTL == pState->getBusProtocol())
    {
        StepperTtlSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, motor_id, {status});
        if(_driver_interfaces_map.count(pState->getBusProtocol()))
            _driver_interfaces_map.at(pState->getBusProtocol())->addSingleCommandToQueue(
                                    std::make_shared<StepperTtlSingleCmd>(stepper_cmd));
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
  if(pState->isStepper() && _driver_interfaces_map.count(pState->getBusProtocol()))
  {
    uint8_t motor_id = pState->getId();
    int32_t offset = pState->to_motor_pos(pState->getOffsetPosition());
    int32_t motor_direction = static_cast<int32_t>(pState->getDirection());

    if (EBusProtocol::CAN == pState->getBusProtocol())
    {
      StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION, motor_id,
                                {offset, delay, motor_direction * calibration_direction, timeout});
      _driver_interfaces_map.at(pState->getBusProtocol())->addSingleCommandToQueue(
                              std::make_shared<StepperSingleCmd>(stepper_cmd));
    }
    else if (EBusProtocol::TTL == pState->getBusProtocol())
    {
        StepperTtlSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION, motor_id);
        _driver_interfaces_map.at(pState->getBusProtocol())->addSingleCommandToQueue(
                              std::make_shared<StepperTtlSingleCmd>(stepper_cmd));
    }

  }
}

/**
 * @brief CalibrationManager::steppersConnected
 * @return
 */
bool CalibrationManager::steppersConnected()
{
    for (auto const& jState : _joint_states_list)
    {
        if (jState && jState->isStepper() && _driver_interfaces_map.count(jState->getBusProtocol()))
        {
            if (!_driver_interfaces_map.at(jState->getBusProtocol())->scanMotorId(jState->getId()))
                    return false;
        }
    }
    return true;
}

/**
 * @brief CalibrationManager::autoCalibration
 * @return
 */
EStepperCalibrationStatus CalibrationManager::autoCalibration()
{
    // 1 - for can only, first move motor 3 a bit up
    if (EBusProtocol::CAN == _joint_states_list.at(1)->getBusProtocol() &&
        EBusProtocol::CAN == _joint_states_list.at(2)->getBusProtocol())
    {
        // Torque ON for motor 2
        setTorqueStepperMotor(_joint_states_list.at(1), true);

        ros::Duration(0.2).sleep();

        // Relative Move Motor 3
        if(_joint_states_list.at(2)->isStepper())
        {
          uint8_t motor_id = _joint_states_list.at(2)->getId();
          int steps = _joint_states_list.at(2)->to_motor_pos(0.25);
          int delay = 500;

          setTorqueStepperMotor(_joint_states_list.at(2), true);

          StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor_id, {steps, delay});
          _driver_interfaces_map.at(_joint_states_list.at(2)->getBusProtocol())->addSingleCommandToQueue(
                                  std::make_shared<StepperSingleCmd>(stepper_cmd));
        }
        ros::Duration(0.5).sleep();
    }

    // 2. Move All Dynamixel to Home Position
    moveDxlToHome();

    // 3. Send calibration cmd 1 + 2 + 3 (from can or ttl depending of which interface is instanciated)
    sendCalibrationToSteppers();

    // wait for calibration status to change
    while ((_driver_interfaces_map.count(EBusProtocol::CAN) && _driver_interfaces_map.at(EBusProtocol::CAN)->isCalibrationInProgress()) ||
           (_driver_interfaces_map.count(EBusProtocol::TTL) && _driver_interfaces_map.at(EBusProtocol::TTL)->isCalibrationInProgress()))
    {
        ros::Duration(0.2).sleep();
    }

    // retrieve values for the calibration
    std::vector<int> sensor_offset_results;
    std::vector<int> sensor_offset_ids;

    for (size_t i = 0; i < 3; ++i)
    {
        uint8_t motor_id = _joint_states_list.at(i)->getId();
        if(_driver_interfaces_map.count(_joint_states_list.at(i)->getBusProtocol()))
        {
          int calibration_result = _driver_interfaces_map.at(_joint_states_list.at(i)->getBusProtocol())->getCalibrationResult(motor_id);

          sensor_offset_results.emplace_back(calibration_result);
          sensor_offset_ids.emplace_back(motor_id);

          ROS_INFO("Calibration Interface - Motor %d, calibration cmd result %d ", motor_id, calibration_result);
        }
    }

    if (sensor_offset_results.at(0) && sensor_offset_results.at(1) && sensor_offset_results.at(2))
    {
        ROS_INFO("Calibration Interface -  New Calibration values : ");

        // 4. Relative Move Motor 1 to 0.0 (back to home)
        // -0.01 to bypass error
        ros::Duration(0.2).sleep();
        relativeMoveStepperMotor(_joint_states_list.at(0),
                           -_joint_states_list.at(0)->to_motor_pos(_joint_states_list.at(0)->getOffsetPosition()),
                           550,
                           false);

        ros::Duration(2.5).sleep();

        // 6. Write sensor_offset_steps to file
        setMotorsCalibrationOffsets(sensor_offset_ids, sensor_offset_results);
    }
    else
    {
        ROS_ERROR("Calibration Interface -  An error occured while calibrating stepper motors");
    }

    // 7 - stop torques
    activateLearningMode(true);

    auto calibration_status = common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;

    if (_driver_interfaces_map.count(EBusProtocol::CAN))
        calibration_status = _driver_interfaces_map.at(EBusProtocol::CAN)->getCalibrationStatus();
    else if (_driver_interfaces_map.count(EBusProtocol::TTL))
        calibration_status = _driver_interfaces_map.at(EBusProtocol::TTL)->getCalibrationStatus();

    return calibration_status;
}

/**
 * @brief CalibrationManager::_can_process_manual_calibration
 * @param result_message
 * @return
 */
bool CalibrationManager::canProcessManualCalibration(std::string &result_message)
{
    if (_driver_interfaces_map.count(EBusProtocol::CAN))
    {
        auto stepper_motor_states = _driver_interfaces_map.at(EBusProtocol::CAN)->getJointStates();

        // 1. Check if motors firmware version is ok
        for (auto const& mState : stepper_motor_states)
        {
            if (mState)
            {
                // TODO(Thuc) check firmware version only need for stepper, need verify
                std::string firmware_version = std::dynamic_pointer_cast<StepperMotorState>(mState)->getFirmwareVersion();
                if (firmware_version.length() == 0)
                {
                    result_message = "Calibration Interface - No firmware version available for motor " +
                                    std::to_string(mState->getId()) +
                                    ". Make sure all motors are connected";

                    ROS_WARN("Calibration Interface - Can't process manual calibration : %s",
                            result_message.c_str());
                    return false;
                }
                if (stoi(firmware_version.substr(0, 1)) < 2)
                {
                    result_message = "Calibration Interface - You need to upgrade stepper firmware for motor " +
                                        std::to_string(mState->getId());

                    ROS_WARN("Calibration Interface - Can't process manual calibration : %s",
                            result_message.c_str());
                    return false;
                }
            }
        }

        // 2. Check if motor offset values have been previously saved (with auto calibration)
        std::vector<int> motor_id_list;
        std::vector<int> steps_list;
        if (!getMotorsCalibrationOffsets(motor_id_list, steps_list))
        {
            result_message = "Calibration Interface - You need to make an "
                            "auto calibration before using the manual calibration";
            ROS_WARN("Calibration Interface - Can't process manual calibration : %s",
                    result_message.c_str());
            return false;
        }

        // 3. Check if all connected motors have a motor offset value
        for (auto const& mState : stepper_motor_states)
        {
            if (mState)
            {
                bool found = false;

                for (int m_id : motor_id_list)
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
                    ROS_WARN("Calibration Interface - Can't process manual calibration : %s", result_message.c_str());
                    return false;
                }
            }
        }
    }

    return true;
}

/**
 * @brief CalibrationManager::sendCalibrationOffset
 * @param id
 * @param offset_to_send
 * @param absolute_steps_at_offset_position
 */
void CalibrationManager::sendCalibrationOffset(const std::shared_ptr<StepperMotorState>& pState,
                                               int offset_to_send, int absolute_steps_at_offset_position)
{
  if(pState->isStepper() && _driver_interfaces_map.count(pState->getBusProtocol()))
  {
    uint8_t motor_id = pState->getId();

    if (EBusProtocol::CAN == pState->getBusProtocol())
    {
      StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION_OFFSET, motor_id, {offset_to_send, absolute_steps_at_offset_position});
      _driver_interfaces_map.at(EBusProtocol::CAN)->addSingleCommandToQueue(
                      std::make_shared<StepperSingleCmd>(stepper_cmd));
    }
    else if (EBusProtocol::TTL == pState->getBusProtocol())
    {
      StepperTtlSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION_OFFSET, motor_id, {offset_to_send, absolute_steps_at_offset_position});
      _driver_interfaces_map.at(EBusProtocol::TTL)->addSingleCommandToQueue(
                      std::make_shared<StepperTtlSingleCmd>(stepper_cmd));
    }

    // (Thuc) call add single command to Queue by if else but not
    // by polymorphism make the program will work for the case all stepper use can or ttl,
    // not 2 protocol in the same time
  }
}

/**
 * @brief CalibrationManager::manualCalibration
 * @return
 */
EStepperCalibrationStatus CalibrationManager::manualCalibration()
{
    ros::Rate rest(0.5);
    ros::Duration sld(0.2);
    std::vector<int> motor_id_list;
    std::vector<int> steps_list;

    if (!getMotorsCalibrationOffsets(motor_id_list, steps_list))
    {
       return EStepperCalibrationStatus::CALIBRATION_FAIL;
    }
    if (_driver_interfaces_map.count(EBusProtocol::CAN))
        _driver_interfaces_map.at(EBusProtocol::CAN)->startCalibration();
    else if (_driver_interfaces_map.count(EBusProtocol::TTL))
        _driver_interfaces_map.at(EBusProtocol::TTL)->startCalibration();

    // 0. Torque ON for motor 2
    auto state = std::dynamic_pointer_cast<common::model::StepperMotorState>(_joint_states_list.at(1));
    if(state)
    {
      int steps_per_rev = state->stepsPerRev();

      for (size_t i = 0; i < motor_id_list.size(); i++)
      {
          int offset_to_send = 0;
          int sensor_offset_steps = steps_list.at(i);
          int absolute_steps_at_offset_position = 0;

          if (motor_id_list.at(i) == _joint_states_list.at(0)->getId())
          {
              offset_to_send = sensor_offset_steps - _joint_states_list.at(0)->to_motor_pos(_joint_states_list.at(0)->getOffsetPosition()) % steps_per_rev;
              if (offset_to_send < 0)
                  offset_to_send += steps_per_rev;
              absolute_steps_at_offset_position = offset_to_send;

              sendCalibrationOffset(_joint_states_list.at(0), offset_to_send, absolute_steps_at_offset_position);
              sld.sleep();
          }
          else if (motor_id_list.at(i) == _joint_states_list.at(1)->getId())
          {
              offset_to_send = sensor_offset_steps - _joint_states_list.at(1)->to_motor_pos(_joint_states_list.at(1)->getOffsetPosition());

              if ("one" == _hardware_version)
              {
                  offset_to_send %= steps_per_rev;
                  if (offset_to_send < 0)
                      offset_to_send += steps_per_rev;
              }

              absolute_steps_at_offset_position = offset_to_send;

              sendCalibrationOffset(_joint_states_list.at(1), offset_to_send, absolute_steps_at_offset_position);
              sld.sleep();
          }
          else if (motor_id_list.at(i) == _joint_states_list.at(2)->getId())
          {
              offset_to_send = sensor_offset_steps - _joint_states_list.at(2)->to_motor_pos(_joint_states_list.at(2)->getOffsetPosition());
              absolute_steps_at_offset_position = sensor_offset_steps;

              sendCalibrationOffset(_joint_states_list.at(2), offset_to_send, absolute_steps_at_offset_position);
              sld.sleep();
          }
      }
      if (_driver_interfaces_map.count(EBusProtocol::CAN))
          return _driver_interfaces_map.at(EBusProtocol::CAN)->getCalibrationStatus();
      else if (_driver_interfaces_map.count(EBusProtocol::TTL))
          return _driver_interfaces_map.at(EBusProtocol::TTL)->getCalibrationStatus();

    }

    return EStepperCalibrationStatus::CALIBRATION_FAIL;
}

/**
 * @brief CalibrationManager::getMotorsCalibrationOffsets
 * @param motor_id_list
 * @param steps_list
 * @return
 */
bool CalibrationManager::getMotorsCalibrationOffsets(std::vector<int> &motor_id_list, std::vector<int> &steps_list)
{
    std::vector<std::string> lines;
    std::string current_line;

    std::ifstream offset_file(_calibration_file_name.c_str());
    if (offset_file.is_open())
    {
        while (getline(offset_file, current_line))
        {
            try
            {
                size_t index = current_line.find(":");
                motor_id_list.push_back(stoi(current_line.substr(0, index)));
                steps_list.push_back(stoi(current_line.erase(0, index + 1)));
            }
            catch (...)
            {
            }
        }
        offset_file.close();
    }
    else
    {
        ROS_WARN("Motor Offset - Unable to open file : %s", _calibration_file_name.c_str());
        return false;
    }
    return true;
}

/**
 * @brief CalibrationManager::moveDxlToHome
 */
void CalibrationManager::moveDxlToHome()
{
  if (_driver_interfaces_map.count(EBusProtocol::TTL))
  {
      // set torque on
      common::model::DxlSyncCmd dynamixel_cmd(EDxlCommandType::CMD_TYPE_TORQUE);
      dynamixel_cmd.addMotorParam(_joint_states_list.at(3)->getHardwareType(), _joint_states_list.at(3)->getId(), 1);
      dynamixel_cmd.addMotorParam(_joint_states_list.at(4)->getHardwareType(), _joint_states_list.at(4)->getId(), 1);
      dynamixel_cmd.addMotorParam(_joint_states_list.at(5)->getHardwareType(), _joint_states_list.at(5)->getId(), 1);

      _driver_interfaces_map.at(EBusProtocol::TTL)->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dynamixel_cmd));
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

      _driver_interfaces_map.at(EBusProtocol::TTL)->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dynamixel_cmd));
      ros::Duration(0.2).sleep();
  }
}

/**
 * @brief CalibrationManager::moveSteppersToHome
 */
void CalibrationManager::moveSteppersToHome()
{
  auto pState = _joint_states_list.at(0);

  if(pState->isStepper() && _driver_interfaces_map.count(pState->getBusProtocol()))
  {

    uint8_t motor_id = pState->getId();
    int steps = -pState->to_motor_pos(pState->getOffsetPosition());
    int delay = 550;
    setTorqueStepperMotor(pState, true);

    if (EBusProtocol::CAN == pState->getBusProtocol())
    {

        StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor_id, {steps, delay});
        _driver_interfaces_map.at(pState->getBusProtocol())->addSingleCommandToQueue(
                                std::make_shared<StepperSingleCmd>(stepper_cmd));
    }
    else if (EBusProtocol::TTL == pState->getBusProtocol())
    {
        StepperTtlSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION, motor_id, {0});
        _driver_interfaces_map.at(pState->getBusProtocol())->addSingleCommandToQueue(
                                std::make_shared<StepperTtlSingleCmd>(stepper_cmd));
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
    if (_driver_interfaces_map.count(EBusProtocol::CAN))
    {

        _driver_interfaces_map.at(EBusProtocol::CAN)->startCalibration();

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


        _driver_interfaces_map.at(EBusProtocol::TTL)->startCalibration();

        StepperTtlSingleCmd stepper_dir_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION_DIRECTION, pStepperMotorState_2->getId(), {1});
        _driver_interfaces_map.at(EBusProtocol::TTL)->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(stepper_dir_cmd));

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

    common::model::DxlSyncCmd dxl_cmd(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
    common::model::StepperTtlSingleCmd stepper_ttl_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);
    common::model::StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE);

    for (auto const& jState : _joint_states_list)
    {
        if (jState)
        {
            if (jState->isDynamixel())
            {
                dxl_cmd.addMotorParam(jState->getHardwareType(), jState->getId(), !activated);
            }
            else if ((jState->isStepper() && EBusProtocol::TTL == jState->getBusProtocol()))
            {
                stepper_ttl_cmd.setId(jState->getId());
                stepper_ttl_cmd.setParams({!activated});
                _driver_interfaces_map.at(EBusProtocol::TTL)->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(stepper_ttl_cmd));
            }
            else
            {
                stepper_cmd.setId(jState->getId());
                stepper_cmd.setParams({!activated});
                _driver_interfaces_map.at(EBusProtocol::CAN)->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(stepper_cmd));
            }
        }
    }

    if (_driver_interfaces_map.count(EBusProtocol::TTL))
    {
        _driver_interfaces_map.at(EBusProtocol::TTL)->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dxl_cmd));
    }
}


/**
 * @brief CalibrationManager::setMotorsCalibrationOffsets
 * @param motor_id_list
 * @param steps_list
 * @return
 */
bool CalibrationManager::setMotorsCalibrationOffsets(const std::vector<int> &motor_id_list,
                                                        const std::vector<int> &steps_list)
{
    if (motor_id_list.size() != steps_list.size())
    {
        ROS_ERROR("Corrupted command : motors id list and params list size mismatch");
        return false;
    }

    size_t found = _calibration_file_name.find_last_of("/");
    std::string folder_name = _calibration_file_name.substr(0, found);

    boost::filesystem::path filepath(_calibration_file_name);
    boost::filesystem::path directory(folder_name);

    // Create dir if not exist
    boost::system::error_code returned_error;
    boost::filesystem::create_directories(directory, returned_error);
    if (returned_error)
    {
        ROS_WARN("CalibrationManager::set_motors_calibration_offsets - Could not create directory : %s",
                 folder_name.c_str());
        return false;
    }

    // Create text to write
    std::string text_to_write = "";
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
    }
    else
    {
        ROS_WARN("CalibrationManager::set_motors_calibration_offsets - Unable to open file : %s",
                 _calibration_file_name.c_str());
        return false;
    }

    return true;
}
}  // namespace joints_interface
