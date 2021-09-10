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
    _ttl_interface(ttl_interface),
    _can_interface(can_interface),
    _joint_states_list(joint_list)
{
    ROS_DEBUG("CalibrationManager::ctor");

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
 * @brief CalibrationManager::getProtocolOfMotor
 * @param bus_proto
 * @return
 */
std::shared_ptr<common::model::IDriverCore>
CalibrationManager::getProtocolOfMotor(EBusProtocol bus_proto) const
{
    switch (bus_proto)
    {
    case EBusProtocol::CAN:
        return _can_interface;
    case EBusProtocol::TTL:
        return _ttl_interface;
    default:
        return nullptr;
    }
}

/**
 * @brief CalibrationManager::CalibrationInprogress
 * @return
 */
bool CalibrationManager::CalibrationInprogress() const
{
    return _calibration_in_progress;
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
        if (!_check_steppers_connected())
        {
            result_message = "Calibration Interface - Please ensure that all motors are connected";
            _calibration_in_progress = false;
            return niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
        }
        _auto_calibration();
        _calibration_in_progress = false;
    }
    else if (MANUAL_CALIBRATION == mode)  // manuel
    {
        _calibration_in_progress = true;
        if (!_can_process_manual_calibration(result_message))
        {
            result_message = "Calibration Interface - Can't proceed to manual calibration";
            _calibration_in_progress = false;
            return niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
        }
        if (!_check_steppers_connected())
        {
            result_message = "Calibration Interface - Please ensure that all motors are connected";
            _calibration_in_progress = false;
            return niryo_robot_msgs::CommandStatus::CALIBRATION_NOT_DONE;
        }

        _manual_calibration();
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
 * @brief CalibrationManager::_motorTorque
 * @param motor
 * @param status
 */
void CalibrationManager::_motorTorque(const std::shared_ptr<JointState>& pState, bool status)
{
    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, pState->getId(), {status});
    getProtocolOfMotor(pState->getBusProtocol())->addSingleCommandToQueue(
                                std::make_shared<StepperSingleCmd>(stepper_cmd));
}

/**
 * @brief CalibrationManager::_moveMotor
 * @param motor
 * @param steps
 * @param delay
 */
void CalibrationManager::_moveMotor(const std::shared_ptr<JointState>& pState, int steps, double delay)
{
    _motorTorque(pState, true);

    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION, pState->getId(), {static_cast<int32_t>(steps)});
    getProtocolOfMotor(pState->getBusProtocol())->addSingleCommandToQueue(
                                std::make_shared<StepperSingleCmd>(stepper_cmd));

    ros::Duration(delay).sleep();
}

/**
 * @brief CalibrationManager::_relativeMoveMotor
 * @param motor
 * @param steps
 * @param delay
 * @param wait
 * @return
 */
int CalibrationManager::_relativeMoveMotor(const std::shared_ptr<JointState>& pState, int steps, int delay, bool wait)
{
    _motorTorque(pState, true);

    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, pState->getId(), {steps, delay});
    getProtocolOfMotor(pState->getBusProtocol())->addSingleCommandToQueue(
                                std::make_shared<StepperSingleCmd>(stepper_cmd));

    if (wait)
    {
        ros::Duration(abs(steps * delay / 1000000) + 0.5).sleep();  // wait for 0.5 sec more to finish
    }
    return 1;
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
    uint8_t motor_id = pState->getId();
    int32_t offset = pState->to_motor_pos(pState->getOffsetPosition());
    int32_t motor_direction = static_cast<int32_t>(pState->getDirection());

    // TODO(Thuc) need implement ttl driver for EStepperCommandType::CMD_TYPE_CALIBRATION cmd
    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION, motor_id,
                                {offset, delay, motor_direction * calibration_direction, timeout});
    getProtocolOfMotor(pState->getBusProtocol())->addSingleCommandToQueue(
                                std::make_shared<StepperSingleCmd>(stepper_cmd));

    ROS_INFO("Calibration Interface - start calibration for motor id %d :", motor_id);
}

/**
 * @brief CalibrationManager::_check_steppers_connected
 * @return
 */
bool CalibrationManager::_check_steppers_connected()
{
    for (auto const& jState : _joint_states_list)
    {
        if (jState && jState->isStepper())
        {
            if (!getProtocolOfMotor(jState->getBusProtocol())->scanMotorId(jState->getId()))
                    return false;
        }
    }
    return true;
}

/**
 * @brief CalibrationManager::_auto_calibration
 * @return
 */
EStepperCalibrationStatus CalibrationManager::_auto_calibration()
{
    ros::Duration sld(0.2);

    // 0. Torque ON for motor 2
    if (_joint_states_list.at(1)->getBusProtocol() == EBusProtocol::CAN && _joint_states_list.at(2)->getBusProtocol() == EBusProtocol::CAN)
    {
        StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, _joint_states_list.at(1)->getId(), {true});
        getProtocolOfMotor(_joint_states_list.at(1)->getBusProtocol())->addSingleCommandToQueue(
                                    std::make_shared<StepperSingleCmd>(stepper_cmd));
        sld.sleep();

        // 1. Relative Move Motor 3
        _relativeMoveMotor(_joint_states_list.at(2), _joint_states_list.at(2)->to_motor_pos(0.25), 500, false);
        ros::Duration(0.5).sleep();
    }

    // 2. Move All Dynamixel to Home Position
    if (_ttl_interface)
    {
        common::model::DxlSyncCmd dynamixel_cmd(EDxlCommandType::CMD_TYPE_TORQUE);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(3)->getHardwareType(), _joint_states_list.at(3)->getId(), 1);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(4)->getHardwareType(), _joint_states_list.at(4)->getId(), 1);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(5)->getHardwareType(), _joint_states_list.at(5)->getId(), 1);

        // TODO(CC): only dxl use sync cmd
        _ttl_interface->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dynamixel_cmd));
        sld.sleep();

        dynamixel_cmd.reset();
        dynamixel_cmd.setType(EDxlCommandType::CMD_TYPE_POSITION);

        dynamixel_cmd.addMotorParam(_joint_states_list.at(3)->getHardwareType(),
                                    _joint_states_list.at(3)->getId(),
                                    static_cast<uint32_t>(_joint_states_list.at(3)->to_motor_pos(0)));

        dynamixel_cmd.addMotorParam(_joint_states_list.at(4)->getHardwareType(),
                                    _joint_states_list.at(4)->getId(),
                                    static_cast<uint32_t>(_joint_states_list.at(4)->to_motor_pos(0)));

        dynamixel_cmd.addMotorParam(_joint_states_list.at(5)->getHardwareType(),
                                    _joint_states_list.at(5)->getId(),
                                    static_cast<uint32_t>(_joint_states_list.at(5)->to_motor_pos(0)));

        // TODO(Thuc): only dxl use sync cmd
        _ttl_interface->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dynamixel_cmd));
        sld.sleep();
    }

    // 3. Send calibration cmd 1 + 2 + 3
    if (_can_interface)
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
            _can_interface->startCalibration();

            setStepperCalibrationCommand(pStepperMotorState_1, 200, 1, _calibration_timeout);
            setStepperCalibrationCommand(pStepperMotorState_2, 1000, 1, _calibration_timeout);
            setStepperCalibrationCommand(pStepperMotorState_3, 1000, -1, _calibration_timeout);
        }

        // wait for calibration status done
        sld.sleep();
    }
    else
    {
        // calibration of steppers Ttl
        StepperTtlSingleCmd torque_cmd_1(EStepperCommandType::CMD_TYPE_TORQUE, 2, {0});
        StepperTtlSingleCmd torque_cmd_2(EStepperCommandType::CMD_TYPE_TORQUE, 3, {0});
        StepperTtlSingleCmd torque_cmd_3(EStepperCommandType::CMD_TYPE_TORQUE, 4, {0});

        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(torque_cmd_1));
        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(torque_cmd_2));
        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(torque_cmd_3));

        StepperTtlSingleCmd stepper_dir_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION_DIRECTION, 3, {1});

        StepperTtlSingleCmd stepper_cmd_1(EStepperCommandType::CMD_TYPE_CALIBRATION, 2);
        StepperTtlSingleCmd stepper_cmd_2(EStepperCommandType::CMD_TYPE_CALIBRATION, 3);
        StepperTtlSingleCmd stepper_cmd_3(EStepperCommandType::CMD_TYPE_CALIBRATION, 4);

        _ttl_interface->startCalibration();
         
        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(stepper_dir_cmd));
        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(stepper_cmd_1));
        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(stepper_cmd_2));
        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(stepper_cmd_3));
        
        sld.sleep();
    }

    while ((_can_interface && _can_interface->isCalibrationInProgress()) || (_ttl_interface && _ttl_interface->isCalibrationInProgress()))
    {
        sld.sleep();
    }

    if (_can_interface)
    {
        std::vector<int> sensor_offset_results;
        for (size_t i = 0; i < 3; ++i)
        {
            uint8_t motor_id = _joint_states_list.at(i)->getId();
            int calibration_result = getProtocolOfMotor(_joint_states_list.at(i)->getBusProtocol())->getCalibrationResult(motor_id);
            sensor_offset_results.emplace_back(calibration_result);
            ROS_INFO("Calibration Interface - Motor %d, calibration cmd result %d ", motor_id, calibration_result);
        }

        if (sensor_offset_results.at(0) && sensor_offset_results.at(1) && sensor_offset_results.at(2))
        {
            ROS_INFO("Calibration Interface -  New Calibration values : ");

            ROS_INFO("Calibration Interface -  motor id %d - calibration value %d",
                    _joint_states_list.at(0)->getId(), sensor_offset_results.at(0));

            ROS_INFO("Calibration Interface -  motor id %d - calibration value %d",
                    _joint_states_list.at(1)->getId(), sensor_offset_results.at(1));

            ROS_INFO("Calibration Interface -  motor id %d - calibration value %d",
                    _joint_states_list.at(2)->getId(), sensor_offset_results.at(2));

            std::vector<int> sensor_offset_ids;
            sensor_offset_ids.push_back(_joint_states_list.at(0)->getId());
            sensor_offset_ids.push_back(_joint_states_list.at(1)->getId());
            sensor_offset_ids.push_back(_joint_states_list.at(2)->getId());
            // 4. Move motor 1,2,3 to 0.0
            // -0.01 to bypass error
            sld.sleep();
            _relativeMoveMotor(_joint_states_list.at(0),
                            -_joint_states_list.at(0)->to_motor_pos(_joint_states_list.at(0)->getOffsetPosition()),
                            550,
                            false);

            ros::Duration(2.5).sleep();

            // 6. Write sensor_offset_steps to file
            set_motors_calibration_offsets(sensor_offset_ids, sensor_offset_results);
        }
        else
        {
            ROS_ERROR("Calibration Interface -  An error occured while calibrating stepper motors");
        }
        // forge stepper command
        for (auto const& jState : _joint_states_list)
        {
            if (jState && jState->isStepper())
            {
                if (jState && jState->isStepper())
                {
                    StepperSingleCmd cmd(EStepperCommandType::CMD_TYPE_TORQUE, jState->getId(), {false});
                    getProtocolOfMotor(jState->getBusProtocol())->addSingleCommandToQueue(
                                                std::make_shared<StepperSingleCmd>(cmd));
                }
            }
        }
    }
    else
    {
        // Forge stepper Ttl
        for (auto const& jState : _joint_states_list)
        {
            if (jState && jState->isStepper())
            {
                StepperTtlSingleCmd cmd(EStepperCommandType::CMD_TYPE_TORQUE, jState->getId(), {0});
                getProtocolOfMotor(jState->getBusProtocol())->addSingleCommandToQueue(
                                            std::make_shared<StepperTtlSingleCmd>(cmd));
            }
        }
    }
    // forge dxl command
    if (_ttl_interface)
    {
        common::model::DxlSyncCmd dynamixel_cmd(EDxlCommandType::CMD_TYPE_TORQUE);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(3)->getHardwareType(), _joint_states_list.at(3)->getId(), 0);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(4)->getHardwareType(), _joint_states_list.at(4)->getId(), 0);
        dynamixel_cmd.addMotorParam(_joint_states_list.at(5)->getHardwareType(), _joint_states_list.at(5)->getId(), 0);

        // TODO(Thuc) only dxl use sync cmd
        _ttl_interface->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dynamixel_cmd));
        sld.sleep();
    }

    common::model::EStepperCalibrationStatus  calibration_status = common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
    if (_can_interface)
        calibration_status = _can_interface->getCalibrationStatus();
    else if (_ttl_interface)
        calibration_status = _ttl_interface->getCalibrationStatus();
    return calibration_status;
}

/**
 * @brief CalibrationManager::_can_process_manual_calibration
 * @param result_message
 * @return
 */
bool CalibrationManager::_can_process_manual_calibration(std::string &result_message)
{
    if (_can_interface)
    {
        auto stepper_motor_states = _can_interface->getJointStates();

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
        if (!get_motors_calibration_offsets(motor_id_list, steps_list))
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
 * @brief CalibrationManager::_send_calibration_offset
 * @param id
 * @param offset_to_send
 * @param absolute_steps_at_offset_position
 */
void CalibrationManager::_send_calibration_offset(uint8_t id, int offset_to_send, int absolute_steps_at_offset_position)
{
    // TODO(THUC) not StepperSingleCmd for ttl stepper
    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION_OFFSET, id, {offset_to_send, absolute_steps_at_offset_position});
    if (_can_interface)
        _can_interface->addSingleCommandToQueue(
                        std::make_shared<StepperSingleCmd>(stepper_cmd));
    else if (_ttl_interface)
        _ttl_interface->addSingleCommandToQueue(
                        std::make_shared<StepperSingleCmd>(stepper_cmd));
    // (Thuc) call add single command to Queue by if else but not
    // by polymorphism make the program will work for the case all stepper use can or ttl,
    // not 2 protocol in the same time
}

/**
 * @brief CalibrationManager::_manual_calibration
 * @return
 */
EStepperCalibrationStatus CalibrationManager::_manual_calibration()
{
    ros::Rate rest(0.5);
    ros::Duration sld(0.2);
    std::vector<int> motor_id_list;
    std::vector<int> steps_list;

    if (!get_motors_calibration_offsets(motor_id_list, steps_list))
    {
       return EStepperCalibrationStatus::CALIBRATION_FAIL;
    }
    if (_can_interface)
        _can_interface->startCalibration();
    else if (_ttl_interface)
        _ttl_interface->startCalibration();

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

              _send_calibration_offset(_joint_states_list.at(0)->getId(), offset_to_send, absolute_steps_at_offset_position);
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

              _send_calibration_offset(_joint_states_list.at(1)->getId(), offset_to_send, absolute_steps_at_offset_position);
              sld.sleep();
          }
          else if (motor_id_list.at(i) == _joint_states_list.at(2)->getId())
          {
              offset_to_send = sensor_offset_steps - _joint_states_list.at(2)->to_motor_pos(_joint_states_list.at(2)->getOffsetPosition());
              absolute_steps_at_offset_position = sensor_offset_steps;

              _send_calibration_offset(_joint_states_list.at(2)->getId(), offset_to_send, absolute_steps_at_offset_position);
              sld.sleep();
          }
      }
      if (_can_interface)
          return _can_interface->getCalibrationStatus();
      else if (_ttl_interface)
          return _ttl_interface->getCalibrationStatus();

    }

    return EStepperCalibrationStatus::CALIBRATION_FAIL;
}

/**
 * @brief CalibrationManager::get_motors_calibration_offsets
 * @param motor_id_list
 * @param steps_list
 * @return
 */
bool CalibrationManager::get_motors_calibration_offsets(std::vector<int> &motor_id_list, std::vector<int> &steps_list)
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
 * @brief CalibrationManager::set_motors_calibration_offsets
 * @param motor_id_list
 * @param steps_list
 * @return
 */
bool CalibrationManager::set_motors_calibration_offsets(const std::vector<int> &motor_id_list,
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
