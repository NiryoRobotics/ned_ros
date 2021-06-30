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
using ::common::model::StepperMotorCmd;
using ::common::model::StepperMotorState;
using ::common::model::EStepperCalibrationStatus;
using ::common::model::SynchronizeMotorCmd;
using ::common::model::EDxlCommandType;

namespace joints_interface
{

/**
 * @brief CalibrationManager::CalibrationManager
 * @param joint_list
 * @param can_driver
 * @param ttl_driver
 */
CalibrationManager::CalibrationManager(ros::NodeHandle &nh, std::vector<std::shared_ptr<JointState> > joint_list,
                                       std::shared_ptr<can_driver::CanDriverCore> can_driver,
                                       std::shared_ptr<ttl_driver::TtlDriverCore> ttl_driver) :
    _can_driver_core(can_driver),
    _ttl_driver_core(ttl_driver),
    _joint_list(joint_list)
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
 */
void CalibrationManager::initParameters(ros::NodeHandle& nh)
{
    nh.getParam("calibration_timeout", _calibration_timeout);
    nh.getParam("calibration_file", _calibration_file_name);

    ROS_DEBUG("Calibration Interface - Calibration timeout %d", _calibration_timeout);
    ROS_DEBUG("Calibration Interface - Calibration file name %s", _calibration_file_name.c_str());
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
void CalibrationManager::_motorTorque(const std::shared_ptr<JointState>& motor, bool status)
{
    StepperMotorCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, motor->getId(), {status});
    _can_driver_core->addSingleCommandToQueue(stepper_cmd);
}

/**
 * @brief CalibrationManager::_moveMotor
 * @param motor
 * @param steps
 * @param delay
 */
void CalibrationManager::_moveMotor(const std::shared_ptr<JointState>& motor, int steps, double delay)
{
    _motorTorque(motor, true);

    StepperMotorCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION, motor->getId(), {steps});
    _can_driver_core->addSingleCommandToQueue(stepper_cmd);

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
int CalibrationManager::_relativeMoveMotor(const std::shared_ptr<JointState>& motor, int steps, int delay, bool wait)
{
    _motorTorque(motor, true);

    StepperMotorCmd stepper_cmd(EStepperCommandType::CMD_TYPE_RELATIVE_MOVE, motor->getId(), {steps, delay});
    _can_driver_core->addSingleCommandToQueue(stepper_cmd);

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

    StepperMotorCmd stepper_cmd(EStepperCommandType::CMD_TYPE_CALIBRATION, motor_id,
                                {offset, delay, motor_direction * calibration_direction, timeout});
    _can_driver_core->addSingleCommandToQueue(stepper_cmd);

    ROS_INFO("Calibration Interface - start calibration for motor id %d :", motor_id);
}

/**
 * @brief CalibrationManager::_check_steppers_connected
 * @return
 */
bool CalibrationManager::_check_steppers_connected()
{
    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isStepper())
        {
            if (!_can_driver_core->scanMotorId(jState->getId()))
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

    StepperMotorCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE, _joint_list.at(1)->getId(), {true});
    _can_driver_core->addSingleCommandToQueue(stepper_cmd);
    sld.sleep();

    // 1. Relative Move Motor 3
    _relativeMoveMotor(_joint_list.at(2), _joint_list.at(2)->to_motor_pos(0.25), 500, false);
    // _relativeMoveMotor(_joint_list.at(2), rad_pos_to_steps(0.25, _gear_ratio_3, _direction_3), 500, false);
    ros::Duration(0.5).sleep();

    // 2. Move All Dynamixel to Home Position
    SynchronizeMotorCmd dynamixel_cmd;
    dynamixel_cmd.setType(EDxlCommandType::CMD_TYPE_TORQUE);
    dynamixel_cmd.addMotorParam(_joint_list.at(3)->getType(), _joint_list.at(3)->getId(), 1);
    dynamixel_cmd.addMotorParam(_joint_list.at(4)->getType(), _joint_list.at(4)->getId(), 1);
    dynamixel_cmd.addMotorParam(_joint_list.at(5)->getType(), _joint_list.at(5)->getId(), 1);

    _ttl_driver_core->setSyncCommand(dynamixel_cmd);
    sld.sleep();

    dynamixel_cmd.reset();
    dynamixel_cmd.setType(EDxlCommandType::CMD_TYPE_POSITION);

    dynamixel_cmd.addMotorParam(_joint_list.at(3)->getType(),
                                _joint_list.at(3)->getId(),
                                static_cast<uint32_t>(_joint_list.at(3)->to_motor_pos(0)));

    dynamixel_cmd.addMotorParam(_joint_list.at(4)->getType(),
                                _joint_list.at(4)->getId(),
                                static_cast<uint32_t>(_joint_list.at(4)->to_motor_pos(0)));

    dynamixel_cmd.addMotorParam(_joint_list.at(5)->getType(),
                                _joint_list.at(5)->getId(),
                                static_cast<uint32_t>(_joint_list.at(5)->to_motor_pos(0)));

    _ttl_driver_core->setSyncCommand(dynamixel_cmd);
    sld.sleep();


    // 3. Send calibration cmd 1 + 2 + 3

    std::vector<int> sensor_offset_results;

    std::shared_ptr<StepperMotorState> pStepperMotorState_1 =
            std::dynamic_pointer_cast<StepperMotorState>(_joint_list.at(0));

    std::shared_ptr<StepperMotorState> pStepperMotorState_2 =
            std::dynamic_pointer_cast<StepperMotorState>(_joint_list.at(1));

    std::shared_ptr<StepperMotorState> pStepperMotorState_3 =
            std::dynamic_pointer_cast<StepperMotorState>(_joint_list.at(2));

    if (pStepperMotorState_1 && pStepperMotorState_1->isValid() &&
            pStepperMotorState_2 && pStepperMotorState_2->isValid() &&
            pStepperMotorState_3 && pStepperMotorState_3->isValid())
    {
        _can_driver_core->startCalibration();

        setStepperCalibrationCommand(pStepperMotorState_1, 200, 1, _calibration_timeout);
        setStepperCalibrationCommand(pStepperMotorState_2, 1000, 1, _calibration_timeout);
        setStepperCalibrationCommand(pStepperMotorState_3, 1000, -1, _calibration_timeout);
    }

    // wait for calibration status done
    sld.sleep();
    while (_can_driver_core->isCalibrationInProgress())
    {
        sld.sleep();
    }

    for (size_t i = 0; i < 3; ++i)
    {
        uint8_t motor_id = _joint_list.at(i)->getId();
        int calibration_result = _can_driver_core->getCalibrationResult(motor_id);
        sensor_offset_results.emplace_back(calibration_result);
        ROS_INFO("Calibration Interface - Motor %d, calibration cmd result %d ", motor_id, calibration_result);
    }

    if (sensor_offset_results.at(0) && sensor_offset_results.at(1) && sensor_offset_results.at(2))
    {
        ROS_INFO("Calibration Interface -  New Calibration values : ");

        ROS_INFO("Calibration Interface -  motor id %d - calibration value %d",
                 _joint_list.at(0)->getId(), sensor_offset_results.at(0));

        ROS_INFO("Calibration Interface -  motor id %d - calibration value %d",
                 _joint_list.at(1)->getId(), sensor_offset_results.at(1));

        ROS_INFO("Calibration Interface -  motor id %d - calibration value %d",
                 _joint_list.at(2)->getId(), sensor_offset_results.at(2));

        std::vector<int> sensor_offset_ids;
        sensor_offset_ids.push_back(_joint_list.at(0)->getId());
        sensor_offset_ids.push_back(_joint_list.at(1)->getId());
        sensor_offset_ids.push_back(_joint_list.at(2)->getId());

        // 4. Move motor 1,2,3 to 0.0
        // -0.01 to bypass error
        sld.sleep();
        _relativeMoveMotor(_joint_list.at(0),
                           -_joint_list.at(0)->to_motor_pos(_joint_list.at(0)->getOffsetPosition()),
                           550,
                           false);

        ros::Duration(2.5).sleep();

        // forge stepper command
        for (auto const& jState : _joint_list)
        {
            if (jState && jState->isStepper())
            {
                StepperMotorCmd cmd(EStepperCommandType::CMD_TYPE_TORQUE, jState->getId(), {false});
                _can_driver_core->addSingleCommandToQueue(cmd);
            }
        }

        // forge dxl command
        dynamixel_cmd.reset();
        dynamixel_cmd.setType(EDxlCommandType::CMD_TYPE_TORQUE);
        dynamixel_cmd.addMotorParam(_joint_list.at(3)->getType(), _joint_list.at(3)->getId(), 0);
        dynamixel_cmd.addMotorParam(_joint_list.at(4)->getType(), _joint_list.at(4)->getId(), 0);
        dynamixel_cmd.addMotorParam(_joint_list.at(5)->getType(), _joint_list.at(5)->getId(), 0);

        _ttl_driver_core->setSyncCommand(dynamixel_cmd);
        sld.sleep();

        // 6. Write sensor_offset_steps to file
        set_motors_calibration_offsets(sensor_offset_ids, sensor_offset_results);
    }
    else
    {
        ROS_ERROR("Calibration Interface -  An error occured while calibrating stepper motors");
    }

    return _can_driver_core->getCalibrationStatus();
}

/**
 * @brief CalibrationManager::_can_process_manual_calibration
 * @param result_message
 * @return
 */
bool CalibrationManager::_can_process_manual_calibration(std::string &result_message)
{
    auto stepper_motor_states = _can_driver_core->getStepperStates();

    // 1. Check if motors firmware version is ok
    for (auto const& mState : stepper_motor_states)
    {
        if (mState)
        {
            std::string firmware_version = mState->getFirmwareVersion();
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
        result_message = "Calibration Interface - You need to make one auto calibration before using the manual calibration";
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
    StepperMotorCmd stepper_cmd(EStepperCommandType::CMD_TYPE_POSITION_OFFSET, id, {offset_to_send, absolute_steps_at_offset_position});
    _can_driver_core->addSingleCommandToQueue(stepper_cmd);
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
    _can_driver_core->startCalibration();
    // 0. Torque ON for motor 2

    int steps_per_rev = common::model::StepperMotorState::stepsPerRev();

    for (size_t i = 0; i < motor_id_list.size(); i++)
    {
        int offset_to_send = 0;
        int sensor_offset_steps = steps_list.at(i);
        int absolute_steps_at_offset_position = 0;

        if (motor_id_list.at(i) == _joint_list.at(0)->getId())
        {
            offset_to_send = sensor_offset_steps - _joint_list.at(0)->to_motor_pos(_joint_list.at(0)->getOffsetPosition()) % steps_per_rev;
            if (offset_to_send < 0)
                offset_to_send += steps_per_rev;
            absolute_steps_at_offset_position = offset_to_send;

            _send_calibration_offset(_joint_list.at(0)->getId(), offset_to_send, absolute_steps_at_offset_position);
            sld.sleep();
        }
        else if (motor_id_list.at(i) == _joint_list.at(1)->getId())
        {
            offset_to_send = sensor_offset_steps - _joint_list.at(1)->to_motor_pos(_joint_list.at(1)->getOffsetPosition());
            absolute_steps_at_offset_position = sensor_offset_steps;

            _send_calibration_offset(_joint_list.at(1)->getId(), offset_to_send, absolute_steps_at_offset_position);
            sld.sleep();
        }
        else if (motor_id_list.at(i) == _joint_list.at(2)->getId())
        {
            offset_to_send = sensor_offset_steps - _joint_list.at(2)->to_motor_pos(_joint_list.at(2)->getOffsetPosition());
            absolute_steps_at_offset_position = sensor_offset_steps;

            _send_calibration_offset(_joint_list.at(2)->getId(), offset_to_send, absolute_steps_at_offset_position);
            sld.sleep();
        }
    }

    return _can_driver_core->getCalibrationStatus();
}

/**
 * @brief CalibrationManager::get_motors_calibration_offsets
 * @param motor_id_list
 * @param steps_list
 * @return
 */
bool CalibrationManager::get_motors_calibration_offsets(std::vector<int> &motor_id_list, std::vector<int> &steps_list)
{
    std::string file_name;
    ros::param::get("/niryo_robot_hardware_interface/calibration_file", file_name);

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
