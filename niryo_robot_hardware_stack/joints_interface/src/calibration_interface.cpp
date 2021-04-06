// /*
//     calibration_interface.cpp
//     Copyright (C) 2020 Niryo
//     All rights reserved.

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <http://www.gnu.org/licenses/>.
// */

#include "joints_interface/calibration_interface.hpp"
#include "joints_interface/Math.hpp"
#include <ros/console.h>
#include <functional>
#include <boost/filesystem.hpp>

using namespace std;

namespace JointsInterface {

    CalibrationInterface::CalibrationInterface(vector<JointState> &joint_list,
                                               shared_ptr<StepperDriver::StepperDriverCore> &stepper,
                                               shared_ptr<DynamixelDriver::DynamixelDriverCore> &dynamixel) :
        _joint_list(joint_list), _stepper(stepper), _dynamixel(dynamixel)
    {

        _nh.getParam("/niryo_robot_hardware_interface/calibration_timeout", _calibration_timeout);
        ROS_DEBUG("Calibration Interface - Calibration timeout %d", _calibration_timeout);

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_gear_ratio", _gear_ratio_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_gear_ratio", _gear_ratio_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_gear_ratio", _gear_ratio_3);
        ROS_DEBUG("Calibration Interface - Gear ratios : (1 : %lf, 2 : %lf, 3 : %lf)", _gear_ratio_1, _gear_ratio_2, _gear_ratio_3);

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_offset_position", _offset_position_stepper_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_offset_position", _offset_position_stepper_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_offset_position", _offset_position_stepper_3);
        ROS_DEBUG("Calibration Interface - Angle offsets steppers: (1 : %lf, 2 : %lf, 3 : %lf)", _offset_position_stepper_1, _offset_position_stepper_2, _offset_position_stepper_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_1_offset_position", _offset_position_dxl_1);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_2_offset_position", _offset_position_dxl_2);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_offset_position", _offset_position_dxl_3);
        ROS_DEBUG("Calibration Interface - Angle offsets dxl: (1 : %lf, 2 : %lf, 3 : %lf)", _offset_position_dxl_1, _offset_position_dxl_2, _offset_position_dxl_3);

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_direction", _direction_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_direction", _direction_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_direction", _direction_3);
        ROS_DEBUG("Calibration Interface - Direction : (1 : %lf, 2 : %lf, 3 : %lf)", _direction_1, _direction_2, _direction_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/xl430_middle_position", _xl430_middle_position);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/xc430_middle_position", _xc430_middle_position);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/xl320_middle_position", _xl320_middle_position);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/xl330_middle_position", _xl330_middle_position);

        ROS_DEBUG("Calibration Interface - (xl430 middle position %d, xc430 middle position %d, xl320 middle position %d, xl330 middle position %d)",
                                        _xl430_middle_position, _xc430_middle_position, _xl320_middle_position, _xl330_middle_position);

        _motor_calibration_list.clear();
        _calibration_in_progress = false;

        ROS_INFO("Calibration Interface - Calibration interface started");
    }

    bool CalibrationInterface::CalibrationInprogress()
    {
        return _calibration_in_progress;
    }

    int CalibrationInterface::startCalibration(int mode, string &result_message)
    {
        if (mode == 1) // auto
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
        else if (mode == 2) // manuel
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

    void CalibrationInterface::_motorTorque(JointState &motor, bool status)
    {
        StepperDriver::StepperMotorCmd stepper_cmd;

        stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);
        vector<uint8_t> id{motor.getId()};
        stepper_cmd.setMotorsId(id);
        vector<int32_t> param{status};
        stepper_cmd.setParams(param);

        _stepper->setStepperCommands(stepper_cmd);
        ros::Duration(0.2).sleep();
    }

    void CalibrationInterface::_moveMotor(JointState &motor, int steps, float delay)
    {
        _motorTorque(motor, true);

        StepperDriver::StepperMotorCmd stepper_cmd;
        stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_POSITION);

        vector<uint8_t> id{motor.getId()};
        stepper_cmd.setMotorsId(id);

        vector<int32_t> param{steps};
        stepper_cmd.setParams(param);

        _stepper->setStepperCommands(stepper_cmd);

        ros::Duration(delay).sleep();
    }

    int CalibrationInterface::_relativeMoveMotor(JointState &motor, int steps, int delay, bool wait)
    {
        _motorTorque(motor, true);

        StepperDriver::StepperMotorCmd stepper_cmd;
        stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_RELATIVE_MOVE);
        vector<int32_t> param{steps, delay};
        stepper_cmd.setParams(param);
        vector<uint8_t> id{motor.getId()};
        stepper_cmd.setMotorsId(id);

        _stepper->setStepperCommands(stepper_cmd);
        param.clear();
        if (wait)
        {
            ros::Duration(abs(steps * delay / 1000000) + 0.5).sleep(); // wait for 0.5 sec more to finish
        }
        return 1;
    }

    void CalibrationInterface::_setCalibrationCommand(
        JointState motor, int offset, int delay, int motor_direction, int calibration_direction, int timeout,
        int32_t& calibration_result)
    {
        StepperDriver::StepperMotorCmd stepper_cmd;

        stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_CALIBRATION);
        vector<uint8_t> id{motor.getId()};
        stepper_cmd.setMotorsId(id);

        vector<int32_t> params{offset, delay, motor_direction * calibration_direction, timeout};
        stepper_cmd.setParams(params);

        _stepper->setStepperCommands(stepper_cmd);

        ROS_INFO("Calibration Interface - Wait for calibration result for motor id %d :", id[0]);
        ros::Duration(0.2).sleep();
        while (_stepper->getCalibrationResult(id[0], calibration_result) == StepperDriver::e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_IN_PROGRESS)
        {
            ros::Duration(0.2).sleep();
        }
        ROS_INFO("Calibration Interface - Motor %d, calibration cmd result %d ", id[0], calibration_result);
    }

    bool CalibrationInterface::_check_steppers_connected()
    {
        for (int stepper_id = 0; stepper_id < 3; stepper_id++)
        {
            if (!_stepper->scanMotorId(_joint_list.at(stepper_id).getId()))
                return false;
        }
        return true;
    }

    void CalibrationInterface::_auto_calibration()
    {
        _stepper->startCalibration(true);
        // 0. Torque ON for motor 2

        ros::Duration sld(0.2);
        sld.sleep();
        StepperDriver::StepperMotorCmd stepper_cmd;

        stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);

        vector<uint8_t> id{_joint_list.at(1).getId()};
        stepper_cmd.setMotorsId(id);
        vector<int32_t> stepper_param{true};
        stepper_cmd.setParams(stepper_param);
        _stepper->setStepperCommands(stepper_cmd);
        sld.sleep();

        // 1. Relative Move Motor 3
        _relativeMoveMotor(_joint_list.at(2), rad_pos_to_steps(0.25, _gear_ratio_3, _direction_3), 500, false);
        ros::Duration(0.5).sleep();

        // 2. Move All Dynamixel to Home Position
        DynamixelDriver::SynchronizeMotorCmd dynamixel_cmd;

        dynamixel_cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_TORQUE);
        id.clear();
        id = {
            _joint_list.at(3).getId(),
            _joint_list.at(4).getId(),
            _joint_list.at(5).getId()};
        dynamixel_cmd.setMotorsId(id);
        vector<uint32_t> dxl_param{1, 1, 1};
        dynamixel_cmd.setParams(dxl_param);
        _dynamixel->setDxlCommands(dynamixel_cmd);
        sld.sleep();

        // CC to be generalized
        dxl_param = {
            rad_pos_to_xc430_pos(-_offset_position_dxl_1),
            rad_pos_to_xc430_pos(-_offset_position_dxl_2),
            rad_pos_to_xl330_pos(-_offset_position_dxl_3)};
        _dynamixel->setTrajectoryControllerCommands(dxl_param);
        sld.sleep();

        _joint_list.at(3).setNeedCalibration(false);
        _joint_list.at(4).setNeedCalibration(false);
        _joint_list.at(5).setNeedCalibration(false);

        // 3. Send calibration cmd 1 + 2 + 3

        int32_t stepper_1_calibration_result{0};
        int32_t stepper_2_calibration_result{0};
        int32_t stepper_3_calibration_result{0};

        //_stepper->setCalibrationResult(_joint_list.at(0).getId(), 0);
        thread stepper_1_calibration_thread = thread(&CalibrationInterface::_setCalibrationCommand, this, _joint_list.at(0),
                                                                rad_pos_to_steps(_offset_position_stepper_1, _gear_ratio_1, _direction_1),
                                                                200, _direction_1, 1, _calibration_timeout, std::ref(stepper_1_calibration_result));
        sld.sleep();
        thread stepper_2_calibration_thread = thread(&CalibrationInterface::_setCalibrationCommand, this, _joint_list.at(1),
                                                                rad_pos_to_steps(_offset_position_stepper_2, _gear_ratio_2, _direction_2),
                                                                1000, _direction_2, 1, _calibration_timeout, std::ref(stepper_2_calibration_result));
        sld.sleep();
        thread stepper_3_calibration_thread = thread(&CalibrationInterface::_setCalibrationCommand, this, _joint_list.at(2),
                                                                rad_pos_to_steps(_offset_position_stepper_3, _gear_ratio_3, _direction_3),
                                                                1000, _direction_3, -1, _calibration_timeout, std::ref(stepper_3_calibration_result));
        sld.sleep();

        stepper_1_calibration_thread.join();
        stepper_2_calibration_thread.join();
        stepper_3_calibration_thread.join();

        if (! stepper_1_calibration_result
                || ! stepper_2_calibration_result
                || ! stepper_3_calibration_result)
        {
            ROS_ERROR("Calibration Interface -  An error occured while calibrating stepper motors");
            return;
        }
        ROS_INFO("Calibration Interface -  New Calibration values : ");
        ROS_INFO("Calibration Interface -  motor id %d - calibration value %d", _joint_list.at(0).getId(), stepper_1_calibration_result);
        ROS_INFO("Calibration Interface -  motor id %d - calibration value %d", _joint_list.at(1).getId(), stepper_2_calibration_result);
        ROS_INFO("Calibration Interface -  motor id %d - calibration value %d", _joint_list.at(2).getId(), stepper_3_calibration_result);

        vector<int> sensor_offset_ids;
        sensor_offset_ids.push_back(_joint_list.at(0).getId());
        sensor_offset_ids.push_back(_joint_list.at(1).getId());
        sensor_offset_ids.push_back(_joint_list.at(2).getId());

        vector<int> sensor_offset_steps;
        sensor_offset_steps.push_back(stepper_1_calibration_result);
        sensor_offset_steps.push_back(stepper_2_calibration_result);
        sensor_offset_steps.push_back(stepper_3_calibration_result);

        // 4. Move motor 1,2,3 to 0.0
        // -0.01 to bypass error
        sld.sleep();
        _relativeMoveMotor(_joint_list.at(0), -rad_pos_to_steps(_offset_position_stepper_1, _gear_ratio_1, _direction_1), 550, false);
        ros::Duration(2.5).sleep();


        dynamixel_cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_TORQUE);
        stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);

        dynamixel_cmd.setMotorsId(vector<uint8_t>{
            _joint_list.at(3).getId(),
            _joint_list.at(4).getId(),
            _joint_list.at(5).getId()});

        stepper_cmd.setMotorsId(vector<uint8_t>{
            _joint_list.at(0).getId(),
            _joint_list.at(1).getId(),
            _joint_list.at(2).getId()});

        vector<int32_t> stepper_params{0, 0, 0};
        vector<uint32_t> dxl_params{0, 0, 0};

        dynamixel_cmd.setParams(dxl_params);
        stepper_cmd.setParams(stepper_params);

        _stepper->setStepperCommands(stepper_cmd);
        _dynamixel->setDxlCommands(dynamixel_cmd);
        sld.sleep();

        // 6. Write sensor_offset_steps to file
        set_motors_calibration_offsets(sensor_offset_ids, sensor_offset_steps);

        _stepper->startCalibration(false);
        _joint_list.at(0).setNeedCalibration(false);
        _joint_list.at(1).setNeedCalibration(false);
        _joint_list.at(2).setNeedCalibration(false);
    }

    bool CalibrationInterface::_can_process_manual_calibration(string &result_message)
    {
        vector<StepperDriver::StepperMotorState> stepper_motor_states;

        stepper_motor_states = _stepper->getStepperStates();

        // 1. Check if motors firmware version is ok
        for (int i = 0; i < stepper_motor_states.size(); i++)
        {
            string firmware_version = stepper_motor_states.at(i).getFirmwareVersion();
            if (firmware_version.length() == 0)
            {
                result_message = "Calibration Interface - No firmware version available for motor " + to_string(stepper_motor_states.at(i).getId()) + ". Make sure all motors are connected";
                ROS_WARN("Calibration Interface - Can't process manual calibration : %s", result_message.c_str());
                return false;
            }
            if (stoi(firmware_version.substr(0, 1)) < 2)
            {
                result_message = "Calibration Interface - You need to upgrade stepper firmware for motor " + to_string(stepper_motor_states.at(i).getId());
                ROS_WARN("Calibration Interface - Can't process manual calibration : %s", result_message.c_str());
                return false;
            }
        }

        // 2. Check if motor offset values have been previously saved (with auto calibration)
        vector<int> motor_id_list;
        vector<int> steps_list;
        if (!get_motors_calibration_offsets(motor_id_list, steps_list))
        {
            result_message = "Calibration Interface - You need to make one auto calibration before using the manual calibration";
            ROS_WARN("Calibration Interface - Can't process manual calibration : %s", result_message.c_str());
            return false;
        }

        // 3. Check if all connected motors have a motor offset value
        for (int i = 0; i < stepper_motor_states.size(); i++)
        {
            for (int j = 0; j < motor_id_list.size(); j++)
            {
                if (motor_id_list.at(j) == stepper_motor_states.at(i).getId())
                {
                    break;
                }
                if (j == motor_id_list.size() - 1)
                {
                    result_message = "Calibration Interface - Motor " + to_string(stepper_motor_states.at(i).getId()) + " does not have a saved offset value, " + "you need to do one auto calibration";
                    ROS_WARN("Calibration Interface - Can't process manual calibration : %s", result_message.c_str());
                    return false;
                }
            }
        }

        return true;
    }

    void CalibrationInterface::_send_calibration_offset(uint8_t id, int offset_to_send, int absolute_steps_at_offset_position)
    {
        StepperDriver::StepperMotorCmd stepper_cmd;

        stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_POSITION_OFFSET);

        vector<uint8_t> id_cmd = {id};
        vector<int32_t> params_cmd = {offset_to_send, absolute_steps_at_offset_position};

        stepper_cmd.setMotorsId(id_cmd);
        stepper_cmd.setParams(params_cmd);
        _stepper->setStepperCommands(stepper_cmd);
        // rest.sleep();
        id_cmd.clear();
        params_cmd.clear();
    }

    int CalibrationInterface::_manual_calibration()
    {
        ros::Rate rest(0.5);
        ros::Duration sld(0.2);
        vector<int> motor_id_list;
        vector<int> steps_list;

        if (!get_motors_calibration_offsets(motor_id_list, steps_list))
        {
            return static_cast<int>(StepperDriver::e_CanStepperCalibrationStatus::CAN_STEPPERS_CALIBRATION_FAIL);
        }
        _stepper->startCalibration(true);
        // 0. Torque ON for motor 2

        int steps_per_rev = int(STEPPERS_MICROSTEPS * STEPPERS_MOTOR_STEPS_PER_REVOLUTION);

        for (int i = 0; i < motor_id_list.size(); i++)
        {
            int offset_to_send = 0;
            int sensor_offset_steps = steps_list.at(i);
            int absolute_steps_at_offset_position = 0;

            if (motor_id_list.at(i) == _joint_list.at(0).getId())
            {
                offset_to_send = (sensor_offset_steps - rad_pos_to_steps(_offset_position_stepper_1, _gear_ratio_1, _direction_1)) % steps_per_rev;
                if (offset_to_send < 0)
                    offset_to_send += steps_per_rev;
                absolute_steps_at_offset_position = offset_to_send;

                _send_calibration_offset(_joint_list.at(0).getId(), offset_to_send, absolute_steps_at_offset_position);
                _joint_list.at(0).setNeedCalibration(false);
                sld.sleep();
            }

            else if (motor_id_list.at(i) == _joint_list.at(1).getId())
            {
                offset_to_send = (sensor_offset_steps - rad_pos_to_steps(_offset_position_stepper_2, _gear_ratio_2, _direction_2));
                absolute_steps_at_offset_position = sensor_offset_steps;

                _send_calibration_offset(_joint_list.at(1).getId(), offset_to_send, absolute_steps_at_offset_position);
                _joint_list.at(1).setNeedCalibration(false);
                sld.sleep();
            }

            else if (motor_id_list.at(i) == _joint_list.at(2).getId())
            {
                offset_to_send = sensor_offset_steps - rad_pos_to_steps(_offset_position_stepper_3, _gear_ratio_3, _direction_3);
                absolute_steps_at_offset_position = sensor_offset_steps;

                _send_calibration_offset(_joint_list.at(2).getId(), offset_to_send, absolute_steps_at_offset_position);
                _joint_list.at(2).setNeedCalibration(false);
                sld.sleep();
            }
        }
        _stepper->startCalibration(false);
    }


    bool CalibrationInterface::get_motors_calibration_offsets(vector<int> &motor_id_list, vector<int> &steps_list)
    {
        string file_name;
        ros::param::get("/niryo_robot_hardware_interface/calibration_file", file_name);

        vector<string> lines;
        string current_line;

        ifstream offset_file(file_name.c_str());
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
                catch (exception &e)
                {
                }
            }
            offset_file.close();
        }
        else
        {
            ROS_WARN("Motor Offset - Unable to open file : %s", file_name.c_str());
            return false;
        }
        return true;
    }

    bool CalibrationInterface::set_motors_calibration_offsets(vector<int> &motor_id_list, vector<int> &steps_list)
    {
        if (motor_id_list.size() != steps_list.size())
        {
            return false;
        }
        string file_name;
        ros::param::get("/niryo_robot_hardware_interface/calibration_file", file_name);

        size_t found = file_name.find_last_of("/");
        string folder_name = file_name.substr(0, found);

        boost::filesystem::path filepath(file_name);
        boost::filesystem::path directory(folder_name);

        // Create dir if not exist
        boost::system::error_code returned_error;
        boost::filesystem::create_directories(directory, returned_error);
        if (returned_error)
        {
            ROS_WARN("Motor Offset - Could not create directory : %s", folder_name.c_str());
            return false;
        }

        // Create text to write
        string text_to_write = "";
        for (int i = 0; i < motor_id_list.size(); i++)
        {
            text_to_write += to_string(motor_id_list.at(i));
            text_to_write += ":";
            text_to_write += to_string(steps_list.at(i));
            if (i < motor_id_list.size() - 1)
            {
                text_to_write += "\n";
            }
        }

        // Write to file
        ofstream offset_file(file_name.c_str());
        if (offset_file.is_open())
        {
            ROS_DEBUG("Motor Offset - Writing calibration offsets to file : \n%s", text_to_write.c_str());
            offset_file << text_to_write.c_str();
            offset_file.close();
        }
        else
        {
            ROS_WARN("Motor Offset - Unable to open file : %s", file_name.c_str());
            return false;
        }

        return true;
    }
} // JointsInterface
