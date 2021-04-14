/*
    JointHardwareInterface.cpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "joints_interface/JointHardwareInterface.hpp"
#include "dynamixel_driver/SendCustomDxlValue.h"

#include "model/motor_type_enum.hpp"
#include "util/util_defs.hpp"

using namespace std;
using namespace common::model;

namespace JointsInterface {

    JointHardwareInterface::JointHardwareInterface(std::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel,
                                                   std::shared_ptr<StepperDriver::StepperDriverCore> stepper) :
        _dynamixel(dynamixel),
        _stepper(stepper),
        _nb_joints(0),
        _learning_mode(true)
    {

        initParameters();

        initJoints();

        initMotors();

        _calibration_interface.reset(new CalibrationInterface(_joint_list, _stepper, _dynamixel));
    }


    void JointHardwareInterface::initParameters()
    {
        _gear_ratio_1 = {0.0}; _gear_ratio_2 = {0.0}; _gear_ratio_3 = {0.0};
        _home_position_1 = {0.0}; _home_position_2 = {0.0}; _home_position_3 = {0.0};
        _offset_position_stepper_1 = {0.0}; _offset_position_stepper_2 = {0.0}; _offset_position_stepper_3 = {0.0};
        _offset_position_dxl_1 = {0.0}; _offset_position_dxl_2 = {0.0}; _offset_position_dxl_3 = {0.0};
        _direction_1 = {0.0}; _direction_2 = {0.0}; _direction_3 = {0.0};
        _max_effort_1 = {0}; _max_effort_2 = {0}; _max_effort_3 = {0};

        _p_gain_1 = {0}; _p_gain_2 = {0}; _p_gain_3 = {0};
        _i_gain_1 = {0}; _i_gain_2 = {0}; _i_gain_3 = {0};
        _d_gain_1 = {0}; _d_gain_2 = {0}; _d_gain_3 = {0};
        _ff1_gain_1 = {0}; _ff1_gain_2 = {0}; _ff1_gain_3 = {0};
        _ff2_gain_1 = {0}; _ff2_gain_2 = {0}; _ff2_gain_3 = {0};

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_gear_ratio", _gear_ratio_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_gear_ratio", _gear_ratio_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_gear_ratio", _gear_ratio_3);
        ROS_DEBUG("Joints Hardware Interface - Joint Hardware Interface - Gear ratios : (1 : %lf, 2 : %lf, 3 : %lf)",
                  _gear_ratio_1, _gear_ratio_2, _gear_ratio_3);

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_home_position", _home_position_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_home_position", _home_position_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_home_position", _home_position_3);
        ROS_DEBUG("Joints Hardware Interface - Home positions : (1 : %lf, 2 : %lf, 3 : %lf)",
                  _home_position_1, _home_position_2, _home_position_3);

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_offset_position", _offset_position_stepper_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_offset_position", _offset_position_stepper_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_offset_position", _offset_position_stepper_3);
        ROS_DEBUG("Joints Hardware Interface - Angle offsets steppers: (1 : %lf, 2 : %lf, 3 : %lf)",
                  _offset_position_stepper_1, _offset_position_stepper_2, _offset_position_stepper_3);

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_direction", _direction_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_direction", _direction_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_direction", _direction_3);
        ROS_DEBUG("Joints Hardware Interface - Direction : (1 : %lf, 2 : %lf, 3 : %lf)",
                  _direction_1, _direction_2, _direction_3);

        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_max_effort", _max_effort_1);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_max_effort", _max_effort_2);
        _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_max_effort", _max_effort_3);
        ROS_DEBUG("Joints Hardware Interface - Max effort : (1 : %d, 2 : %d, 3 : %d)",
                  _max_effort_1, _max_effort_2, _max_effort_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_1_offset_position", _offset_position_dxl_1);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_2_offset_position", _offset_position_dxl_2);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_offset_position", _offset_position_dxl_3);
        ROS_DEBUG("Joints Hardware Interface - Angle offsets dxl: (1 : %lf, 2 : %lf, 3 : %lf)",
                  _offset_position_dxl_1, _offset_position_dxl_2, _offset_position_dxl_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_1_P_gain", _p_gain_1);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_2_P_gain", _p_gain_2);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_P_gain", _p_gain_3);
        ROS_DEBUG("Joints Hardware Interface - Proportional Gain dxl: (1 : %d, 2 : %d, 3 : %d)",
                  _p_gain_1, _p_gain_2, _p_gain_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_1_I_gain", _i_gain_1);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_2_I_gain", _i_gain_2);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_I_gain", _i_gain_3);
        ROS_DEBUG("Joints Hardware Interface - Integral Gain dxl: (1 : %d, 2 : %d, 3 : %d)",
                  _i_gain_1, _i_gain_2, _i_gain_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_D_gain", _d_gain_1);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_D_gain", _d_gain_2);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_D_gain", _d_gain_3);
        ROS_DEBUG("Joints Hardware Interface - Integral Gain dxl: (1 : %d, 2 : %d, 3 : %d)",
                  _d_gain_1, _d_gain_2, _d_gain_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_FF1_gain", _ff1_gain_1);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_FF1_gain", _ff1_gain_2);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_FF1_gain", _ff1_gain_3);
        ROS_DEBUG("Joints Hardware Interface - Integral Gain dxl: (1 : %d, 2 : %d, 3 : %d)",
                  _ff1_gain_1, _ff1_gain_2, _ff1_gain_3);

        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_FF2_gain", _ff2_gain_1);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_FF2_gain", _ff2_gain_2);
        _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_FF2_gain", _ff2_gain_3);
        ROS_DEBUG("Joints Hardware Interface - Integral Gain dxl: (1 : %d, 2 : %d, 3 : %d)",
                  _ff2_gain_1, _ff2_gain_2, _ff2_gain_3);
    }

    void JointHardwareInterface::initJoints()
    {
        //retrieve nb joints with checking that the config param exists for both name and id
        while(_nh.hasParam("/niryo_robot_hardware_interface/joint_" + std::to_string(_nb_joints + 1) + "_id") &&
              _nh.hasParam("/niryo_robot_hardware_interface/joint_" + std::to_string(_nb_joints + 1) + "_name"))
            _nb_joints++;


        // connect and register joint state interface
        std::vector<hardware_interface::JointStateHandle> state_handle;
        std::vector<hardware_interface::JointHandle> position_handle;

        _vel.clear();
        _eff.clear();

        for (int j = 0; j < _nb_joints; j++)
        {
            string joint_name = "";
            string joint_type = "";
            int joint_id_config = 0;
            _vel.push_back(0);
            _eff.push_back(0);

            _nh.getParam("/niryo_robot_hardware_interface/joint_" + std::to_string(j + 1) + "_id", joint_name);
            _nh.getParam("/niryo_robot_hardware_interface/joint_" + std::to_string(j + 1) + "_name", joint_id_config);
            _nh.getParam("/niryo_robot_hardware_interface/joint_" + std::to_string(j + 1) + "_type", joint_type);

            ROS_INFO("JointHardwareInterface::initJoints - New Joints config found : Name : %s ; id : %d", joint_name.c_str(), joint_id_config);

            uint8_t joint_id = static_cast<uint8_t>(joint_id_config);

            hardware_interface::JointStateHandle jStateHandle(joint_name, &_pos.at(j), &_vel.at(j), &_eff.at(j));
            state_handle.emplace_back(jStateHandle);
            _joint_state_interface.registerHandle(jStateHandle);

            registerInterface(&_joint_state_interface);

            hardware_interface::JointHandle jPosHandle(_joint_state_interface.getHandle(joint_name), &_cmd.at(j));

            position_handle.emplace_back(jPosHandle);
            _joint_position_interface.registerHandle(jPosHandle);

            registerInterface(&_joint_position_interface);

            // Create motors with previous params
            _joint_list.clear();
            _list_stepper_id.clear();
            _map_stepper_name.clear();
            _list_dxl_id.clear();
            _map_dxl_name.clear();
            std::vector<JointState> joints_vect;
            //use config instead

            MotorTypeEnum eType = MotorTypeEnum(joint_type.c_str());
            joints_vect.emplace_back(JointState(joint_name,
                                                eType,
                                                joint_id));

            if(EMotorType::MOTOR_TYPE_STEPPER == eType) {
                _list_stepper_id.emplace_back(joint_id);
                _map_stepper_name[joint_id] = joint_name;
            }
            else if(EMotorType::MOTOR_TYPE_UNKNOWN != eType) {
                _list_dxl_id.push_back(joint_id);
                _map_dxl_name[joint_id] = joint_name;
            }

            _joint_list.emplace_back(joints_vect.back());
        }
    }

    void JointHardwareInterface::initMotors()
    {
        if(_joint_list.size() >= 5)
        {
            _joint_list.at(0).setNeedCalibration(true);
            _joint_list.at(1).setNeedCalibration(true);
            _joint_list.at(2).setNeedCalibration(true);
            _joint_list.at(3).setNeedCalibration(false);
            _joint_list.at(4).setNeedCalibration(false);
            _joint_list.at(5).setNeedCalibration(false);
        }

        sendInitMotorsParams();

        activateLearningMode();
    }

    void JointHardwareInterface::sendInitMotorsParams()
    {
        StepperMotorCmd cmd;
        std::vector<int32_t> stepper_params{8, 8, 8};
        cmd.setParams(stepper_params);
        cmd.setType(EStepperCommandType::CMD_TYPE_MICRO_STEPS);
        cmd.setMotorsId(_list_stepper_id);

        _stepper->setStepperCommands(cmd);
        ros::Duration(0.05).sleep();

        stepper_params.clear();
        stepper_params = {
            _max_effort_1,
            _max_effort_2,
            _max_effort_3};

        cmd.setParams(stepper_params);
        cmd.setType(EStepperCommandType::CMD_TYPE_MAX_EFFORT);
        cmd.setMotorsId(_list_stepper_id);

        _stepper->setStepperCommands(cmd);
        ros::Duration(0.05).sleep();

        // * Joint 4
        if(!setMotorPID(2, _p_gain_1, _i_gain_1, _d_gain_1, _ff1_gain_1, _ff2_gain_1))
            ROS_ERROR("Joints Hardware Interface - Error setting motor PID for joint 4");

        // * Joint 5
        if(!setMotorPID(3, _p_gain_2, _i_gain_2, _d_gain_2, _ff1_gain_2, _ff2_gain_2))
            ROS_ERROR("Joints Hardware Interface - Error setting motor PID for joint 5");

        // * Joint 6
        if(!setMotorPID(6, _p_gain_3, _i_gain_3, _d_gain_3, _ff1_gain_3, _ff2_gain_3))
            ROS_ERROR("Joints Hardware Interface - Error setting motor PID for joint 6");

    }

    void JointHardwareInterface::setCommandToCurrentPosition()
    {
        ROS_DEBUG("Joints Hardware Interface - Set command to current position called");
        _joint_position_interface.getHandle("joint_1").setCommand(_pos[0]);
        _joint_position_interface.getHandle("joint_2").setCommand(_pos[1]);
        _joint_position_interface.getHandle("joint_3").setCommand(_pos[2]);
        _joint_position_interface.getHandle("joint_4").setCommand(_pos[3]);
        _joint_position_interface.getHandle("joint_5").setCommand(_pos[4]);
        _joint_position_interface.getHandle("joint_6").setCommand(_pos[5]);
    }

    void JointHardwareInterface::read()
    {
        // std::vector<StepperDriver::StepperMotorState> stepper_motor_state;
        std::vector<int32_t> stepper_motor_state = _stepper->getTrajectoryControllerStates();
        std::vector<DxlMotorState> dxl_motor_state = _dynamixel->getDxlStates();
        _pos[0] = StepperDriver::StepperDriver::steps_to_rad_pos(stepper_motor_state.at(0), _gear_ratio_1, _direction_1);
        _pos[1] = StepperDriver::StepperDriver::steps_to_rad_pos(stepper_motor_state.at(1), _gear_ratio_2, _direction_2);
        _pos[2] = StepperDriver::StepperDriver::steps_to_rad_pos(stepper_motor_state.at(2), _gear_ratio_3, _direction_3);

        // Quick fix
        double dxl1_pose = _offset_position_dxl_1 + DynamixelDriver::motor_pos_to_rad_pos<DynamixelDriver::XL430Driver>(dxl_motor_state.at(0).getPositionState());
        double dxl2_pose = _offset_position_dxl_2 + DynamixelDriver::motor_pos_to_rad_pos<DynamixelDriver::XL430Driver>(DynamixelDriver::XL430Driver::MIDDLE_POSITION * 2 - dxl_motor_state.at(1).getPositionState());
        double dxl3_pose = _offset_position_dxl_3 + DynamixelDriver::motor_pos_to_rad_pos<DynamixelDriver::XL320Driver>(dxl_motor_state.at(2).getPositionState());
        _pos[3] = abs(dxl1_pose) < 2 * M_PI ? dxl1_pose : _pos[3];
        _pos[4] = abs(dxl2_pose) < 2 * M_PI ? dxl2_pose : _pos[4];
        _pos[5] = abs(dxl3_pose) < 2 * M_PI ? dxl3_pose : _pos[5];

        // Require new calibration for stepper because it was disconnected
        if (!_stepper->isConnectionOk())
            this->newCalibration();
    }

    void JointHardwareInterface::write()
    {
        if(_stepper && _dynamixel) {
            std::vector<int32_t> stepper_cmds{
                StepperDriver::StepperDriver::rad_pos_to_steps(_cmd.at(0), _gear_ratio_1, _direction_1),
                StepperDriver::StepperDriver::rad_pos_to_steps(_cmd.at(1), _gear_ratio_2, _direction_2),
                StepperDriver::StepperDriver::rad_pos_to_steps(_cmd.at(2), _gear_ratio_3, _direction_3)};

            std::vector<uint32_t> dxl_cmds{
                DynamixelDriver::rad_pos_to_motor_pos<DynamixelDriver::XL430Driver>(_cmd.at(3) - _offset_position_dxl_1),
                DynamixelDriver::rad_pos_to_motor_pos<DynamixelDriver::XL430Driver>(DynamixelDriver::XL430Driver::MIDDLE_POSITION * 2
                                                                                 - DynamixelDriver::motor_pos_to_rad_pos<DynamixelDriver::XL430Driver>(_cmd.at(4) - _offset_position_dxl_2)),
                DynamixelDriver::rad_pos_to_motor_pos<DynamixelDriver::XL320Driver>(_cmd.at(5) - _offset_position_dxl_3)};

            _stepper->setTrajectoryControllerCommands(stepper_cmds);
            _dynamixel->setTrajectoryControllerCommands(dxl_cmds);
        }
    }

    bool JointHardwareInterface::needCalibration() const
    {
        bool result = false;
        for (JointState jState : _joint_list)
        {
            if (jState.needCalibration())
            {
                result = true;
                break;
            }
        }
        ROS_DEBUG_THROTTLE(2, "JointHardwareInterface::needCalibration - Need calibration returned: %d", static_cast<int>(result));
        return result;
    }

    int JointHardwareInterface::calibrateJoints(int mode, std::string &result_message)
    {
        if (isCalibrationInProgress())
        {
            result_message = "JointHardwareInterface::calibrateJoints - Calibration already in process";
            return niryo_robot_msgs::CommandStatus::ABORTED;
        }
        if (needCalibration())
        {
            return _calibration_interface->startCalibration(mode, result_message);
        }

        result_message = "JointHardwareInterface::calibrateJoints - Calibration already done";
        return niryo_robot_msgs::CommandStatus::SUCCESS;
    }

    void JointHardwareInterface::newCalibration()
    {
        _joint_list.at(0).setNeedCalibration(true);
        _joint_list.at(1).setNeedCalibration(true);
        _joint_list.at(2).setNeedCalibration(true);
        return;
    }

    void JointHardwareInterface::activateLearningMode()
    {
        ROS_DEBUG("JointHardwareInterface::activateLearningMode - activate learning mode");

        if(_stepper && _dynamixel) {
            SynchronizeMotorCmd dxl_cmd;
            StepperMotorCmd stepper_cmd;

            dxl_cmd.setType(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
            stepper_cmd.setType(EStepperCommandType::CMD_TYPE_TORQUE);

            stepper_cmd.setMotorsId(_list_stepper_id);

            std::vector<int32_t> stepper_params{0, 0, 0};
            std::vector<uint32_t> dxl_params{0};

            dxl_cmd.setParams(dxl_params);
            stepper_cmd.setParams(stepper_params);

            _stepper->setStepperCommands(stepper_cmd);
            _dynamixel->setDxlCommands(dxl_cmd);

            _learning_mode = true;
        }

    }

    void JointHardwareInterface::deactivateLearningMode()
    {
        ROS_DEBUG("JointHardwareInterface::deactivateLearningMode - deactivate learning mode");
        if(_stepper && _dynamixel) {
            SynchronizeMotorCmd dxl_cmd;
            StepperMotorCmd stepper_cmd;

            dxl_cmd.setType(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
            stepper_cmd.setType(EStepperCommandType::CMD_TYPE_TORQUE);

            stepper_cmd.setMotorsId(_list_stepper_id);

            std::vector<int32_t> stepper_params{1, 1, 1};
            std::vector<uint32_t> dxl_params{1};
            dxl_cmd.setParams(dxl_params);
            stepper_cmd.setParams(stepper_params);

            _stepper->setStepperCommands(stepper_cmd);
            _dynamixel->setDxlCommands(dxl_cmd);
            _learning_mode = false;
        }
    }

    void JointHardwareInterface::synchronizeMotors(bool synchronise)
    {
        ROS_DEBUG("JointHardwareInterface::synchronizeMotors");
        if(_stepper) {
            StepperMotorCmd stepper_cmd;

            stepper_cmd.setType(EStepperCommandType::CMD_TYPE_SYNCHRONIZE);

            stepper_cmd.setMotorsId(_list_stepper_id);

            std::vector<int32_t> stepper_params{
                static_cast<int8_t>(synchronise),
                static_cast<int8_t>(synchronise),
                static_cast<int8_t>(synchronise)};

            stepper_cmd.setParams(stepper_params);

            _stepper->setStepperCommands(stepper_cmd);
        }
    }

    bool JointHardwareInterface::isCalibrationInProgress() const
    {
        return _calibration_interface->CalibrationInprogress();
    }

    const std::vector<JointState> &JointHardwareInterface::getJointsState() const
    {
        return _joint_list;
    }

    std::string JointHardwareInterface::jointIdToJointName(uint8_t id)
    {
        if(_map_stepper_name.count(id))
            return _map_stepper_name.at(id);
        else if(_map_dxl_name.count(id))
            return _map_dxl_name.at(id);

        return "";
    }

    /**
     * @brief JointHardwareInterface::setMotorPID
     * @param motor_id
     * @param motor_type
     * @param p_gain
     * @param i_gain
     * @param d_gain
     * @param ff1 : feed forward gaim 1 : only for XL430, XL330 and XC430 motors
     * @param ff2 : feed forward gaim 2 : only for XL430, XL330 and XC430 motors
     * @return
     */
    bool JointHardwareInterface::setMotorPID(uint8_t motor_id, uint32_t p_gain,
                                             uint32_t i_gain, uint32_t d_gain,
                                             uint32_t ff1, uint32_t ff2)
    {
        bool res = false;

        ROS_DEBUG("Joints Hardware Interface - Setting PID for motor id: %d", motor_id);

        // ** DXL PID configuration ** //

        // P Gain
        SingleMotorCmd dxl_cmd_p(EDxlCommandType::CMD_TYPE_P_GAIN, motor_id, p_gain);

        if(dxl_cmd_p.isValid())
            _dynamixel->addDxlCommandToQueue(dxl_cmd_p);

        SingleMotorCmd dxl_cmd_i(EDxlCommandType::CMD_TYPE_I_GAIN, motor_id, i_gain);

        if(dxl_cmd_i.isValid())
            _dynamixel->addDxlCommandToQueue(dxl_cmd_i);

        SingleMotorCmd dxl_cmd_d(EDxlCommandType::CMD_TYPE_D_GAIN, motor_id, d_gain);

        if(dxl_cmd_d.isValid())
            _dynamixel->addDxlCommandToQueue(dxl_cmd_d);

        SingleMotorCmd dxl_cmd_ff1(EDxlCommandType::CMD_TYPE_FF1_GAIN, motor_id, ff1);

        if(dxl_cmd_ff1.isValid())
            _dynamixel->addDxlCommandToQueue(dxl_cmd_ff1);

        SingleMotorCmd dxl_cmd_ff2(EDxlCommandType::CMD_TYPE_FF2_GAIN, motor_id, ff2);

        if(dxl_cmd_ff2.isValid())
            _dynamixel->addDxlCommandToQueue(dxl_cmd_ff2);


        return res;
    }
} // JointsInterface
