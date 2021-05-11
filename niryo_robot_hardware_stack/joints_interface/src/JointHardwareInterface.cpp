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
#include "joints_interface/Math.hpp"
#include "dynamixel_driver/SendCustomDxlValue.h"

JointHardwareInterface::JointHardwareInterface(
    boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> &dynamixel,
    boost::shared_ptr<StepperDriver::StepperDriverCore> &stepper)
    : _dynamixel(dynamixel), _stepper(stepper)
{
    _learning_mode = true;

    for (int i = 0; i < 6; i++)
    {
        _nh.getParam("/niryo_robot_hardware_interface/joint_" + std::to_string(i + 1) + "_id", _joints_id[i]);
        _nh.getParam("/niryo_robot_hardware_interface/joint_" + std::to_string(i + 1) + "_name", _joints_name[i]);
    }

    ROS_INFO("Joints Hardware Interface - Joints' Name : (1 : %s, 2 : %s, 3 : %s)", _joints_name[0].c_str(), _joints_name[1].c_str(), _joints_name[2].c_str());
    ROS_INFO("Joints Hardware Interface - Joints' Name : (4 : %s, 5 : %s, 6 : %s)", _joints_name[3].c_str(), _joints_name[4].c_str(), _joints_name[5].c_str());
    ROS_INFO("Joints Hardware Interface - Joints' ID : (1 : %d, 2 : %d, 3 : %d)", _joints_id[0], _joints_id[1], _joints_id[2]);
    ROS_INFO("Joints Hardware Interface - Joints' ID : (4 : %d, 5 : %d, 6 : %d)", _joints_id[3], _joints_id[4], _joints_id[5]);

    // connect and register joint state interface
    std::vector<hardware_interface::JointStateHandle> state_handle;
    for (int i = 0; i < 6; i++)
    {
        state_handle.push_back(hardware_interface::JointStateHandle(_joints_name[i], &_pos[i], &_vel[i], &_eff[i]));
        _joint_state_interface.registerHandle(state_handle[i]);
    }
    registerInterface(&_joint_state_interface);

    std::vector<hardware_interface::JointHandle> position_handle;
    for (int i = 0; i < 6; i++)
    {
        position_handle.push_back(hardware_interface::JointHandle(_joint_state_interface.getHandle(_joints_name[i]), &_cmd[i]));
        _joint_position_interface.registerHandle(position_handle[i]);
    }
    registerInterface(&_joint_position_interface);

    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_gear_ratio", _gear_ratio_1);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_gear_ratio", _gear_ratio_2);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_gear_ratio", _gear_ratio_3);
    ROS_DEBUG("Joints Hardware Interface - Joint Hardware Interface - Gear ratios : (1 : %lf, 2 : %lf, 3 : %lf)", _gear_ratio_1, _gear_ratio_2, _gear_ratio_3);

    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_home_position", _home_position_1);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_home_position", _home_position_2);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_home_position", _home_position_3);
    ROS_DEBUG("Joints Hardware Interface - Home positions : (1 : %lf, 2 : %lf, 3 : %lf)", _home_position_1, _home_position_2, _home_position_3);

    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_offset_position", _offset_position_stepper_1);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_offset_position", _offset_position_stepper_2);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_offset_position", _offset_position_stepper_3);
    ROS_DEBUG("Joints Hardware Interface - Angle offsets steppers: (1 : %lf, 2 : %lf, 3 : %lf)", _offset_position_stepper_1, _offset_position_stepper_2, _offset_position_stepper_3);

    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_direction", _direction_1);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_direction", _direction_2);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_direction", _direction_3);
    ROS_DEBUG("Joints Hardware Interface - Direction : (1 : %lf, 2 : %lf, 3 : %lf)", _direction_1, _direction_2, _direction_3);

    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_1_max_effort", _max_effort_1);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_2_max_effort", _max_effort_2);
    _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_3_max_effort", _max_effort_3);
    ROS_DEBUG("Joints Hardware Interface - Max effort : (1 : %d, 2 : %d, 3 : %d)", _max_effort_1, _max_effort_2, _max_effort_3);

    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_1_offset_position", _offset_position_dxl_1);
    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_2_offset_position", _offset_position_dxl_2);
    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_offset_position", _offset_position_dxl_3);
    ROS_DEBUG("Joints Hardware Interface - Angle offsets dxk: (1 : %lf, 2 : %lf, 3 : %lf)", _offset_position_dxl_1, _offset_position_dxl_2, _offset_position_dxl_3);

    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_1_P_gain", _p_gain_1);
    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_2_P_gain", _p_gain_2);
    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_P_gain", _p_gain_3);
    ROS_DEBUG("Joints Hardware Interface - Proportional Gain dxk: (1 : %d, 2 : %d, 3 : %d)", _p_gain_1, _p_gain_2, _p_gain_3);

    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_1_I_gain", _i_gain_1);
    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_2_I_gain", _i_gain_2);
    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_I_gain", _i_gain_3);
    ROS_DEBUG("Joints Hardware Interface - Integral Gain dxk: (1 : %d, 2 : %d, 3 : %d)", _i_gain_1, _i_gain_2, _i_gain_3);

    _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_3_D_gain", _d_gain_3);
    ROS_DEBUG("Joints Hardware Interface - Integral Gain dxk: (3 : %d)",  _d_gain_3);

    // Create motors with previous params
    _joint_list.clear();
    _list_stepper_id.clear();
    _map_stepper_name.clear();
    _list_dxl_id.clear();
    _map_dxl_name.clear();
    std::vector<JointState> joints_vect;
    for (int i = 0; i < 3; i++)
    {
        joints_vect.push_back(JointState(_joints_name[i], (uint8_t)StepperDriver::StepperMotorType::MOTOR_TYPE_STEPPER, _joints_id[i]));
        _list_stepper_id.push_back(_joints_id[i]);
        _map_stepper_name[_joints_id[i]] = _joints_name[i];
        _joint_list.push_back(joints_vect[i]);
    }
    for (int i = 3; i < 5; i++)
    {
        joints_vect.push_back(JointState(_joints_name[i], (uint8_t)DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL430, _joints_id[i]));
        _list_dxl_id.push_back(_joints_id[i]);
        _map_dxl_name[_joints_id[i]] = _joints_name[i];
        _joint_list.push_back(joints_vect[i]);
    }
    for (int i = 5; i < 6; i++)
    {
        joints_vect.push_back(JointState(_joints_name[i], (uint8_t)DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL320, _joints_id[i]));
        _list_dxl_id.push_back(_joints_id[i]);
        _map_dxl_name[_joints_id[i]] = _joints_name[i];
        _joint_list.push_back(joints_vect[i]);
    }
    initMotors();

    _calibration_interface.reset(new CalibrationInterface(_joint_list
    , _stepper, _dynamixel));
}

void JointHardwareInterface::initMotors()
{
    _joint_list.at(0).setNeedCalibration(true);
    _joint_list.at(1).setNeedCalibration(true);
    _joint_list.at(2).setNeedCalibration(true);
    _joint_list.at(3).setNeedCalibration(false);
    _joint_list.at(4).setNeedCalibration(false);
    _joint_list.at(5).setNeedCalibration(false);

    sendInitMotorsParams();

    activateLearningMode();
}

void JointHardwareInterface::sendInitMotorsParams()
{
    StepperDriver::StepperMotorCmd cmd;
    std::vector<int32_t> stepper_params{8, 8, 8};
    cmd.setParams(stepper_params);
    cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_MICRO_STEPS);
    cmd.setMotorsId(_list_stepper_id);

    _stepper->setStepperCommands(cmd);
    ros::Duration(0.05).sleep();

    stepper_params.clear();
    stepper_params = {
        _max_effort_1,
        _max_effort_2,
        _max_effort_3};

    cmd.setParams(stepper_params);
    cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_MAX_EFFORT);
    cmd.setMotorsId(_list_stepper_id);

    _stepper->setStepperCommands(cmd);
    ros::Duration(0.05).sleep();

    // ** DXL PID configuration ** //
    ros::ServiceClient dxl_client = _nh.serviceClient<dynamixel_driver::SendCustomDxlValue>("/niryo_robot/dynamixel_driver/send_custom_dxl_value");
    dynamixel_driver::SendCustomDxlValue dxl_cmd_srv;

    // * Joint 4
    dxl_cmd_srv.request.motor_type = 2;
    dxl_cmd_srv.request.id = 2;
    dxl_cmd_srv.request.byte_number = 2;
    dxl_cmd_srv.request.value = _p_gain_1;
    dxl_cmd_srv.request.reg_address = 80;
    if (dxl_client.call(dxl_cmd_srv))
        ROS_DEBUG("Joints Hardware Interface - Set joint 4 P Gain OK");

    dxl_cmd_srv.request.value = _i_gain_1;
    dxl_cmd_srv.request.reg_address = 82;
    if (dxl_client.call(dxl_cmd_srv))
        ROS_DEBUG("Joints Hardware Interface - Set joint 4 I Gain OK");

    // * Joint 5
    dxl_cmd_srv.request.motor_type = 2;
    dxl_cmd_srv.request.id = 3;
    dxl_cmd_srv.request.byte_number = 2;
    dxl_cmd_srv.request.value = _p_gain_2;
    dxl_cmd_srv.request.reg_address = 80;
    if (dxl_client.call(dxl_cmd_srv))
        ROS_DEBUG("Joints Hardware Interface - Set joint 5 P Gain OK");

    dxl_cmd_srv.request.value = _i_gain_2;
    dxl_cmd_srv.request.reg_address = 82;
    if (dxl_client.call(dxl_cmd_srv))
        ROS_DEBUG("Joints Hardware Interface - Set joint 5 I Gain OK");

    // * Joint 6
    dxl_cmd_srv.request.motor_type = 3;
    dxl_cmd_srv.request.id = 6;
    dxl_cmd_srv.request.byte_number = 1;
    dxl_cmd_srv.request.value = _p_gain_3;
    dxl_cmd_srv.request.reg_address = 29;
    if (dxl_client.call(dxl_cmd_srv))
        ROS_DEBUG("Joints Hardware Interface - Set joint 6 P Gain OK");

    dxl_cmd_srv.request.value = _i_gain_3;
    dxl_cmd_srv.request.reg_address = 28;
    if (dxl_client.call(dxl_cmd_srv))
        ROS_DEBUG("Joints Hardware Interface - Set joint 6 I Gain OK");

    dxl_cmd_srv.request.value = _d_gain_3;
    dxl_cmd_srv.request.reg_address = 27;
    if (dxl_client.call(dxl_cmd_srv))
        ROS_DEBUG("Joints Hardware Interface - Set joint 6 D Gain OK");
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
    std::vector<DynamixelDriver::DxlMotorState> dxl_motor_state = _dynamixel->getDxlStates();
    _pos[0] = steps_to_rad_pos(stepper_motor_state.at(0), _gear_ratio_1, _direction_1);
    _pos[1] = steps_to_rad_pos(stepper_motor_state.at(1), _gear_ratio_2, _direction_2);
    _pos[2] = steps_to_rad_pos(stepper_motor_state.at(2), _gear_ratio_3, _direction_3);

    // Quick fix
    double dxl1_pose = _offset_position_dxl_1 + xl430_pos_to_rad_pos(dxl_motor_state.at(0).getPositionState());
    double dxl2_pose = _offset_position_dxl_2 + xl430_pos_to_rad_pos(XL430_MIDDLE_POSITION * 2 - dxl_motor_state.at(1).getPositionState());
    double dxl3_pose = _offset_position_dxl_3 + xl320_pos_to_rad_pos(dxl_motor_state.at(2).getPositionState());
    _pos[3] = abs(dxl1_pose) < 2 * M_PI ? dxl1_pose : _pos[3];
    _pos[4] = abs(dxl2_pose) < 2 * M_PI ? dxl2_pose : _pos[4];
    _pos[5] = abs(dxl3_pose) < 2 * M_PI ? dxl3_pose : _pos[5];

    // Require new calibration for stepper because it was disconnected
    if (!_stepper->isConnectionOk())
        this->newCalibration();
}

void JointHardwareInterface::write()
{
    std::vector<int32_t> stepper_cmds{
        rad_pos_to_steps(_cmd[0], _gear_ratio_1, _direction_1),
        rad_pos_to_steps(_cmd[1], _gear_ratio_2, _direction_2),
        rad_pos_to_steps(_cmd[2], _gear_ratio_3, _direction_3)};

    std::vector<uint32_t> dxl_cmds{
        rad_pos_to_xl430_pos(_cmd[3] - _offset_position_dxl_1),
        (XL430_MIDDLE_POSITION * 2 - rad_pos_to_xl430_pos(_cmd[4] - _offset_position_dxl_2)),
        rad_pos_to_xl320_pos(_cmd[5] - _offset_position_dxl_3)};

    _stepper->setTrajectoryControllerCommands(stepper_cmds);
    _dynamixel->setTrajectoryControllerCommands(dxl_cmds);
}

bool JointHardwareInterface::needCalibration()
{
    bool result = false;
    for (int i = 0; i < _joint_list.size(); i++)
    {
        if (_joint_list.at(i).needCalibration())
        {
            result = true;
            break;
        }
    }
    ROS_DEBUG_THROTTLE(2, "Joints Hardware Interface - Need calibration returned: %d", static_cast<int>(result));
    return result;
}

int JointHardwareInterface::calibrateJoints(int mode, std::string &result_message)
{
    if (isCalibrationInProgress())
    {
        result_message = "Joints Hardware Interface - Calibration already in process";
        return niryo_robot_msgs::CommandStatus::ABORTED;
    }
    if (needCalibration())
    {
        return _calibration_interface->startCalibration(mode, result_message);
    }
    else
    {
        result_message = "Joints Hardware Interface - Calibration already done";
        return niryo_robot_msgs::CommandStatus::SUCCESS;
    }
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
    ROS_DEBUG("Joints Hardware Interface - activate learning mode");
    DynamixelDriver::SynchronizeMotorCmd dxl_cmd;
    StepperDriver::StepperMotorCmd stepper_cmd;

    dxl_cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_LEARNING_MODE);
    stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);

    stepper_cmd.setMotorsId(_list_stepper_id);

    std::vector<int32_t> stepper_params{0, 0, 0};
    std::vector<uint32_t> dxl_params{0};

    dxl_cmd.setParams(dxl_params);
    stepper_cmd.setParams(stepper_params);

    _stepper->setStepperCommands(stepper_cmd);
    _dynamixel->setDxlCommands(dxl_cmd);

    _learning_mode = true;
}

void JointHardwareInterface::deactivateLearningMode()
{
    ROS_DEBUG("Joints Hardware Interface - deactivate learning mode");
    DynamixelDriver::SynchronizeMotorCmd dxl_cmd;
    StepperDriver::StepperMotorCmd stepper_cmd;
    bool enable_torque = false;

    dxl_cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_LEARNING_MODE);
    stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);

    stepper_cmd.setMotorsId(_list_stepper_id);

    std::vector<int32_t> stepper_params{1, 1, 1};
    std::vector<uint32_t> dxl_params{1};
    dxl_cmd.setParams(dxl_params);
    stepper_cmd.setParams(stepper_params);

    _stepper->setStepperCommands(stepper_cmd);
    _dynamixel->setDxlCommands(dxl_cmd);
    _learning_mode = false;
}

void JointHardwareInterface::synchronizeMotors(bool synchronise)
{
    ROS_DEBUG("JointHardwareInterface::synchronizeMotors");
    StepperDriver::StepperMotorCmd stepper_cmd;

    stepper_cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_SYNCHRONIZE);

    stepper_cmd.setMotorsId(_list_stepper_id);

    std::vector<int32_t> stepper_params{
        (int8_t)synchronise,
        (int8_t)synchronise,
        (int8_t)synchronise};

    stepper_cmd.setParams(stepper_params);

    _stepper->setStepperCommands(stepper_cmd);
}

bool JointHardwareInterface::isCalibrationInProgress()
{
    return _calibration_interface->CalibrationInprogress();
}

std::vector<JointState> &JointHardwareInterface::getJointsState()
{
    return _joint_list;
}

std::string JointHardwareInterface::jointIdToJointName(int id, uint8_t motor_type)
{
    if (motor_type == (uint8_t)StepperDriver::StepperMotorType::MOTOR_TYPE_STEPPER)
    {
        std::map<uint8_t, std::string>::iterator  it= _map_stepper_name.find(id);
        if (it !=  _map_stepper_name.end())
            return it->second;
    }
    else if (motor_type == (uint8_t)DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL430 or motor_type ==(uint8_t)DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL320)
    {
        std::map<uint8_t, std::string>::iterator  it= _map_dxl_name.find(id);
        if (it !=  _map_dxl_name.end())
            return it->second;
    }
    return "";
}
