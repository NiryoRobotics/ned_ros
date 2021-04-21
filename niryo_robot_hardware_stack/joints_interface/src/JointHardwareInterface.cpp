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

#include "model/motor_type_enum.hpp"
#include "util/util_defs.hpp"

using namespace std;
using namespace common::model;

namespace JointsInterface {

    JointHardwareInterface::JointHardwareInterface(shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel,
                                                   shared_ptr<StepperDriver::StepperDriverCore> stepper) :
        _dynamixel(dynamixel),
        _stepper(stepper),
        _learning_mode(true)
    {

        initJoints();

        sendInitMotorsParams();

        activateLearningMode();

        _calibration_interface.reset(new CalibrationInterface(_joint_list, _stepper, _dynamixel));
    }

    /**
     * @brief JointHardwareInterface::read
     */
    void JointHardwareInterface::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
    {
        std::vector<StepperMotorState> stepper_motor_state = _stepper->getStepperStates();
        std::vector<DxlMotorState> dxl_motor_state = _dynamixel->getDxlStates();

        for(size_t j = 0; j < _joint_list.size(); ++j)
        {
            if(_joint_list.at(j) && _joint_list.at(j)->isValid()) {
                if(_joint_list.at(j)->isStepper())
                    _joint_list.at(j)->pos = _joint_list.at(j)->to_rad_pos(stepper_motor_state.at(j).getPositionState());
            }
        }

        double dxl1_pose = _joint_list.at(3)->getOffsetPosition() + _joint_list.at(3)->to_rad_pos(dxl_motor_state.at(0).getPositionState());
        double dxl2_pose = _joint_list.at(4)->getOffsetPosition() + _joint_list.at(4)->to_rad_pos(dxl_motor_state.at(1).getPositionState());
        double dxl3_pose = _joint_list.at(5)->getOffsetPosition() + _joint_list.at(5)->to_rad_pos(dxl_motor_state.at(2).getPositionState());
        _joint_list.at(3)->pos = (abs(dxl1_pose) < 2 * M_PI) ? dxl1_pose : _joint_list.at(3)->pos;
        _joint_list.at(4)->pos = (abs(dxl1_pose) < 2 * M_PI) ? dxl2_pose : _joint_list.at(4)->pos;
        _joint_list.at(5)->pos = (abs(dxl1_pose) < 2 * M_PI) ? dxl3_pose : _joint_list.at(5)->pos;


//        _pos[0] = steps_to_rad_pos(stepper_motor_state.at(0), _gear_ratio_1, _direction_1);
//        _pos[1] = steps_to_rad_pos(stepper_motor_state.at(1), _gear_ratio_2, _direction_2);
//        _pos[2] = steps_to_rad_pos(stepper_motor_state.at(2), _gear_ratio_3, _direction_3);

//        // Quick fix
//        double dxl1_pose = _offset_position_dxl_1 + xl430_pos_to_rad_pos(dxl_motor_state.at(0).getPositionState());
//        double dxl2_pose = _offset_position_dxl_2 + xl430_pos_to_rad_pos(XL430_MIDDLE_POSITION * 2 - dxl_motor_state.at(1).getPositionState());
//        double dxl3_pose = _offset_position_dxl_3 + xl320_pos_to_rad_pos(dxl_motor_state.at(2).getPositionState());
//        _pos[3] = abs(dxl1_pose) < 2 * M_PI ? dxl1_pose : _pos[3];
//        _pos[4] = abs(dxl2_pose) < 2 * M_PI ? dxl2_pose : _pos[4];
//        _pos[5] = abs(dxl3_pose) < 2 * M_PI ? dxl3_pose : _pos[5];


        if (!_stepper->isConnectionOk())
            this->newCalibration();
    }

    /**
     * @brief JointHardwareInterface::write
     */
    void JointHardwareInterface::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
    {
        vector<int32_t> stepper_cmds;

        for(size_t j = 0; j < _joint_list.size(); ++j)
        {
            if(_joint_list.at(j) && _joint_list.at(j)->isValid() && _joint_list.at(j)->isStepper()) {

                int32_t pos = _joint_list.at(j)->rad_pos_to_motor_pos(_joint_list.at(j)->cmd);
                stepper_cmds.emplace_back(pos);
            }
        }

        double precision = 0.0001;
        SynchronizeMotorCmd dxlMotorCmd(EDxlCommandType::CMD_TYPE_POSITION);
        for(std::shared_ptr<common::model::JointState> jState : _joint_list) {
            if(jState->isDynamixel() && (jState->cmd - jState->pos) < precision) //set a new command only if pos has not yet been reached
                dxlMotorCmd.addMotorParam(jState->getType(), jState->getId(), static_cast<uint32_t>(jState->rad_pos_to_motor_pos(jState->cmd - jState->getOffsetPosition())));
            //carefull for dxl 2
        }

        _stepper->setTrajectoryControllerCommands(stepper_cmds); //CC append ? do a queue ?
        if(dxlMotorCmd.isValid())
            _dynamixel->addDxlSyncCommandToQueue(dxlMotorCmd); // CC warning -> risk of queue overflow
    }

    /**
     * @brief JointHardwareInterface::initJoints : build the joints by gathering information in config files and instanciating correct state (dxl or stepper)
     */
    void JointHardwareInterface::initJoints()
    {
        size_t nb_joints = 0;

        //retrieve nb joints with checking that the config param exists for both name and id
        while(_nh.hasParam("/niryo_robot_hardware_interface/joint_" + to_string(nb_joints + 1) + "_id") &&
              _nh.hasParam("/niryo_robot_hardware_interface/joint_" + to_string(nb_joints + 1) + "_name") &&
              _nh.hasParam("/niryo_robot_hardware_interface/joint_" + to_string(nb_joints + 1) + "_type"))
            nb_joints++;

        // connect and register joint state interface
        vector<hardware_interface::JointHandle> position_handle;

        _joint_list.clear();
        _list_stepper_id.clear();
        _map_stepper_name.clear();
        _list_dxl_id.clear();
        _map_dxl_name.clear();

        double cmd[6] = {0, 0.64, -1.39};
        double pos[6] = {0, 0.64, -1.39};

        int currentIdStepper = 1;
        int currentIdDxl = 1;

        for (size_t j = 0; j < nb_joints; j++)
        {
            int joint_id_config = 0;
            string joint_name = "";
            string joint_type = "";

            _nh.getParam("/niryo_robot_hardware_interface/joint_" + to_string(j + 1) + "_id", joint_id_config);
            _nh.getParam("/niryo_robot_hardware_interface/joint_" + to_string(j + 1) + "_name", joint_name);
            _nh.getParam("/niryo_robot_hardware_interface/joint_" + to_string(j + 1) + "_type", joint_type);

            MotorTypeEnum eType = MotorTypeEnum(joint_type.c_str());

            // CC check if this operator== is working...

            //gather info in joint  states (polymorphic)
            if(eType == EMotorType::MOTOR_TYPE_STEPPER) {  //stepper
                // cc use factory
                shared_ptr<StepperMotorState> stepperState = make_shared<StepperMotorState>(joint_name, eType, static_cast<uint8_t>(joint_id_config));

                //CC use factory in state directly

                double offsetPos = 0.0;
                double gear_ratio = 0.0;
                double direction = 0.0;
                double max_effort = 0.0;

                _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_" + to_string(currentIdStepper) + "_offset_position", offsetPos);
                _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_" + to_string(currentIdStepper) + "_gear_ratio", gear_ratio);
                _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_" + to_string(currentIdStepper) + "_direction", direction);
                _nh.getParam("/niryo_robot_hardware_interface/steppers/stepper_" + to_string(currentIdStepper) + "_max_effort", max_effort);

                //add parameters
                stepperState->setOffsetPosition(offsetPos);
                stepperState->setGearRatio(gear_ratio);
                stepperState->setDirection(direction);
                stepperState->setMaxEffort(max_effort);
                stepperState->setNeedCalibration(true);

                stepperState->pos = pos[currentIdStepper - 1];
                stepperState->cmd = cmd[currentIdStepper - 1];

                _joint_list.emplace_back(stepperState);

                _list_stepper_id.emplace_back(stepperState->getId());
                _map_stepper_name[stepperState->getId()] = stepperState->getName();

                currentIdStepper++;
            }
            else if(eType != EMotorType::MOTOR_TYPE_UNKNOWN) {  //dynamixel
                shared_ptr<DxlMotorState> dxlState = make_shared<DxlMotorState>(joint_name, eType, static_cast<uint8_t>(joint_id_config));

                double offsetPos = 0.0;
                int PGain = 0;
                int IGain = 0;
                int DGain = 0;
                int FF1Gain = 0;
                int FF2Gain = 0;

                _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_" + to_string(currentIdDxl) + "_offset_position", offsetPos);

                _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_" + to_string(currentIdDxl) + "_P_gain", PGain);
                _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_" + to_string(currentIdDxl) + "_I_gain", IGain);
                _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_" + to_string(currentIdDxl) + "_D_gain", DGain);
                _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_" + to_string(currentIdDxl) + "_FF1_gain", FF1Gain);
                _nh.getParam("/niryo_robot_hardware_interface/dynamixels/dxl_" + to_string(currentIdDxl) + "_FF2_gain", FF2Gain);

                dxlState->setOffsetPosition(offsetPos);
                dxlState->setPGain(static_cast<uint32_t>(PGain));
                dxlState->setIGain(static_cast<uint32_t>(IGain));
                dxlState->setDGain(static_cast<uint32_t>(DGain));
                dxlState->setFF1Gain(static_cast<uint32_t>(FF1Gain));
                dxlState->setFF2Gain(static_cast<uint32_t>(FF2Gain));

                dxlState->setNeedCalibration(false);

                _joint_list.emplace_back(dxlState);


                _list_dxl_id.push_back(dxlState->getId());
                _map_dxl_name[dxlState->getId()] = dxlState->getName();

                currentIdDxl++;
            }

            // register the joints
            if(j < _joint_list.size() && _joint_list.at(j)) {
                shared_ptr<JointState> jState = _joint_list.at(j);

                ROS_INFO("JointHardwareInterface::initJoints - New Joints config found : %s", jState->str().c_str());

                hardware_interface::JointStateHandle jStateHandle(jState->getName(),
                                                                  &_joint_list.at(j)->pos,
                                                                  &_joint_list.at(j)->vel,
                                                                  &_joint_list.at(j)->eff);

                _joint_state_interface.registerHandle(jStateHandle);

                registerInterface(&_joint_state_interface);

                hardware_interface::JointHandle jPosHandle(_joint_state_interface.getHandle(jState->getName()), &_joint_list.at(j)->cmd);

                position_handle.emplace_back(jPosHandle);
                _joint_position_interface.registerHandle(jPosHandle);

                registerInterface(&_joint_position_interface);
            }
        }
    }

    void JointHardwareInterface::sendInitMotorsParams()
    {
        StepperMotorCmd cmd;
        // CMD_TYPE_MICRO_STEPS cmd
        cmd.setType(EStepperCommandType::CMD_TYPE_MICRO_STEPS);
        vector<int32_t> stepper_params{8, 8, 8};
        cmd.setParams(stepper_params);
        cmd.setMotorsId(_list_stepper_id);

        _stepper->setStepperCommands(cmd);
        ros::Duration(0.05).sleep();

        // CMD_TYPE_MAX_EFFORT cmd
        cmd.reset();
        stepper_params.clear();
        cmd.setType(EStepperCommandType::CMD_TYPE_MAX_EFFORT);
        cmd.setMotorsId(_list_stepper_id);

        for(size_t i = 0; i < _joint_list.size(); ++i)
        {
            if(_joint_list.at(i) && _joint_list.at(i)->isStepper())
                stepper_params.emplace_back(dynamic_pointer_cast<StepperMotorState>(_joint_list.at(i))->getMaxEffort());
        }
        cmd.setParams(stepper_params);
        _stepper->setStepperCommands(cmd);
        ros::Duration(0.05).sleep();

        // * dynamixels joints PID
        for(size_t i = 0; i < _joint_list.size(); ++i)
        {
            if(_joint_list.at(i) && _joint_list.at(i)->isDynamixel()) {
                shared_ptr<DxlMotorState> dxlState = dynamic_pointer_cast<DxlMotorState>(_joint_list.at(i));
                if(!setMotorPID(dxlState))
                    ROS_ERROR("JointHardwareInterface::sendInitMotorsParams - Error setting motor PID for dynamixel id %d", static_cast<int>(dxlState->getId()));
            }
        }
    }

    void JointHardwareInterface::setCommandToCurrentPosition()
    {
        ROS_DEBUG("Joints Hardware Interface - Set command to current position called");
        for(size_t j = 0; j < _joint_list.size(); ++j)
            _joint_position_interface.getHandle("joint_" + to_string(j + 1)).setCommand(_joint_list.at(j)->pos);
    }

    /**
     * @brief JointHardwareInterface::needCalibration
     * @return
     */
    bool JointHardwareInterface::needCalibration() const
    {
        bool result = false;
        for (shared_ptr<JointState> jState : _joint_list)
        {
            if (jState->needCalibration())
            {
                result = true;
                break;
            }
        }
        ROS_DEBUG_THROTTLE(2, "JointHardwareInterface::needCalibration - Need calibration returned: %d", static_cast<int>(result));
        return result;
    }

    /**
     * @brief JointHardwareInterface::calibrateJoints
     * @param mode
     * @param result_message
     * @return
     */
    int JointHardwareInterface::calibrateJoints(int mode, string &result_message)
    {
        result_message = "";
        int calib_res = niryo_robot_msgs::CommandStatus::ABORTED;

        if (isCalibrationInProgress())
        {
            result_message = "JointHardwareInterface::calibrateJoints - Calibration already in process";
            calib_res = niryo_robot_msgs::CommandStatus::ABORTED;
        }
        else if (needCalibration())
        {
            calib_res = _calibration_interface->startCalibration(mode, result_message);
        }
        else {
            result_message = "JointHardwareInterface::calibrateJoints - Calibration already done";
            calib_res = niryo_robot_msgs::CommandStatus::SUCCESS;
        }

        return calib_res;
    }

    /**
     * @brief JointHardwareInterface::newCalibration : setNeedCalibration for all steppers
     */
    void JointHardwareInterface::newCalibration()
    {
        for(shared_ptr<JointState> jState: _joint_list) {
            if(jState->isStepper())
                jState->setNeedCalibration(true);
        }
    }

    /**
     * @brief JointHardwareInterface::activateLearningMode
     */
    void JointHardwareInterface::activateLearningMode()
    {
        ROS_DEBUG("JointHardwareInterface::activateLearningMode - activate learning mode");

        if(_stepper && _dynamixel) {
            SynchronizeMotorCmd dxl_cmd;

            dxl_cmd.setType(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
            for(auto const& jState: _joint_list) {
                if(jState->isDynamixel())
                    dxl_cmd.addMotorParam(jState->getType(), jState->getId(), 0);
            }

            StepperMotorCmd stepper_cmd;
            vector<int32_t> stepper_params(_list_stepper_id.size(), 0);

            stepper_cmd.setType(EStepperCommandType::CMD_TYPE_TORQUE);
            stepper_cmd.setMotorsId(_list_stepper_id);
            stepper_cmd.setParams(stepper_params);

            _stepper->setStepperCommands(stepper_cmd);
            _dynamixel->addDxlSyncCommandToQueue(dxl_cmd);

            _learning_mode = true;
        }

    }

    /**
     * @brief JointHardwareInterface::deactivateLearningMode
     */
    void JointHardwareInterface::deactivateLearningMode()
    {
        ROS_DEBUG("JointHardwareInterface::deactivateLearningMode - deactivate learning mode");
        if(_stepper && _dynamixel) {
            SynchronizeMotorCmd dxl_cmd;
            dxl_cmd.setType(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
            for(auto const& jState: _joint_list) {
                if(jState->isDynamixel())
                    dxl_cmd.addMotorParam(jState->getType(), jState->getId(), 1);
            }

            StepperMotorCmd stepper_cmd;
            vector<int32_t> stepper_params(_list_stepper_id.size(), 1);

            stepper_cmd.setType(EStepperCommandType::CMD_TYPE_TORQUE);
            stepper_cmd.setMotorsId(_list_stepper_id);
            stepper_cmd.setParams(stepper_params);

            _stepper->setStepperCommands(stepper_cmd);
            _dynamixel->addDxlSyncCommandToQueue(dxl_cmd);

            _learning_mode = true;
        }
    }

    /**
     * @brief JointHardwareInterface::synchronizeMotors
     * @param synchronize
     */
    void JointHardwareInterface::synchronizeMotors(bool synchronize)
    {
        ROS_DEBUG("JointHardwareInterface::synchronizeMotors");

        if(_stepper) {
            StepperMotorCmd stepper_cmd;
            vector<int32_t> stepper_params(_list_stepper_id.size(), synchronize);

            stepper_cmd.setType(EStepperCommandType::CMD_TYPE_SYNCHRONIZE);
            stepper_cmd.setMotorsId(_list_stepper_id);
            stepper_cmd.setParams(stepper_params);

            _stepper->setStepperCommands(stepper_cmd);
        }
    }

    /**
     * @brief JointHardwareInterface::isCalibrationInProgress
     * @return
     */
    bool JointHardwareInterface::isCalibrationInProgress() const
    {
        return _calibration_interface->CalibrationInprogress();
    }

    /**
     * @brief JointHardwareInterface::getJointsState
     * @return
     */
    const vector<shared_ptr<JointState> >& JointHardwareInterface::getJointsState() const
    {
        return _joint_list;
    }

    /**
     * @brief JointHardwareInterface::jointIdToJointName
     * @param id
     * @return
     */
    string JointHardwareInterface::jointIdToJointName(uint8_t id) const
    {
        if(_map_stepper_name.count(id))
            return _map_stepper_name.at(id);
        else if(_map_dxl_name.count(id))
            return _map_dxl_name.at(id);

        return "";
    }

    /**
     * @brief JointHardwareInterface::setMotorPID : if param is < 0, does not set anything
     * @param motor_id
     * @param motor_type
     * @param p_gain
     * @param i_gain
     * @param d_gain
     * @param ff1 : feed forward gain 1 : only for XL430, XL330 and XC430 motors
     * @param ff2 : feed forward gain 2 : only for XL430, XL330 and XC430 motors
     * @return
     */
    bool JointHardwareInterface::setMotorPID(const shared_ptr<DxlMotorState>& dxlState)
    {
        uint8_t motor_id = dxlState->getId();

        ROS_DEBUG("JointHardwareInterface::setMotorPID - Setting PID for motor id: %d", static_cast<int>(motor_id));

        // ** DXL PID configuration ** //

        // P Gain
        if(dxlState->getPGain() > 0) {
            SingleMotorCmd dxl_cmd_p(EDxlCommandType::CMD_TYPE_P_GAIN, motor_id, dxlState->getPGain());

            if(dxl_cmd_p.isValid())
                _dynamixel->addDxlCommandToQueue(dxl_cmd_p);
        }

        if(dxlState->getIGain() > 0) {
            SingleMotorCmd dxl_cmd_i(EDxlCommandType::CMD_TYPE_I_GAIN, motor_id, dxlState->getIGain());

            if(dxl_cmd_i.isValid())
                _dynamixel->addDxlCommandToQueue(dxl_cmd_i);
        }

        if(dxlState->getDGain() > 0) {
            SingleMotorCmd dxl_cmd_d(EDxlCommandType::CMD_TYPE_D_GAIN, motor_id, dxlState->getDGain());

            if(dxl_cmd_d.isValid())
                _dynamixel->addDxlCommandToQueue(dxl_cmd_d);
        }

        if(dxlState->getFF1Gain() > 0) {
            SingleMotorCmd dxl_cmd_ff1(EDxlCommandType::CMD_TYPE_FF1_GAIN, motor_id, dxlState->getFF1Gain());

            if(dxl_cmd_ff1.isValid())
                _dynamixel->addDxlCommandToQueue(dxl_cmd_ff1);
        }

        if(dxlState->getFF2Gain() > 0) {
            SingleMotorCmd dxl_cmd_ff2(EDxlCommandType::CMD_TYPE_FF2_GAIN, motor_id, dxlState->getFF2Gain());

            if(dxl_cmd_ff2.isValid())
                _dynamixel->addDxlCommandToQueue(dxl_cmd_ff2);
        }

        return true;
    }
} // JointsInterface
