/*
    joint_hardware_interface.cpp
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

#include "joints_interface/joint_hardware_interface.hpp"

// c++
#include <vector>
#include <string>
#include <utility>
#include <typeinfo>

// niryo
#include "common/model/motor_type_enum.hpp"
#include "common/model/bus_protocol_enum.hpp"
#include "common/util/util_defs.hpp"

using ::std::shared_ptr;
using ::std::string;
using ::std::to_string;
using ::std::dynamic_pointer_cast;

using ::common::model::MotorTypeEnum;
using ::common::model::EMotorType;
using ::common::model::EBusProtocol;
using ::common::model::BusProtocolEnum;
using ::common::model::StepperMotorState;
using ::common::model::DxlMotorState;
using ::common::model::StepperSingleCmd;
using ::common::model::StepperTtlSingleCmd;
using ::common::model::EStepperCalibrationStatus;
using ::common::model::EStepperCommandType;
using ::common::model::EDxlCommandType;

namespace joints_interface
{

/**
 * @brief JointHardwareInterface::JointHardwareInterface
 * @param rootnh
 * @param robot_hwnh
 * @param ttl_interface
 * @param can_interface
 */
JointHardwareInterface::JointHardwareInterface(ros::NodeHandle& rootnh,
                                               ros::NodeHandle& robot_hwnh,
                                               std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                                               std::shared_ptr<can_driver::CanInterfaceCore> can_interface) :
    _ttl_interface(ttl_interface),
    _can_interface(can_interface)
{
    ROS_DEBUG("JointHardwareInterface::ctor");

    init(rootnh, robot_hwnh);

    sendInitMotorsParams();

    activateLearningMode();

    _calibration_manager = std::make_unique<CalibrationManager>(robot_hwnh, _joint_list, _ttl_interface, _can_interface);
}

/**
 * @brief JointHardwareInterface::read
 */
void JointHardwareInterface::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    int newPositionState = 0.0;

    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isValid())
        {
            if (jState->getBusProtocol() == EBusProtocol::CAN)
                newPositionState = _can_interface->getState(jState->getId()).getPositionState();
            if (jState->getBusProtocol() == EBusProtocol::TTL)
                newPositionState = _ttl_interface->getState(jState->getId()).getPositionState();

            jState->pos = jState->to_rad_pos(newPositionState);
        }
    }

    if ((_can_interface && !_can_interface->isConnectionOk()) || (_ttl_interface && !_ttl_interface->isConnectionOk()))
        this->setNeedCalibration();
}

/**
 * @brief JointHardwareInterface::write: update the position of each joint using the received command from the joint handle
 */
void JointHardwareInterface::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    std::vector<std::pair<uint8_t, int32_t> > can_cmd;
    std::vector<std::pair<uint8_t, uint32_t> > ttl_cmd;

    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isValid())
        {
            if (jState->getBusProtocol() == EBusProtocol::CAN)
                can_cmd.emplace_back(jState->getId(), jState->to_motor_pos(jState->cmd));
            if (jState->getBusProtocol() == EBusProtocol::TTL)
                ttl_cmd.emplace_back(jState->getId(), jState->to_motor_pos(jState->cmd));
        }
    }

    if (_can_interface)
        _can_interface->setTrajectoryControllerCommands(can_cmd);

    if (_ttl_interface)
        _ttl_interface->setTrajectoryControllerCommands(ttl_cmd);
}

/**
 * @brief JointHardwareInterface::initJoints : build the joints by gathering information in config files and instanciating
 * correct state (dxl or stepper)
 */
bool JointHardwareInterface::init(ros::NodeHandle& rootnh, ros::NodeHandle &robot_hwnh)
{
    size_t nb_joints = 0;

    // retrieve nb joints with checking that the config param exists for both name and id
    while (robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "_id") &&
          robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "_name") &&
          robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "_type") &&
          robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "_bus"))
        nb_joints++;

    // connect and register joint state interface
    _joint_list.clear();
    _map_stepper_name.clear();
    _map_dxl_name.clear();

    int currentIdStepper = 1;
    int currentIdDxl = 1;

    for (size_t j = 0; j < nb_joints; j++)
    {
        int joint_id_config = 0;
        string joint_name = "";
        string joint_type = "";
        string joint_bus = "";

        robot_hwnh.getParam("joint_" + to_string(j + 1) + "_id", joint_id_config);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "_name", joint_name);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "_type", joint_type);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "_bus", joint_bus);
        MotorTypeEnum eType = MotorTypeEnum(joint_type.c_str());
        BusProtocolEnum eBusProto = BusProtocolEnum(joint_bus.c_str());

        // gather info in joint  states (polymorphic)
        // CC use factory in state directly ?
        if (eType == EMotorType::STEPPER)
        {  // stepper
            auto stepperState = std::make_shared<StepperMotorState>(joint_name,
                                                                    eType,
                                                                    eBusProto,
                                                                    static_cast<uint8_t>(joint_id_config));
            if (stepperState)
            {
                double offsetPos = 0.0;
                double gear_ratio = 0.0;
                int direction = 1;
                double max_effort = 0.0;

                std::string currentStepperNamespace = "steppers/stepper_" + to_string(currentIdStepper);

                rootnh.getParam(currentStepperNamespace + "/offset_position", offsetPos);
                rootnh.getParam(currentStepperNamespace + "/gear_ratio", gear_ratio);
                rootnh.getParam(currentStepperNamespace + "/direction", direction);
                rootnh.getParam(currentStepperNamespace + "/max_effort", max_effort);

                // add parameters
                stepperState->setOffsetPosition(offsetPos);
                stepperState->setGearRatio(gear_ratio);
                stepperState->setDirection(direction);
                stepperState->setMaxEffort(max_effort);

                _joint_list.emplace_back(stepperState);
                _map_stepper_name[stepperState->getId()] = stepperState->getName();

                currentIdStepper++;
            }
        }
        else if (eType != EMotorType::UNKNOWN)
        {  // dynamixel
            auto dxlState = std::make_shared<DxlMotorState>(joint_name,
                                                            eType,
                                                            eBusProto,
                                                            static_cast<uint8_t>(joint_id_config));
            if (dxlState)
            {
                double offsetPos = 0.0;
                int direction = 1;
                int positionPGain = 0;
                int positionIGain = 0;
                int positionDGain = 0;
                int velocityPGain = 0;
                int velocityIGain = 0;
                int FF1Gain = 0;
                int FF2Gain = 0;

                std::string currentDxlNamespace = "dynamixels/dxl_" + to_string(currentIdDxl);

                rootnh.getParam(currentDxlNamespace + "/offset_position", offsetPos);
                rootnh.getParam(currentDxlNamespace + "/direction", direction);

                rootnh.getParam(currentDxlNamespace + "/position_P_gain", positionPGain);
                rootnh.getParam(currentDxlNamespace + "/position_I_gain", positionIGain);
                rootnh.getParam(currentDxlNamespace + "/position_D_gain", positionDGain);

                rootnh.getParam(currentDxlNamespace + "/velocity_P_gain", velocityPGain);
                rootnh.getParam(currentDxlNamespace + "/velocity_I_gain", velocityIGain);

                rootnh.getParam(currentDxlNamespace + "/FF1_gain", FF1Gain);
                rootnh.getParam(currentDxlNamespace + "/FF2_gain", FF2Gain);

                dxlState->setOffsetPosition(offsetPos);
                dxlState->setDirection(direction);

                dxlState->setPositionPGain(static_cast<uint32_t>(positionPGain));
                dxlState->setPositionIGain(static_cast<uint32_t>(positionIGain));
                dxlState->setPositionDGain(static_cast<uint32_t>(positionDGain));

                dxlState->setVelocityPGain(static_cast<uint32_t>(velocityPGain));
                dxlState->setVelocityIGain(static_cast<uint32_t>(velocityIGain));

                dxlState->setFF1Gain(static_cast<uint32_t>(FF1Gain));
                dxlState->setFF2Gain(static_cast<uint32_t>(FF2Gain));

                _joint_list.emplace_back(dxlState);

                _map_dxl_name[dxlState->getId()] = dxlState->getName();

                currentIdDxl++;
            }
        }

        // register the joints
        if (j < _joint_list.size() && _joint_list.at(j))
        {
            auto jState = _joint_list.at(j);
            if (jState)
            {
                ROS_INFO("JointHardwareInterface::initJoints - New Joints config found : %s", jState->str().c_str());

                hardware_interface::JointStateHandle jStateHandle(jState->getName(),
                                                                  &_joint_list.at(j)->pos,
                                                                  &_joint_list.at(j)->vel,
                                                                  &_joint_list.at(j)->eff);

                _joint_state_interface.registerHandle(jStateHandle);


                hardware_interface::JointHandle jPosHandle(_joint_state_interface.getHandle(jState->getName()),
                                                           &_joint_list.at(j)->cmd);

                _joint_position_interface.registerHandle(jPosHandle);
            }
        }
    }  // end for (size_t j = 0; j < nb_joints; j++)

    // register the interfaces
    registerInterface(&_joint_state_interface);
    registerInterface(&_joint_position_interface);

    return true;
}

/**
 * @brief JointHardwareInterface::sendInitMotorsParams
 */
void JointHardwareInterface::sendInitMotorsParams()
{
    ROS_DEBUG("JointHardwareInterface::sendInitMotorsParams");

    // CMD_TYPE_MICRO_STEPS cmd
    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isStepper() && jState->getBusProtocol() == EBusProtocol::CAN)
        {
            StepperSingleCmd cmd(
                        EStepperCommandType::CMD_TYPE_MICRO_STEPS,
                        jState->getId(),
                        {8});
            _can_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(cmd));
        }
    }
    ros::Duration(0.05).sleep();

    // CMD_TYPE_MAX_EFFORT cmd
    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isStepper() && jState->getBusProtocol() == EBusProtocol::CAN)
        {
            StepperSingleCmd cmd(EStepperCommandType::CMD_TYPE_MAX_EFFORT, jState->getId(),
                                {
                                    static_cast<int32_t>(dynamic_pointer_cast<StepperMotorState>(jState)->getMaxEffort())
                                });
            _can_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(cmd));
        }
    }
    ros::Duration(0.05).sleep();

    //  dynamixels joints PID
    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isDynamixel())
        {
            auto dxlState = dynamic_pointer_cast<DxlMotorState>(jState);
            if (_ttl_interface && jState->getBusProtocol() == EBusProtocol::TTL)
            {
                if (!_ttl_interface->setMotorPID(dxlState))
                {
                    ROS_ERROR("JointHardwareInterface::sendInitMotorsParams - Error setting motor PID for dynamixel id %d",
                            static_cast<int>(dxlState->getId()));
                }
            }
        }
    }
}

/**
 * @brief JointHardwareInterface::setCommandToCurrentPosition
 */
void JointHardwareInterface::setCommandToCurrentPosition()
{
    ROS_DEBUG("Joints Hardware Interface - Set command to current position called");
    for (auto const& jState : _joint_list)
    {
        if (jState)
            _joint_position_interface.getHandle(jState->getName()).setCommand(jState->pos);
    }
}

/**
 * @brief JointHardwareInterface::needCalibration
 * @return
 */
bool JointHardwareInterface::needCalibration() const
{
    bool result = false;
    if (_can_interface)
        result = (EStepperCalibrationStatus::CALIBRATION_OK != _can_interface->getCalibrationStatus());
    else
        result = (EStepperCalibrationStatus::CALIBRATION_OK != _ttl_interface->getCalibrationStatus());
    ROS_DEBUG_THROTTLE(2, "JointHardwareInterface::needCalibration - Need calibration returned: %d",
                       static_cast<int>(result));
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
        calib_res = _calibration_manager->startCalibration(mode, result_message);
    }
    else
    {
        result_message = "JointHardwareInterface::calibrateJoints - Calibration already done";
        calib_res = niryo_robot_msgs::CommandStatus::SUCCESS;
    }

    return calib_res;
}

/**
 * @brief JointHardwareInterface::newCalibration : setNeedCalibration for all steppers
 */
void JointHardwareInterface::setNeedCalibration()
{
    if (_can_interface)
        _can_interface->resetCalibration();
    // TODO(Thuc) implement if calibration with ttl
}

/**
 * @brief JointHardwareInterface::activateLearningMode
 */
void JointHardwareInterface::activateLearningMode()
{
    ROS_DEBUG("JointHardwareInterface::activateLearningMode - activate learning mode");

    common::model::DxlSyncCmd dxl_cmd(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
    common::model::StepperSyncCmd stepper_ttl_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);
    common::model::StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE);

    for (auto const& jState : _joint_list)
    {
        if (jState)
        {
            if (jState->isDynamixel())
            {
                dxl_cmd.addMotorParam(jState->getType(), jState->getId(), 0);
            }
            else if ((jState->isStepper() && jState->getBusProtocol() == EBusProtocol::TTL))
            {
                stepper_ttl_cmd.addMotorParam(jState->getType(), jState->getId(), 0);
            }
            else
            {
                stepper_cmd.setId(jState->getId());
                stepper_cmd.setParams({0});
                _can_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(stepper_cmd));
            }
        }
    }

    // TODO(Thuc): only dxl use sync cmd
    if (_ttl_interface)
    {
        _ttl_interface->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dxl_cmd));
        if (!_can_interface)
        {
            _ttl_interface->setSyncCommand(std::make_shared<common::model::StepperSyncCmd>(stepper_ttl_cmd));
        }
    }
    _learning_mode = true;
}

/**
 * @brief JointHardwareInterface::deactivateLearningMode
 */
void JointHardwareInterface::deactivateLearningMode()
{
    ROS_DEBUG("JointHardwareInterface::deactivateLearningMode - deactivate learning mode");

    common::model::DxlSyncCmd dxl_cmd(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_TORQUE);

    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isValid())
        {
            if (jState->isDynamixel())
            {
                dxl_cmd.addMotorParam(jState->getType(), jState->getId(), 1);
            }
            else if (jState->isStepper())
            {
                stepper_cmd.setId(jState->getId());
                stepper_cmd.setParams({1});

                if (jState->getBusProtocol() == EBusProtocol::CAN)
                {
                    if (_can_interface)
                        _can_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(stepper_cmd));
                }
                else
                {
                    if (_ttl_interface)
                    {
                        // TODO(Thuc): verify it. stepper use 0 torque in learning mode
                        _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(stepper_cmd));
                    }
                }
            }
        }
    }

    if (_ttl_interface)
        _ttl_interface->setSyncCommand(std::make_shared<common::model::DxlSyncCmd>(dxl_cmd));

    _learning_mode = true;
}

/**
 * @brief JointHardwareInterface::synchronizeMotors
 * @param synchronize
 */
void JointHardwareInterface::synchronizeMotors(bool synchronize)
{
    ROS_DEBUG("JointHardwareInterface::synchronizeMotors");

    for (auto const& jState : _joint_list)
    {
        if (jState && jState->isValid() && jState->isStepper() && jState->getBusProtocol() == EBusProtocol::CAN)
        {
            StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_SYNCHRONIZE, jState->getId(), {synchronize});
                _can_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(stepper_cmd));
        }
    }
}

/**
 * @brief JointHardwareInterface::jointIdToJointName
 * @param id
 * @param motor_type
 * @return
 */
string JointHardwareInterface::jointIdToJointName(uint8_t id, EMotorType motor_type) const
{
    if (EMotorType::STEPPER == motor_type && _map_stepper_name.count(id))
        return _map_stepper_name.at(id);
    else if (_map_dxl_name.count(id))
        return _map_dxl_name.at(id);

    return "";
}

}  // namespace joints_interface
