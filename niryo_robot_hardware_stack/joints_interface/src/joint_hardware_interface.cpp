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
#include "common/model/single_motor_cmd.hpp"
#include "common/model/synchronize_motor_cmd.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/component_type_enum.hpp"
#include "common/model/bus_protocol_enum.hpp"
#include "common/util/util_defs.hpp"

using ::std::shared_ptr;
using ::std::string;
using ::std::to_string;
using ::std::dynamic_pointer_cast;

using ::common::model::HardwareTypeEnum;
using ::common::model::EHardwareType;
using ::common::model::EBusProtocol;
using ::common::model::BusProtocolEnum;
using ::common::model::StepperMotorState;
using ::common::model::DxlMotorState;
using ::common::model::DxlSyncCmd;
using ::common::model::StepperSingleCmd;
using ::common::model::StepperTtlSyncCmd;
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
    activateLearningMode(true);
    _calibration_manager = std::make_unique<CalibrationManager>(robot_hwnh, _joint_list, _ttl_interface, _can_interface);
}

/**
 * @brief JointHardwareInterface::initJoints : build the joints by gathering information in config files and instanciating
 * correct state (dxl or stepper)
 */
bool JointHardwareInterface::init(ros::NodeHandle& rootnh, ros::NodeHandle &robot_hwnh)
{
    size_t nb_joints = 0;

    // retrieve nb joints with checking that the config param exists for both name and id
    while (robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/id") &&
          robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/name") &&
          robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/type") &&
          robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/bus"))
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

        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/id", joint_id_config);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/name", joint_name);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/type", joint_type);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/bus", joint_bus);
        HardwareTypeEnum eType = HardwareTypeEnum(joint_type.c_str());
        BusProtocolEnum eBusProto = BusProtocolEnum(joint_bus.c_str());

        // gather info in joint  states (polymorphic)
        // CC use factory in state directly ?
        if (eType == EHardwareType::STEPPER || eType == EHardwareType::FAKE_STEPPER_MOTOR)
        {  // stepper
            std::string currentStepperNamespace = "steppers/stepper_" + to_string(currentIdStepper);

            auto stepperState = std::make_shared<StepperMotorState>(joint_name,
                                                                    eType,
                                                                    common::model::EComponentType::JOINT,
                                                                    eBusProto,
                                                                    static_cast<uint8_t>(joint_id_config));
            if (stepperState)
            {
                double offsetPos = 0.0;
                double gear_ratio = 1.0;
                int direction = 1;
                double max_effort = 0.0;

                robot_hwnh.getParam(currentStepperNamespace + "/offset_position", offsetPos);
                robot_hwnh.getParam(currentStepperNamespace + "/gear_ratio", gear_ratio);
                robot_hwnh.getParam(currentStepperNamespace + "/direction", direction);
                robot_hwnh.getParam(currentStepperNamespace + "/max_effort", max_effort);

                // add parameters
                stepperState->setOffsetPosition(offsetPos);
                stepperState->setGearRatio(gear_ratio);
                stepperState->setDirection(direction);
                stepperState->setMaxEffort(max_effort);

                _joint_list.emplace_back(stepperState);
                _map_stepper_name[stepperState->getId()] = stepperState->getName();

                if (eBusProto == EBusProtocol::CAN)
                    _can_interface->addJoint(stepperState);
                else if (eBusProto == EBusProtocol::TTL)
                    _ttl_interface->addJoint(stepperState);

                currentIdStepper++;
            }
        }
        else if (eType != EHardwareType::UNKNOWN)
        {  // dynamixel
            auto dxlState = std::make_shared<DxlMotorState>(joint_name,
                                                            eType,
                                                            common::model::EComponentType::JOINT,
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

                robot_hwnh.getParam(currentDxlNamespace + "/offset_position", offsetPos);
                robot_hwnh.getParam(currentDxlNamespace + "/direction", direction);

                robot_hwnh.getParam(currentDxlNamespace + "/position_P_gain", positionPGain);
                robot_hwnh.getParam(currentDxlNamespace + "/position_I_gain", positionIGain);
                robot_hwnh.getParam(currentDxlNamespace + "/position_D_gain", positionDGain);

                robot_hwnh.getParam(currentDxlNamespace + "/velocity_P_gain", velocityPGain);
                robot_hwnh.getParam(currentDxlNamespace + "/velocity_I_gain", velocityIGain);

                robot_hwnh.getParam(currentDxlNamespace + "/FF1_gain", FF1Gain);
                robot_hwnh.getParam(currentDxlNamespace + "/FF2_gain", FF2Gain);

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

                if (eBusProto == EBusProtocol::CAN)
                {
                  ROS_ERROR("JointHardwareInterface::init : Dynamixel motors are not available on CAN Bus");
                }
                else if (eBusProto == EBusProtocol::TTL)
                    _ttl_interface->addJoint(dxlState);

                currentIdDxl++;
            }
        }

        // register the joints
        if (j < _joint_list.size() && _joint_list.at(j))
        {
            auto jState = _joint_list.at(j);
            if (jState)
            {
                ROS_DEBUG("JointHardwareInterface::initJoints - New Joints config found : %s", jState->str().c_str());

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
            ros::Duration(0.05).sleep();
        }
    }
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
            ros::Duration(0.05).sleep();
        }
    }
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
 * @brief JointHardwareInterface::read
 */
void JointHardwareInterface::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    int newPositionState = 0.0;

    for (auto& jState : _joint_list)
    {
        if (jState && jState->isValid())
        {
            newPositionState = jState->getPositionState();

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
    result_message.clear();
    int calib_res = niryo_robot_msgs::CommandStatus::ABORTED;

    if (!isCalibrationInProgress())
    {
        if (needCalibration())
        {
          calib_res = _calibration_manager->startCalibration(mode, result_message);
        }
        else
        {
            result_message = "JointHardwareInterface::calibrateJoints - Calibration already done";
            calib_res = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
    }
    else
    {
        result_message = "JointHardwareInterface::calibrateJoints - Calibration already in process";
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
    else
        _ttl_interface->resetCalibration();
}

/**
 * @brief JointHardwareInterface::activateLearningMode
 */
void JointHardwareInterface::activateLearningMode(bool activated)
{
    ROS_DEBUG("JointHardwareInterface::activateLearningMode - activate learning mode");

    DxlSyncCmd dxl_cmd(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
    StepperTtlSyncCmd stepper_ttl_sync_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);
    StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);

    for (auto const& jState : _joint_list)
    {
        if (jState)
        {
            if (jState->isDynamixel())
            {
                dxl_cmd.addMotorParam(jState->getHardwareType(), jState->getId(), activated);
            }
            else if ((jState->isStepper() && jState->getBusProtocol() == EBusProtocol::TTL))
            {
                stepper_ttl_sync_cmd.addMotorParam(jState->getHardwareType(), jState->getId(), activated);
            }
            else
            {
                stepper_cmd.setId(jState->getId());
                stepper_cmd.setParams({activated});
                if(_can_interface)
                    _can_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(stepper_cmd));
            }
        }
    }

    if (_ttl_interface)
    {
        if(dxl_cmd.isValid())
            _ttl_interface->setSyncCommand(std::make_shared<DxlSyncCmd>(dxl_cmd));
        if(stepper_ttl_sync_cmd.isValid())
            _ttl_interface->setSyncCommand(std::make_shared<StepperTtlSyncCmd>(stepper_ttl_sync_cmd));
    }

    _learning_mode = activated;
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

}  // namespace joints_interface
