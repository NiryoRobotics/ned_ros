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
#include <utility>
#include <vector>
#include <string>
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
using ::common::model::DxlSingleCmd;
using ::common::model::StepperSingleCmd;
using ::common::model::StepperTtlSyncCmd;
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
    _ttl_interface(std::move(ttl_interface)),
    _can_interface(std::move(can_interface))
{
    ROS_DEBUG("JointHardwareInterface::ctor");

    init(rootnh, robot_hwnh);

    _calibration_manager = std::make_unique<CalibrationManager>(robot_hwnh, _joint_list,
                                                                _ttl_interface,
                                                                _can_interface);
}

/**
 * @brief JointHardwareInterface::initJoints : build the joints by gathering information in config files and instanciating
 * correct state (dxl or stepper)
 */
bool JointHardwareInterface::init(ros::NodeHandle& /*rootnh*/, ros::NodeHandle &robot_hwnh)
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
        string joint_name;
        string joint_type;
        string joint_bus;

        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/id", joint_id_config);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/name", joint_name);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/type", joint_type);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/bus", joint_bus);
        HardwareTypeEnum eType = HardwareTypeEnum(joint_type.c_str());
        BusProtocolEnum eBusProto = BusProtocolEnum(joint_bus.c_str());
        // gather info in joint  states (polymorphic)
        // CC use factory in state directly ?
        if (eType == EHardwareType::STEPPER || eType == EHardwareType::FAKE_STEPPER_MOTOR)
        {
            // stepper
            std::string currentNamespace = "steppers/stepper_" + to_string(currentIdStepper);

            auto stepperState = std::make_shared<StepperMotorState>(joint_name,
                                                                    eType,
                                                                    common::model::EComponentType::JOINT,
                                                                    eBusProto,
                                                                    static_cast<uint8_t>(joint_id_config));

            if (initStepperState(robot_hwnh, stepperState, currentNamespace))
            {
                _joint_list.emplace_back(stepperState);
                _map_stepper_name[stepperState->getId()] = stepperState->getName();

                if (EBusProtocol::CAN == eBusProto)
                    _can_interface->addJoint(stepperState);
                else if (EBusProtocol::TTL == eBusProto)
                    _ttl_interface->addJoint(stepperState);
            }

            currentIdStepper++;
        }
        else if (eType != EHardwareType::UNKNOWN)
        {
            // dynamixel
            std::string currentNamespace = "dynamixels/dxl_" + to_string(currentIdDxl);

            auto dxlState = std::make_shared<DxlMotorState>(joint_name,
                                                            eType,
                                                            common::model::EComponentType::JOINT,
                                                            static_cast<uint8_t>(joint_id_config));

            if (initDxlState(robot_hwnh, dxlState, currentNamespace))
            {
                _joint_list.emplace_back(dxlState);
                _map_dxl_name[dxlState->getId()] = dxlState->getName();

                if (EBusProtocol::CAN == eBusProto)
                {
                  ROS_ERROR("JointHardwareInterface::init : Dynamixel motors are not available on CAN Bus");
                }
                else if (EBusProtocol::TTL == eBusProto)
                    _ttl_interface->addJoint(dxlState);
            }
            currentIdDxl++;
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
 * @brief JointHardwareInterface::initStepper
 * @param robot_hwnh
 * @param stepperState
 * @param currentNamespace
 * @return
 */
bool JointHardwareInterface::initStepperState(ros::NodeHandle &robot_hwnh,
                                         const std::shared_ptr<StepperMotorState>& stepperState,
                                         const std::string& currentNamespace) const
{
    bool res = false;
    if (stepperState)
    {
        double offsetPos = 0.0;
        double gear_ratio = 1.0;
        int direction = 1;
        double max_effort = 0.0;
        double home_position = 0.0;
        double limit_position = 0.0;

        robot_hwnh.getParam(currentNamespace + "/offset_position", offsetPos);
        robot_hwnh.getParam(currentNamespace + "/gear_ratio", gear_ratio);
        robot_hwnh.getParam(currentNamespace + "/direction", direction);
        robot_hwnh.getParam(currentNamespace + "/max_effort", max_effort);
        robot_hwnh.getParam(currentNamespace + "/home_position", home_position);
        robot_hwnh.getParam(currentNamespace + "/limit_position", limit_position);

        // acceleration and velocity profiles
        common::model::VelocityProfile profile{};
        int data{};
        if (robot_hwnh.hasParam(currentNamespace + "/v_start"))
        {
            robot_hwnh.getParam(currentNamespace + "/v_start", data);
            profile.v_start = static_cast<uint32_t>(data);
        }

        if (robot_hwnh.hasParam(currentNamespace + "/a_1"))
        {
            robot_hwnh.getParam(currentNamespace + "/a_1", data);
            profile.a_1 = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentNamespace + "/v_1"))
        {
            robot_hwnh.getParam(currentNamespace + "/v_1", data);
            profile.v_1 = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentNamespace + "/a_max"))
        {
            robot_hwnh.getParam(currentNamespace + "/a_max", data);
            profile.a_max = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentNamespace + "/v_max"))
        {
            robot_hwnh.getParam(currentNamespace + "/v_max", data);
            profile.v_max = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentNamespace + "/d_max"))
        {
            robot_hwnh.getParam(currentNamespace + "/d_max", data);
            profile.d_max = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentNamespace + "/d_1"))
        {
            robot_hwnh.getParam(currentNamespace + "/d_1", data);
            profile.d_1 = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentNamespace + "/v_stop"))
        {
            robot_hwnh.getParam(currentNamespace + "/v_stop", data);
            profile.v_stop = static_cast<uint32_t>(data);
        }

        // add parameters
        stepperState->setOffsetPosition(offsetPos);
        stepperState->setGearRatio(gear_ratio);
        stepperState->setDirection(static_cast<int8_t>(direction));
        stepperState->setMaxEffort(max_effort);
        stepperState->setVelocityProfile(profile);
        stepperState->setHomePosition(home_position);
        stepperState->setLimitPosition(limit_position);

        res = true;
    }
    return res;
}

/**
 * @brief JointHardwareInterface::initDxl
 * @param robot_hwnh
 * @param dxlState
 * @param currentNamespace
 * @return
 */
bool JointHardwareInterface::initDxlState(ros::NodeHandle &robot_hwnh,
                                     const std::shared_ptr<DxlMotorState>& dxlState,
                                     const std::string& currentNamespace) const
{
    bool res = false;
    if (dxlState)
    {
        double offsetPos = 0.0;
        double home_position = 0.0;
        int direction = 1;
        int positionPGain = 0;
        int positionIGain = 0;
        int positionDGain = 0;
        int velocityPGain = 0;
        int velocityIGain = 0;
        int FF1Gain = 0;
        int FF2Gain = 0;

        robot_hwnh.getParam(currentNamespace + "/offset_position", offsetPos);
        robot_hwnh.getParam(currentNamespace + "/direction", direction);

        robot_hwnh.getParam(currentNamespace + "/position_P_gain", positionPGain);
        robot_hwnh.getParam(currentNamespace + "/position_I_gain", positionIGain);
        robot_hwnh.getParam(currentNamespace + "/position_D_gain", positionDGain);

        robot_hwnh.getParam(currentNamespace + "/velocity_P_gain", velocityPGain);
        robot_hwnh.getParam(currentNamespace + "/velocity_I_gain", velocityIGain);

        robot_hwnh.getParam(currentNamespace + "/FF1_gain", FF1Gain);
        robot_hwnh.getParam(currentNamespace + "/FF2_gain", FF2Gain);
        robot_hwnh.getParam(currentNamespace + "/home_position", home_position);

        dxlState->setOffsetPosition(offsetPos);
        dxlState->setHomePosition(home_position);
        dxlState->setDirection(static_cast<int8_t>(direction));

        dxlState->setPositionPGain(static_cast<uint32_t>(positionPGain));
        dxlState->setPositionIGain(static_cast<uint32_t>(positionIGain));
        dxlState->setPositionDGain(static_cast<uint32_t>(positionDGain));

        dxlState->setVelocityPGain(static_cast<uint32_t>(velocityPGain));
        dxlState->setVelocityIGain(static_cast<uint32_t>(velocityIGain));

        dxlState->setFF1Gain(static_cast<uint32_t>(FF1Gain));
        dxlState->setFF2Gain(static_cast<uint32_t>(FF2Gain));

        res = true;
    }

    return res;
}

/**
 * @brief JointHardwareInterface::sendInitMotorsParams
 * TODO(CC) : find out where the inits should be done (for tool and conveyor it is in addHardwareComponent() method)
 */
void JointHardwareInterface::sendInitMotorsParams(bool learningMode)
{
    ROS_DEBUG("JointHardwareInterface::sendInitMotorsParams");

    for (auto const& jState : _joint_list)
    {
        if (jState)
        {
            if (jState->isStepper())
            {
                auto stepperState = std::dynamic_pointer_cast<StepperMotorState>(jState);
                if (stepperState)
                {
                    if (jState->getBusProtocol() == EBusProtocol::CAN)
                    {
                      // CMD_TYPE_MICRO_STEPS cmd
                      StepperSingleCmd cmd_micro(
                                  EStepperCommandType::CMD_TYPE_MICRO_STEPS,
                                  jState->getId(),
                                  {static_cast<int32_t>(stepperState->getMicroSteps())});
                      _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(cmd_micro));
                      ros::Duration(0.05).sleep();

                      // CMD_TYPE_MAX_EFFORT cmd
                      StepperSingleCmd cmd_max_effort(
                                  EStepperCommandType::CMD_TYPE_MAX_EFFORT,
                                  jState->getId(),
                                  {static_cast<int32_t>(stepperState->getMaxEffort())});
                      _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(cmd_max_effort));
                      ros::Duration(0.05).sleep();
                    }
                    else if (jState->getBusProtocol() == EBusProtocol::TTL)
                    {
                      // CMD_TYPE_VELOCITY_PROFILE cmd
                      StepperTtlSingleCmd cmd_profile(
                                  EStepperCommandType::CMD_TYPE_VELOCITY_PROFILE,
                                  stepperState->getId(),
                                  stepperState->getVelocityProfile().to_list());
                      _ttl_interface->addSingleCommandToQueue(std::make_unique<StepperTtlSingleCmd>(cmd_profile));
                      ros::Duration(0.05).sleep();
                    }
                }
            }

            if (jState->isDynamixel())
            {
                auto dxlState = dynamic_pointer_cast<DxlMotorState>(jState);
                if (dxlState && jState->getBusProtocol() == EBusProtocol::TTL)
                {
                    // CMD_TYPE_PID cmd
                    DxlSingleCmd cmd_pid(
                                  EDxlCommandType::CMD_TYPE_PID,
                                  dxlState->getId(), {dxlState->getPositionPGain(),
                                                      dxlState->getPositionIGain(),
                                                      dxlState->getPositionDGain(),
                                                      dxlState->getVelocityPGain(),
                                                      dxlState->getVelocityIGain(),
                                                      dxlState->getFF1Gain(),
                                                      dxlState->getFF2Gain()});

                    _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(cmd_pid));
                    ros::Duration(0.05).sleep();
                }
            }
        }
    }

    activateLearningMode(learningMode);
}

/**
 * @brief JointHardwareInterface::read
 * Reads the current state of the robot and update pos and vel of
 */
void JointHardwareInterface::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    for (auto& jState : _joint_list)
    {
        if (jState && jState->isValid())
        {
            jState->pos = jState->to_rad_pos(jState->getPosition());
            jState->vel = jState->to_rad_vel(jState->getVelocity());
        }
    }

    if (!needCalibration() && ((_can_interface && !_can_interface->isConnectionOk()) || (_ttl_interface && !_ttl_interface->isConnectionOk())))
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
        _can_interface->setTrajectoryControllerCommands(std::move(can_cmd));

    if (_ttl_interface)
      _ttl_interface->setTrajectoryControllerCommands(std::move(ttl_cmd));
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
    return (EStepperCalibrationStatus::OK != _calibration_manager->getCalibrationStatus() &&
            EStepperCalibrationStatus::IN_PROGRESS != _calibration_manager->getCalibrationStatus());
}

/**
 * @brief JointHardwareInterface::isCalibrationInProgress
 * @return
 */
bool JointHardwareInterface::isCalibrationInProgress() const
{
    return (EStepperCalibrationStatus::IN_PROGRESS == _calibration_manager->getCalibrationStatus());
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
            // sleep for 3 seconds, waiting for light and sound
            ros::Duration(3.0).sleep();

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
    while (!_ttl_interface->isSingleQueueFree())
    {
        ros::Duration(0.05).sleep();
    }

    ROS_DEBUG("JointHardwareInterface::activateLearningMode - activate learning mode");

    DxlSyncCmd dxl_cmd(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
    StepperTtlSyncCmd stepper_ttl_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);

    for (auto const& jState : _joint_list)
    {
        if (jState)
        {
            if (jState->getBusProtocol() == EBusProtocol::TTL)
            {
                if (jState->isDynamixel())
                  dxl_cmd.addMotorParam(jState->getHardwareType(), jState->getId(), activated);
                else
                  stepper_ttl_cmd.addMotorParam(jState->getHardwareType(), jState->getId(), activated);
            }
            else
            {
                StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);

                stepper_cmd.setId(jState->getId());
                stepper_cmd.setParams({activated});
                if (_can_interface)
                    _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(stepper_cmd));
            }
        }
    }

    if (_ttl_interface)
    {
        if (dxl_cmd.isValid())
          _ttl_interface->addSyncCommandToQueue(std::make_unique<DxlSyncCmd>(dxl_cmd));

        if (stepper_ttl_cmd.isValid())
          _ttl_interface->addSyncCommandToQueue(std::make_unique<StepperTtlSyncCmd>(stepper_ttl_cmd));
    }
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
                _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(stepper_cmd));
        }
    }
}

}  // namespace joints_interface
