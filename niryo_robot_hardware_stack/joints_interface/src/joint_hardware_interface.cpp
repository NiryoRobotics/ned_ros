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
#include <memory>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

// niryo
#include "common/model/bus_protocol_enum.hpp"
#include "common/model/component_type_enum.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/synchronize_motor_cmd.hpp"
#include "common/util/util_defs.hpp"

#include "niryo_robot_database/GetSettings.h"
#include "niryo_robot_database/SetSettings.h"

using ::std::dynamic_pointer_cast;
using ::std::shared_ptr;
using ::std::string;
using ::std::to_string;

using ::common::model::BusProtocolEnum;
using ::common::model::DxlMotorState;
using ::common::model::DxlSingleCmd;
using ::common::model::DxlSyncCmd;
using ::common::model::EBusProtocol;
using ::common::model::EDxlCommandType;
using ::common::model::EHardwareType;
using ::common::model::EStepperCalibrationStatus;
using ::common::model::EStepperCommandType;
using ::common::model::HardwareTypeEnum;
using ::common::model::StepperMotorState;
using ::common::model::StepperSingleCmd;
using ::common::model::StepperTtlSingleCmd;
using ::common::model::StepperTtlSyncCmd;

namespace joints_interface
{
/**
 * @brief JointHardwareInterface::JointHardwareInterface
 * @param rootnh
 * @param robot_hwnh
 * @param ttl_interface
 * @param can_interface
 */
JointHardwareInterface::JointHardwareInterface(ros::NodeHandle& rootnh, ros::NodeHandle& robot_hwnh,
                                               std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                                               std::shared_ptr<can_driver::CanInterfaceCore> can_interface)
  : _ttl_interface(std::move(ttl_interface)), _can_interface(std::move(can_interface))
{
  ROS_DEBUG("JointHardwareInterface::ctor");

  // Used to get or initialize motors home position
  _get_settings_client =
      robot_hwnh.serviceClient<niryo_robot_database::GetSettings>("/niryo_robot_database/settings/get");
  _set_settings_client =
      robot_hwnh.serviceClient<niryo_robot_database::SetSettings>("/niryo_robot_database/settings/set");
  _get_home_position_service = robot_hwnh.advertiseService("/niryo_robot_joints_interface/get_home_position",
                                                           &JointHardwareInterface::callbackGetHomePosition, this);
  _set_home_position_service = robot_hwnh.advertiseService("/niryo_robot_joints_interface/set_home_position",
                                                           &JointHardwareInterface::callbackSetHomePosition, this);
  _reset_home_position_service = robot_hwnh.advertiseService("/niryo_robot_joints_interface/reset_home_position",
                                                             &JointHardwareInterface::callbackResetHomePosition, this);

  init(rootnh, robot_hwnh);

  _calibration_manager =
      std::make_unique<CalibrationManager>(robot_hwnh, _joint_state_list, _ttl_interface, _can_interface);
}

/**
 * @brief JointHardwareInterface::initJoints : build the joints by gathering information in config files and
 * instanciatRing correct state (dxl or stepper)
 */
bool JointHardwareInterface::init(ros::NodeHandle& /*rootnh*/, ros::NodeHandle& robot_hwnh)
{
  bool torque_status{ false };

  robot_hwnh.getParam("/niryo_robot/hardware_version", _hardware_version);
  if (_hardware_version == "ned2" || _hardware_version == "ned3pro")
    torque_status = true;
  else if (_hardware_version == "ned" || _hardware_version == "one")
    torque_status = false;

  size_t nb_joints = 0;

  // retrieve nb joints with checking that the config param exists for both name and id
  while (robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/id") &&
         robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/name") &&
         robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/type") &&
         robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/bus"))
    nb_joints++;

  // connect and register joint state interface
  _joint_state_list.clear();
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
    if (eType == EHardwareType::STEPPER || eType == EHardwareType::NED3PRO_STEPPER ||
        eType == EHardwareType::FAKE_STEPPER_MOTOR)
    {
      // stepper
      std::string currentNamespace = "steppers/stepper_" + to_string(currentIdStepper);

      auto stepperState = std::make_shared<StepperMotorState>(joint_name, eType, common::model::EComponentType::JOINT,
                                                              eBusProto, static_cast<uint8_t>(joint_id_config));

      if (initStepperState(robot_hwnh, stepperState, currentNamespace))
      {
        _joint_state_list.emplace_back(stepperState);
        _map_stepper_name[stepperState->getId()] = stepperState->getName();

        int result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;

        // Try 3 times
        for (int tries = 0; tries < 3; tries++)
        {
          if (EBusProtocol::CAN == eBusProto)
            result = _can_interface->addJoint(stepperState);
          else if (EBusProtocol::TTL == eBusProto)
            result = _ttl_interface->addJoint(stepperState);

          // on success, we initialize the joint and go out of loop
          if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
          {
            result = initHardware(stepperState, torque_status);
            if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
            {
              ROS_INFO("JointHardwareInterface::init - add stepper joint success");
              break;
            }

            ROS_WARN(
                "JointHardwareInterface::init - "
                "initialize stepper joint failure, return : %d. Retrying (%d)...",
                result, tries);
          }
          else
          {
            ROS_WARN(
                "JointHardwareInterface::init - "
                "add stepper joint failure, return : %d. Retrying (%d)...",
                result, tries);
          }
        }

        if (niryo_robot_msgs::CommandStatus::SUCCESS != result)
        {
          ROS_ERROR("JointHardwareInterface::init - Fail to add joint, return : %d", result);
          stepperState->setConnectionStatus(false);
          ros::Duration(0.05).sleep();
        }
      }
      else
      {
        ROS_ERROR("JointHardwareInterface::init - stepper state init failed");
      }

      currentIdStepper++;
    }
    else if (eType != EHardwareType::UNKNOWN)
    {
      // dynamixel
      std::string currentNamespace = "dynamixels/dxl_" + to_string(currentIdDxl);

      auto dxlState = std::make_shared<DxlMotorState>(joint_name, eType, common::model::EComponentType::JOINT,
                                                      static_cast<uint8_t>(joint_id_config));

      if (initDxlState(robot_hwnh, dxlState, currentNamespace))
      {
        _joint_state_list.emplace_back(dxlState);
        _map_dxl_name[dxlState->getId()] = dxlState->getName();

        int result = niryo_robot_msgs::CommandStatus::TTL_READ_ERROR;

        // Try 3 times
        for (int tries = 0; tries < 3; tries++)
        {
          if (EBusProtocol::CAN == eBusProto)
            ROS_ERROR("JointHardwareInterface::init - Dynamixel motors are not available on CAN Bus");
          else if (EBusProtocol::TTL == eBusProto)
            result = _ttl_interface->addJoint(dxlState);

          // on success, we initialize the joint and go out of loop
          if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
          {
            result = initHardware(dxlState, torque_status);
            if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
            {
              ROS_INFO("JointHardwareInterface::init - add dxl joint success");
              break;
            }

            ROS_WARN(
                "JointHardwareInterface::init - "
                "init dxl joint failure, return : %d. Retrying (%d)...",
                result, tries);
          }
          else
          {
            ROS_WARN(
                "JointHardwareInterface::init - "
                "add dxl joint failure, return : %d. Retrying (%d)...",
                result, tries);
          }
        }

        if (niryo_robot_msgs::CommandStatus::SUCCESS != result)
        {
          ROS_ERROR("JointHardwareInterface::init - Fail to add joint, return : %d", result);
          dxlState->setConnectionStatus(false);
          ros::Duration(0.05).sleep();
        }
      }
      else
      {
        ROS_ERROR("JointHardwareInterface::init - dxl state init failed");
      }

      currentIdDxl++;
    }

    // register the joints
    if (j < _joint_state_list.size() && _joint_state_list.at(j))
    {
      auto jState = _joint_state_list.at(j);
      if (jState)
      {
        ROS_DEBUG("JointHardwareInterface::initJoints - New Joints config found : %s", jState->str().c_str());

        hardware_interface::JointStateHandle jStateHandle(jState->getName(), &_joint_state_list.at(j)->pos,
                                                          &_joint_state_list.at(j)->vel, &_joint_state_list.at(j)->eff);

        _joint_state_interface.registerHandle(jStateHandle);

        hardware_interface::JointHandle jPosHandle(_joint_state_interface.getHandle(jState->getName()),
                                                   &_joint_state_list.at(j)->cmd);

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
 * @brief Fetch joints home position from database if they are not already cached
 *
 */
void JointHardwareInterface::updateJointsHomePosition()
{
  // Check if a custom home position is already cached
  if (!_cached_custom_home_positions)
  {
    // Else try to fetch home position from database
    if (_get_settings_client.waitForExistence(ros::Duration(5.0)))
    {
      std::vector<float> tmp_custom_home_positions;
      for (auto& jState : _joint_state_list)
      {
        auto get_request = niryo_robot_database::GetSettings();
        auto setting_name = "custom_home_position_" + jState->getName();
        get_request.request.name = setting_name;
        if (_get_settings_client.call(get_request) &&
            get_request.response.status == niryo_robot_msgs::CommandStatus::SUCCESS &&
            !get_request.response.value.empty())
        {
          tmp_custom_home_positions.push_back(std::stof(get_request.response.value));
        }
      }

      // Check if all joints have a valid custom home position in database
      if (tmp_custom_home_positions.size() == _joint_state_list.size())
      {
        int counter = 0;
        for (auto& jState : _joint_state_list)
        {
          jState->setHomePosition(tmp_custom_home_positions[counter]);
          counter++;
        }

        _cached_custom_home_positions = true;
      }
    }
  }
}

bool JointHardwareInterface::callbackGetHomePosition(niryo_robot_msgs::GetFloatList::Request&,
                                                     niryo_robot_msgs::GetFloatList::Response& res)
{
  ROS_DEBUG("JointHardwareInterface::callbackGetHomePosition - Get home position requested");

  res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
  res.message = "Home positions retrieved successfully";

  updateJointsHomePosition();

  for (auto& jState : _joint_state_list)
  {
    res.values.push_back(jState->getHomePosition());
  }

  return true;
}

bool JointHardwareInterface::callbackSetHomePosition(niryo_robot_msgs::SetFloatList::Request& req,
                                                     niryo_robot_msgs::SetFloatList::Response& res)
{
  ROS_DEBUG("JointHardwareInterface::callbackSetHomePosition - Set new home position requested");

  niryo_robot_database::SetSettings set_service = niryo_robot_database::SetSettings();

  // Check if requested home position values match the joints number
  if (req.values.size() != _joint_state_list.size())
  {
    res.status = niryo_robot_msgs::CommandStatus::FAILURE;
    res.message = "Failed to set home positions : " + std::to_string(_joint_state_list.size()) +
                  " values required for home positions.";
    return true;
  }

  // Check if requested home positions are in valid ranges
  std::string out_of_range_positions = "";
  for (size_t i = 0; i < req.values.size(); ++i)
  {
    if (!_joint_state_list[i]->isValidPosition(req.values[i]))
    {
      out_of_range_positions += _joint_state_list[i]->getName() + " ";
    }
  }
  if (!out_of_range_positions.empty())
  {
    res.status = niryo_robot_msgs::CommandStatus::FAILURE;
    res.message =
        "Failed to set home positions : home position for axis (" + out_of_range_positions + ") are out of range.";
    return true;
  }

  int counter = 0;
  std::vector<float> tmp_custom_home_positions;
  for (auto& jState : _joint_state_list)
  {
    set_service.request.name = "custom_home_position_" + jState->getName();
    set_service.request.type = "float";
    set_service.request.value = std::to_string(req.values[counter]);
    if (_set_settings_client.call(set_service))
    {
      if (set_service.response.status == niryo_robot_msgs::CommandStatus::SUCCESS)
      {
        jState->setHomePosition(req.values[counter]);
      }
      else
      {
        res.status = niryo_robot_msgs::CommandStatus::FAILURE;
        res.message = "Failed to set home position in db for axis " + jState->getName();
        return true;
      }
    }
    else
    {
      res.status = niryo_robot_msgs::CommandStatus::FAILURE;
      res.message = "Failed to call service /niryo_robot_database/settings/set";
      return true;
    }

    counter++;
  }

  _cached_custom_home_positions = true;

  res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
  res.message = "Home positions set successfully";

  return true;
}

bool JointHardwareInterface::callbackResetHomePosition(niryo_robot_msgs::Trigger::Request&,
                                                       niryo_robot_msgs::Trigger::Response& res)
{
  ROS_DEBUG("JointHardwareInterface::callbackResetHomePosition - Reset home position requested");

  auto set_service = niryo_robot_database::SetSettings();

  for (size_t i = 0; i < _joint_state_list.size(); ++i)
  {
    auto& jState = _joint_state_list[i];
    set_service.request.name = "custom_home_position_" + jState->getName();
    set_service.request.type = "float";
    set_service.request.value = "";
    _set_settings_client.call(set_service);

    jState->resetHomePosition();
  }

  _cached_custom_home_positions = false;

  res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
  res.message = "Home positions reset to default values";
  return true;
}

/**
 * @brief JointHardwareInterface::initStepper
 * @param robot_hwnh
 * @param stepperState
 * @param currentNamespace
 * @return
 */
bool JointHardwareInterface::initStepperState(ros::NodeHandle& robot_hwnh,
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
    double default_home_position = 0.0;
    double limit_position_min = 0.0;
    double limit_position_max = 0.0;
    double motor_ratio = 0.0;
    int torque_percentage = 0;

    robot_hwnh.getParam(currentNamespace + "/offset_position", offsetPos);
    robot_hwnh.getParam(currentNamespace + "/gear_ratio", gear_ratio);
    robot_hwnh.getParam(currentNamespace + "/direction", direction);
    robot_hwnh.getParam(currentNamespace + "/max_effort", max_effort);
    robot_hwnh.getParam(currentNamespace + "/default_home_position", default_home_position);
    robot_hwnh.getParam(currentNamespace + "/limit_position_min", limit_position_min);
    robot_hwnh.getParam(currentNamespace + "/limit_position_max", limit_position_max);
    robot_hwnh.getParam(currentNamespace + "/motor_ratio", motor_ratio);
    robot_hwnh.getParam(currentNamespace + "/torque_percentage", torque_percentage);

    // acceleration and velocity profiles (with conversion from RPM and RPM-2)
    common::model::VelocityProfile profile{};
    double data{};
    if (robot_hwnh.hasParam(currentNamespace + "/v_start"))
    {
      robot_hwnh.getParam(currentNamespace + "/v_start", data);
      // v in 0.01 RPM
      profile.v_start = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_TO_RPM * 100);
    }
    if (robot_hwnh.hasParam(currentNamespace + "/a_1"))
    {
      robot_hwnh.getParam(currentNamespace + "/a_1", data);
      profile.a_1 = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_SQ_TO_RPM_SQ);
    }
    if (robot_hwnh.hasParam(currentNamespace + "/v_1"))
    {
      robot_hwnh.getParam(currentNamespace + "/v_1", data);
      profile.v_1 = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_TO_RPM * 100);
    }
    if (robot_hwnh.hasParam(currentNamespace + "/a_max"))
    {
      robot_hwnh.getParam(currentNamespace + "/a_max", data);
      if ("ned3pro" == _hardware_version)
        profile.a_max = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_SQ_TO_RPM_SQ / 214.577);
      else
        profile.a_max = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_SQ_TO_RPM_SQ);
    }
    if (robot_hwnh.hasParam(currentNamespace + "/v_max"))
    {
      robot_hwnh.getParam(currentNamespace + "/v_max", data);
      if ("ned3pro" == _hardware_version)
        profile.v_max = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_TO_RPM * 1000);
      else
        profile.v_max = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_TO_RPM * 100);
    }
    if (robot_hwnh.hasParam(currentNamespace + "/d_max"))
    {
      robot_hwnh.getParam(currentNamespace + "/d_max", data);
      profile.d_max = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_SQ_TO_RPM_SQ);
    }
    if (robot_hwnh.hasParam(currentNamespace + "/d_1"))
    {
      robot_hwnh.getParam(currentNamespace + "/d_1", data);
      profile.d_1 = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_SQ_TO_RPM_SQ);
    }
    if (robot_hwnh.hasParam(currentNamespace + "/v_stop"))
    {
      robot_hwnh.getParam(currentNamespace + "/v_stop", data);
      profile.v_stop = static_cast<uint32_t>(data * RADIAN_PER_SECONDS_TO_RPM * 100);
    }

    // add parameters
    stepperState->setDefaultHomePosition(default_home_position);
    stepperState->setOffsetPosition(offsetPos);
    stepperState->setGearRatio(gear_ratio);
    stepperState->setDirection(static_cast<int8_t>(direction));
    stepperState->setMaxEffort(max_effort);
    stepperState->setVelocityProfile(profile);
    stepperState->setLimitPositionMax(limit_position_max);
    stepperState->setLimitPositionMin(limit_position_min);
    stepperState->setMotorRatio(motor_ratio);
    stepperState->setTorquePercentage(torque_percentage);

    // update ratio used to convert rad to pos motor
    stepperState->updateMultiplierRatio();

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
bool JointHardwareInterface::initDxlState(ros::NodeHandle& robot_hwnh, const std::shared_ptr<DxlMotorState>& dxlState,
                                          const std::string& currentNamespace) const
{
  bool res = false;
  if (dxlState)
  {
    double offsetPos = 0.0;
    double default_home_position = 0.0;
    int direction = 1;
    int positionPGain = 0;
    int positionIGain = 0;
    int positionDGain = 0;
    int velocityPGain = 0;
    int velocityIGain = 0;
    int FF1Gain = 0;
    int FF2Gain = 0;
    int velocityProfile = 0;
    int accelerationProfile = 0;
    double limit_position_min = 0.0;
    double limit_position_max = 0.0;

    robot_hwnh.getParam(currentNamespace + "/offset_position", offsetPos);
    robot_hwnh.getParam(currentNamespace + "/direction", direction);

    robot_hwnh.getParam(currentNamespace + "/position_P_gain", positionPGain);
    robot_hwnh.getParam(currentNamespace + "/position_I_gain", positionIGain);
    robot_hwnh.getParam(currentNamespace + "/position_D_gain", positionDGain);

    robot_hwnh.getParam(currentNamespace + "/velocity_P_gain", velocityPGain);
    robot_hwnh.getParam(currentNamespace + "/velocity_I_gain", velocityIGain);

    robot_hwnh.getParam(currentNamespace + "/FF1_gain", FF1Gain);
    robot_hwnh.getParam(currentNamespace + "/FF2_gain", FF2Gain);

    robot_hwnh.getParam(currentNamespace + "/velocity_profile", velocityProfile);
    robot_hwnh.getParam(currentNamespace + "/acceleration_profile", accelerationProfile);

    robot_hwnh.getParam(currentNamespace + "/default_home_position", default_home_position);
    robot_hwnh.getParam(currentNamespace + "/limit_position_min", limit_position_min);
    robot_hwnh.getParam(currentNamespace + "/limit_position_max", limit_position_max);

    dxlState->setOffsetPosition(offsetPos);
    dxlState->setDirection(static_cast<int8_t>(direction));
    dxlState->setDefaultHomePosition(default_home_position);

    dxlState->setPositionPGain(static_cast<uint32_t>(positionPGain));
    dxlState->setPositionIGain(static_cast<uint32_t>(positionIGain));
    dxlState->setPositionDGain(static_cast<uint32_t>(positionDGain));

    dxlState->setVelocityPGain(static_cast<uint32_t>(velocityPGain));
    dxlState->setVelocityIGain(static_cast<uint32_t>(velocityIGain));

    dxlState->setFF1Gain(static_cast<uint32_t>(FF1Gain));
    dxlState->setFF2Gain(static_cast<uint32_t>(FF2Gain));

    dxlState->setVelProfile(static_cast<uint32_t>(velocityProfile));
    dxlState->setAccProfile(static_cast<uint32_t>(accelerationProfile));

    dxlState->setLimitPositionMin(limit_position_min);
    dxlState->setLimitPositionMax(limit_position_max);

    res = true;
  }

  return res;
}

/**
 * @brief JointHardwareInterface::read
 * Reads the current state of the robot and update pos and vel of
 */
void JointHardwareInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  for (auto& jState : _joint_state_list)
  {
    if (jState && jState->isValid())
    {
      jState->pos = jState->to_rad_pos(jState->getPosition());
      // jState->vel = jState->to_rad_vel(jState->getVelocity());
    }
  }

  if (!needCalibration() &&
      ((_can_interface && !_can_interface->isConnectionOk()) || (_ttl_interface && !_ttl_interface->isConnectionOk())))
    this->setNeedCalibration();
}

/**
 * @brief JointHardwareInterface::write: update the position of each joint using the received command from the joint
 * handle
 */
void JointHardwareInterface::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  std::vector<std::pair<uint8_t, int32_t>> can_cmd;
  std::vector<std::pair<uint8_t, uint32_t>> ttl_cmd;

  for (auto const& jState : _joint_state_list)
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
  for (auto const& jState : _joint_state_list)
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
 * @brief JointHardwareInterface::rebootAll
 * @param torque_on
 * @return
 */
bool JointHardwareInterface::rebootAll(bool torque_on)
{
  _ttl_interface->waitSingleQueueFree();

  bool res = true;
  for (auto const& jState : _joint_state_list)
  {
    if (jState->getBusProtocol() == EBusProtocol::TTL)
    {
      // first set torque state
      if (jState->isStepper())
        _ttl_interface->addSingleCommandToQueue(
            std::make_unique<StepperTtlSingleCmd>(EStepperCommandType::CMD_TYPE_TORQUE, jState->getId(),
                                                  std::initializer_list<uint32_t>{ jState->getTorquePercentage() }));

      ros::Duration(0.2).sleep();

      if (_ttl_interface->rebootHardware(jState))
      {
        initHardware(jState, torque_on);
      }
      else
      {
        ROS_ERROR("JointHardwareInterface::init - Fail to reboot motor id %d", jState->getId());
        res = false;
      }
    }
    // reboot not available for CAN
  }

  // reset learning mode correctly
  activateLearningMode(!torque_on);

  return res;
}

/**
 * @brief JointHardwareInterface::initHardware
 * @param motor_state
 * @param torque_on
 * initializes all joints
 */
int JointHardwareInterface::initHardware(const std::shared_ptr<common::model::JointState>& motor_state, bool torque_on)
{
  ROS_DEBUG("TtlInterfaceCore::initHardware");

  if (motor_state)
  {
    if (motor_state->isStepper())
    {
      auto stepperState = std::dynamic_pointer_cast<common::model::StepperMotorState>(motor_state);
      if (stepperState)
      {
        if (motor_state->getBusProtocol() == EBusProtocol::CAN)
        {
          // CMD_TYPE_MICRO_STEPS cmd
          StepperSingleCmd cmd_micro(EStepperCommandType::CMD_TYPE_MICRO_STEPS, motor_state->getId(),
                                     { static_cast<int32_t>(stepperState->getMicroSteps()) });
          _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(cmd_micro));
          ros::Duration(0.05).sleep();

          // CMD_TYPE_MAX_EFFORT cmd
          StepperSingleCmd cmd_max_effort(EStepperCommandType::CMD_TYPE_MAX_EFFORT, motor_state->getId(),
                                          { static_cast<int32_t>(stepperState->getMaxEffort()) });
          _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(cmd_max_effort));
          ros::Duration(0.05).sleep();
        }
        else if (motor_state->getBusProtocol() == EBusProtocol::TTL)
        {
          // CMD_TYPE_VELOCITY_PROFILE cmd
          _ttl_interface->addSingleCommandToQueue(std::make_unique<StepperTtlSingleCmd>(
              EStepperCommandType::CMD_TYPE_VELOCITY_PROFILE, stepperState->getId(),
              stepperState->getVelocityProfile().to_list()));
          // TORQUE cmd
          _ttl_interface->addSingleCommandToQueue(std::make_unique<StepperTtlSingleCmd>(
              EStepperCommandType::CMD_TYPE_TORQUE, stepperState->getId(),
              std::initializer_list<uint32_t>{ stepperState->getTorquePercentage() }));
        }
      }
    }
    else if (motor_state->isDynamixel())
    {
      auto dxlState = std::dynamic_pointer_cast<common::model::DxlMotorState>(motor_state);
      if (dxlState)
      {
        // TORQUE cmd off to ensure command can be written on the motor
        _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(
            DxlSingleCmd(EDxlCommandType::CMD_TYPE_TORQUE, dxlState->getId(), { false })));

        // set PID
        _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(
            EDxlCommandType::CMD_TYPE_PID, dxlState->getId(),
            std::initializer_list<uint32_t>(
                { dxlState->getPositionPGain(), dxlState->getPositionIGain(), dxlState->getPositionDGain(),
                  dxlState->getVelocityPGain(), dxlState->getVelocityIGain(), dxlState->getFF1Gain(),
                  dxlState->getFF2Gain(), dxlState->getVelProfile(), dxlState->getAccProfile() })));
        // set velocity and acceleration profile
        _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(
            EDxlCommandType::CMD_TYPE_PROFILE, dxlState->getId(),
            std::initializer_list<uint32_t>({ dxlState->getVelProfile(), dxlState->getAccProfile() })));

        // set startup configuration so that torque is on when motor is alimented
        _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(
            EDxlCommandType::CMD_TYPE_STARTUP, dxlState->getId(), std::initializer_list<uint32_t>({ 1 })));

        // TORQUE cmd on if ned2, off otherwise
        _ttl_interface->addSingleCommandToQueue(std::make_unique<DxlSingleCmd>(
            DxlSingleCmd(EDxlCommandType::CMD_TYPE_TORQUE, dxlState->getId(), { torque_on })));
      }
    }
  }
  else
  {
    return niryo_robot_msgs::CommandStatus::FAILURE;
  }

  // wait for empty cmd queue (all init cmd processed correctly)
  _ttl_interface->waitSingleQueueFree();

  return niryo_robot_msgs::CommandStatus::SUCCESS;
}

/**
 * @brief JointHardwareInterface::calibrateJoints
 * @param mode
 * @param result_message
 * @return
 */
int JointHardwareInterface::calibrateJoints(int mode, string& result_message)
{
  result_message.clear();
  int calib_res = niryo_robot_msgs::CommandStatus::ABORTED;

  if (!isCalibrationInProgress())
  {
    if (needCalibration())
    {
      // 0. Fetch home position to make sure it is up to date before performing calibration
      updateJointsHomePosition();

      // 1. change status in interfaces (needed to trigger lights and sound at startup)
      if (_can_interface)
        _can_interface->startCalibration();
      else
        _ttl_interface->startCalibration();

      // sleep for 2 seconds, waiting for light and sound
      ros::Duration(2.0).sleep();

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
 * @brief JointHardwareInterface::factoryCalibrateJoints
 * @param mode
 * @param result_message
 * @return
 */
int JointHardwareInterface::factoryCalibrateJoints(FactoryCalibration::Request::_command_type command,
                                                   FactoryCalibration::Request::_ids_type ids, string& result_message)
{
  if (FactoryCalibration::Request::START == command)
  {
    // 1. change status in interfaces (needed to trigger lights and sound at startup)
    _ttl_interface->startCalibration();

    result_message = "Calibration Interface - Calibration started";
    return _calibration_manager->startFactoryCalibration(command, ids, result_message);
  }

  if (FactoryCalibration::Request::STOP == command)
  {
    result_message = "Calibration Interface - Calibration done";
    return _calibration_manager->startFactoryCalibration(command, ids, result_message);
  }

  result_message =
      std::string("JointHardwareInterface::factoryCalibrateJoints - Command not available: " + std::to_string(command));
  return niryo_robot_msgs::CommandStatus::ABORTED;
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
  _ttl_interface->waitSingleQueueFree();

  ROS_DEBUG("JointHardwareInterface::activateLearningMode - activate learning mode");

  DxlSyncCmd dxl_cmd(EDxlCommandType::CMD_TYPE_LEARNING_MODE);
  StepperTtlSyncCmd stepper_ttl_cmd(EStepperCommandType::CMD_TYPE_TORQUE);

  for (auto const& jState : _joint_state_list)
  {
    if (jState)
    {
      if (jState->getBusProtocol() == EBusProtocol::TTL)
      {
        if (jState->isDynamixel())
          dxl_cmd.addMotorParam(jState->getHardwareType(), jState->getId(), activated);
        else
          stepper_ttl_cmd.addMotorParam(jState->getHardwareType(), jState->getId(),
                                        activated ? 0 : jState->getTorquePercentage());
      }
      else
      {
        StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_LEARNING_MODE);

        stepper_cmd.setId(jState->getId());
        stepper_cmd.setParams({ activated });
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

  for (auto const& jState : _joint_state_list)
  {
    if (jState && jState->isValid() && jState->isStepper() && jState->getBusProtocol() == EBusProtocol::CAN)
    {
      StepperSingleCmd stepper_cmd(EStepperCommandType::CMD_TYPE_SYNCHRONIZE, jState->getId(), { synchronize });
      _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(stepper_cmd));
    }
  }
}

}  // namespace joints_interface
