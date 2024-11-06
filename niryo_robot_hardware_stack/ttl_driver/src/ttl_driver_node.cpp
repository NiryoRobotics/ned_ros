/*
    ttl_driver_node.cpp
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

// ros
#include <memory>
#include <ros/ros.h>
#include <string>

// niryo
#include "ros/serialization.h"
#include "ttl_driver/ttl_interface_core.hpp"

using ::common::model::BusProtocolEnum;
using ::common::model::DxlMotorState;
using ::common::model::EBusProtocol;
using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;
using ::common::model::StepperMotorState;
using ::std::string;
using ::std::to_string;

// Method add joints
void addJointToTtlInterface(const std::shared_ptr<ttl_driver::TtlInterfaceCore>& ttl_interface)
{
  size_t nb_joints = 0;

  ros::NodeHandle robot_hwnh("joints_interface");
  // retrieve nb joints with checking that the config param exists for both name and id
  while (robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/id") &&
         robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/name") &&
         robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/type") &&
         robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/bus"))
    nb_joints++;

  // connect and register joint state interface

  int currentIdDxl = 1;
  int currentIdStepper = 1;

  for (size_t j = 0; j < nb_joints; j++)
  {
    ROS_DEBUG("Initialize stepper motors");
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

    if (eType == EHardwareType::STEPPER || eType == EHardwareType::FAKE_STEPPER_MOTOR)
    {  // stepper
      std::string currentStepperNamespace = "steppers/stepper_" + to_string(currentIdStepper);

      auto stepperState = std::make_shared<StepperMotorState>(joint_name, eType, common::model::EComponentType::JOINT,
                                                              eBusProto, static_cast<uint8_t>(joint_id_config));
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

        robot_hwnh.getParam(currentStepperNamespace + "/offset_position", offsetPos);
        robot_hwnh.getParam(currentStepperNamespace + "/gear_ratio", gear_ratio);
        robot_hwnh.getParam(currentStepperNamespace + "/direction", direction);
        robot_hwnh.getParam(currentStepperNamespace + "/max_effort", max_effort);
        robot_hwnh.getParam(currentStepperNamespace + "/default_home_position", default_home_position);
        robot_hwnh.getParam(currentStepperNamespace + "/limit_position_min", limit_position_min);
        robot_hwnh.getParam(currentStepperNamespace + "/limit_position_max", limit_position_max);
        robot_hwnh.getParam(currentStepperNamespace + "/motor_ratio", motor_ratio);

        // acceleration and velocity profiles
        common::model::VelocityProfile profile{};
        int data{};
        if (robot_hwnh.hasParam(currentStepperNamespace + "/v_start"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/v_start", data);
          profile.v_start = static_cast<uint32_t>(data);
        }

        if (robot_hwnh.hasParam(currentStepperNamespace + "/a_1"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/a_1", data);
          profile.a_1 = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentStepperNamespace + "/v_1"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/v_1", data);
          profile.v_1 = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentStepperNamespace + "/a_max"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/a_max", data);
          profile.a_max = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentStepperNamespace + "/v_max"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/v_max", data);
          profile.v_max = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentStepperNamespace + "/d_max"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/d_max", data);
          profile.d_max = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentStepperNamespace + "/d_1"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/d_1", data);
          profile.d_1 = static_cast<uint32_t>(data);
        }
        if (robot_hwnh.hasParam(currentStepperNamespace + "/v_stop"))
        {
          robot_hwnh.getParam(currentStepperNamespace + "/v_stop", data);
          profile.v_stop = static_cast<uint32_t>(data);
        }

        // add parameters
        stepperState->setOffsetPosition(offsetPos);
        stepperState->setGearRatio(gear_ratio);
        stepperState->setDirection(static_cast<int8_t>(direction));
        stepperState->setMaxEffort(max_effort);
        stepperState->setDefaultHomePosition(default_home_position);
        stepperState->setLimitPositionMax(limit_position_max);
        stepperState->setLimitPositionMin(limit_position_min);
        stepperState->setMotorRatio(motor_ratio);
        stepperState->setVelocityProfile(profile);

        if (eBusProto == EBusProtocol::TTL)
          ttl_interface->addJoint(stepperState);

        currentIdStepper++;
      }
    }
    else if (eType != EHardwareType::UNKNOWN)
    {  // dynamixel
      ROS_DEBUG("Initialize dxl motors");
      auto dxlState = std::make_shared<DxlMotorState>(joint_name, eType, common::model::EComponentType::JOINT,
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
        int velocityProfile = 0;
        int accelerationProfile = 0;
        double limit_position_min = 0.0;
        double limit_position_max = 0.0;
        double default_home_position = 0.0;

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

        robot_hwnh.getParam(currentDxlNamespace + "/velocity_profile", velocityProfile);
        robot_hwnh.getParam(currentDxlNamespace + "/acceleration_profile", accelerationProfile);

        robot_hwnh.getParam(currentDxlNamespace + "/default_home_position", default_home_position);
        robot_hwnh.getParam(currentDxlNamespace + "/limit_position_min", limit_position_min);
        robot_hwnh.getParam(currentDxlNamespace + "/limit_position_max", limit_position_max);

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

        if (eBusProto == EBusProtocol::TTL)
          ttl_interface->addJoint(dxlState);

        currentIdDxl++;
      }
    }
  }  // end for (size_t j = 0; j < nb_joints; j++)
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ttl_driver_node");

  ROS_DEBUG("Launching ttl_driver_node");

  ros::NodeHandle nodeHandle("~");

  std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_node = std::make_shared<ttl_driver::TtlInterfaceCore>(nodeHandle);

  addJointToTtlInterface(ttl_node);

  ros::spin();
  return 0;
}
