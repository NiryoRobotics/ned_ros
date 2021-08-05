/*
    joints_driver.cpp
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

#include "joints_driver/joints_driver.hpp"
#include "ttl_driver/ttl_interface_core.hpp"
#include "can_driver/can_interface_core.hpp"

#include <string>
#include <vector>

using ::std::vector;
using ::std::string;
using ::ttl_driver::TtlInterfaceCore;
using ::can_driver::CanInterfaceCore;

namespace joint_driver
{
/**
 * @brief JointDriver::JointDriver
*/
JointDriver::JointDriver(ros::NodeHandle &nh)
{
  ROS_DEBUG("JointsDriver - ctor");

  init(nh);
}

/**
 * @brief JointDriver::~JointDriver
*/
JointDriver::~JointDriver()
{
}

/**
 * @brief JointDriver::~init
 * @param nh
*/
void JointDriver::init(ros::NodeHandle &nh)
{
  ROS_DEBUG("JointDriver: initialize");
  size_t nb_joints = 0;
  vector<string> drivers_list;
  vector<string> protocols_list;

  // get params in file config
  // get config in joints_driver package
  nh.getParam("drivers_params/drivers_list", drivers_list);

  // get config in joint interface
  while (nh.hasParam("joint_" + std::to_string(nb_joints + 1) + "_name") &&
          nh.hasParam("joint_" + std::to_string(nb_joints + 1) + "_protocol"))
    nb_joints++;

  for (auto it : drivers_list)
  {
    if (it == "can" && _haveCan == false)
    {
      ROS_DEBUG("JointDriver: Create Can driver core");
      ros::NodeHandle nh_can(nh, "can_driver");
      _haveCan = true;
      _canInterfaceCore.reset(new CanInterfaceCore(nh_can));
    }
    else if (it == "ttl" && _haveTtl == false)
    {
      ROS_DEBUG("JointsDriver: Create Ttl driver core");
      ros::NodeHandle nh_ttl(nh, "ttl_driver");
      _haveTtl = true;
      _ttlInterfaceCore.reset(new TtlInterfaceCore(nh_ttl));
    }
  }

  for (size_t j = 0; j < nb_joints; j++)
  {
    string joint_name;
    string protocol;

    nh.getParam("joint_" + std::to_string(j + 1) + "_name", joint_name);
    nh.getParam("joint_" + std::to_string(j + 1) + "_protocol", protocol);

    if (protocol == "can")
    {
      _m_name_proto.insert(std::make_pair(joint_name, _canInterfaceCore));
    }
    else if (protocol == "ttl")
    {
      _m_name_proto.insert(std::make_pair(joint_name, _ttlInterfaceCore));
    }
    else
      ROS_ERROR("Can't recognize protocol used for motor %s. Verify file config", joint_name.c_str());
  }
}

/**
 * @brief JointDriver::getTtlInterfaceCore
*/
std::shared_ptr<ttl_driver::TtlInterfaceCore> JointDriver::getTtlInterfaceCore() const
{
    return _ttlInterfaceCore;
}

/**
 * @brief JointDriver::getCanInterfaceCore
*/
std::shared_ptr<can_driver::CanInterfaceCore>
JointDriver::getCanInterfaceCore() const
{
    return _canInterfaceCore;
}

/**
 * @brief JointDriver::getProtocolOfMotor
*/
std::shared_ptr<common::model::IDriverCore> JointDriver::getProtocolOfMotor(std::string name) const
{
    return _m_name_proto.at(name);
}

/**
 * @brief JointDriver::haveCan
*/
bool JointDriver::haveCan() const
{
    return _haveCan;
}

/**
 * @brief JointDriver::haveTtl
*/
bool JointDriver::haveTtl() const
{
    return _haveTtl;
}

}  // namespace joint_driver
