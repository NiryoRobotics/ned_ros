#include <joints_driver/joints_driver.hpp>
#include <ttl_driver/ttl_interface_core.hpp>
#include <can_driver/can_driver_core.hpp>

#include <string>
#include <vector>

using ::std::vector;
using ::std::string;
using ::ttl_driver::TtlInterfaceCore;
using ::can_driver::CanDriverCore;

namespace joint_driver
{
JointDriver::JointDriver(ros::NodeHandle &nh)
{
  ROS_DEBUG("JointsDriver - ctor");

  _haveCan = false;
  _haveTtl = false;
  init(nh);
}

JointDriver::~JointDriver()
{
}

void JointDriver::init(ros::NodeHandle &nh)
{
  ROS_DEBUG("JointDriver: initialize");
  size_t nb_joints = 0;
  vector<string> drivers_list;
  vector<string> protocols_list;

  // get params in file config
  // get config in joints_driver package
  ros::NodeHandle nh_joints_drv(nh, "joints_driver");
  nh_joints_drv.getParam("drivers_params/drivers_list", drivers_list);

  // get config in joint interface
  while (nh_joints_drv.hasParam("joint_" + std::to_string(nb_joints + 1) + "_name") &&
          nh_joints_drv.hasParam("joint_" + std::to_string(nb_joints + 1) + "_protocol"))
    nb_joints++;

  for (auto it : drivers_list)
  {
    if (it == "can" && _haveCan == false)
    {
      ROS_DEBUG("JointDriver: Create Can driver core");
      ros::NodeHandle nh_can(nh, "can_driver");
      _haveCan = true;
      _canDriverCore.reset(new CanDriverCore(nh_can));
    }
    else if (it == "ttl" && _haveTtl == false)
    {
      ROS_DEBUG("JointsDriver: Create Ttl driver core");
      ros::NodeHandle nh_ttl(nh, "ttl_driver");
      _haveTtl = true;
      _ttlDriverCore.reset(new TtlInterfaceCore(nh_ttl));
    }
  }

  for (size_t j = 0; j < nb_joints; j++)
  {
    string joint_name;
    string protocol;

    nh_joints_drv.getParam("joint_" + std::to_string(j + 1) + "_name", joint_name);
    nh_joints_drv.getParam("joint_" + std::to_string(j + 1) + "_protocol", protocol);
    
    if (protocol == "can")
    {
      _m_name_proto.insert(std::make_pair(joint_name, _canDriverCore));
    }
    else if (protocol == "ttl")
    {
      _m_name_proto.insert(std::make_pair(joint_name, _ttlDriverCore));
    }
    else
      ROS_ERROR("Can't recognize protocol used for motor %s. Verify file config", joint_name.c_str());
  }
}

/**
 * @brief JointDriver::getTtlDriverCore
*/
std::shared_ptr<ttl_driver::TtlInterfaceCore> JointDriver::getTtlDriverCore() const
{
    return _ttlDriverCore;
}

/**
 * @brief JointDriver::getCanDriverCore
*/
std::shared_ptr<can_driver::CanDriverCore> JointDriver::getCanDriverCore() const
{
    return _canDriverCore;
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

} // namespace joint_driver