#include <joints_driver/joints_driver.hpp>
#include <ttl_driver/ttl_driver_core.hpp>
#include <can_driver/can_driver_core.hpp>

#include <string>
#include <vector>

using ::std::vector;
using ::std::string;
using ::ttl_driver::TtlDriverCore;
using ::can_driver::CanDriverCore;

namespace joint_driver
{

JointDriver::JointDriver(ros::NodeHandle nh)
{
  _nh = nh;
  ROS_DEBUG("JointsDriver - ctor");
  init();
}

JointDriver::~JointDriver()
{
}

void JointDriver::init()
{
  vector<string> drivers_list;
  _nh.getParam("/niryo_robot_hardware_interface/joints_driver/drivers_params/drivers_list", drivers_list);

  for (auto it = drivers_list.begin(); it != drivers_list.end(); it++)
  {
    if (*it == "can")
    {
      CanDriverCore can_driver(_nh);
    }
    else if (*it == "ttl")
    {
      TtlDriverCore ttl_driver(_nh);
    }
  }
}

} // namespace joint_driver