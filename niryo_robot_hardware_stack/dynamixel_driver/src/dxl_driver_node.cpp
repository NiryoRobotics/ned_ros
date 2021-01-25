#include <ros/ros.h>
#include "dynamixel_driver/dxl_driver_core.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_driver_node");
  ros::NodeHandle nodeHandle("~");

  DynamixelDriver::DynamixelDriverCore dxl_node();

  ros::spin();
  return 0;
}