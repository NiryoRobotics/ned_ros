// ros
#include <ros/ros.h>
#include <joints_driver/joints_driver.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "driver_node");

  ROS_DEBUG("Launching driver_node");

  ros::NodeHandle nodeHandle("~");

  joint_driver::JointDriver drv_node(nodeHandle);

  ros::spin();
  return 0;
}
