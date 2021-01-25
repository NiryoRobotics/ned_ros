#include <ros/ros.h>
#include "stepper_driver/stepper_driver_core.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stepper_driver_node");
  ros::NodeHandle nodeHandle("~");

  StepperDriver::StepperDriverCore stepper_node();

  ros::spin();
  return 0;
}